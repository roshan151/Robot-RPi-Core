"""Short-horizon tracking between REST answers. VISION-DESIGN.md Part 4.2.

REST answers arrive at ~2 Hz, 100-500 ms late. Between them there is no new
information unless the Pi makes some, so the Pi runs sparse Lucas-Kanade
optical flow on the small `lores` stream at TRACK_HZ:

    1. goodFeaturesToTrack inside the box
    2. calcOpticalFlowPyrLK to the next frame, with a forward-backward check
    3. median translation, median spread ratio for scale
    4. fewer than MIN_INLIERS surviving points -> the track is LOST, honestly

Re-anchoring is the subtle part. A detection for seq S describes the world as it
was at S, 0.5-1.5 s ago. Slamming it into the current box makes the head jump
at REST rate. Instead: find the track's OWN box at seq S (history), IOU-match
the detection against that, and apply the DIFFERENCE to the current box. That
corrects drift without teleporting to a stale measurement.

All boxes here are in tracker-image pixels (lores). Callers convert detections
into that space before anchoring. cv2 is imported lazily so the rest of the
package imports on a machine without OpenCV.
"""

from __future__ import annotations

import itertools
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Sequence

import numpy as np

from .geometry import BBox, clip_bbox, iou, median

_ids = itertools.count(1)


@dataclass
class Track:
    id: int
    cls: str
    bbox: BBox
    confidence: float
    born_t: float
    last_anchor_t: float
    last_anchor_seq: int
    lost: bool = False
    lost_since: Optional[float] = None
    misses: int = 0                      # detection cycles with no matching detection
    identity: Optional[str] = None       # sticky once assigned (identity.py)
    identity_score: float = 0.0
    history: deque = field(default_factory=lambda: deque(maxlen=40))   # (seq, bbox), ~2.5 s at 15 Hz

    def box_at(self, seq: int) -> Optional[BBox]:
        """Own box at seq, or the nearest earlier one we kept."""
        best = None
        for s, b in self.history:
            if s <= seq:
                best = b
            else:
                break
        return best

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)


@dataclass
class TrackerConfig:
    min_inliers: int = 8
    max_corners: int = 30
    fb_max_err_px: float = 1.5           # forward-backward consistency
    anchor_iou: float = 0.25
    max_misses: int = 4                  # detection cycles unmatched before a track is dropped
    reacquire_s: float = 5.0             # a LOST track is kept this long, then dies
    max_tracks: int = 8
    classes: Optional[frozenset] = None  # None = track every class detected


class TrackManager:
    """Owns every track. Call `step()` per tracker frame, `anchor()` per detect result."""

    def __init__(self, width: int, height: int, config: Optional[TrackerConfig] = None) -> None:
        self.wh = (width, height)
        self.cfg = config or TrackerConfig()
        self.tracks: dict[int, Track] = {}
        self._prev: Optional[np.ndarray] = None

    # ------------------------------------------------------------ per frame

    def step(self, gray: np.ndarray, seq: int, t: float) -> None:
        """Advance every live track from the previous frame to this one."""
        prev, self._prev = self._prev, gray
        for tr in list(self.tracks.values()):
            if prev is not None and not tr.lost:
                moved = _flow_box(prev, gray, tr.bbox, self.cfg)
                if moved is None:
                    tr.lost, tr.lost_since = True, t
                else:
                    tr.bbox = clip_bbox(moved, self.wh)
            tr.history.append((seq, tr.bbox))
            if tr.lost and tr.lost_since is not None and t - tr.lost_since > self.cfg.reacquire_s:
                del self.tracks[tr.id]

    # ------------------------------------------------------- per detection

    def anchor(self, detections: Sequence[tuple[str, float, BBox]], seq: int, t: float) -> list[Track]:
        """Fold one detect result (already in tracker pixels) into the tracks.

        Returns the tracks created by this call.
        """
        dets = [d for d in detections if self.cfg.classes is None or d[0] in self.cfg.classes]
        # Greedy IOU matching against each track's box AT THE DETECTION'S SEQ.
        pairs = []
        for ti, tr in self.tracks.items():
            then = tr.box_at(seq) or tr.bbox
            for di, (cls, _conf, box) in enumerate(dets):
                if cls == tr.cls:
                    score = iou(then, box)
                    if score >= self.cfg.anchor_iou:
                        pairs.append((score, ti, di, then))
        pairs.sort(reverse=True)
        used_t, used_d = set(), set()
        for _score, ti, di, then in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            tr = self.tracks[ti]
            cls, conf, box = dets[di]
            if tr.lost:
                # Nothing has moved the box since it was lost, so the detection
                # is the best we have. Take it whole.
                tr.bbox = box
            else:
                tr.bbox = clip_bbox(_apply_correction(tr.bbox, then, box), self.wh)
            tr.confidence = conf
            tr.lost, tr.lost_since, tr.misses = False, None, 0
            tr.last_anchor_t, tr.last_anchor_seq = t, seq

        for ti, tr in list(self.tracks.items()):
            if ti not in used_t:
                tr.misses += 1
                if tr.misses > self.cfg.max_misses and not tr.lost:
                    tr.lost, tr.lost_since = True, t

        created = []
        for di, (cls, conf, box) in enumerate(dets):
            if di in used_d or len(self.tracks) >= self.cfg.max_tracks:
                continue
            tr = Track(id=next(_ids), cls=cls, bbox=box, confidence=conf, born_t=t,
                       last_anchor_t=t, last_anchor_seq=seq)
            tr.history.append((seq, box))
            self.tracks[tr.id] = tr
            created.append(tr)
        return created

    # ------------------------------------------------------------- queries

    def live(self, cls: Optional[str] = None) -> list[Track]:
        return [t for t in self.tracks.values() if not t.lost and (cls is None or t.cls == cls)]

    def get(self, track_id: int) -> Optional[Track]:
        return self.tracks.get(track_id)

    def reset(self) -> None:
        self.tracks.clear()
        self._prev = None


# ---------------------------------------------------------------------------
# internals
# ---------------------------------------------------------------------------

def _apply_correction(current: BBox, then: BBox, measured: BBox) -> BBox:
    """Shift and rescale `current` by how far `then` was from `measured`."""
    tcx, tcy = (then[0] + then[2]) / 2, (then[1] + then[3]) / 2
    mcx, mcy = (measured[0] + measured[2]) / 2, (measured[1] + measured[3]) / 2
    sw = (measured[2] - measured[0]) / max(1.0, then[2] - then[0])
    sh = (measured[3] - measured[1]) / max(1.0, then[3] - then[1])
    ccx, ccy = (current[0] + current[2]) / 2 + (mcx - tcx), (current[1] + current[3]) / 2 + (mcy - tcy)
    hw, hh = (current[2] - current[0]) * sw / 2, (current[3] - current[1]) * sh / 2
    return (ccx - hw, ccy - hh, ccx + hw, ccy + hh)


def _flow_box(prev: np.ndarray, cur: np.ndarray, bbox: BBox, cfg: TrackerConfig) -> Optional[BBox]:
    import cv2  # lazy: only the tracker needs OpenCV

    h, w = prev.shape[:2]
    x1, y1, x2, y2 = (int(round(v)) for v in bbox)
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    if x2 - x1 < 4 or y2 - y1 < 4:
        return None
    mask = np.zeros_like(prev)
    mask[y1:y2, x1:x2] = 255
    p0 = cv2.goodFeaturesToTrack(prev, maxCorners=cfg.max_corners, qualityLevel=0.01,
                                 minDistance=3, mask=mask)
    if p0 is None or len(p0) < cfg.min_inliers:
        return None
    lk = dict(winSize=(15, 15), maxLevel=2,
              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(prev, cur, p0, None, **lk)
    p0r, st2, _ = cv2.calcOpticalFlowPyrLK(cur, prev, p1, None, **lk)
    fb = np.linalg.norm((p0 - p0r).reshape(-1, 2), axis=1)
    good = (st1.ravel() == 1) & (st2.ravel() == 1) & (fb < cfg.fb_max_err_px)
    if int(good.sum()) < cfg.min_inliers:
        return None
    a, b = p0.reshape(-1, 2)[good], p1.reshape(-1, 2)[good]
    dx = median((b[:, 0] - a[:, 0]).tolist())
    dy = median((b[:, 1] - a[:, 1]).tolist())
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    ra = np.linalg.norm(a - ca, axis=1)
    rb = np.linalg.norm(b - cb, axis=1)
    ok = ra > 1e-3
    scale = median((rb[ok] / ra[ok]).tolist()) if ok.any() else 1.0
    scale = float(min(1.2, max(0.8, scale)))     # one frame never changes size by 20%
    cx, cy = (bbox[0] + bbox[2]) / 2 + dx, (bbox[1] + bbox[3]) / 2 + dy
    hw, hh = (bbox[2] - bbox[0]) * scale / 2, (bbox[3] - bbox[1]) * scale / 2
    return (cx - hw, cy - hh, cx + hw, cy + hh)
