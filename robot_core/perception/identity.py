"""Identity is a property of a track, not a frame. VISION-DESIGN.md Part 6.

    new person track -> wait for a usable crop -> embed -> match
    two consecutive matches, same label, above threshold -> identity, STICKY
    never re-query that track; a new track starts from scratch

This module is the bookkeeping only. It says WHEN a track wants an embed and
WHAT to conclude from each answer; the caller does the HTTP (off the capture
thread) and reports back. That split keeps it testable with no network.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

UNKNOWN = "unknown"


@dataclass
class _State:
    attempts: int = 0
    last_label: Optional[str] = None
    agree: int = 0
    last_try_t: float = -1e9
    pending: bool = False
    decided: Optional[str] = None
    score: float = 0.0


@dataclass
class IdentityConfig:
    min_quality: float = 0.6          # the server's quality score; below this, don't match
    agree_needed: int = 2             # consecutive agreeing matches before assigning
    retry_interval_s: float = 0.7     # spacing between attempts on one track
    max_attempts: int = 8             # after this many, the track stays UNKNOWN
    min_box_px: float = 40.0          # person boxes shorter than this are too far to bother


class IdentityResolver:
    def __init__(self, config: Optional[IdentityConfig] = None) -> None:
        self.cfg = config or IdentityConfig()
        self._s: dict[int, _State] = {}

    def wants_embed(self, track_id: int, box_height_px: float, now: float) -> bool:
        s = self._s.setdefault(track_id, _State())
        return (s.decided is None and not s.pending
                and s.attempts < self.cfg.max_attempts
                and box_height_px >= self.cfg.min_box_px
                and now - s.last_try_t >= self.cfg.retry_interval_s)

    def started(self, track_id: int, now: float) -> None:
        s = self._s.setdefault(track_id, _State())
        s.pending, s.last_try_t = True, now
        s.attempts += 1

    def no_face(self, track_id: int) -> None:
        """404 from embed: back of the head, most likely. Not a disagreement."""
        s = self._s.setdefault(track_id, _State())
        s.pending = False
        self._maybe_give_up(s)

    def failed(self, track_id: int) -> None:
        """Network or server error. Doesn't count toward the attempt budget."""
        s = self._s.setdefault(track_id, _State())
        s.pending = False
        s.attempts = max(0, s.attempts - 1)

    def result(self, track_id: int, quality: float, label: Optional[str],
               similarity: float) -> Optional[str]:
        """Report one embed+match. `label` is None when below threshold.

        Returns the identity if this answer decided it, else None.
        """
        s = self._s.setdefault(track_id, _State())
        s.pending = False
        if s.decided is not None:
            return None
        if quality < self.cfg.min_quality:
            self._maybe_give_up(s)
            return None
        s.score = similarity
        if label is None:
            # Below threshold. Resets the streak but decides nothing: a face
            # turned half away scores low, and the next frame may be fine.
            s.agree, s.last_label = 0, None
            self._maybe_give_up(s)
            return None
        s.agree = s.agree + 1 if label == s.last_label else 1
        s.last_label = label
        if s.agree >= self.cfg.agree_needed:
            s.decided = label
            return label
        self._maybe_give_up(s)
        return None

    def identity(self, track_id: int) -> Optional[str]:
        s = self._s.get(track_id)
        return s.decided if s else None

    def score(self, track_id: int) -> float:
        s = self._s.get(track_id)
        return s.score if s else 0.0

    def forget(self, track_id: int) -> None:
        self._s.pop(track_id, None)

    def prune(self, live_ids: set[int]) -> None:
        for tid in list(self._s):
            if tid not in live_ids:
                del self._s[tid]

    def _maybe_give_up(self, s: _State) -> None:
        if s.decided is None and s.attempts >= self.cfg.max_attempts:
            s.decided = UNKNOWN
