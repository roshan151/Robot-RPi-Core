"""Face enroll and face match, as jobs the voice agent runs with its session closed.

Plain functions over a camera and a VisionClient — no ROS, no audio. The Live
session is torn down before either runs (the microphone is free, the camera is
not shared), and the agent answers by gesture afterwards: `enroll` and `match`
return whether it worked / whether the face is known.

With a `head` (camera on the pan/tilt gimbal) the robot looks for the face: it
sweeps pan ±45° at a few tilts from level upwards (the voice node limits the
tilt axis to 0..+45, so the head never looks below its base position), stops on
the first face it sees, centres it, and holds there while it captures. Without
a head the camera just looks straight ahead. Enroll never searches (the person
steps in front of the robot) but it does frame: the head starts at
FACE_ENROLL_TILT_DEG and keeps re-aiming at the face whenever it drifts off
centre, and only shots with the whole face inside the frame are kept.
"""

from __future__ import annotations

import itertools
import logging
import time
from typing import Callable, Optional

from robot_core import settings, status
from robot_core.sensors.camera import jpeg
from robot_core.vision_client import VisionClient, VisionError

log = logging.getLogger(__name__)

LOST_FRAMES = 3          # this many frames without the face and the search resumes


class _Eyes:
    """One place that looks: captures, asks the Mac for a face, and (with a
    head) searches for it, then holds still on it."""

    def __init__(self, camera, vision: VisionClient, head=None, search: bool = True) -> None:
        self.camera, self.vision, self.head, self.search = camera, vision, head, search
        self.last_frame = None
        self.locked, self._lost, self._n = False, 0, 0
        self.last_jpeg = b""                  # what the last look() sent to the Mac
        self._poses = itertools.cycle(
            (pan, tilt)
            for i, tilt in enumerate(settings.FACE_SEARCH_TILTS)
            for pan in (settings.FACE_SEARCH_PANS if i % 2 == 0 else settings.FACE_SEARCH_PANS[::-1]))

    def look(self) -> Optional[dict]:
        head = self.head
        if head and self.search and not self.locked:
            pan, tilt = next(self._poses)
            head.gimbal.move_to(pan, tilt, wait=True)
            head.gimbal.wait_settled()
        frame = self.camera.capture(head.gimbal) if head else self.camera.capture()
        self._n += 1
        self.last_frame = frame
        self.last_jpeg = jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE)
        face = self.vision.face_embed(self.last_jpeg, self._n)
        if head and not self.search:
            if face:
                self._frame(frame, face["face_bbox"])
        elif head:
            if face:
                self._lost = 0
                if not self.locked:
                    self.locked = True
                    self._centre(frame, face["face_bbox"])
            elif self.locked:
                self._lost += 1
                self.locked = self._lost < LOST_FRAMES
        return face

    def _centre(self, frame, bbox) -> None:
        x1, y1, x2, y2 = _to_frame(frame, bbox)
        self.head.aim_at_pixel(frame, (x1 + x2) / 2, (y1 + y2) / 2)

    def _frame(self, frame, bbox) -> None:
        """Enroll: re-aim when the face is off centre or cut by the frame edge.
        The detector's box runs past the edge for a cut-off face, so its centre
        still points the head the right way."""
        h, w = frame.main.shape[:2]
        x1, y1, x2, y2 = _to_frame(frame, bbox)
        off = max(abs((x1 + x2) / 2 - w / 2) / w, abs((y1 + y2) / 2 - h / 2) / h)
        if off > settings.FACE_FRAME_DEADBAND or not full_face(frame, bbox):
            self._centre(frame, bbox)
            self.head.gimbal.wait_settled()


def _to_frame(frame, bbox):
    """A box from the (downscaled) wire image, in camera-frame pixels."""
    h, w = frame.main.shape[:2]
    scale = min(1.0, settings.FACE_WIRE_LONG_EDGE / max(h, w))
    return tuple(v / scale for v in bbox)


def full_face(frame, bbox) -> bool:
    """The whole face box lies inside the frame, FACE_EDGE_MARGIN in from each edge."""
    h, w = frame.main.shape[:2]
    x1, y1, x2, y2 = _to_frame(frame, bbox)
    mx, my = settings.FACE_EDGE_MARGIN * w, settings.FACE_EDGE_MARGIN * h
    return x1 >= mx and y1 >= my and x2 <= w - mx and y2 <= h - my


def enroll(camera, vision: VisionClient, name: str, seconds: float = settings.FACE_ENROLL_SECONDS,
           max_shots: int = 16, clock: Callable[[], float] = time.monotonic, head=None) -> bool:
    """Find a face (up to FACE_SEARCH_SECONDS), then collect good single-face
    frames for `seconds` and save them under `name`.

    Same rules as vision_test.py enroll: a second face in view rejects the whole
    capture, blurry / angled frames (quality < FACE_MIN_QUALITY) and faces cut by
    the frame edge are skipped, and shots are spread evenly across the window.
    With a head it frames the face (no search sweep): the person comes to the
    camera and the head follows their face to keep all of it in view.
    """
    eyes = _Eyes(camera, vision, head, search=False)
    if head:
        head.gimbal.move_to(0.0, settings.FACE_ENROLL_TILT_DEG, wait=True)
        head.gimbal.wait_settled()
    status.timer(f"search {name}", settings.FACE_SEARCH_SECONDS)
    shots, t0, t_face, last = [], clock(), None, float("-inf")
    try:
        while True:
            now = clock()
            if t_face is None and now - t0 >= settings.FACE_SEARCH_SECONDS:
                break                                  # never found anyone
            if t_face is not None and now - t_face >= seconds:
                break
            face = eyes.look()
            if face and t_face is None:
                t_face = clock()
                status.timer(f"enroll {name}", seconds)
            if face and face["face_count"] > 1:
                log.warning("enroll rejected: %d faces in view", face["face_count"])
                status.result("enroll rejected")
                return False
            if face and not full_face(eyes.last_frame, face["face_bbox"]):
                status.detail("whole face not in view")
                continue
            if face and face["quality"] >= settings.FACE_MIN_QUALITY \
                    and clock() - last >= seconds / max_shots:
                shots.append(eyes.last_jpeg)
                last = clock()
        if len(shots) < settings.FACE_MIN_SHOTS:
            status.error("F01" if t_face is None else "F03")
            status.result("enroll failed")
            log.warning("enroll: only %d usable frames, nothing saved", len(shots))
            return False
        vision.face_enroll(shots, name)
    except (VisionError, RuntimeError) as exc:      # RuntimeError: the camera gave up
        log.warning("enroll failed: %s", exc)
        status.result("enroll failed")
        return False
    log.info("enrolled %s from %d frames", name, len(shots))
    status.result(f"{name} enrolled")
    return True


def match(camera, vision: VisionClient, seconds: float = settings.FACE_MATCH_SECONDS,
          clock: Callable[[], float] = time.monotonic, head=None) -> Optional[str]:
    """The enrolled name of whoever is in front of the camera, or None after
    `seconds` (which includes the time spent searching)."""
    eyes = _Eyes(camera, vision, head)
    status.timer("match", seconds)
    t0 = clock()
    try:
        while clock() - t0 < seconds:
            face = eyes.look()
            if not face:
                continue
            res = vision.face_match(face["embedding"])
            best = res["matches"][0] if res["matches"] else None
            if best and best["similarity"] >= res["threshold"]:
                status.result(f"{best['label']} matched")
                return best["label"]
    except (VisionError, RuntimeError) as exc:
        log.warning("match failed: %s", exc)
    status.error("F01")
    status.result("no match")
    return None
