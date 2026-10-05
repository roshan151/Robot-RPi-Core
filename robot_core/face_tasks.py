"""Face enroll and face match, as jobs the voice agent runs with its session closed.

Plain functions over a camera and a VisionClient — no ROS, no audio. The Live
session is torn down before either runs (the microphone is free, the camera is
not shared), and the agent answers by gesture afterwards: `enroll` and `match`
return whether it worked / whether the face is known.

With a `head` (camera on the pan/tilt gimbal) the robot looks for the face: it
sweeps pan ±45° at a few tilts from level upwards (the voice node limits the
tilt axis to 0..+45, so the head never looks below its base position), stops on
the first face it sees, centres it, and holds there while it captures. Without
a head the camera just looks straight ahead.
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

    def __init__(self, camera, vision: VisionClient, head=None) -> None:
        self.camera, self.vision, self.head = camera, vision, head
        self.locked, self._lost, self._n = False, 0, 0
        self.last_jpeg = b""                  # what the last look() sent to the Mac
        self._poses = itertools.cycle(
            (pan, tilt)
            for i, tilt in enumerate(settings.FACE_SEARCH_TILTS)
            for pan in (settings.FACE_SEARCH_PANS if i % 2 == 0 else settings.FACE_SEARCH_PANS[::-1]))

    def look(self) -> Optional[dict]:
        head = self.head
        if head and not self.locked:
            pan, tilt = next(self._poses)
            head.gimbal.move_to(pan, tilt, wait=True)
            head.gimbal.wait_settled()
        frame = self.camera.capture(head.gimbal) if head else self.camera.capture()
        self._n += 1
        self.last_jpeg = jpeg(frame.main)
        face = self.vision.face_embed(self.last_jpeg, self._n)
        if head:
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
        h, w = frame.main.shape[:2]
        scale = min(1.0, 640.0 / max(h, w))           # the frame was downscaled to 640 for the wire
        x1, y1, x2, y2 = (v / scale for v in bbox)
        self.head.aim_at_pixel(frame, (x1 + x2) / 2, (y1 + y2) / 2)


def enroll(camera, vision: VisionClient, name: str, seconds: float = settings.FACE_ENROLL_SECONDS,
           max_shots: int = 16, clock: Callable[[], float] = time.monotonic, head=None) -> bool:
    """Find a face (up to FACE_SEARCH_SECONDS), then collect good single-face
    frames for `seconds` and save them under `name`.

    Same rules as vision_test.py enroll: a second face in view rejects the whole
    capture, blurry / angled frames (quality < FACE_MIN_QUALITY) are skipped, and
    shots are spread evenly across the window.
    """
    eyes = _Eyes(camera, vision, head)
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
    except VisionError as exc:
        log.warning("enroll failed: %s", exc)
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
    except VisionError as exc:
        log.warning("match failed: %s", exc)
    status.error("F01")
    status.result("no match")
    return None
