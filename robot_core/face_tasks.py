"""Face enroll and face match, as jobs the voice agent runs with its session closed.

Plain functions over a camera and a VisionClient — no ROS, no audio. The Live
session is torn down before either runs (the microphone is free, the camera is
not shared), and the agent answers by gesture afterwards: `enroll` and `match`
return whether it worked / whether the face is known.

With a `head` (camera on the pan/tilt gimbal) match scans for faces: pan -90
(right) to +90 (left) at each tilt in FACE_SEARCH_TILTS, from level upwards (the
voice node limits tilt to 0..+45, so the head never looks below its base
position). Every face seen gets the head turned to centre it and a fresh photo,
and the face in the middle of that photo is matched; a known face ends the task,
an unknown one is skipped from then on and the scan carries on. Without a head
the camera just looks straight ahead. Enroll never searches (the person
steps in front of the robot) but it does frame: the head starts at
FACE_ENROLL_TILT_DEG and keeps re-aiming at the face whenever it drifts off
centre, and only shots with the whole face inside the frame are kept.
"""

from __future__ import annotations

import logging
import time
from typing import Callable, Optional

from robot_core import robot_log, settings, status, task_cancel
from robot_core.sensors.camera import jpeg
from robot_core.vision_client import VisionClient, VisionError

log = logging.getLogger(__name__)

SAME_FACE_DEG = 10.0     # a face within this many degrees of one already matched is that face


class _Eyes:
    """Enroll's camera: captures, asks the Mac for a face and (with a head)
    keeps that face framed."""

    def __init__(self, camera, vision: VisionClient, head=None) -> None:
        self.camera, self.vision, self.head = camera, vision, head
        self.last_frame = None
        self._n = 0
        self.last_jpeg = b""                  # what the last look() sent to the Mac

    def look(self) -> Optional[dict]:
        head = self.head
        frame = self.camera.capture(head.gimbal) if head else self.camera.capture()
        self._n += 1
        self.last_frame = frame
        self.last_jpeg = jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE)
        face = self.vision.face_embed(self.last_jpeg, self._n)
        if head and face:
            self._frame(frame, face["face_bbox"])
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
    eyes = _Eyes(camera, vision, head)
    if head:
        head.gimbal.move_to(0.0, settings.FACE_ENROLL_TILT_DEG, wait=True)
        head.gimbal.wait_settled()
    status.timer(f"search {name}", settings.FACE_SEARCH_SECONDS)
    shots, t0, t_face, last = [], clock(), None, float("-inf")
    try:
        while True:
            if task_cancel.requested():
                status.result("enroll cancelled")
                return False                           # nothing saved
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
    """The enrolled name of whoever is in view, or None.

    With a head: one scan (see the module docstring), ended early by a known face
    or after `seconds`. Without one: looks straight ahead until `seconds` run out."""
    status.timer("match", seconds)
    t0, seen, n = clock(), False, 0

    def embed(img) -> Optional[dict]:
        nonlocal n
        n += 1
        return vision.face_embed(img, n)

    try:
        if head is None:
            while clock() - t0 < seconds:
                if task_cancel.requested():
                    return _match_cancelled()
                face = embed(jpeg(camera.capture().main, settings.FACE_WIRE_LONG_EDGE))
                seen = seen or bool(face)
                name = _known(vision, face, 0.0, 0.0)
                if name:
                    return name
        else:
            checked = []            # where the faces already matched are: the next view overlaps
            for pan, tilt in _scan():
                if clock() - t0 >= seconds:
                    break
                if task_cancel.requested():
                    return _match_cancelled()
                head.gimbal.move_to(pan, tilt, wait=True)
                head.gimbal.wait_settled()
                frame = camera.capture(head.gimbal)
                face = embed(jpeg(frame.main, settings.FACE_WIRE_LONG_EDGE))
                boxes = (face.get("face_bboxes") or [face["face_bbox"]]) if face else []
                robot_log.event("face.look", pan=pan, tilt=tilt, faces=len(boxes))
                for bbox in boxes:                         # largest first
                    seen = True
                    if task_cancel.requested():
                        return _match_cancelled()
                    x1, y1, x2, y2 = _to_frame(frame, bbox)
                    where = head.pixel_angles(frame, (x1 + x2) / 2, (y1 + y2) / 2)
                    if any(abs(where[0] - p) < SAME_FACE_DEG and abs(where[1] - t) < SAME_FACE_DEG
                           for p, t in checked):
                        continue                           # this one was matched already
                    checked.append(where)
                    head.gimbal.move_to(*where, wait=True)  # centre it, then a fresh photo
                    head.gimbal.wait_settled()
                    name = _known(vision, embed(_centre_crop(camera.capture(head.gimbal), x2 - x1, y2 - y1)),
                                  *where)
                    if name:
                        return name
    except VisionError as exc:
        log.warning("match failed: %s", exc)
        status.error("V02" if "timed out" in str(exc).lower() else "V01", "mac vision service")
        status.result("match failed")
        return None
    except RuntimeError as exc:                     # the camera gave up
        log.warning("match failed: %s", exc)
        status.error("F04", str(exc))
        status.result("match failed")
        return None
    if not seen:
        status.error("F01")
    status.result("no match")
    return None


def _scan():
    """(pan, tilt) for one scan: each tilt row in turn, panning across and back."""
    for i, tilt in enumerate(settings.FACE_SEARCH_TILTS):
        for pan in (settings.FACE_SEARCH_PANS if i % 2 == 0 else settings.FACE_SEARCH_PANS[::-1]):
            yield pan, tilt


def _centre_crop(frame, face_w: float, face_h: float) -> bytes:
    """The middle of a photo taken with the face centred, three face-widths wide, so
    the Mac (which embeds the largest face it sees) embeds this face and nobody else's."""
    h, w = frame.main.shape[:2]
    side = int(min(max(3 * max(face_w, face_h), 320), h, w))
    x0, y0 = (w - side) // 2, (h - side) // 2
    return jpeg(frame.main[y0:y0 + side, x0:x0 + side], settings.FACE_WIRE_LONG_EDGE)


def _known(vision: VisionClient, face: Optional[dict], pan: float, tilt: float) -> Optional[str]:
    """The face's enrolled name if it matches well enough, else None."""
    if not face:
        return None
    res = vision.face_match(face["embedding"])
    best = res["matches"][0] if res["matches"] else None
    robot_log.event("face.check", pan=round(pan), tilt=round(tilt), quality=face.get("quality"),
                    best=best and best["label"], similarity=best and round(best["similarity"], 3),
                    threshold=res["threshold"])
    if best and best["similarity"] >= res["threshold"]:
        status.result(f"{best['label']} matched")
        return best["label"]
    return None


def _match_cancelled() -> None:
    status.result("match cancelled")
    return None
