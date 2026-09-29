"""Camera capture: two sizes of the same moment, stamped at the sensor.

PLAN Phase 0 item 5 / VISION-DESIGN V0. Nothing touches the SD card.

    main   1280x720 BGR   face crops, and the 640x360 frame sent to /v1/detect
    lores   320x180 gray  the local optical-flow tracker, at TRACK_HZ

Both streams are 16:9 so a box scales between them with one factor per axis.
(A 320x240 lores next to a 16:9 main would squash the tracker's pixels.)

`t_capture` comes from the sensor timestamp in the request metadata, converted
to CLOCK_MONOTONIC, not from time.monotonic() after capture returns — the gap is
encode/copy time, exactly the error the frame stamp exists to remove.

PiCamera needs picamera2 (apt: python3-picamera2). OpenCVCamera is the bench
fallback: a USB webcam or a laptop camera, no sensor timestamp.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

log = logging.getLogger(__name__)

MAIN_SIZE = (1280, 720)
LORES_SIZE = (320, 180)


@dataclass
class Frame:
    seq: int
    t_capture: float          # CLOCK_MONOTONIC seconds
    main_bgr: np.ndarray      # MAIN_SIZE, uint8 BGR
    gray: np.ndarray          # LORES_SIZE, uint8


class PiCamera:
    def __init__(self, fps: float = 15.0, hflip: bool = False, vflip: bool = False) -> None:
        from picamera2 import Picamera2          # lazy: only exists on the Pi
        from libcamera import Transform           # noqa: F401  (ships with picamera2)

        self._cam = Picamera2()
        cfg = self._cam.create_video_configuration(
            main={"size": MAIN_SIZE, "format": "RGB888"},      # RGB888 is BGR in memory
            lores={"size": LORES_SIZE, "format": "YUV420"},
            transform=Transform(hflip=int(hflip), vflip=int(vflip)),
            controls={"FrameRate": float(fps)},
            buffer_count=4,
        )
        self._cam.configure(cfg)
        self._seq = 0

    def start(self) -> "PiCamera":
        self._cam.start()
        return self

    def capture(self) -> Frame:
        req = self._cam.capture_request()
        try:
            main = req.make_array("main")
            yuv = req.make_array("lores")
            md = req.get_metadata()
        finally:
            req.release()
        gray = yuv[: LORES_SIZE[1], : LORES_SIZE[0]].copy()     # Y plane
        self._seq += 1
        return Frame(self._seq, _sensor_to_monotonic(md.get("SensorTimestamp")), main, gray)

    def close(self) -> None:
        try:
            self._cam.stop()
        finally:
            self._cam.close()


class OpenCVCamera:
    """Bench fallback: any camera OpenCV can open. Same Frame shape."""

    def __init__(self, index: int = 0) -> None:
        import cv2

        self._cv2 = cv2
        self._cap = cv2.VideoCapture(index)
        self._seq = 0

    def start(self) -> "OpenCVCamera":
        if not self._cap.isOpened():
            raise RuntimeError("OpenCV could not open the camera")
        return self

    def capture(self) -> Frame:
        cv2 = self._cv2
        ok, img = self._cap.read()
        t = time.monotonic()
        if not ok:
            raise RuntimeError("camera read failed")
        main = cv2.resize(img, MAIN_SIZE, interpolation=cv2.INTER_AREA)
        gray = cv2.cvtColor(cv2.resize(main, LORES_SIZE, interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
        self._seq += 1
        return Frame(self._seq, t, main, gray)

    def close(self) -> None:
        self._cap.release()


def open_camera(kind: str = "auto", **kw) -> "PiCamera | OpenCVCamera":
    """kind: "pi", "opencv", or "auto" (picamera2 if importable)."""
    if kind in ("pi", "auto"):
        try:
            return PiCamera(**kw).start()
        except ImportError:
            if kind == "pi":
                raise
            log.warning("picamera2 not available; falling back to OpenCV camera 0")
    return OpenCVCamera().start()


# ---------------------------------------------------------------------------
# encoding
# ---------------------------------------------------------------------------

def jpeg(bgr: np.ndarray, *, long_edge: Optional[int] = None, quality: int = 75) -> bytes:
    """Encode (optionally downscaled) to JPEG bytes in memory."""
    import cv2

    img = bgr
    if long_edge:
        h, w = img.shape[:2]
        s = long_edge / float(max(h, w))
        if s < 1.0:
            img = cv2.resize(img, (int(round(w * s)), int(round(h * s))), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()


def crop(bgr: np.ndarray, bbox: tuple[float, float, float, float]) -> np.ndarray:
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = (int(round(v)) for v in bbox)
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    return bgr[y1:y2, x1:x2]


def _sensor_to_monotonic(sensor_ns: Optional[int]) -> float:
    """libcamera stamps frames on the kernel's boot-time clock. Convert by
    measuring how long ago that was, then subtracting from monotonic now."""
    now_mono = time.monotonic()
    if not sensor_ns:
        return now_mono
    age = time.clock_gettime(time.CLOCK_BOOTTIME) - sensor_ns / 1e9
    if not 0.0 <= age < 1.0:           # clock mismatch: don't invent a stamp
        return now_mono
    return now_mono - age
