"""The head camera, mounted upside down.

Two streams, one capture (VISION-DESIGN rule "capture high, send low"):
  * main  — full resolution BGR, for plant photos and crops
  * lores — small greyscale, for the visual-motion check (and later the tracker)

The mount is inverted, so the image is rotated 180° *in the ISP*
(picamera2 `Transform(hflip=1, vflip=1)`): free, and every consumer — the
detector, the photos, the motion check — sees an upright image. Note that
picamera2's "RGB888" is BGR in memory, which is exactly what OpenCV wants.

Falls back to OpenCV's VideoCapture (with a software flip) on a laptop or with
a USB camera.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

log = logging.getLogger(__name__)
FRAME_TIMEOUT_S = 4.0       # no frame for this long: the sensor link stalled, so restart the camera once


@dataclass(frozen=True)
class Frame:
    seq: int
    t_capture: float          # host monotonic seconds
    main: np.ndarray          # HxWx3 BGR
    lores: np.ndarray         # hxw uint8 grey
    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    settled: bool = True


class Camera:
    def __init__(self, main_size: Tuple[int, int] = (1920, 1080),
                 lores_size: Tuple[int, int] = (320, 240),
                 inverted: bool = True, backend: str = "auto", index: int = 0) -> None:
        self.inverted = inverted
        self.lores_size, self.main_size = lores_size, main_size
        self._seq = 0
        self._lock = threading.Lock()
        self._picam = None
        self._cap = None
        if backend in ("auto", "picamera2"):
            try:
                self._start()
                log.info("camera: picamera2 %s + lores %s, inverted=%s", main_size, lores_size, inverted)
            except Exception as exc:                        # noqa: BLE001
                if backend == "picamera2":
                    raise
                log.warning("camera: picamera2 unavailable (%s); falling back to OpenCV", exc)
        if self._picam is None:
            import cv2
            self._cap = cv2.VideoCapture(index)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, main_size[0])
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, main_size[1])
            if not self._cap.isOpened():
                raise RuntimeError(f"cannot open camera {index}")

    def _start(self) -> None:
        from libcamera import Transform
        from picamera2 import Picamera2
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": self.main_size, "format": "RGB888"},
            lores={"size": self.lores_size, "format": "YUV420"},
            transform=Transform(hflip=int(self.inverted), vflip=int(self.inverted)),
            buffer_count=3))
        cam.start()
        self._picam = cam

    def _request(self):
        """The next frame, never waiting forever. libcamera sometimes loses the sensor
        ("frontend has timed out": a loose ribbon, a power dip) and picamera2 would then
        block for good. Restart the camera once; if it is still silent, raise."""
        for attempt in (1, 2):
            try:
                return self._picam.capture_request(wait=FRAME_TIMEOUT_S)
            except Exception as exc:                        # noqa: BLE001 — TimeoutError, libcamera errors
                if attempt == 2:
                    raise RuntimeError(f"camera gave no frames, even after a reset ({type(exc).__name__})") from exc
                log.warning("camera: no frames for %.0f s, resetting it", FRAME_TIMEOUT_S)
                try:
                    self._picam.stop()
                    self._picam.close()
                except Exception:                           # noqa: BLE001
                    pass
                time.sleep(1.0)
                self._start()

    def capture(self, gimbal=None) -> Frame:
        import cv2
        with self._lock:
            pan, tilt = gimbal.angles() if gimbal else (0.0, 0.0)
            settled = gimbal.settled() if gimbal else True
            if self._picam is not None:
                req = self._request()
                try:
                    main = req.make_array("main")
                    yuv = req.make_array("lores")
                finally:
                    req.release()
                lores = np.ascontiguousarray(yuv[: self.lores_size[1], : self.lores_size[0]])
            else:
                ok, main = self._cap.read()
                if not ok:
                    raise RuntimeError("camera read failed")
                if self.inverted:
                    main = cv2.flip(main, -1)
                lores = cv2.resize(cv2.cvtColor(main, cv2.COLOR_BGR2GRAY), self.lores_size,
                                   interpolation=cv2.INTER_AREA)
            self._seq += 1
            return Frame(self._seq, time.monotonic(), main, lores, pan, tilt, settled)

    def capture_lores(self):
        """(t, lores grey) only — cheap enough to run at 5 Hz for the motion
        check without copying a full-resolution frame every time."""
        import cv2
        with self._lock:
            if self._picam is not None:
                req = self._request()
                try:
                    yuv = req.make_array("lores")
                finally:
                    req.release()
                return time.monotonic(), np.ascontiguousarray(
                    yuv[: self.lores_size[1], : self.lores_size[0]])
            ok, main = self._cap.read()
            if not ok:
                raise RuntimeError("camera read failed")
            if self.inverted:
                main = cv2.flip(main, -1)
            return time.monotonic(), cv2.resize(cv2.cvtColor(main, cv2.COLOR_BGR2GRAY),
                                                self.lores_size, interpolation=cv2.INTER_AREA)

    def close(self) -> None:
        if self._picam is not None:
            self._picam.stop()
            self._picam.close()
        if self._cap is not None:
            self._cap.release()


def jpeg(img: np.ndarray, long_edge: Optional[int] = 640, quality: int = 85) -> bytes:
    """Encode for the wire. long_edge=None keeps full resolution (plant photos)."""
    import cv2
    if long_edge:
        h, w = img.shape[:2]
        s = long_edge / max(h, w)
        if s < 1.0:
            img = cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError("JPEG encode failed")
    return buf.tobytes()
