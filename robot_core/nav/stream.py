"""The camera, captured continuously and streamed to the Mac for visual SLAM.

One thread captures at `hz` (10 by default) and keeps the newest frame together with the
robot's pose and the gimbal's angles at that instant; whoever needs a frame (follow) takes
`latest()` instead of capturing its own, so the camera has one owner. Another thread sends
each frame to the Mac (POST /v1/nav/frame) as JPEG with its timestamp, pose and pan/tilt:
ORB-SLAM3 needs a steady stream, and the pose and pan/tilt are what turn its camera poses
into robot poses. The ToF grid nearest the capture goes too: its metric ranges give SLAM its scale. Sending never holds up capture: a frame that
finds the previous one still on its way is dropped. With no Mac, frames are just not sent.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional, Tuple

log = logging.getLogger(__name__)


class FrameStream:
    def __init__(self, camera, gimbal, pose: Callable[[], Optional[tuple]], send=None,
                 hz: float = 10.0, long_edge: int = 640, quality: int = 70, tilt_offset_deg: float = 0.0,
                 tof=None, min_move_m: float = 0.02, min_turn_deg: float = 1.5,
                 idle_hz: float = 1.0, min_quality: int = 45) -> None:
        """`send(jpeg, t, pose, pan, tilt, tof)`: hand a frame to the Mac (VisionClient.nav_frame), or None.
        `tof()`: the latest ToF grid (tof8x8.Grid) or None; sent as its rows and its age at capture.
        The tilt sent is the REAL one: the gimbal's plus `tilt_offset_deg` (how far up it looks at 0).
        Motion-gating skips redundant stationary frames above `idle_hz`, and adaptive quality scales
        JPEG quality down toward `min_quality` when Wi-Fi RTT climbs."""
        self.tilt_offset = tilt_offset_deg
        self.camera, self.gimbal, self.pose, self.send, self.tof = camera, gimbal, pose, send, tof
        self.period, self.long_edge, self.quality = 1.0 / hz, long_edge, quality
        self.min_move_m, self.min_turn_deg = float(min_move_m), float(min_turn_deg)
        self.idle_period = 1.0 / max(idle_hz, 0.1)
        self.min_quality = int(min_quality)
        self._last_sent: Optional[Tuple] = None               # (t_mono, pose, pan, tilt)
        self._rtt_s: float = 0.05
        self._latest: Optional[Tuple] = None                  # (frame, pose)
        self._cond = threading.Condition()
        self._slot: Optional[Tuple] = None
        self._slot_ev = threading.Event()
        self._stop = threading.Event()
        self._threads = []
        self.sent = self.dropped = self.failed = self.skipped_idle = 0

    def start(self) -> "FrameStream":
        self._threads = [threading.Thread(target=self._capture, name="cam-capture", daemon=True)]
        if self.send is not None:
            self._threads.append(threading.Thread(target=self._sender, name="cam-send", daemon=True))
        for t in self._threads:
            t.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._slot_ev.set()
        for t in self._threads:
            t.join(timeout=2.0)

    def latest(self, newer_than: int = 0, timeout: float = 3.0):
        """(frame, pose at capture) newer than frame seq `newer_than`; raises RuntimeError if the
        camera has gone quiet."""
        with self._cond:
            ok = self._cond.wait_for(lambda: self._latest is not None and self._latest[0].seq > newer_than,
                                     timeout)
            if not ok:
                raise RuntimeError("camera stream: no new frame")
            return self._latest

    def _capture(self) -> None:
        fails = 0
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                frame = self.camera.capture(self.gimbal)
                pose = self.pose()
                grid = self.tof() if self.tof is not None else None
                fails = 0
            except Exception as exc:                             # noqa: BLE001
                fails += 1
                if fails == 1:
                    log.warning("camera stream: capture failed: %s", exc)
                self._stop.wait(0.2)
                continue
            with self._cond:
                self._latest = (frame, pose)
                self._cond.notify_all()
            if self.send is not None:
                if self._slot is not None:
                    self.dropped += 1
                self._slot = (frame, pose, grid)
                self._slot_ev.set()
            self._stop.wait(max(0.0, self.period - (time.monotonic() - t0)))

    def _should_send(self, now: float, pose, pan: float, tilt: float) -> bool:
        """Motion-gate stationary frames to `idle_hz` to cut Wi-Fi bandwidth outdoors."""
        if self._last_sent is None:
            return True
        t0, p0, pan0, tilt0 = self._last_sent
        if now - t0 >= self.idle_period:
            return True
        if abs(pan - pan0) >= self.min_turn_deg or abs(tilt - tilt0) >= self.min_turn_deg:
            return True
        if pose is not None and p0 is not None:
            import math
            if math.hypot(pose[0] - p0[0], pose[1] - p0[1]) >= self.min_move_m:
                return True
            if abs(math.degrees(pose[2] - p0[2])) >= self.min_turn_deg:
                return True
        return False

    def _adaptive_quality(self) -> int:
        """Step JPEG quality down when RTT > 0.25 s so weak outdoor Wi-Fi doesn't back up."""
        if self._rtt_s <= 0.25:
            return self.quality
        frac = min(1.0, (self._rtt_s - 0.25) / 0.75)
        return max(self.min_quality, int(round(self.quality - frac * (self.quality - self.min_quality))))

    def _sender(self) -> None:
        from robot_core.sensors.camera import jpeg
        quiet_until = 0.0
        while not self._stop.is_set():
            self._slot_ev.wait(1.0)
            self._slot_ev.clear()
            item, self._slot = self._slot, None
            now = time.monotonic()
            if item is None or now < quiet_until:
                continue
            frame, pose, grid = item
            real_tilt = frame.tilt_deg + self.tilt_offset
            if not self._should_send(now, pose, frame.pan_deg, real_tilt):
                self.skipped_idle += 1
                continue
            tof = None if grid is None else {"rows": grid.rows, "age_s": round(grid.t - frame.t_capture, 3)}
            try:
                t_send = time.monotonic()
                self.send(jpeg(frame.main, self.long_edge, self._adaptive_quality()), frame.t_capture, pose,
                          frame.pan_deg, real_tilt, tof)
                rtt = time.monotonic() - t_send
                self._rtt_s = 0.7 * self._rtt_s + 0.3 * rtt
                self._last_sent = (time.monotonic(), pose, frame.pan_deg, real_tilt)
                self.sent += 1
            except Exception as exc:                             # noqa: BLE001 — the Mac is optional
                self.failed += 1
                self._rtt_s = min(1.5, self._rtt_s + 0.3)
                if self.failed == 1 or self.failed % 50 == 0:
                    log.info("camera stream: send failed (%s); retrying in 2 s", exc)
                quiet_until = time.monotonic() + 2.0
