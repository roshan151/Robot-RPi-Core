"""The head as one instrument: camera + TF-Luna on a pan/tilt gimbal.

Because the rangefinder looks wherever the camera looks, the head can:

  sweep()        pan slowly while reading continuously → a 90° slice of a
                 2-D lidar scan (the SLAM keyframe is four of these)
  range_at()     aim, settle, median of N readings → distance along a bearing
  aim_at_pixel() turn a detector box into pan/tilt, so the next range_at()
                 lands on the object
  floor_probe()  tilt down to where the floor should be, d metres ahead:
                 shorter than expected = low obstacle the level sweep missed,
                 longer = a drop (stairs). Run before every drive.

Angles: pan > 0 turns the head LEFT (counter-clockwise, REP-103), tilt > 0
looks UP. If a servo moves the other way, set `invert` on its AxisConfig — the
geometry here assumes these signs.
"""

from __future__ import annotations

import bisect
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .camera import Frame


@dataclass
class HeadGeometry:
    height_m: float = 0.25        # TF-Luna / camera above the floor, head level
    pan_axis_x_m: float = 0.05    # pan axis ahead of the wheel-axle centre (base_link)
    hfov_deg: float = 66.0        # Camera Module 3 standard; 102 for the Wide
    servo_lag_s: float = 0.04     # a servo trails its command by a frame or two
    sweep_dps: float = 30.0       # 90° in 3 s → ~3 readings per degree at 100 Hz


@dataclass(frozen=True)
class Probe:
    status: str                    # clear | obstacle | drop | unknown
    distance_m: Optional[float]    # how far ahead the problem starts
    points: List[Tuple[float, float]]   # base_link [x, y] to mark on the map


class Head:
    def __init__(self, gimbal, lidar, camera, geom: HeadGeometry = HeadGeometry()) -> None:
        self.gimbal, self.lidar, self.camera, self.geom = gimbal, lidar, camera, geom

    @property
    def sensor_xy(self) -> Tuple[float, float]:
        return (self.geom.pan_axis_x_m, 0.0)

    # ------------------------------------------------------------ primitives

    def look(self, pan: float, tilt: float) -> Frame:
        self.gimbal.move_to(pan, tilt, wait=True)
        self.gimbal.wait_settled()
        return self.camera.capture(self.gimbal)

    def range_at(self, pan: Optional[float] = None, tilt: Optional[float] = None,
                 n: int = 12) -> Optional[float]:
        if pan is not None or tilt is not None:
            self.gimbal.move_to(pan, tilt, wait=True)
        self.gimbal.wait_settled()
        return self.lidar.read(n=n, timeout=1.0)

    def sweep(self, pan_from: float = -45.0, pan_to: float = 45.0,
              tilt: float = 0.0) -> List[Tuple[float, Optional[float]]]:
        """Pan continuously, pairing every TF-Luna reading with the pan angle at
        that instant. Returns [(bearing_rad in base_link, range_m | None)]."""
        g = self.geom
        self.gimbal.move_to(pan_from, tilt, wait=True)
        self.gimbal.wait_settled()
        track_t, track_pan = [], []
        t0 = time.monotonic()
        self.gimbal.move_to(pan_to, tilt, speed_dps=g.sweep_dps, wait=False)
        while True:
            now = time.monotonic()
            track_t.append(now)
            track_pan.append(self.gimbal.angles()[0])
            if self.gimbal.settled(0.05) or now - t0 > 30:
                break
            time.sleep(0.01)
        out = []
        for r in self.lidar.since(t0, track_t[-1]):
            pan = _interp(track_t, track_pan, r.t - g.servo_lag_s)
            out.append((math.radians(pan), r.range_m))
        return out

    # ------------------------------------------------------------- geometry

    def fx(self, width: int) -> float:
        return (width / 2.0) / math.tan(math.radians(self.geom.hfov_deg) / 2.0)

    def pixel_angles(self, frame: Frame, u: float, v: float) -> Tuple[float, float]:
        """Absolute (pan, tilt) in degrees that would centre pixel (u, v)."""
        h, w = frame.main.shape[:2]
        f = self.fx(w)
        return (frame.pan_deg + math.degrees(math.atan2(w / 2.0 - u, f)),
                frame.tilt_deg + math.degrees(math.atan2(h / 2.0 - v, f)))

    def aim_at_pixel(self, frame: Frame, u: float, v: float) -> Tuple[float, float]:
        pan, tilt = self.pixel_angles(frame, u, v)
        return self.gimbal.move_to(pan, tilt, wait=True)

    def point_at(self, pan_deg: float, tilt_deg: float, rng: float) -> Tuple[float, float]:
        """base_link [x, y] of a range reading taken at (pan, tilt)."""
        horiz = rng * math.cos(math.radians(tilt_deg))
        b = math.radians(pan_deg)
        return (self.geom.pan_axis_x_m + horiz * math.cos(b), horiz * math.sin(b))

    # ---------------------------------------------------------------- probes

    def floor_probe(self, distance_m: float, pans: Sequence[float] = (-12.0, 0.0, 12.0),
                    short: float = 0.85, long: float = 1.3) -> Probe:
        """Is the floor where it should be, `distance_m` ahead?"""
        h = self.geom.height_m
        d = max(0.3, distance_m)
        # Close in, the floor is steeper than the head can look down: look as
        # far down as it goes and judge that spot (a farther one) instead.
        tilt = max(-math.degrees(math.atan2(h, d)), self.gimbal.tilt_limits()[0])
        expected = h / math.sin(math.radians(-tilt))
        worst, first, pts = "clear", None, []
        for pan in pans:
            r = self.range_at(pan, tilt)
            if r is None:
                if worst == "clear":
                    worst = "unknown"          # dark floor or absorbing rug
                continue
            if r < short * expected:
                x, y = self.point_at(pan, tilt, r)
                pts.append((x, y))
                worst = "obstacle" if worst != "drop" else worst
                first = x if first is None else min(first, x)
            elif r > long * expected:
                x, y = self.point_at(pan, tilt, expected)
                pts.append((x, y))
                worst = "drop"
                first = x if first is None else min(first, x)
        self.gimbal.move_to(0.0, 0.0, wait=False)
        return Probe(worst, first, pts)

    def home(self) -> None:
        self.gimbal.home(wait=True)


def _interp(ts: Sequence[float], vs: Sequence[float], t: float) -> float:
    i = bisect.bisect_left(ts, t)
    if i <= 0:
        return vs[0]
    if i >= len(ts):
        return vs[-1]
    a = (t - ts[i - 1]) / max(1e-9, ts[i] - ts[i - 1])
    return vs[i - 1] + a * (vs[i] - vs[i - 1])
