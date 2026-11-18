"""The head as one instrument: camera + range sensor on a pan/tilt gimbal.

The range sensor is the 8x8 ToF (tof8x8.py) or, before it, the TF-Luna; both
answer `read()` / `latest()` / `since()`. With the ToF fitted the head can also
see the shape of the floor: `drive_pose()`, `floor_profile()`, `terrain_ahead()`.

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

from .. import terrain
from .camera import Frame
from .tof8x8 import SIZE, ZONE_DEG, zone_angles


FLOOR_REACH_M = 3.0      # a ray that should meet the floor nearer than this and returns nothing is a hole


@dataclass
class HeadGeometry:
    height_m: float = 0.25        # TF-Luna / camera above the floor, head level
    pan_axis_x_m: float = 0.05    # pan axis ahead of the wheel-axle centre (base_link)
    hfov_deg: float = 66.0        # Camera Module 3 standard; 102 for the Wide
    servo_lag_s: float = 0.04     # a servo trails its command by a frame or two
    sweep_dps: float = 30.0       # 90° in 3 s → ~3 readings per degree at 100 Hz
    drive_tilt_deg: float = -30.0     # ToF: head tilt for looking at the floor ahead (the drive pose)
    tof_perpendicular: bool = True    # ToF ranges are distances to the sensor's plane, not along each ray
    tilt_offset_deg: float = 0.0      # how far up the head really looks at tilt 0 (+10 = 10 deg up)
    pan_axis_y_m: float = 0.0         # ToF left of the robot's centre line (negative: to the right)


@dataclass(frozen=True)
class Probe:
    status: str                    # clear | obstacle | drop | unknown | unseen (ToF: unreadable floor, not driven)
    distance_m: Optional[float]    # how far ahead the problem starts
    points: List[Tuple[float, float]]   # base_link [x, y] to mark on the map


class Head:
    def __init__(self, gimbal, lidar, camera, geom: HeadGeometry = HeadGeometry(),
                 pitch=None, terrain_params: Optional[terrain.Params] = None) -> None:
        """`pitch()` -> the robot's nose-up angle in degrees (the IMU's), or None for level."""
        self.gimbal, self.lidar, self.camera, self.geom = gimbal, lidar, camera, geom
        self.pitch = pitch
        self.terrain = terrain_params or terrain.Params()
        self.last_level = None                        # the last floor scan's correction (terrain.level)

    @property
    def has_grid(self) -> bool:
        """True with the 8x8 ToF fitted (the TF-Luna has one beam and no grid)."""
        return hasattr(self.lidar, "grid")

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

    # ------------------------------------------------------ the floor (8x8 ToF)

    def drive_pose(self) -> None:
        """Look straight ahead and down at the floor in front of the tracks."""
        self.gimbal.move_to(0.0, self.geom.drive_tilt_deg, wait=True)
        self.gimbal.wait_settled()

    def _zones(self):
        """Every ToF zone from where the head points now, as (row, col, x, y, h, d):
        x ahead, y left, h above the floor, d from the sensor, in metres. A zone
        with no return whose ray should have met the floor within the sensor's
        reach comes back with h None and (x, y) where the floor should have been;
        other empty zones are left out. The robot's own pitch (the IMU's) is taken
        out, so a nose lifted by a sill does not turn flat floor into a slope."""
        self.gimbal.wait_settled()
        grid = self.lidar.grid(newer_than=time.monotonic())
        if grid is None:
            return []
        pan, tilt = (math.radians(a) for a in self.gimbal.angles())
        tilt += math.radians(self.geom.tilt_offset_deg)
        body = math.radians((self.pitch() if self.pitch else 0.0) or 0.0)
        ox, oy, height = self.geom.pan_axis_x_m, self.geom.pan_axis_y_m, self.geom.height_m
        out = []
        for r in range(SIZE):
            for c in range(SIZE):
                up_deg, left_deg = zone_angles(r, c)
                ty, tz = math.tan(math.radians(left_deg)), math.tan(math.radians(up_deg))
                fwd, left, up = 1.0, ty, tz                                     # per metre of depth
                fwd, up = (fwd * math.cos(tilt) - up * math.sin(tilt),          # head tilt
                           fwd * math.sin(tilt) + up * math.cos(tilt))
                fwd, left = (fwd * math.cos(pan) - left * math.sin(pan),        # head pan
                             fwd * math.sin(pan) + left * math.cos(pan))
                fwd, up = (fwd * math.cos(body) - up * math.sin(body),          # the robot's pitch
                           fwd * math.sin(body) + up * math.cos(body))
                along = math.sqrt(1.0 + ty * ty + tz * tz)                      # ray length per metre of depth
                d = grid.rows[r][c]
                if d is not None:
                    depth = d if self.geom.tof_perpendicular else d / along
                    out.append((r, c, ox + depth * fwd, oy + depth * left, height + depth * up, depth * along))
                elif up < -1e-6:
                    depth = height / -up                                        # where this ray meets the floor
                    if depth * along <= FLOOR_REACH_M:
                        out.append((r, c, ox + depth * fwd, oy + depth * left, None, depth * along))
        return out

    def zones(self):
        """`_zones()`, for callers outside: every ToF zone from where the head points now."""
        return self._zones()

    def floor_profile(self) -> List[terrain.Point]:
        """The zones as (x ahead, y left, height above the floor); height None is a
        ray that should have met the floor there and did not. Empty without a grid."""
        return [(x, y, h) for _, _, x, y, h, _ in self._zones()]

    def scan_look(self) -> Tuple[terrain.Slice, List[Tuple[float, float, float]]]:
        """Everything one look adds to the map, from one grid: the slice (see
        terrain.scan_slice: the nearest thing in the robot's way along each of the
        eight bearings, which of those are low, where the floor falls away, how far
        the floor was seen), and every reading as a point (x, y, height) in base_link."""
        cols: dict = {}
        cloud = []
        for _, c, x, y, h, d in self._zones():
            cols.setdefault(c, []).append((d, x, y, h))
            if h is not None:
                cloud.append((x, y, h))
        sl = terrain.scan_slice([terrain.Column(tuple(sorted(v, key=lambda z: z[0]))) for v in cols.values()],
                                self.terrain)
        return sl, cloud

    def scan_slice(self) -> terrain.Slice:
        return self.scan_look()[0]

    def cloud(self) -> List[Tuple[float, float, float]]:
        """Every ToF reading from where the head points now, as (x, y, height) in base_link."""
        return [(x, y, h) for _, _, x, y, h, _ in self._zones() if h is not None]

    def floor_scan(self) -> List[terrain.Point]:
        """The floor ahead from the drive pose, twice: the second look half a zone
        lower, so its rows land between the first look's. Sixteen strips of floor
        where one look gives eight, which is what finds a low sill."""
        self.drive_pose()
        pts = self.floor_profile()
        lower = self.geom.drive_tilt_deg - ZONE_DEG / 2.0
        if lower >= self.gimbal.tilt_limits()[0]:
            self.gimbal.move_to(0.0, lower, wait=True)
            pts += self.floor_profile()
        if self.terrain.auto_level:                   # heights from the floor in front, not the servo's word
            pts, self.last_level = terrain.level(pts)
        self.last_floor = pts                         # for local_plan: what was in the way, all across the view
        return pts

    def floor_look(self, tilt_deg: float) -> List[terrain.Point]:
        """One more look at the floor, from `tilt_deg` (higher than the drive pose: further out)."""
        self.gimbal.move_to(0.0, tilt_deg, wait=True)
        pts = self.floor_profile()
        if self.terrain.auto_level:                   # its own floor fit, else the last scan's
            lv, fit = terrain.level(pts)
            if fit is not None:
                return lv
            if self.last_level is not None:
                a, b, c = self.last_level[0], math.tan(math.radians(self.last_level[1])), \
                    math.tan(math.radians(self.last_level[2]))
                return [(x, y, None if h is None else h - (a + b * x + c * y)) for x, y, h in pts]
        return pts

    def terrain_ahead(self, reach_m: float) -> terrain.Terrain:
        """Look down and judge the path for the next `reach_m`. The head is left
        looking at the floor; the caller puts it back."""
        return terrain.classify(self.floor_scan(), self.terrain, reach_m)

    # ---------------------------------------------------------------- probes

    def floor_probe(self, distance_m: float, pans: Sequence[float] = (-12.0, 0.0, 12.0),
                    short: float = 0.85, long: float = 1.3) -> Probe:
        """Is the floor where it should be, `distance_m` ahead?"""
        if self.has_grid:
            return self._floor_probe_grid(distance_m)
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

    def _floor_probe_grid(self, distance_m: float) -> Probe:
        """The same answer from the ToF grid: one look, the whole path."""
        pts = self.floor_scan()
        t = terrain.classify(pts, self.terrain, max(0.3, distance_m))
        self.gimbal.move_to(0.0, 0.0, wait=False)
        if terrain.may_drive(t, self.terrain):
            return Probe("clear" if t.kind == terrain.FLAT else "unknown", None, [])
        if t.kind == terrain.UNKNOWN:              # a floor the sensor cannot read, and not allowed to drive it
            return Probe("unseen", None, [(self.geom.pan_axis_x_m + max(0.3, distance_m), 0.0)])
        half, tol = self.terrain.corridor_m / 2.0, self.terrain.flat_tol_m
        if t.kind == terrain.DROP:
            return Probe("drop", t.distance_m, [(t.distance_m, 0.0)])
        # blocked, or climbable: nothing climbs yet, so a low step is an obstacle too
        marks = [(x, y) for x, y, h in pts if h is not None and abs(y) <= half and h > tol and x <= distance_m + 0.3]
        return Probe("obstacle", t.distance_m, marks or [(t.distance_m, 0.0)])

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
