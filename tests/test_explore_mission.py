"""The explore mission against a tiny simulated world.

Real `Head` geometry (pixel→angle, range→point, floor probe) and real mission
logic; fake servos, rangefinder, camera, wheels and vision service. The
simulated detector puts a box exactly where a real camera would see the plant,
so a passing test means the plant is localised by the same maths the robot runs.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from robot_core.explore import ExploreConfig, Explorer
from robot_core.odometry import Pose2D, wrap
from robot_core.sensors.camera import Frame
from robot_core.sensors.head import Head, HeadGeometry
from robot_core.sensors.tfluna import Reading

W, H = 640, 480
GEOM = HeadGeometry(height_m=0.25, pan_axis_x_m=0.05, hfov_deg=66.0, servo_lag_s=0.0)
ROOM = [((-2, -2), (4, -2)), ((4, -2), (4, 3)), ((4, 3), (-2, 3)), ((-2, 3), (-2, -2))]
PLANT = (1.5, 1.0)
PLANT_R = 0.12


class World:
    def __init__(self, plant=PLANT, box=None):
        self.pose = Pose2D()
        self.plant = plant
        self.box = box                   # low obstacle (x, y, r) below the level sweep

    def cast(self, origin, ang, tilt_deg=0.0):
        if tilt_deg < -1:                # looking at the floor
            d = GEOM.height_m / math.sin(math.radians(-tilt_deg))
            horiz = d * math.cos(math.radians(tilt_deg))
            if self.box:
                bx, by, br = self.box
                t = _ray_circle(origin, ang, (bx, by), br)
                if t is not None and t < horiz:
                    return t / math.cos(math.radians(tilt_deg))
            return d
        ox, oy = origin
        best = 8.0
        for (x1, y1), (x2, y2) in ROOM:
            dx, dy = math.cos(ang), math.sin(ang)
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t > 0 and 0 <= u <= 1:
                best = min(best, t)
        if self.plant:
            t = _ray_circle(origin, ang, self.plant, PLANT_R)
            if t is not None:
                best = min(best, t)
        return best

    def sensor(self):
        return self.pose.compose(Pose2D(GEOM.pan_axis_x_m, 0, 0))


def _ray_circle(o, ang, c, r):
    dx, dy = math.cos(ang), math.sin(ang)
    fx, fy = o[0] - c[0], o[1] - c[1]
    b = fx * dx + fy * dy
    disc = b * b - (fx * fx + fy * fy - r * r)
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    return t if t > 0 else None


class FakeGimbal:
    def __init__(self):
        self.pan = self.tilt = 0.0
        self.log = []

    def move_to(self, pan=None, tilt=None, speed_dps=None, wait=True, timeout=10):
        if pan is not None:
            self.pan = max(-45, min(45, pan))
        if tilt is not None:
            self.tilt = max(-30, min(45, tilt))
        self.log.append((self.pan, self.tilt))
        return self.pan, self.tilt

    def angles(self):
        return self.pan, self.tilt

    def tilt_limits(self):
        return -30.0, 45.0

    def settled(self, s=0.15):
        return True

    def wait_settled(self, *a, **k):
        return True

    def home(self, wait=True):
        self.move_to(0, 0)


class FakeLidar:
    def __init__(self, world, gimbal):
        self.w, self.g = world, gimbal

    def _now(self):
        s = self.w.sensor()
        return self.w.cast((s.x, s.y), s.theta + math.radians(self.g.pan), self.g.tilt)

    def read(self, n=10, timeout=1.0):
        return self._now()

    def since(self, t0, t1=None):
        # A sweep: pretend readings arrived every 0.5° across ±45°.
        s = self.w.sensor()
        out = []
        for k, pan in enumerate(np.arange(-45, 45.01, 0.5)):
            r = self.w.cast((s.x, s.y), s.theta + math.radians(pan))
            out.append(Reading(t0 + k * 1e-3, r if r < 8 else None, 500))
        return out


class FakeCamera:
    def __init__(self, gimbal):
        self.g, self.seq = gimbal, 0

    def capture(self, gimbal=None):
        self.seq += 1
        return Frame(self.seq, time.monotonic(), np.zeros((H, W, 3), np.uint8),
                     np.zeros((60, 80), np.uint8), self.g.pan, self.g.tilt, True)


class SweepingHead(Head):
    """Real Head, but sweep() uses FakeLidar's synthetic timeline."""

    def sweep(self, pan_from=-45.0, pan_to=45.0, tilt=0.0):
        self.gimbal.move_to(pan_from, tilt)
        rs = self.lidar.since(0.0)          # always generated -45 → +45
        return [(math.radians(-45 + 0.5 * i), r.range_m) for i, r in enumerate(rs)]


class FakeBase:
    def __init__(self, world):
        self.w = world
        self.m2o = None
        self.drives = []

    def odom(self):
        return self.w.pose

    def drive(self, m):
        self.drives.append(m)
        self.w.pose = self.w.pose.compose(Pose2D(m, 0, 0))
        return "done"

    def turn_ccw(self, rad):
        self.w.pose = Pose2D(self.w.pose.x, self.w.pose.y, wrap(self.w.pose.theta + rad))
        return "done"

    def set_map_to_odom(self, p):
        self.m2o = p

    def relocalized(self):
        pass


class FakeVision:
    def __init__(self, world, gimbal, matches=None, same=None, keyframes_until_done=2):
        self.w, self.g = world, gimbal
        self.matches = matches or []
        self.same = same
        self.enrolled = []
        self.keyframes = []
        self.until_done = keyframes_until_done
        self.fx = (W / 2) / math.tan(math.radians(GEOM.hfov_deg) / 2)

    def detect(self, jpeg, seq, min_confidence=0.4):
        dets = []
        if self.w.plant:
            s = self.w.sensor()
            cam_th = s.theta + math.radians(self.g.pan)
            b = wrap(math.atan2(self.w.plant[1] - s.y, self.w.plant[0] - s.x) - cam_th)
            if abs(b) < math.radians(GEOM.hfov_deg / 2):
                u = W / 2 - self.fx * math.tan(b)
                d = math.hypot(self.w.plant[0] - s.x, self.w.plant[1] - s.y)
                half = self.fx * PLANT_R / d
                v_pot = H / 2 + self.fx * math.tan(math.radians(self.g.tilt))
                dets.append({"class": "potted plant", "confidence": 0.8,
                             "bbox": [u - half, v_pot - 3 * half, u + half, v_pot + half]})
        return {"detections": dets, "image_size": [W, H]}

    def slam_reset(self, name, pose, load=False, **scope):
        return {}

    def slam_keyframe(self, odom, rays, sensor_xy, obstacles, plan=True):
        self.keyframes.append({"odom": odom, "rays": rays, "obstacles": obstacles})
        done = len(self.keyframes) >= self.until_done
        p = self.w.pose
        return {"kf_id": len(self.keyframes) - 1, "map_to_odom": [0, 0, 0],
                "match": {"accepted": True, "score": 0.8}, "explored_m2": 10.0, "done": done,
                "goal": None if done else {"path": [[p.x, p.y], [p.x - 0.5, p.y]]}}

    def plant_embed(self, crop):
        return [1.0]

    def plant_match(self, emb, pos):
        return {"matches": self.matches, "threshold": 0.62}

    def plant_photo(self, label):
        return b"old-photo"

    def same_plant(self, now, before):
        return self.same

    def slam_viewpoints(self, start, target, radius, n):
        a0 = math.atan2(start[1] - target[1], start[0] - target[0])
        out = []
        for k in range(n):
            a = a0 + 2 * math.pi * k / n
            x, y = target[0] + radius * math.cos(a), target[1] + radius * math.sin(a)
            out.append({"angle_deg": math.degrees(a), "pose": [x, y, a + math.pi]})
        return out

    def slam_plan(self, start, goal):
        return [list(start), list(goal)]

    def plant_enroll(self, jpegs, label, meta):
        self.enrolled.append((label, jpegs, meta))
        return {"label": label or "plant_001", "photos_saved": len(jpegs), "folder": "/tmp/p"}

    def slam_save(self):
        return {}


def make(world=None, **vision_kw):
    world = world or World()
    g = FakeGimbal()
    head = SweepingHead(g, FakeLidar(world, g), FakeCamera(g), GEOM)
    base = FakeBase(world)
    vision = FakeVision(world, g, **vision_kw)
    return Explorer(base, head, vision, ExploreConfig(max_minutes=1)), world, vision, base


def test_new_plant_is_localised_orbited_and_enrolled():
    ex, world, vision, _ = make()
    ex.run()
    assert len(vision.enrolled) == 1
    label, photos, meta = vision.enrolled[0]
    assert label == ""                                     # new plant
    assert len(photos) == 4 and len(meta["views"]) == 4   # full 360 in 4 views
    px, py = meta["position"]
    # range hits the near side of the pot; pot_radius_m pushes it back to the centre
    assert math.hypot(px - PLANT[0], py - PLANT[1]) < 0.08
    assert all(v["detected"] for v in meta["views"])       # plant in every photo
    kf = vision.keyframes[0]
    assert len(kf["rays"]) > 600                           # 4 sectors of sweep
    assert ex.stats.plants_new == ["plant_001"]


def test_known_plant_in_the_same_place_gets_a_new_visit():
    ex, _, vision, _ = make(matches=[{"label": "plant_007", "similarity": 0.9,
                                      "distance_m": 0.1, "position": list(PLANT)}])
    ex.run()
    assert vision.enrolled[0][0] == "plant_007"


def test_similar_plant_elsewhere_asks_the_vlm():
    far = [{"label": "plant_007", "similarity": 0.8, "distance_m": 3.0, "position": [5, 5]}]
    ex, _, vision, _ = make(matches=far, same=True)
    ex.run()
    assert vision.enrolled[0][0] == "plant_007"           # moved, VLM confirmed
    ex, _, vision, _ = make(matches=far, same=False)
    ex.run()
    assert vision.enrolled[0][0] == ""                    # a sibling: new plant


def test_low_obstacle_found_by_floor_probe_stops_the_drive_and_reaches_the_map():
    world = World(plant=None, box=(-0.45, 0.0, 0.08))      # right on the planned path
    ex, _, vision, base = make(world, keyframes_until_done=3)
    ex.run()
    assert vision.keyframes[1]["obstacles"], "obstacle not reported to SLAM"
    assert ex.stats.obstacles >= 1
    # never drove into it: robot stays short of the box edge
    assert world.pose.x > -0.45 + 0.08 + 0.05


def test_plants_outside_pan_range_are_left_for_the_next_sector():
    world = World(plant=(0.3, 2.0))       # ~80° left: visible at pan +30, unreachable by pan
    ex, _, vision, _ = make(world)
    sightings = ex.look_for_plants(world.pose)
    assert sightings == []


def test_plants_outside_the_radius_or_behind_the_start_are_skipped():
    from robot_core.explore import ExploreConfig, Explorer
    ex = Explorer(None, None, None, ExploreConfig(start_pose=(1.0, 0.0, 0.0), explore_radius_m=10, no_behind=True))
    assert ex._in_scope((4.0, 1.0)) and not ex._in_scope((0.0, 0.0)) and not ex._in_scope((12.0, 0.0))
    ex.cfg.no_behind = False
    assert ex._in_scope((0.0, 0.0))
    ex.cfg.explore_radius_m = 0                                   # 0 = unlimited
    assert ex._in_scope((50.0, 0.0))
