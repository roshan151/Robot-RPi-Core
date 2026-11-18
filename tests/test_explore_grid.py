"""Explore with the 8x8 ToF, against a small simulated room in 3-D.

The fake sensor casts each of its 64 rays through the room (floor, walls, a plant,
a low box, a place where the floor ends) from wherever the real `Head` points, so
what is tested is the robot's own geometry: grid -> heights -> map data, and the
floor check before every drive.
"""

from __future__ import annotations

import math
import time

from robot_core import terrain
from robot_core.explore import ExploreConfig, Explorer
from robot_core.odometry import Pose2D
from robot_core.sensors import tof8x8
from robot_core.sensors.head import Head, HeadGeometry

from test_explore_mission import PLANT, PLANT_R, ROOM, FakeBase, FakeCamera, FakeGimbal, FakeVision, H, W

GEOM = HeadGeometry(height_m=0.25, pan_axis_x_m=0.05, hfov_deg=66.0, servo_lag_s=0.0, drive_tilt_deg=-30.0)
PLANT_H, BOX_H = 0.5, 0.10


class Room:
    """plant: (x, y) or None. box: (x, y, r), 10 cm tall. floor_ends_x: no floor (a
    stairwell) for x below it. dark: (x0, x1) a strip of floor that returns nothing.
    upper: (z0, z1) a second plant at the same spot, on a shelf of a plant stand."""

    def __init__(self, plant=PLANT, box=None, floor_ends_x=None, dark=None, upper=None):
        self.pose = Pose2D()
        self.plant, self.box, self.floor_ends_x, self.dark, self.upper = plant, box, floor_ends_x, dark, upper

    def plants(self):
        """(z of the pot's base, z of the top) of every plant at `plant`."""
        return [] if not self.plant else [(0.0, PLANT_H)] + ([self.upper] if self.upper else [])

    def sensor(self):
        return self.pose.compose(Pose2D(GEOM.pan_axis_x_m, 0, 0))

    def solid(self, x, y, z):
        """What is at this point: "floor", "thing", "void" (reflects nothing), or None (air)."""
        if z <= 0.0:
            if self.floor_ends_x is not None and x < self.floor_ends_x:
                return "floor" if z <= -0.4 else None          # the next level down, 40 cm below
            if self.dark and self.dark[0] <= x <= self.dark[1]:
                return "void"
            return "floor"
        if self.plant and math.hypot(x - self.plant[0], y - self.plant[1]) <= PLANT_R \
                and any(z0 <= z <= z1 for z0, z1 in self.plants()):
            return "thing"
        if self.box and z <= BOX_H and math.hypot(x - self.box[0], y - self.box[1]) <= self.box[2]:
            return "thing"
        (x0, y0), (x1, y1) = ROOM[0][0], ROOM[1][1]             # the room's corners
        if not (x0 < x < x1 and y0 < y < y1):
            return "thing"                                      # a wall
        return None


class GridGimbal(FakeGimbal):
    def move_to(self, pan=None, tilt=None, speed_dps=None, wait=True, timeout=10):
        if pan is not None:
            self.pan = max(-90, min(90, pan))
        if tilt is not None:
            self.tilt = max(-45, min(45, tilt))
        self.log.append((self.pan, self.tilt))
        return self.pan, self.tilt

    def tilt_limits(self):
        return -45.0, 45.0


class GridLidar:
    """An 8x8 ToF in the room: perpendicular ranges, None beyond 3.5 m or off a void."""

    def __init__(self, room, gimbal):
        self.room, self.g = room, gimbal

    def grid(self, newer_than=None, timeout=1.0):
        s = self.room.sensor()
        pan, tilt = s.theta + math.radians(self.g.pan), math.radians(self.g.tilt)
        rows = []
        for r in range(8):
            row = []
            for c in range(8):
                up, left = tof8x8.zone_angles(r, c)
                ty, tz = math.tan(math.radians(left)), math.tan(math.radians(up))
                fwd, upz = math.cos(tilt) - tz * math.sin(tilt), math.sin(tilt) + tz * math.cos(tilt)
                dx = fwd * math.cos(pan) - ty * math.sin(pan)
                dy = fwd * math.sin(pan) + ty * math.cos(pan)
                hit = None
                for i in range(1, 351):                          # march 1 cm of depth at a time
                    d = i * 0.01
                    what = self.room.solid(s.x + d * dx, s.y + d * dy, GEOM.height_m + d * upz)
                    if what:
                        hit = d if what != "void" else None
                        break
                row.append(hit)
            rows.append(tuple(row))
        return tof8x8.Grid(time.monotonic(), tuple(rows))

    def read(self, n=10, timeout=1.0):
        return self.grid().centre()


class GridVision(FakeVision):
    def __init__(self, room, gimbal, goal=(-0.5, 0.0), **kw):
        super().__init__(room, gimbal, **kw)
        self.goal = goal

    def detect(self, jpeg, seq, min_confidence=0.4):
        """A box around every plant in view, from its pot's base to its top, where a
        camera on the head would see it (so it moves in the picture as the head tilts)."""
        dets = []
        s = self.w.sensor()
        tilt = math.radians(self.g.tilt)
        for z0, z1 in self.w.plants():
            b = math.atan2(self.w.plant[1] - s.y, self.w.plant[0] - s.x) - (s.theta + math.radians(self.g.pan))
            b = math.atan2(math.sin(b), math.cos(b))
            if abs(b) >= math.radians(GEOM.hfov_deg / 2):
                continue
            dist = math.hypot(self.w.plant[0] - s.x, self.w.plant[1] - s.y)
            depth = dist * math.cos(b)

            def row(z):                                         # image row of height z at the plant
                fwd = depth * math.cos(tilt) + (z - GEOM.height_m) * math.sin(tilt)
                up = -depth * math.sin(tilt) + (z - GEOM.height_m) * math.cos(tilt)
                return H / 2 - self.fx * up / fwd

            u, half = W / 2 - self.fx * math.tan(b), self.fx * PLANT_R / depth
            top, bottom = max(0.0, row(z1)), min(float(H), row(z0))
            if bottom - top > 8:                                # enough of it in the picture
                dets.append({"class": "potted plant", "confidence": 0.8, "bbox": [u - half, top, u + half, bottom]})
        return {"detections": dets, "image_size": [W, H]}

    def slam_keyframe(self, odom, rays, sensor_xy, obstacles, plan=True, low=(), drops=(), clear=(), cloud=()):
        out = super().slam_keyframe(odom, rays, sensor_xy, obstacles, plan)
        self.keyframes[-1].update(low=list(low), drops=list(drops), clear=list(clear), cloud=list(cloud))
        if out["goal"]:
            p = self.w.pose
            out["goal"] = {"path": [[p.x, p.y], [p.x + self.goal[0], p.y + self.goal[1]]]}
        return out

    def describe_plant(self, jpeg):
        return {"name": "monstera", "species": "Monstera deliciosa", "condition": "healthy", "confidence": 0.8}


def make(room=None, params=None, **vision_kw):
    room = room or Room()
    g = GridGimbal()
    head = Head(g, GridLidar(room, g), FakeCamera(g), GEOM, terrain_params=params or terrain.Params())
    base = FakeBase(room)
    vision = GridVision(room, g, **vision_kw)
    return Explorer(base, head, vision, ExploreConfig(max_minutes=1)), room, vision, base


def test_one_keyframe_sees_all_round_with_one_turn_and_maps_the_walls():
    ex, room, vision, base = make(Room(plant=None))
    turns = []
    base.turn_ccw = lambda rad, _t=base.turn_ccw: turns.append(rad) or _t(rad)
    kf, rays, _ = ex.scan_keyframe()
    assert turns == [math.pi], "one 180 degree turn, where the single beam needed three of 90"
    assert len(rays) > 100
    # Walls nearer than the sensor's 3.5 m give rays; where the wall is further, the floor
    # seen on the way gives free space instead. Between them there is no blind sector.
    clear = ex._kf_clear
    bearings = sorted(math.degrees(b) % 360 for b, _ in list(rays) + list(clear))
    gaps = [b - a for a, b in zip(bearings, bearings[1:] + [bearings[0] + 360])]
    assert max(gaps) < 20, "no blind sector"
    assert clear and all(0.5 < r < 3.1 for _, r in clear), "free space only as far as floor was seen"
    # Every ray ends on a wall of the 6 x 5 m room (those within the sensor's 3.5 m).
    for b, r in rays:
        x, y = 0.05 + r * math.cos(b), r * math.sin(b)
        assert min(abs(x - 4), abs(x + 2), abs(y - 3), abs(y + 2)) < 0.12, (x, y)


def test_plant_is_found_ranged_by_the_grid_measured_named_and_enrolled():
    ex, room, vision, _ = make()
    ex.run()
    assert len(vision.enrolled) == 1
    label, photos, meta = vision.enrolled[0]
    assert label == "" and len(photos) == 4
    assert math.hypot(meta["position"][0] - PLANT[0], meta["position"][1] - PLANT[1]) < 0.12
    w, h = meta["size_m"]
    assert abs(w - 2 * PLANT_R) < 0.08 and abs(h - PLANT_H) < 0.12
    assert meta["about"]["name"] == "monstera"
    assert len(meta["position"]) == 3 and meta["position"][2] == 0.0 and meta["height_m"] == 0.0   # on the floor
    assert abs(meta["top_m"] - PLANT_H) < 0.12 and meta["on_stand"] is False


def test_two_plants_on_a_plant_stand_are_two_plants_at_one_spot_and_two_heights():
    room = Room(upper=(0.70, 1.05))                              # a second pot on a shelf 70 cm up
    ex, room, vision, _ = make(room)
    ex.run()
    assert len(vision.enrolled) == 2, "one plant per shelf"
    metas = sorted((m for _, _, m in vision.enrolled), key=lambda m: m["height_m"])
    for m in metas:
        assert math.hypot(m["position"][0] - PLANT[0], m["position"][1] - PLANT[1]) < 0.15
        assert m["on_stand"] is True
    assert metas[0]["height_m"] == 0.0 and abs(metas[1]["height_m"] - 0.70) < 0.12
    assert [m["level"] for m in metas] == [0, 1]
    assert abs(metas[1]["top_m"] - 1.05) < 0.15
    # Each was photographed looking at its own shelf: the upper one with the head tilted up.
    tilts = [max(v["tilt_deg"] for v in m["views"]) for m in metas]
    assert tilts[1] > tilts[0] + 15


def test_a_known_plant_on_the_other_shelf_is_not_mistaken_for_this_one():
    room = Room(upper=(0.70, 1.05))
    known = [{"label": "plant_007", "similarity": 0.9, "distance_m": 0.7,
              "position": [PLANT[0], PLANT[1], 0.70]}]           # the look-alike already known is the UPPER one
    ex, room, vision, _ = make(room, matches=known)
    ex.run()
    by_height = {round(m["height_m"], 1): label for label, _, m in vision.enrolled}
    assert by_height[0.7] == "plant_007" and by_height[0.0] == "", "the floor plant is new; the shelf plant is the known one"


def test_every_reading_goes_to_the_map_as_a_3d_point():
    ex, room, vision, _ = make(Room(plant=None))
    ex.run()
    cloud = vision.keyframes[0]["cloud"]
    assert len(cloud) > 600
    zs = [z for _, _, z in cloud]
    assert min(zs) > -0.05 and max(zs) > 1.0, "the floor, the walls, and the looks upward"
    assert all(-2.2 < x < 4.2 and -2.2 < y < 3.2 for x, y, _ in cloud), "all inside the room"


def test_a_low_box_the_level_beam_would_miss_goes_on_the_map_as_low():
    room = Room(plant=None, box=(0.7, 0.3, 0.08))                # 10 cm tall: under the sensor's level view
    ex, _, vision, _ = make(room)
    ex.run()
    low = vision.keyframes[0]["low"]
    assert low and min(math.hypot(x - 0.7, y - 0.3) for x, y in low) < 0.15
    assert ex.stats.low_obstacles >= 1


