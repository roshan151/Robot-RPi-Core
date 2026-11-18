"""Terrain rules on synthetic floors, and the head's floor profile from a fake ToF grid."""

from __future__ import annotations

import math
import time
from types import SimpleNamespace

import pytest

from robot_core import terrain
from robot_core.sensors import tof8x8
from robot_core.sensors.head import Head, HeadGeometry
from robot_core.terrain import BLOCKED, CLIMBABLE, DROP, FLAT, UNKNOWN, Params, classify

P = Params(h_max_m=0.03, min_landing_m=0.20, max_drop_m=0.03)


def floor(height, x0=0.15, x1=1.0, step=0.025, ys=(-0.1, -0.03, 0.03, 0.1)):
    """Points every 2.5 cm ahead; `height(x)` is the surface there."""
    n = int(round((x1 - x0) / step))
    return [(x0 + i * step, y, height(x0 + i * step)) for i in range(n + 1) for y in ys]


def test_flat_floor_with_sensor_noise_is_flat():
    assert classify(floor(lambda x: 0.008 * math.sin(40 * x)), P, 0.5).kind == FLAT


def test_a_door_sill_is_climbable_and_says_where_and_how_high():
    t = classify(floor(lambda x: 0.02 if 0.40 <= x < 0.44 else 0.0), P, 0.6)
    assert t.kind == CLIMBABLE and t.distance_m == pytest.approx(0.40) and t.height_m == pytest.approx(0.02)


def test_a_low_platform_is_climbable_when_it_stays_flat():
    assert classify(floor(lambda x: 0.025 if x >= 0.35 else 0.0), P, 0.5).kind == CLIMBABLE


def test_a_box_is_blocked():
    t = classify(floor(lambda x: 0.12 if x >= 0.40 else 0.0, x1=0.45), P, 0.6)
    assert t.kind == BLOCKED and t.distance_m == pytest.approx(0.40)


def test_stairs_up_are_blocked_the_first_step_alone_would_have_passed():
    stairs = floor(lambda x: 0.0 if x < 0.35 else 0.025 if x < 0.45 else 0.17)
    assert classify(stairs, P, 0.5).kind == BLOCKED


def test_a_step_with_nothing_seen_beyond_it_is_blocked():
    assert classify(floor(lambda x: 0.02 if x >= 0.40 else 0.0, x1=0.42), P, 0.6).kind == BLOCKED


def test_only_what_is_within_reach_counts():
    box_far = floor(lambda x: 0.12 if x >= 0.80 else 0.0)
    assert classify(box_far, P, 0.4).kind == FLAT
    assert classify(box_far, P, 0.9).kind == BLOCKED


def test_beside_the_path_does_not_count():
    pts = floor(lambda x: 0.0) + [(0.4, 0.30, 0.15), (0.4, -0.30, 0.15)]       # chair legs either side
    assert classify(pts, P, 0.6).kind == FLAT


def test_too_little_floor_seen_is_unknown():
    assert classify([], P, 0.5).kind == UNKNOWN
    assert classify(floor(lambda x: 0.0, x0=0.70), P, 0.5).kind == UNKNOWN     # nothing within reach
    assert classify([(0.30, 0.0, 0.0)] * 4, P, 0.5).kind == UNKNOWN            # one strip of floor says little


class Gimbal:
    def __init__(self, tilt_min=-45.0): self.pan, self.tilt, self.tilt_min, self.moves = 0.0, 0.0, tilt_min, []
    def move_to(self, pan=None, tilt=None, wait=False, **kw):
        self.pan = self.pan if pan is None else pan
        self.tilt = self.tilt if tilt is None else max(tilt, self.tilt_min)
        self.moves.append((self.pan, self.tilt))
        return self.pan, self.tilt
    def wait_settled(self, *a, **kw): return True
    def angles(self): return self.pan, self.tilt
    def tilt_limits(self): return (self.tilt_min, 45.0)


def grid_of(surface, height_m, tilt_deg, perpendicular=True):
    """What the ToF would read, tilted `tilt_deg`, over a floor whose height at x ahead is surface(x)."""
    rows = []
    for r in range(8):
        row = []
        for c in range(8):
            up, left = tof8x8.zone_angles(r, c)
            ty, tz = math.tan(math.radians(left)), math.tan(math.radians(up))
            t = math.radians(tilt_deg)
            dx, dz = math.cos(t) - tz * math.sin(t), math.sin(t) + tz * math.cos(t)   # per metre of depth
            depth = None
            for i in range(1, 700):                       # march along the ray until it meets the surface
                d = i * 0.005
                if height_m + d * dz <= surface(d * dx):
                    depth = d
                    break
            if depth is None:
                row.append(None)
            else:
                row.append(depth if perpendicular else depth * math.sqrt(1 + ty * ty + tz * tz))
        rows.append(tuple(row))
    return tof8x8.Grid(time.monotonic() + 60, tuple(rows))


class Lidar:
    """A ToF over `surface`: reads what it would see from wherever the gimbal points
    (plus `nose_up`, the robot's own pitch, which the gimbal does not know about)."""
    def __init__(self, surface, gimbal, perpendicular=True, nose_up=0.0):
        self.surface, self.gimbal, self.perpendicular, self.nose_up = surface, gimbal, perpendicular, nose_up
    def grid(self, newer_than=None, timeout=1.0):
        return grid_of(self.surface, 0.25, self.gimbal.tilt + self.nose_up, self.perpendicular)


def head_over(surface, pitch=None, perpendicular=True, nose_up=0.0):
    geom = HeadGeometry(height_m=0.25, pan_axis_x_m=0.0, drive_tilt_deg=-30.0, tof_perpendicular=perpendicular)
    g = Gimbal()
    return Head(g, Lidar(surface, g, perpendicular, nose_up), None, geom, pitch=pitch, terrain_params=P)


def test_floor_profile_puts_a_flat_floor_at_height_zero():
    for perpendicular in (True, False):
        h = head_over(lambda x: 0.0, perpendicular=perpendicular)
        h.drive_pose()
        pts = h.floor_profile()
        assert len(pts) > 40
        assert max(abs(z) for _, _, z in pts) < 0.01
        assert min(x for x, _, _ in pts) < 0.25 and max(x for x, _, _ in pts) > 1.0


def test_terrain_ahead_reads_flat_a_box_and_a_drop_through_the_grid():
    assert head_over(lambda x: 0.0).terrain_ahead(0.5).kind == FLAT
    assert head_over(lambda x: 0.15 if x >= 0.40 else 0.0).terrain_ahead(0.6).kind == BLOCKED
    assert head_over(lambda x: -0.20 if x >= 0.40 else 0.0).terrain_ahead(0.6).kind == FLAT   # drops: TF-Luna's


def test_the_robots_own_pitch_is_taken_out():
    # Nose up 8 degrees over a flat floor: the sensor looks 8 degrees higher than the head thinks.
    # Nose down 8 degrees: flat floor reads as rising unless the pitch is taken out.
    assert head_over(lambda x: 0.0, nose_up=-8.0).terrain_ahead(0.5).kind != FLAT
    assert head_over(lambda x: 0.0, nose_up=-8.0, pitch=lambda: -8.0).terrain_ahead(0.5).kind == FLAT


def test_the_floor_is_looked_at_twice_the_second_time_between_the_first_rows():
    h = head_over(lambda x: 0.0)
    h.drive_pose()
    one = sorted({round(x, 2) for x, y, _ in h.floor_profile() if abs(y) < 0.05 and x < 0.6})
    two = sorted({round(x, 2) for x, y, _ in h.floor_scan() if abs(y) < 0.05 and x < 0.6})
    assert len(two) == 2 * len(one) and [m[1] for m in h.gimbal.moves[-2:]] == [-30.0, -33.75]
    step_up = head_over(lambda x: 0.025 if x >= 0.40 else 0.0)          # a low platform: seen from above
    assert step_up.terrain_ahead(0.6).kind == CLIMBABLE


def test_floor_probe_uses_the_grid_and_keeps_its_old_answers():
    assert head_over(lambda x: 0.0).floor_probe(0.4).status == "clear"
    p = head_over(lambda x: 0.15 if x >= 0.40 else 0.0).floor_probe(0.5)
    assert p.status == "obstacle" and p.distance_m == pytest.approx(0.40, abs=0.05) and p.points
    assert head_over(lambda x: -0.20 if x >= 0.40 else 0.0).floor_probe(0.5).status == "clear"  # drops: TF-Luna's
    low = head_over(lambda x: 0.025 if x >= 0.40 else 0.0)
    assert low.floor_probe(0.5).status == "obstacle"                  # climbable, but nothing climbs yet
    h = head_over(lambda x: 0.0)
    h.floor_probe(0.4)
    assert h.gimbal.moves[0] == (0.0, -30.0) and h.gimbal.moves[-1] == (0.0, 0.0)   # looks down, then back


def test_a_head_with_the_single_beam_tfluna_still_probes_the_old_way():
    lidar = SimpleNamespace(read=lambda n, timeout: 0.25 / math.sin(math.radians(32.0)))
    h = Head(Gimbal(), lidar, None, HeadGeometry(height_m=0.25))
    assert not h.has_grid and h.floor_probe(0.4).status == "clear"


def test_params_from_robot_yaml_names():
    p = terrain.params_from({"h_max_m": 0.04, "path_width_m": 0.26})
    assert (p.h_max_m, p.corridor_m, p.max_drop_m) == (0.04, 0.26, Params().max_drop_m)


def test_returns_nearer_than_the_robot_s_own_front_are_not_obstacles():
    """Nov 18: the front-mounted TF-Luna read 8-12 cm in the bottom-left zones: a box 15 cm ahead."""
    from robot_core.sensors import tof8x8
    rows = [[0.36] * 8 for _ in range(8)]
    rows[7][:3] = [0.10, 0.08, 0.11]
    g = tof8x8.Grid(0.0, tuple(tuple(r) for r in rows))
    gim = SimpleNamespace(angles=lambda: (0.0, -30.0), wait_settled=lambda: True)
    h = Head(gim, SimpleNamespace(grid=lambda *a, **k: g), None, HeadGeometry(height_m=0.22))
    assert all(d >= 0.15 for *_, d in h.zones()) and len(h.zones()) == 61
