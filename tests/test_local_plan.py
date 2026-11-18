"""local_plan: hazards from a floor scan, and the way round them."""

import math

from robot_core import local_plan as lp, terrain


def test_hazards_are_what_the_floor_check_refuses_and_a_lone_noisy_zone_is_not():
    p = terrain.Params()
    pts = [(0.5, 0.0, 0.0), (0.6, 0.0, 0.1), (0.62, 0.05, 0.1), (0.8, 0.0, -0.2), (0.82, 0.03, -0.2),
           (1.0, 0.0, None), (1.02, 0.02, None), (1.3, 0.0, 0.09), (0.7, 0.4, 0.9), (0.72, 0.42, 0.9)]
    got = lp.hazards_from_floor(pts, p)
    assert (0.6, 0.0) in got, "a box"
    assert (0.82, 0.03) not in got and (1.0, 0.0) not in got, "low floor and no return: drops are the TF-Luna's"
    assert (1.3, 0.0) not in got, "one zone alone is noise"
    assert (0.7, 0.4) not in got, "above the robot: it passes under"
    assert (0.5, 0.0) not in got


def test_plans_round_a_box_and_straight_when_nothing_is_in_the_way():
    box = [(1.0, y / 20.0) for y in range(-4, 5)]
    path = lp.plan((0.0, 0.0), (2.0, 0.0), box, 0.2)
    assert path[0] == (0.0, 0.0) and path[-1] == (2.0, 0.0) and len(path) > 2
    for a, b in zip(path, path[1:]):
        assert lp.line_clear(a, b, box, 0.19)
    assert lp.plan((0.0, 0.0), (2.0, 0.0), [], 0.2) == [(0.0, 0.0), (2.0, 0.0)]
    wall = [(1.0, y / 20.0) for y in range(-40, 41)]
    assert lp.plan((0.0, 0.0), (2.0, 0.0), wall, 0.2) is None


def test_hazards_are_kept_in_the_odometry_frame_and_forgotten_in_time():
    t = [0.0]
    hz = lp.Hazards(keep_s=10.0, clock=lambda: t[0])
    hz.add((1.0, 0.0, math.pi / 2), [(0.5, 0.0)])            # facing +y: 0.5 ahead is (1.0, 0.5)
    (x, y), = hz.points()
    assert abs(x - 1.0) < 1e-9 and abs(y - 0.5) < 1e-9
    t[0] = 11.0
    assert hz.points() == []


def test_leg_turns_are_positive_to_the_right():
    turn, dist = lp.leg((0.0, 0.0, 0.0), (1.0, -1.0))
    assert abs(turn - 45.0) < 1e-6 and abs(dist - math.sqrt(2)) < 1e-9


def test_a_tilted_head_reads_flat_floor_flat_once_levelled():
    import random
    rnd = random.Random(1)
    pts = [(x, y, 0.05 - 0.12 * x + 0.1 * y + rnd.uniform(-0.005, 0.005))   # 7 deg pitch, 6 deg roll, 5 cm off
           for x in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8) for y in (-0.12, -0.04, 0.04, 0.12)]
    pts += [(0.55, 0.0, 0.05 - 0.12 * 0.55 + 0.10), (0.57, 0.02, 0.05 - 0.12 * 0.57 + 0.10)]   # a 10 cm box
    lv, fit = terrain.level(pts)
    assert fit is not None and abs(fit[1] + 6.8) < 1.0 and abs(fit[2] - 5.7) < 1.0
    assert max(abs(h) for x, y, h in lv if h < 0.05) < 0.01, "the floor reads flat"
    assert sum(1 for x, y, h in lv if h > 0.08) == 2, "and the box still stands out"
    assert terrain.level([(0.4, 0.0, 0.0)])[1] is None, "too little floor: left alone"


def test_clearance_comes_from_the_robots_width():
    assert abs(lp.radius(terrain.Params(width_m=0.255, clearance_m=0.08)) - 0.2075) < 1e-9
