import math

import pytest

from robot_core.odometry import (
    DiffDriveOdometry, MotionHealth, Pose2D, effective_wheelbase_m,
    IDLE, MOVING, PUSHED, SLIPPING, STALLED, WRONG_DIRECTION,
)

TPM = 3850.0          # 38.5 ticks/cm
WB = 0.22


def feed(odo, samples, t0=0.0):
    """samples: list of (physical_left, physical_right) totals at 100 ms."""
    st = None
    for i, (l, r) in enumerate(samples):
        # firmware order is (channel left = physical right, channel right = physical left)
        st = odo.update(r, l, i * 100, t0 + i * 0.1)
    return st


def test_wheelbase_from_calibration():
    assert effective_wheelbase_m(38.5, 7.37) == pytest.approx(0.2193, abs=1e-3)


def test_straight_line():
    odo = DiffDriveOdometry(TPM, WB)
    st = feed(odo, [(i * 385, i * 385) for i in range(11)])     # 1 m in 1 s
    assert st.pose.x == pytest.approx(1.0)
    assert st.pose.y == pytest.approx(0.0)
    assert st.v == pytest.approx(0.1 / 0.1)
    assert st.sigma_xy > 0


def test_turn_in_place_ccw():
    odo = DiffDriveOdometry(TPM, WB)
    arc = WB / 2 * math.pi / 2 * TPM            # ticks per wheel for 90°
    st = feed(odo, [(-arc * k / 10, arc * k / 10) for k in range(11)])
    assert st.pose.theta == pytest.approx(math.pi / 2, abs=1e-6)
    assert abs(st.pose.x) < 1e-9


def test_reboot_rebaselines_instead_of_jumping():
    odo = DiffDriveOdometry(TPM, WB)
    feed(odo, [(0, 0), (3850, 3850)])
    st = odo.update(0, 0, 50, 5.0)              # millis went backwards
    assert st.pose.x == pytest.approx(1.0)


def test_pose_algebra_round_trip():
    a, b = Pose2D(1, 2, 0.5), Pose2D(-0.3, 0.7, -1.2)
    rel = a.between(b)
    back = a.compose(rel)
    assert back.as_tuple() == pytest.approx(b.as_tuple())


def _health_run(op, v, w, visual=None, seconds=3.0, **kw):
    h = MotionHealth(**kw)
    odo = DiffDriveOdometry(TPM, WB)
    st = odo.state()
    rep = None
    t = 0.0
    while t < seconds:
        st = type(st)(st.pose, v, w, st.distance_m, 0, 0, t, 1)
        if visual is not None:
            h.on_visual(visual, t)
        rep = h.update(st, op, t)
        t += 0.1
    return rep


def test_health_states():
    assert _health_run(None, 0, 0).state == IDLE
    assert _health_run("straight", 0.2, 0, visual=True).state == MOVING
    assert _health_run("straight", 0.0, 0).state == STALLED
    assert _health_run("straight", 0.2, 0, visual=False).state == SLIPPING
    assert _health_run(None, 0.1, 0).state == PUSHED
    assert _health_run("right", 0, math.radians(40)).state == WRONG_DIRECTION
    assert _health_run("right", 0, -math.radians(40)).state == MOVING


def test_no_stall_during_start_grace():
    assert _health_run("straight", 0.0, 0, seconds=0.7).state == MOVING
