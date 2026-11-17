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


# --------------------------------------------------------------------------- #
# With the IMU: gyro heading, tilt, bump, gyro slip, turn trim
# --------------------------------------------------------------------------- #

def test_gyro_heading_wins_when_the_tracks_slip():
    odo = DiffDriveOdometry(TPM, WB)
    arc = WB / 2 * math.pi / 2 * TPM                       # encoders say 90 degrees...
    st = None
    for k in range(11):
        st = odo.update(int(arc * k / 10), int(-arc * k / 10), k * 100, k * 0.1,
                        yaw=math.radians(70.0) * k / 10)   # ...the gyro says 70
    assert st.pose.theta == pytest.approx(math.radians(70), abs=1e-6)
    assert st.w == pytest.approx(math.radians(70), abs=1e-3)
    assert st.w_wheels == pytest.approx(math.radians(90), abs=0.02)
    plain = DiffDriveOdometry(TPM, WB)
    for k in range(11):
        last = plain.update(int(arc * k / 10), int(-arc * k / 10), k * 100, k * 0.1)
    assert st.sigma_theta < last.sigma_theta, "a gyro heading is trusted more than an encoder one"


def _imu(pitch=0.0, roll=0.0, rate=0.0, jolt=0.0):
    from robot_core.sensors.imu import ImuState
    return ImuState(0.0, pitch, roll, 0.0, rate, jolt, True)


def _odom(v=0.0, w=0.0, w_wheels=None):
    from robot_core.odometry import OdomState
    return OdomState(Pose2D(), v, w, 0.0, 0.0, 0.0, 0.0, 1, w if w_wheels is None else w_wheels)


def test_tilted_stops_the_move_and_bumped_only_reports():
    from robot_core.odometry import BUMPED, TILTED
    h = MotionHealth(max_pitch_deg=25, max_roll_deg=20)
    rep = None
    for i in range(20):
        rep = h.update(_odom(v=0.1), "straight", 2.0 + i * 0.1, imu=_imu(pitch=30))
    assert rep.state == TILTED and rep.stuck and rep.pitch_deg == 30
    h = MotionHealth()
    for i in range(15):
        rep = h.update(_odom(v=0.1), "straight", i * 0.1, imu=_imu(jolt=1.2))
    assert rep.state == BUMPED and not rep.stuck
    assert h.update(_odom(v=0.1), "straight", 5.0, imu=_imu(pitch=5, roll=-3)).state == MOVING


def test_the_nose_dropping_on_a_drive_stops_it_at_once_but_a_sill_or_a_ramp_does_not():
    from robot_core.odometry import TILTED

    def run(pitches, op="straight"):
        h, out = MotionHealth(max_dip_deg=8.0), []
        for i, p in enumerate(pitches):
            out.append(h.update(_odom(v=0.1 if op == "straight" else -0.1), op, i * 0.1, imu=_imu(pitch=p)))
        return out

    edge = run([0, 0, 0, -1, -4, -9, -14])                       # over an edge: the front falls fast
    assert edge[-2].state == TILTED and edge[-2].stuck and "nose" in edge[-2].reason
    assert all(r.state != TILTED for r in edge[:4])
    sill = run([0, 3, 6, 9, 6, 2, 0, 0])                         # up over a sill and back down to level
    assert all(r.state != TILTED for r in sill)
    ramp = run([-0.6 * i for i in range(30)])                    # a slow ramp down to -17: not fast
    assert all(r.state != TILTED for r in ramp)
    back = run([0, 0, 1, 5, 10, 15], op="reverse")               # reversing off an edge: the tail falls
    assert back[-1].state == TILTED and "tail" in back[-1].reason
    nose_up = run([0, 0, 1, 5, 10, 15])                          # nose up going forward is not a fall
    assert all(r.state != TILTED for r in nose_up)


def test_tracks_turning_but_gyro_still_is_slipping():
    h = MotionHealth()
    rep = None
    for i in range(40):                                    # encoders: 60 deg/s; gyro: barely moving
        rep = h.update(_odom(w=math.radians(3), w_wheels=math.radians(60)), "left", i * 0.1, imu=_imu(rate=3))
    assert rep.state == SLIPPING and "gyro" in rep.reason
    h = MotionHealth()
    for i in range(40):
        rep = h.update(_odom(w=math.radians(55), w_wheels=math.radians(60)), "left", i * 0.1, imu=_imu(rate=55))
    assert rep.state == MOVING


def test_turn_correction_makes_up_what_the_slip_cost():
    from robot_core.odometry import turn_correction
    assert turn_correction(-90, -84) == ("right", pytest.approx(6))     # asked 90 right, turned 84
    assert turn_correction(90, 97) == ("right", pytest.approx(7))       # overshot a left turn
    assert turn_correction(90, 82) == ("left", pytest.approx(8))
    assert turn_correction(90, 89) is None                               # inside the tolerance
    assert turn_correction(90, 20) is None                               # that is not slip: do not chase it
