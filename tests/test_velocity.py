"""robot_core/velocity.py: (v, w) -> PWM per track, held with encoder feedback."""

from robot_core.velocity import VelocityParams, WheelSpeedController, params_from

WB = 0.18


def test_starts_gently_and_drives_both_tracks_forward_alike():
    c = WheelSpeedController(WB)
    l1, r1 = c.step(0.2, 0.0, None, 0.1)
    assert abs(c.v - 0.04) < 1e-9, "0.4 m/s^2: 4 cm/s after the first 0.1 s"
    assert l1 == r1 > 60
    for _ in range(10):
        l, r = c.step(0.2, 0.0, None, 0.1)
    assert l == r == round(60 + 0.2 * 750)


def test_turning_left_slows_the_left_track_and_speeds_the_right():
    c = WheelSpeedController(WB)
    for _ in range(20):
        l, r = c.step(0.1, 0.8, None, 0.1)
    assert r > l
    for _ in range(20):
        l, r = c.step(0.0, 0.8, None, 0.1)
    assert l < 0 < r and l == -r, "in place: the tracks run opposite ways"


def test_a_track_that_runs_fast_is_held_to_the_same_speed_as_the_other():
    c = WheelSpeedController(WB)
    speed = [0.0, 0.0]
    gain = (1.0, 1.25)                                     # the right motor runs 25 % faster for the same PWM
    for _ in range(60):
        pwm = c.step(0.2, 0.0, tuple(speed), 0.1)
        speed = [max(0.0, abs(p) - 60) / 750 * g * (1 if p >= 0 else -1) for p, g in zip(pwm, gain)]
    assert abs(speed[0] - 0.2) < 0.01 and abs(speed[1] - 0.2) < 0.01, speed


def test_stopping_ramps_down_then_releases_the_wheels():
    c = WheelSpeedController(WB)
    for _ in range(10):
        c.step(0.2, 0.0, None, 0.1)
    out = [c.step(0.0, 0.0, None, 0.1) for _ in range(8)]
    assert out[0][0] > 0, "not a slam"
    assert out[-1] == (0, 0) and c.v == 0.0


def test_feedback_never_drives_a_track_backwards_and_limits_hold():
    c = WheelSpeedController(WB)
    for _ in range(10):
        l, r = c.step(0.05, 0.0, (1.0, 1.0), 0.1)          # measured far too fast
    assert l >= 0 and r >= 0
    c.reset()
    for _ in range(40):
        c.step(9.0, 9.0, None, 0.1)
    assert c.v <= 0.25 and c.w <= 1.0


def test_params_come_from_robot_yaml_names():
    p = params_from({"velocity_max_mps": 0.3, "velocity_pwm_min": 70})
    assert p.max_mps == 0.3 and p.pwm_min == 70 and p.kp == VelocityParams().kp
