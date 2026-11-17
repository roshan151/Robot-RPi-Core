import time

import numpy as np
import pytest

from robot_core.sensors import tfluna
from robot_core.sensors.gimbal import AxisConfig, Gimbal
from robot_core.sensors.visual_motion import VisualMotionEstimator


def frame(dist_cm, amp, temp_raw=2400):
    b = [0x59, 0x59, dist_cm & 0xFF, dist_cm >> 8, amp & 0xFF, amp >> 8,
         temp_raw & 0xFF, temp_raw >> 8]
    return bytes(b + [sum(b) & 0xFF])


def test_tfluna_parser_resyncs_and_validates():
    buf = bytearray(b"\x00\x59" + frame(123, 500) + b"\x59\x59\x01" + frame(50, 50))
    out = tfluna.parse(buf)
    assert [(d, a) for d, a, _ in out] == [(123, 500), (50, 50)]
    assert tfluna.to_reading(123, 500, 0).range_m == pytest.approx(1.23)
    assert tfluna.to_reading(50, 50, 0).range_m is None          # weak signal
    assert tfluna.to_reading(10, 900, 0).range_m is None         # under 0.2 m


def test_tfluna_reader_median():
    data = [frame(200, 800), frame(210, 800), frame(900, 20)] * 20
    it = iter(data)
    lidar = tfluna.TFLuna(source=lambda n: next(it, b"") or time.sleep(0.01) or b"")
    time.sleep(0.05)
    # everything already arrived before read() started, so it times out on
    # "new" readings and must fall back gracefully
    assert lidar.read(n=5, timeout=0.1) is None
    lidar.close()


def test_gimbal_clamps_and_moves_smoothly():
    g = Gimbal(pan=AxisConfig(channel=1, min_deg=-45, max_deg=45, max_speed_dps=90, accel_dps2=360),
               tilt=AxisConfig(channel=0, min_deg=-90, max_deg=90, max_speed_dps=90, accel_dps2=360),
               backend="fake", hold_on_close=False)
    target = g.move_to(80, -30, wait=True, timeout=5)
    assert target == (45, -30)                                   # pan clamped
    pan, tilt = g.angles()
    assert pan == pytest.approx(45) and tilt == pytest.approx(-30)
    pulses = g.pan.out.pulses
    steps = np.abs(np.diff(pulses))
    assert steps.max() < 90 / 50 * (2000 / 180) + 1              # never a jump
    g.close()
    assert g.angles() == (0.0, 0.0)                              # parked
    assert g.pan.out.enabled is False


def test_gimbal_pulse_clamped_and_us_per_deg_from_params():
    from robot_core.sensors.gimbal import from_params
    g = from_params({"pan_center_us": 1500, "pan_us_per_deg": 8.0,
                     "tilt_center_us": 2300, "tilt_us_per_deg": 20.0}, backend="fake")
    g.move_to(45, 45, wait=True, timeout=5)
    assert g.pan.out.pulses[-1] == pytest.approx(1500 + 45 * 8.0)      # per-axis scale is honoured
    assert g.tilt.out.pulses[-1] == 2400.0                             # 2300 + 900 us would be 3200: clamped
    assert min(g.tilt.out.pulses) >= 500.0 and max(g.tilt.out.pulses) <= 2400.0
    g.close()


def test_gimbal_defaults_unchanged():
    from robot_core.sensors.gimbal import from_params
    g = from_params({}, backend="fake")
    g.move_to(45, 45, wait=True, timeout=5)
    assert g.pan.out.pulses[-1] == pytest.approx(1500 + 45 * 2000 / 180)   # still 1000..2000 us for +-45 deg
    g.close()


def test_gimbal_close_holds_centre_by_default_and_can_release_later():
    from robot_core.sensors.gimbal import from_params
    g = from_params({"pan_center_us": 2300, "tilt_center_us": 1750, "tilt_invert": True}, backend="fake")
    g.move_to(-20, 20, wait=True, timeout=10)
    g.close()
    assert g.angles() == (0.0, 0.0)
    assert g.pan.out.enabled and g.tilt.out.enabled                      # still driving the pulse: no flop
    assert g.pan.out.pulses[-1] == pytest.approx(2300) and g.tilt.out.pulses[-1] == pytest.approx(1750)
    assert np.abs(np.diff(g.pan.out.pulses)).max() < 30 / 50 * (2000 / 180) + 1   # parked in small steps
    g.close(release=True)                                                # a later release still works
    assert not g.pan.out.enabled and not g.tilt.out.enabled


def test_gimbal_angles_never_exceed_what_the_pulse_window_allows():
    from robot_core.sensors.gimbal import from_params
    g = from_params({"pan_center_us": 2300, "tilt_center_us": 1750, "tilt_invert": True}, backend="fake")
    got = g.move_to(45, 45, speed_dps=60, wait=True, timeout=10)
    assert got == pytest.approx((9.0, 45.0), abs=0.01)                   # 2400 us cap: only 9 deg to the left
    assert g.pan.out.pulses[-1] <= 2400.0
    assert g.move_to(-90, -90, speed_dps=60, wait=True, timeout=10) == pytest.approx((-90.0, -45.0))
    assert g.tilt_limits() == (-45.0, 45.0)
    g.close(release=True)


def test_visual_motion_static_vs_shift_vs_local():
    rng = np.random.default_rng(0)
    base = (rng.random((260, 360)) * 255).astype(np.uint8)
    est = VisualMotionEstimator()
    assert est.update(base[:240, :320]) is None
    still = est.update(base[:240, :320])
    assert not still.moving
    moved = est.update(base[6:246, 8:328])                      # camera moved
    assert moved.moving
    est.reset(); est.update(base[:240, :320])
    local = base[:240, :320].copy()
    local[100:140, 150:190] = 255 - local[100:140, 150:190]      # something sways
    assert not est.update(local).moving


def test_gimbal_starts_from_saved_pulse_and_walks_home(tmp_path):
    f = tmp_path / "head.json"
    cfg = AxisConfig(channel=1, min_deg=-90.0, max_deg=90.0)
    g = Gimbal(pan=cfg, backend="fake", state_path=f)
    g.move_to(pan=40.0, wait=True)
    time.sleep(0.1)
    g._stop.set()                                      # power cut / crash: no park at centre
    g = Gimbal(pan=cfg, backend="fake", state_path=f)
    try:
        assert g.pan.pos == pytest.approx(40.0, abs=0.5)    # no jump: first pulse = where it was left
        assert g.pan.target == 0.0
        g.wait(10.0)
        assert g.pan.pos == pytest.approx(0.0, abs=0.5)
    finally:
        g.close()


def test_camera_restarts_once_when_frames_stop_and_gives_up_after():
    import concurrent.futures
    from robot_core.sensors import camera as cam_mod

    class Stub:
        def __init__(self, ok): self.ok, self.calls = ok, 0
        def capture_request(self, wait=None):
            self.calls += 1
            if not self.ok:
                raise concurrent.futures.TimeoutError()
            return "frame"
        def stop(self): pass
        def close(self): pass

    cam = cam_mod.Camera.__new__(cam_mod.Camera)
    cam._picam = Stub(ok=False)
    restarts = []
    cam._start = lambda: (restarts.append(1), setattr(cam, "_picam", Stub(ok=True)))
    old, cam_mod.time = cam_mod.time, type("T", (), {"sleep": staticmethod(lambda s: None)})
    try:
        assert cam._request() == "frame" and len(restarts) == 1
        cam._picam, cam._start = Stub(ok=False), lambda: setattr(cam, "_picam", Stub(ok=False))
        with pytest.raises(RuntimeError, match="no frames"):
            cam._request()
    finally:
        cam_mod.time = old




def test_usb_camera_opens_v4l2_with_mjpeg_one_buffer_and_drops_the_stale_frame(monkeypatch):
    import cv2
    import numpy as np
    from robot_core.sensors import camera as cam_mod

    class FakeCap:
        def __init__(self, index, api=None, params=()):
            self.api, self.n, self.grabbed = api, 0, 0
            self.props = dict(zip(params[::2], params[1::2]))     # given at open, not set() one by one
        def set(self, prop, value): self.props[prop] = value
        def isOpened(self): return True
        def grab(self):
            self.n += 1
            self.grabbed += 1
            return True
        def read(self):
            self.n += 1
            return True, np.full((1080, 1920, 3), self.n, np.uint8)
        def release(self): pass

    monkeypatch.setattr(cv2, "VideoCapture", FakeCap)
    cam = cam_mod.Camera(main_size=(1920, 1080), inverted=False, backend="opencv")
    cap = cam._cap
    assert cap.api == cv2.CAP_V4L2, "V4L2 by name: left to choose, OpenCV took GStreamer (~9 s to open)"
    assert cap.props[cv2.CAP_PROP_FOURCC] == cv2.VideoWriter_fourcc(*"MJPG")
    assert cap.props[cv2.CAP_PROP_BUFFERSIZE] == 1
    f = cam.capture()
    assert cap.grabbed == 1 and int(f.main[0, 0, 0]) == 2, "the queued frame is dropped first"
    assert f.main.shape == (1080, 1920, 3) and f.lores.shape == (240, 320)


# --- servo supply: a head that cannot move must never be "moved" -----------------------

def test_gimbal_freezes_while_servo_supply_is_off_and_resumes_without_a_jump():
    """Incident 1: servo switch off while commands arrive, then switched on. The pulse the
    line holds must stay where the servo last followed it, so switching on is a non-event."""
    rail = {"on": True}
    g = Gimbal(backend="fake", power_sense=lambda: rail["on"])
    try:
        g.move_to(10, 10, speed_dps=60, wait=True, timeout=5)
        held = (g.pan.out.pulses[-1], g.tilt.out.pulses[-1])
        written = (len(g.pan.out.pulses), len(g.tilt.out.pulses))
        rail["on"] = False
        assert g.move_to(80, -40, wait=True, timeout=2) == g.angles()          # refused, not queued
        time.sleep(0.3)
        assert g.angles() == pytest.approx((10, 10), abs=0.01)
        rail["on"] = True
        time.sleep(0.3)                                                         # power returns
        assert g.angles() == pytest.approx((10, 10), abs=0.01)                  # nothing was waiting to run
        assert (len(g.pan.out.pulses), len(g.tilt.out.pulses)) == written       # no pulse written meanwhile
        assert (g.pan.out.pulses[-1], g.tilt.out.pulses[-1]) == held
        g.move_to(20, 0, speed_dps=60, wait=True, timeout=5)                    # and it works again
        assert np.abs(np.diff(g.pan.out.pulses)).max() < 60 / 50 * (2000 / 180) + 1
    finally:
        g.close(release=True)


def test_gimbal_drops_a_move_in_flight_when_servo_supply_dies():
    rail = {"on": True}
    g = Gimbal(backend="fake", power_sense=lambda: rail["on"])
    try:
        g.move_to(60, 40, speed_dps=30, wait=False)
        time.sleep(0.4)
        rail["on"] = False
        time.sleep(0.2)
        frozen = g.angles()
        assert 0 < frozen[0] < 60                                               # it had started to move
        time.sleep(0.4)
        rail["on"] = True
        time.sleep(0.4)
        assert g.angles() == pytest.approx(frozen, abs=0.01)                    # the order did not survive the outage
    finally:
        g.close(release=True)


def test_no_position_is_saved_while_the_servo_supply_is_off(tmp_path):
    """The servos ignore pulses with the supply off, so the commanded pose is not the
    real one: saving it would make the next start's first pulse dash there."""
    import json
    f = tmp_path / "head.json"
    rail = {"on": True}
    g = Gimbal(backend="fake", state_path=f, power_sense=lambda: rail["on"])
    try:
        g.move_to(-60, 30, speed_dps=60, wait=True, timeout=5)
        time.sleep(0.1)                                   # the arrival save lands just after wait() returns
        saved = json.loads(f.read_text())
        rail["on"] = False
        time.sleep(0.2)
        g._save_state()
        assert json.loads(f.read_text()) == saved
    finally:
        g.close(release=True)
    assert not f.exists(), "could not park: the saved position is dropped, the next start sends the centre"
    again = Gimbal(backend="fake", state_path=f)
    try:
        assert again.angles() == (0.0, 0.0) and again.pan.out.pulses[0] == again.pan.cfg.center_us
    finally:
        again.close(release=True)


def test_a_parked_head_is_saved_at_centre(tmp_path):
    import json
    f = tmp_path / "head.json"
    g = Gimbal(backend="fake", state_path=f, power_sense=lambda: True)
    g.move_to(-40, 20, speed_dps=60, wait=True, timeout=5)
    g.close(release=True)
    assert json.loads(f.read_text()) == {"pan_us": g.pan.cfg.center_us, "tilt_us": g.tilt.cfg.center_us}


def test_a_failing_power_sensor_reads_as_off():
    def broken():
        raise OSError("gpio gone")
    g = Gimbal(backend="fake", power_sense=broken)
    try:
        assert not g.powered()
        assert g.move_to(30, 30, wait=True, timeout=2) == (0.0, 0.0)
    finally:
        g.close(release=True)


def test_gimbal_saves_its_position_while_still_moving(tmp_path):
    """A reset mid-slew must not leave the soft start with a stale position."""
    import json
    f = tmp_path / "head.json"
    g = Gimbal(backend="fake", state_path=f)
    try:
        g.move_to(pan=60, speed_dps=30, wait=False)
        time.sleep(1.0)
        assert not g.settled()                                                  # still moving
        assert json.loads(f.read_text())["pan_us"] > 1500 + 5 * (2000 / 180)    # and the file already knows
    finally:
        g.close(release=True)


def test_servo_speed_is_a_hard_ceiling_whatever_a_caller_asks_for():
    g = Gimbal(backend="fake")                                   # servo_speed_dps default: 30
    try:
        g.move_to(pan=40, tilt=20, speed_dps=500, wait=True, timeout=10)
        for out in (g.pan.out, g.tilt.out):
            assert np.abs(np.diff(out.pulses)).max() < 30 / 50 * (2000 / 180) + 1      # never faster than 30 deg/s
    finally:
        g.close(release=True)


# --- fail-safe: anything off stops the head instead of letting it snap -------------------

def _gimbal_status(monkeypatch):
    """Capture what the gimbal puts on the OLED."""
    from robot_core import status
    seen = []
    monkeypatch.setattr(status, "error", lambda code, text="", hold=0: seen.append(code))
    return seen


def test_pwm_overwritten_behind_our_back_stops_the_head_and_latches(tmp_path, monkeypatch):
    """Another process / driver reprogramming the channel: stop, say so, stay stopped after a restart."""
    seen = _gimbal_status(monkeypatch)
    f = tmp_path / "head.json"
    g = Gimbal(backend="fake", state_path=f)
    try:
        g.pan.out.verify = lambda: "enable=1 period=20000000 duty=0 ns, but wrote 1600 us"
        deadline = time.monotonic() + 2
        while not g.fault and time.monotonic() < deadline:
            time.sleep(0.02)
        assert g.fault.startswith("pwm:") and "G04" in seen
        assert not g.pan.out.enabled and not g.tilt.out.enabled           # both outputs stopped
        assert g.move_to(30, 30, wait=True, timeout=1) == g.angles() == (0.0, 0.0)
        assert "needs a reset" in g.refuse() and not g.powered()
    finally:
        g.close()
    again = Gimbal(backend="fake", state_path=f)                          # a restart does not quietly try again
    try:
        assert again.fault.startswith("latched:") and "G07" in seen and not again.powered()
    finally:
        again.close()


def test_first_pulse_is_checked_before_anything_can_move(tmp_path, monkeypatch):
    from robot_core.sensors import gimbal as gm
    seen = _gimbal_status(monkeypatch)

    class Bad(gm.FakePwm):
        def verify(self):
            return "never reached the line"
    monkeypatch.setattr(gm, "HardwarePwm", lambda ch, chip=0: Bad())
    monkeypatch.setattr(gm, "LOCK_PATH", str(tmp_path / "lock"))
    g = Gimbal(backend="hardware", state_path=tmp_path / "head.json")
    try:
        assert g.fault.startswith("pwm:") and "G04" in seen and not g.pan.out.enabled
        assert g.move_to(10, 10, wait=True, timeout=1) == (0.0, 0.0)
    finally:
        g.close()


def test_a_pulse_that_jumps_in_one_frame_trips_it(monkeypatch):
    seen = _gimbal_status(monkeypatch)
    g = Gimbal(backend="fake")
    try:
        g.move_to(10, 0, wait=True, timeout=5)
        with g._lock:
            g.pan.pos, g.pan.target = 80.0, 10.0                          # the angle jumps behind the planner's back
            g._moving = True
        deadline = time.monotonic() + 2
        while not g.fault and time.monotonic() < deadline:
            time.sleep(0.02)
        assert g.fault.startswith("pulse:") and "G05" in seen and not g.pan.out.enabled
    finally:
        g.close()


def test_a_crashed_loop_stops_the_head_instead_of_leaving_a_pulse_running(monkeypatch):
    seen = _gimbal_status(monkeypatch)
    g = Gimbal(backend="fake")
    try:
        def boom(dt):
            raise OSError("sysfs write failed")
        g.pan.step = boom
        g.move_to(20, 0, wait=True, timeout=2)                            # wait() must not hang for the full timeout
        deadline = time.monotonic() + 2
        while not g.fault and time.monotonic() < deadline:
            time.sleep(0.02)
        assert g.fault.startswith("loop:") and "G06" in seen and not g.pan.out.enabled and not g.tilt.out.enabled
    finally:
        g.close()


def test_non_numbers_and_negative_speeds_are_refused_not_clamped_to_the_limit():
    g = Gimbal(backend="fake")
    try:
        for bad in ((float("nan"), None, None), (None, float("inf"), None), (10, 10, float("nan")), (10, 10, -30)):
            assert g.move_to(*bad, wait=False) == (0.0, 0.0)
        time.sleep(0.2)
        assert g.angles() == (0.0, 0.0) and not g.fault                   # refused, not a fault: the next order works
        assert g.move_to(10, 10, wait=True, timeout=5) == (10, 10)
    finally:
        g.close(release=True)


def test_only_one_gimbal_drives_the_pwm_at_a_time(tmp_path, monkeypatch):
    from robot_core.sensors import gimbal as gm
    seen = _gimbal_status(monkeypatch)
    made = []
    monkeypatch.setattr(gm, "HardwarePwm", lambda ch, chip=0: made.append(ch) or gm.FakePwm())
    monkeypatch.setattr(gm, "LOCK_PATH", str(tmp_path / "lock"))
    first = Gimbal(backend="hardware", state_path=tmp_path / "head.json")
    second = Gimbal(backend="hardware", state_path=tmp_path / "head.json")
    try:
        assert not first.fault and len(made) == 2
        assert second.fault.startswith("busy:") and "G08" in seen and len(made) == 2     # never touched the hardware
        assert not second.powered() and not (tmp_path / "head.fault").exists()           # busy is not latched
    finally:
        second.close()
        first.close()
    third = Gimbal(backend="hardware", state_path=tmp_path / "head.json")                # the lock was handed on
    try:
        assert not third.fault
    finally:
        third.close()


def test_reset_clears_a_latched_fault(tmp_path, monkeypatch):
    from robot_core.sensors import gimbal as gm
    f = tmp_path / "head.json"
    f.with_suffix(".fault").write_text("pwm: ch1 something")
    monkeypatch.setattr(gm, "params_from_robot_yaml", lambda: {"head_state_file": str(f)})
    monkeypatch.setattr("sys.argv", ["gimbal", "reset"])
    gm.main()
    assert not f.with_suffix(".fault").exists()
    g = Gimbal(backend="fake", state_path=f)
    try:
        assert not g.fault and g.powered()
    finally:
        g.close()


def test_hardware_verify_reads_the_kernel_channel_back(tmp_path):
    from robot_core.sensors.gimbal import HardwarePwm
    pwm = HardwarePwm.__new__(HardwarePwm)                                # no sysfs here: point it at a fake channel dir
    pwm._dir, pwm._started, pwm._us = tmp_path, True, 1600.0

    def chan(enable, period, duty):
        for name, v in (("enable", enable), ("period", period), ("duty_cycle", duty)):
            (tmp_path / name).write_text(f"{v}\n")
    chan(1, 20_000_000, 1_600_000)
    assert pwm.verify() is None
    chan(1, 20_000_000, 0)
    assert "duty=0" in pwm.verify()
    chan(0, 20_000_000, 1_600_000)
    assert "enable=0" in pwm.verify()
    chan(1, 10_000_000, 1_600_000)
    assert "period=10000000" in pwm.verify()
    (tmp_path / "duty_cycle").unlink()
    assert "unreadable" in pwm.verify()
    pwm._started = False
    assert pwm.verify() is None                                           # stopped on purpose: nothing to check


def test_head_look_reports_a_fault_and_lets_go_of_the_dead_gimbal(tmp_path):
    from robot_core.head_look import HeadLook
    f = tmp_path / "head.json"
    g = Gimbal(backend="fake", state_path=f)
    made = []
    head = HeadLook(lambda: made.append(1) or (g if len(made) == 1 else Gimbal(backend="fake", state_path=f)))
    try:
        assert head.look(20)["ok"]
        g._trip("pwm", "ch1 test")
        r = head.look(20)
        assert not r["ok"] and "fault" in r["error"] and head._gimbal is None
        r = head.look(20)                                                 # re-opens: the latch keeps it dead
        assert not r["ok"] and "needs a reset" in r["error"]
    finally:
        head.release()


# --------------------------------------------------------------------------- #
# IMU (Fermion 10 DOF) and 8x8 ToF
# --------------------------------------------------------------------------- #

def _fusion_run(fusion, accel, gyro, seconds, hz=100):
    for _ in range(int(seconds * hz)):
        fusion.update(accel, gyro, 1.0 / hz)


def test_imu_pitch_and_roll_from_gravity_with_the_documented_signs():
    import math
    from robot_core.sensors.imu import Fusion
    s, c = math.sin(math.radians(20)), math.cos(math.radians(20))
    f = Fusion()
    _fusion_run(f, (s, 0.0, c), (0, 0, 0), 2)              # nose up 20 deg: gravity shows on the forward axis
    assert f.pitch == pytest.approx(20, abs=0.5) and f.roll == pytest.approx(0, abs=0.5)
    f = Fusion()
    _fusion_run(f, (0.0, s, c), (0, 0, 0), 2)              # right side down 20 deg
    assert f.roll == pytest.approx(20, abs=0.5) and f.pitch == pytest.approx(0, abs=0.5)


def test_imu_zeroes_the_gyro_at_rest_and_yaw_then_follows_a_turn():
    from robot_core.sensors.imu import Fusion
    f = Fusion()
    _fusion_run(f, (0, 0, 1), (0.4, -0.7, 1.3), 2)         # standing still: all of this is the gyro's offset
    assert f.calibrated and f.bias == pytest.approx([0.4, -0.7, 1.3])
    yaw0 = f.yaw
    f.moving = True
    _fusion_run(f, (0, 0, 1), (0.4, -0.7, 1.3 + 45.0), 2)  # 45 deg/s to the left for 2 s
    assert f.yaw - yaw0 == pytest.approx(90, abs=0.5) and f.yaw_rate == pytest.approx(45)
    f.moving = False
    _fusion_run(f, (0, 0, 1), (0.4, -0.7, 1.3), 60)        # a minute at rest
    assert abs(f.yaw - yaw0 - 90) < 1.0, "yaw holds to within a degree a minute"


def test_imu_does_not_take_a_slow_steady_turn_for_its_zero():
    from robot_core.sensors.imu import Fusion
    f = Fusion()
    _fusion_run(f, (0, 0, 1), (0, 0, 0), 2)
    _fusion_run(f, (0, 0, 1), (0, 0, 10.0), 3)             # steady 10 deg/s, nobody said "moving"
    assert f.bias[2] == pytest.approx(0.0) and f.yaw == pytest.approx(30, abs=0.5)


def test_imu_gyro_carries_pitch_through_acceleration_and_reports_the_jolt():
    from robot_core.sensors.imu import Fusion
    f = Fusion()
    _fusion_run(f, (0, 0, 1), (0, 0, 0), 2)
    _fusion_run(f, (0.9, 0, 1.6), (0, -20.0, 0), 0.5)      # hard shove while pitching up at 20 deg/s
    assert f.pitch == pytest.approx(10, abs=0.5), "the accelerometer is ignored while it reads more than gravity"
    assert f.state(0.0).jolt_g > 0.8


def test_imu_axis_map_and_the_reader_thread():
    from robot_core.sensors.imu import Imu, axis_map
    assert axis_map(("y", "-x", "z"))((1.0, 2.0, 3.0)) == (2.0, -1.0, 3.0)
    with pytest.raises(ValueError):
        axis_map(("x", "x", "z"))
    # Board mounted with its Y forward and X to the right: nose up shows on the board's Y.
    imu = Imu(axes=("y", "-x", "z"), hz=200, source=lambda: (0.0, 0.342, 0.940, 0.0, 0.0, 0.0))
    try:
        assert imu.wait_calibrated(3.0)
        st = imu.state()
        assert st.pitch_deg == pytest.approx(20, abs=1.0) and st.roll_deg == pytest.approx(0, abs=1.0)
    finally:
        imu.close()


class _TofLink:
    """The module's side of the I2C conversation: answers "all data" with `mm`."""
    def __init__(self, mm, mode8=True):
        self.mm, self.mode8, self.out, self.writes = mm, mode8, [], []

    def write(self, pkt):
        from robot_core.sensors import tof8x8 as t
        self.writes.append(list(pkt))
        cmd = pkt[2]
        if cmd == t.CMD_SET_MODE:
            self.mode8 = True
            return
        vals = self.mm if self.mode8 else self.mm[:16]
        data = b"".join(int(v).to_bytes(2, "little") for v in vals)
        self.out = [0xFF, 0xFF, t.OK, cmd, len(data) & 0xFF, len(data) >> 8, *data]

    def read_byte(self):
        return self.out.pop(0) if self.out else 0xFF

    def close(self): pass


def test_tof_protocol_bytes_and_decoding():
    from robot_core.sensors import tof8x8 as t
    assert t.request(t.CMD_ALL_DATA) == [0, 1, 2]
    assert t.request(t.CMD_SET_MODE, [0, 0, 0, 8]) == [0, 5, 1, 0, 0, 0, 8]
    mm = [1000 + i for i in range(64)]
    mm[0], mm[1], mm[2] = 0, 5, 4000                        # nothing, too near, too far
    rows = t.decode(b"".join(v.to_bytes(2, "little") for v in mm))
    assert rows[0][:4] == (None, None, None, pytest.approx(1.003)) and rows[7][7] == pytest.approx(1.063)
    assert t.decode(b"\x00" * 32) is None                   # a 4x4 frame is not a grid
    flipped = t.decode(b"".join(v.to_bytes(2, "little") for v in mm), flip_rows=True, flip_cols=True)
    assert flipped[0][0] == pytest.approx(1.063)
    assert t.zone_angles(0, 0) == (26.25, 26.25) and t.zone_angles(7, 7) == (-26.25, -26.25)


def test_tof_on_its_side_is_turned_back_upright():
    """Board a quarter-turn clockwise (seen from behind): what is above shows in its LEFT
    columns, what is to the right in its TOP rows. tof_tilt: 90 turns the grid back."""
    from robot_core.sensors import tof8x8 as t
    world = [[r * 10 + c for c in range(8)] for r in range(8)]                 # what an upright board would read
    raw = [[world[c][7 - r] for c in range(8)] for r in range(8)]              # what the turned board reads
    pack = lambda g: b"".join((100 + v).to_bytes(2, "little") for row in g for v in row)
    want = tuple(tuple((100 + v) / 1000 for v in row) for row in world)
    assert t.decode(pack(raw), tilt_deg=90) == want
    assert t.decode(pack(world), tilt_deg=90 * 4) == want
    assert t.decode(pack(raw), tilt_deg=-270) == want                          # -270 is the same as 90
    back = [[world[7 - c][r] for c in range(8)] for r in range(8)]             # turned anticlockwise instead
    assert t.decode(pack(back), tilt_deg=-90) == want
    with pytest.raises(ValueError):
        t.Tof8x8(tilt_deg=45, link=object())


def test_tof_reader_gives_a_grid_and_stands_in_for_the_tfluna():
    from robot_core.sensors import tof8x8 as t
    mm = [2000] * 64
    for r, c in t.CENTRE:
        mm[r * 8 + c] = 1500
    tof = t.Tof8x8(hz=50, link=_TofLink(mm))
    try:
        g = tof.grid(newer_than=time.monotonic(), timeout=2.0)
        assert g is not None and g.rows[0][0] == pytest.approx(2.0) and g.centre() == pytest.approx(1.5)
        assert tof.read(n=5, timeout=1.0) == pytest.approx(1.5)        # what Head.range_at() calls
        assert tof.latest().range_m == pytest.approx(1.5)
        assert tof.since(0.0) and all(r.range_m == pytest.approx(1.5) for r in tof.since(0.0))
    finally:
        tof.close()


def test_tof_switches_a_module_left_in_4x4_back_to_8x8(monkeypatch):
    from robot_core.sensors import tof8x8 as t
    link = _TofLink([1200] * 64, mode8=False)
    tof = t.Tof8x8(hz=50, link=link)
    monkeypatch.setattr(tof._stop, "wait", lambda s=None: time.sleep(min(s or 0, 0.02)) or False)
    try:
        assert tof.grid(newer_than=time.monotonic(), timeout=3.0) is not None
        assert [0, 5, t.CMD_SET_MODE, 0, 0, 0, 8] in link.writes
    finally:
        tof._stop.set()
        tof.close()


def test_tof_range_inside_a_camera_box():
    from robot_core.sensors import tof8x8 as t
    rows = [[3.0] * 8 for _ in range(8)]
    for r in range(2, 7):
        for c in (3, 4):
            rows[r][c] = 1.4                                 # a person in the middle, wall behind
    g = t.Grid(0.0, tuple(tuple(r) for r in rows))
    W, H = 1920, 1080
    assert t.range_in_box(g, (800, 300, 1120, 1080), W, H, 80.0) == pytest.approx(1.4)
    assert t.range_in_box(g, (300, 0, 700, 1080), W, H, 80.0) == pytest.approx(3.0)  # off to the left: the wall
    assert t.range_in_box(g, (0, 0, 250, 1080), W, H, 80.0) is None                  # the camera sees wider than the ToF
    assert t.range_in_box(g, (950, 530, 970, 550), W, H, 80.0) == pytest.approx(1.4)  # tiny box: the nearest zone it touches
    assert t.range_in_box(None, (800, 300, 1120, 1080), W, H, 80.0) is None
    assert t.range_in_box(g, None, W, H, 80.0) is None


def test_bmi323_is_set_up_and_read_with_its_dummy_bytes(monkeypatch):
    """The BMI323's 16-bit registers: writes low byte first, reads start with two dummy bytes."""
    import struct
    import sys
    import types
    from robot_core.sensors import imu

    class Bus:
        chip = 0x0043

        def __init__(self, n):
            self.writes, self.regs = [], {0x00: Bus.chip, 0x01: 0x0000}

        def write_i2c_block_data(self, addr, reg, data):
            self.writes.append((addr, reg, data))

        def read_i2c_block_data(self, addr, reg, n):
            if reg == 0x03:
                words = [2048, 0, -2048, 655, 0, -655]           # +1 g x, -1 g z; +10, -10 deg/s
                return list(b"\xff\xff" + struct.pack("<6h", *words))
            return list(b"\xff\xff" + struct.pack("<H", self.regs[reg]))

        def close(self):
            pass

    monkeypatch.setitem(sys.modules, "smbus2", types.SimpleNamespace(SMBus=Bus))
    monkeypatch.setattr(imu.time, "sleep", lambda s: None)
    c = imu.Chips(1, 0x69)
    assert c._bus.writes == [(0x69, 0x7E, [0xAF, 0xDE]),                 # soft reset
                             (0x69, 0x20, [0x38, 0x40]),                 # accel: normal, +-16 g, 100 Hz
                             (0x69, 0x21, [0x28, 0x40])]                 # gyro: normal, +-500 deg/s, 100 Hz
    ax, ay, az, gx, gy, gz = c.read()
    assert (ax, ay, az) == pytest.approx((1.0, 0.0, -1.0))
    assert (gx, gz) == pytest.approx((10.0, -10.0), abs=0.01)
    Bus.chip = 0x0241                                                   # some parts report 0x41
    imu.Chips(1, 0x69)
    Bus.chip = 0x0011                                                   # something else at 0x69
    with pytest.raises(RuntimeError, match="not a BMI323"):
        imu.Chips(1, 0x69)


def test_board_flat_chips_up_x_forward_reads_level():
    """robot.yaml's imu_axes for this board: Z printed into the board, so at rest it reads -1 g."""
    from robot_core.sensors.imu import axis_map
    up = axis_map(["x", "-y", "-z"])
    assert up((0.0, 0.0, -1.0)) == (0.0, 0.0, 1.0)                        # level: gravity on robot up
    assert up((0.0, -0.5, 0.0)) == (0.0, 0.5, 0.0)                        # board -y is the robot's left


def test_tof_over_usb_reads_text_grids():
    from robot_core.sensors import tof8x8 as t
    f = t.TextFrames()
    rows = [" ".join(str(1000 + r * 8 + c) for c in range(8)) for r in range(8)]
    out = [f.feed(line, i * 0.001) for i, line in enumerate(rows)]
    assert out[:7] == [None] * 7 and out[7] == list(range(1000, 1064))
    # A header resets a half-read grid; a leading row number is dropped; a pause starts again.
    assert f.feed(rows[0], 1.0) is None and f.feed("--- frame ---", 1.001) is None
    got = [f.feed(f"{r}: {line}", 1.002 + r * 0.001) for r, line in enumerate(rows)]
    assert got[7] == list(range(1000, 1064))
    assert f.feed(rows[0], 2.0) is None and f.feed(rows[1], 2.5) is None    # 0.5 s apart: not one grid
    assert f.feed(",".join(["4000"] * 64), 3.0) == [4000] * 64              # one line of 64

    class Port:
        def __init__(self):
            self.lines = [b"begin\r\n"] + [(l + "\r\n").encode() for l in rows] + [b""] * 1000
        def readline(self):
            import time
            time.sleep(0.0005)
            return self.lines.pop(0) if self.lines else b""
        def close(self):
            pass

    tof = t.Tof8x8(port="test", link=Port(), tilt_deg=90)
    try:
        g = tof.grid(newer_than=0.0, timeout=2.0)
        assert g is not None
        assert g.rows[0][0] == pytest.approx(1.056)                         # turned upright: was row 7, col 0
    finally:
        tof.close()
