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
               backend="fake")
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
