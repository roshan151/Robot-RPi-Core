"""Fermion 10 DOF IMU (DFRobot SEN0697: BMI323 + BMM350 + BMP581) on the Pi's I2C:
pitch, roll and yaw.

One of the board's three chips is used:

    BMI323 accelerometer + gyroscope   0x69 (0x68 with its pad closed)
        which way is down -> pitch and roll at rest
        how fast it turns -> yaw, and pitch/roll while moving

The magnetometer (BMM350, 0x15) is not read: it sits next to two motors and their
magnets. The barometer (BMP581, 0x47) is not read either.

The BMI323 has 16-bit registers: a write is the register then the value low byte
first; every I2C read starts with two dummy bytes, then the data, low byte first.
Accelerometer x, y, z then gyro x, y, z are six words from register 0x03.

Wiring is in the README ("IMU and ToF sensors").

Angles, in degrees, for the robot (not the board; `axes` maps one to the other):

    pitch  > 0  nose up
    roll   > 0  right side down
    yaw    > 0  turned left (counter-clockwise), unwrapped: two turns left is 720

Pitch and roll come from a complementary filter: the gyro for quick changes, the
accelerometer to stop them drifting. Yaw is the gyro alone, so it drifts; the
gyro's zero is measured whenever the robot stands still (`set_moving(False)`),
which keeps that to about a degree a minute.

Same shape as tfluna.py: a background thread, `state()` for the latest value, and
`source=` to feed samples from a test instead of the bus.
"""

from __future__ import annotations

import logging
import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from typing import Callable, Optional, Sequence, Tuple

log = logging.getLogger(__name__)

GYRO_ADDR = 0x69                    # the BMI323, pad open (as shipped)
REG_CHIP_ID, REG_ERR, REG_DATA, REG_ACC_CONF, REG_GYR_CONF, REG_CMD = 0x00, 0x01, 0x03, 0x20, 0x21, 0x7E
SOFT_RESET = 0xDEAF
CHIP_IDS = (0x43, 0x41)             # the datasheet's id, and the one some parts report
# mode normal (4) << 12 | range << 4 | output rate 0x8 = 100 samples a second
ACC_CONF = 0x4000 | 3 << 4 | 0x8    # +-16 g, so a bump does not clip
GYR_CONF = 0x4000 | 2 << 4 | 0x8    # +-500 deg/s: a tank turns far slower, and it keeps the resolution
ACCEL_G_PER_LSB = 16.0 / 32768
GYRO_LSB_PER_DPS = 32768 / 500.0

Sample = Tuple[float, float, float, float, float, float]   # ax ay az (g), gx gy gz (deg/s)


@dataclass(frozen=True)
class ImuState:
    t: float                # host monotonic seconds
    pitch_deg: float
    roll_deg: float
    yaw_deg: float          # unwrapped, counter-clockwise positive
    yaw_rate_dps: float
    jolt_g: float           # biggest departure from 1 g in the last half second
    calibrated: bool        # the gyro's zero has been measured at least once


class Chips:
    """The BMI323 on the bus. `read()` -> one Sample in the board's own axes (printed on it)."""

    def __init__(self, bus: int = 1, addr: int = GYRO_ADDR) -> None:
        from smbus2 import SMBus
        self._bus, self._addr = SMBus(bus), addr
        self._write(REG_CMD, SOFT_RESET)
        time.sleep(0.05)
        chip = self._words(REG_CHIP_ID, 1)[0] & 0xFF
        if chip not in CHIP_IDS:
            raise RuntimeError(f"0x{addr:02x} on I2C bus {bus} answers but is not a BMI323 "
                               f"(chip id 0x{chip:02x}): check imu_gyro_addr and the README")
        self._write(REG_ACC_CONF, ACC_CONF)
        self._write(REG_GYR_CONF, GYR_CONF)
        time.sleep(0.05)
        err = self._words(REG_ERR, 1)[0]
        if err & 0x61:                                  # fatal, accel or gyro configuration error
            raise RuntimeError(f"BMI323 rejected its configuration (error register 0x{err:04x})")

    def _write(self, reg: int, value: int) -> None:
        self._bus.write_i2c_block_data(self._addr, reg, [value & 0xFF, value >> 8 & 0xFF])

    def _words(self, reg: int, n: int, signed: bool = False) -> Tuple[int, ...]:
        raw = bytes(self._bus.read_i2c_block_data(self._addr, reg, 2 + 2 * n))[2:]   # two dummy bytes first
        return struct.unpack(f"<{n}{'h' if signed else 'H'}", raw)

    def read(self) -> Sample:
        ax, ay, az, gx, gy, gz = self._words(REG_DATA, 6, signed=True)
        return (ax * ACCEL_G_PER_LSB, ay * ACCEL_G_PER_LSB, az * ACCEL_G_PER_LSB,
                gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS)

    def close(self) -> None:
        self._bus.close()


def axis_map(axes: Sequence[str]):
    """`axes` names the board axis that points robot-forward, robot-left and
    robot-up, each "x", "y" or "z" with an optional "-": ("y", "-x", "z")."""
    picks = []
    for a in axes:
        a = a.strip().lower()
        sign = -1.0 if a.startswith("-") else 1.0
        picks.append(("xyz".index(a.lstrip("+-")), sign))
    if sorted(i for i, _ in picks) != [0, 1, 2]:
        raise ValueError(f"imu axes must use x, y and z once each, got {list(axes)}")
    return lambda v: tuple(v[i] * s for i, s in picks)


class Fusion:
    """Samples in (robot axes) -> angles out. No clock, no thread: `update(sample, dt)`."""

    def __init__(self, accel_weight: float = 0.02, still_spread_dps: float = 1.0,
                 window: int = 100, jolt_window: int = 50,
                 auto_tare_level: bool = False, max_tare_deg: float = 12.0) -> None:
        self.k = accel_weight
        self.still_spread = still_spread_dps
        self.pitch = self.roll = self.yaw = self.yaw_rate = 0.0      # degrees
        self.bias = [0.0, 0.0, 0.0]
        self.calibrated = False
        self.auto_tare_level = bool(auto_tare_level)
        self.max_tare_deg = float(max_tare_deg)
        self.pitch_tare = self.roll_tare = 0.0
        self.level_tared = False
        self.moving = False                    # told from outside: no zeroing during a move
        self._gyro: deque = deque(maxlen=window)
        self._accel: deque = deque(maxlen=window)
        self._pr: deque = deque(maxlen=window)
        self._jolt: deque = deque(maxlen=jolt_window)
        self._started = False

    def update(self, accel: Sequence[float], gyro: Sequence[float], dt: float) -> None:
        ax, ay, az = accel
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        self._jolt.append(abs(norm - 1.0))
        p_acc = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        r_acc = math.degrees(math.atan2(ay, az))
        self._zero(gyro, norm, p_acc, r_acc)
        gx, gy, gz = (g - b for g, b in zip(gyro, self.bias))
        # Right-hand turns about forward / left / up: roll right-side-down, nose DOWN, yaw left.
        self.roll += gx * dt
        self.pitch -= gy * dt
        self.yaw += gz * dt
        self.yaw_rate = gz
        if abs(norm - 1.0) < 0.15:             # only gravity on it: let it pull the angles back
            k = self.k if self._started else 1.0
            self.pitch += k * (p_acc - self.pitch)
            self.roll += k * (r_acc - self.roll)
            self._started = True

    def _zero(self, gyro: Sequence[float], norm: float, p_acc: float, r_acc: float) -> None:
        """A full window with the gyro steady and only gravity on the accelerometer
        is the robot standing still: its mean is the gyro's zero. On the first
        stationary window, optionally auto-tares pitch & roll (within max_tare_deg)
        so mounting/chassis drift never turns flat floor into false obstacles."""
        if self.moving:
            self._gyro.clear()
            self._accel.clear()
            self._pr.clear()
            return
        self._gyro.append(tuple(gyro))
        self._accel.append(norm)
        self._pr.append((p_acc, r_acc))
        if len(self._gyro) < self._gyro.maxlen:
            return
        cols = list(zip(*self._gyro))
        steady = all(max(c) - min(c) < self.still_spread for c in cols) \
            and max(self._accel) - min(self._accel) < 0.05
        mean = [sum(c) / len(c) for c in cols]
        # Once zeroed, a mean far from the last zero is a slow steady turn, not drift.
        if steady and (not self.calibrated or all(abs(m - b) < 3.0 for m, b in zip(mean, self.bias))):
            self.bias, self.calibrated = mean, True
            if self.auto_tare_level and not self.level_tared and abs(sum(self._accel) / len(self._accel) - 1.0) < 0.15:
                p0 = sum(p for p, _ in self._pr) / len(self._pr)
                r0 = sum(r for _, r in self._pr) / len(self._pr)
                if abs(p0) <= self.max_tare_deg and abs(r0) <= self.max_tare_deg:
                    self.pitch_tare, self.roll_tare, self.level_tared = p0, r0, True
                    log.info("imu: auto-tared level at pitch=%.2f deg, roll=%.2f deg", p0, r0)
                else:
                    log.warning("imu: startup tilt (pitch=%.1f, roll=%.1f deg) exceeds %.1f deg; skipping auto-tare",
                                p0, r0, self.max_tare_deg)
        self._gyro.clear()
        self._accel.clear()
        self._pr.clear()

    def state(self, t: float) -> ImuState:
        return ImuState(t, self.pitch - self.pitch_tare, self.roll - self.roll_tare,
                        self.yaw, self.yaw_rate, max(self._jolt, default=0.0), self.calibrated)


class Imu:
    def __init__(self, bus: int = 1, axes: Sequence[str] = ("x", "y", "z"), hz: float = 100.0,
                 gyro_addr: int = GYRO_ADDR,
                 source: Optional[Callable[[], Sample]] = None, pitch_offset_deg: float = 0.0,
                 auto_tare_level: bool = True, max_tare_deg: float = 12.0) -> None:
        """`pitch_offset_deg`: fallback flat-floor pitch offset before startup auto-tare completes.
        `auto_tare_level`: when True, measures flat-floor pitch and roll during the first stationary
        calibration window (within `max_tare_deg`) so mounting/chassis drift never turns flat floor
        into false obstacles."""
        self._chips, self._pitch0 = None, float(pitch_offset_deg)
        if source is None:
            self._chips = Chips(bus, gyro_addr)
            source = self._chips.read
        self._read, self._map, self._period = source, axis_map(axes), 1.0 / hz
        self._fusion = Fusion(window=max(10, int(hz)), jolt_window=max(5, int(hz / 2)),
                              auto_tare_level=auto_tare_level, max_tare_deg=max_tare_deg)
        self._lock = threading.Lock()
        self._state: Optional[ImuState] = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="imu", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        last = time.monotonic()
        failing = False
        while not self._stop.wait(self._period):
            try:
                s = self._read()
                failing = False
            except Exception as e:                          # noqa: BLE001
                if not failing:
                    log.warning("imu: read failed (%s: %s); retrying", type(e).__name__, e)
                failing = True
                last = time.monotonic()
                time.sleep(0.5)
                continue
            now = time.monotonic()
            dt, last = min(now - last, 0.1), now            # a long gap is not integrated as one step
            with self._lock:
                self._fusion.update(self._map(s[:3]), self._map(s[3:]), dt)
                self._state = self._fusion.state(now)

    def state(self) -> Optional[ImuState]:
        """The latest angles, or None before the first sample."""
        with self._lock:
            st = self._state
            tared = self._fusion.level_tared
        return st if st is None or tared or not self._pitch0 else replace(st, pitch_deg=st.pitch_deg - self._pitch0)

    def set_moving(self, moving: bool) -> None:
        """The gyro's zero is only measured while this is False (the drivetrain says so)."""
        with self._lock:
            self._fusion.moving = bool(moving)

    def wait_calibrated(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            st = self.state()
            if st and st.calibrated:
                return True
            time.sleep(0.05)
        return False

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self._chips is not None:
            self._chips.close()


class FakeImu:
    """For tests: set the fields, read them back."""

    def __init__(self, pitch_deg: float = 0.0, roll_deg: float = 0.0, yaw_deg: float = 0.0) -> None:
        self.pitch_deg, self.roll_deg, self.yaw_deg = pitch_deg, roll_deg, yaw_deg
        self.yaw_rate_dps = self.jolt_g = 0.0
        self.moving = False

    def state(self) -> ImuState:
        return ImuState(time.monotonic(), self.pitch_deg, self.roll_deg, self.yaw_deg,
                        self.yaw_rate_dps, self.jolt_g, True)

    def set_moving(self, moving: bool) -> None:
        self.moving = bool(moving)

    def wait_calibrated(self, timeout: float = 5.0) -> bool:
        return True

    def close(self) -> None:
        pass
