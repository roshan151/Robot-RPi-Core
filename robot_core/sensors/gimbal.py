"""Pan/tilt head: smooth, limited, and always parked at centre on shutdown.

Smooth and slow
---------------
Hobby servos have no speed input — command a new angle and they get there as
fast as the motor allows. So the smoothing is done here: a 50 Hz loop (the
servo's own frame rate) walks the pulse width along a trapezoidal profile —
accelerate, cruise at `max_speed_dps`, decelerate — so the servo only ever sees
tiny steps. Hardware PWM (GPIO12/13 via `dtoverlay=pwm-2chan`) keeps each step
jitter-free; software PWM would twitch.

Limits live here, with the actuator (PLAN standing rule #4)
-----------------------------------------------------------
Pan is clamped to ±90° from the start position, tilt to ±45° (+ is up),
whatever a caller asks for - and further, to what the servo's pulse window
(servo_min_us..servo_max_us around its centre) can actually reach.

Soft start
----------
A servo has no memory: whatever pulse arrives first, it dashes to at full speed.
So opening the gimbal sends NO pulse: the head stays exactly where it is. The
pulse each axis last held is saved to a tiny file (see `state_path`) whenever
the head comes to rest, and the next open assumes the head is still there. An
axis gets its first pulse only when it is first asked to move, and moves from
that assumed spot at the normal slow speed. (No saved file, or a head moved by
hand while off: that first move can still jump, once, because the servo cannot
report where it is.)

Parking
-------
`close()` slews both axes back to 0 at normal speed, waits for them to arrive,
then keeps HOLDING the centre pulse (`hold_on_close`, the default). Cutting the
PWM instead leaves the servos limp and the head flops away from centre the
moment the program ends; with the pulse still running it stays put, and the
next open starts from where it already is. `close(release=True)` (or
`hold_on_close=False`) stops the PWM when you want the servos to relax.
Nodes call close() from `destroy_node()`, which runs on Ctrl+C and on systemd's
SIGTERM. A hard power cut cannot park anything — the start position is assumed
on the next boot, so after a crash run `python -m robot_core.sensors.gimbal home`
(it uses the centres in robot.yaml) before trusting the angles.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from pathlib import Path
from typing import Optional, Protocol, Tuple

log = logging.getLogger(__name__)

FRAME_HZ = 50.0
PERIOD_US = 1e6 / FRAME_HZ


class PwmOutput(Protocol):
    def set_pulse_us(self, us: float) -> None: ...
    def disable(self) -> None: ...


class HardwarePwm:
    """One Pi hardware-PWM channel (`pip install rpi-hardware-pwm`).

    Pi 4: chip 0, channel 0 = GPIO12, channel 1 = GPIO13, with
    `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4` in config.txt.
    """

    def __init__(self, channel: int, chip: int = 0) -> None:
        from rpi_hardware_pwm import HardwarePWM
        self._pwm = HardwarePWM(pwm_channel=channel, hz=FRAME_HZ, chip=chip)
        self._started = False

    def set_pulse_us(self, us: float) -> None:
        duty = 100.0 * us / PERIOD_US
        if self._started:
            self._pwm.change_duty_cycle(duty)
        else:
            self._pwm.start(duty)
            self._started = True

    def disable(self) -> None:
        if self._started:
            self._pwm.stop()
            self._started = False


class FakePwm:
    """Records pulses. Used by the tests and by `backend="fake"` on a laptop."""

    def __init__(self) -> None:
        self.pulses: list = []
        self.enabled = False

    def set_pulse_us(self, us: float) -> None:
        self.pulses.append(us)
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False


@dataclass
class AxisConfig:
    channel: int
    min_deg: float
    max_deg: float
    center_us: float = 1500.0       # the start position — trim per servo
    us_per_deg: float = 2000.0 / 180.0
    invert: bool = False            # flip if +angle moves the wrong way
    max_speed_dps: float = 30.0     # cruise speed (deg/s): slow and deliberate
    accel_dps2: float = 180.0
    min_us: float = 500.0           # hard electrical limits: no trim or angle can push a pulse
    max_us: float = 2400.0          # outside these (an SG90 just buzzes against its end stop)


PAN_DEFAULT = AxisConfig(channel=1, min_deg=-90.0, max_deg=90.0)     # GPIO13
TILT_DEFAULT = AxisConfig(channel=0, min_deg=-45.0, max_deg=45.0)    # GPIO12


class _Axis:
    def __init__(self, cfg: AxisConfig, out: PwmOutput) -> None:
        self.cfg, self.out = cfg, out
        self.pos = 0.0          # commanded angle right now (deg)
        self.live = False       # has this servo been sent a pulse yet? (not until its first move)
        self.vel = 0.0
        self.target = 0.0
        self.speed = cfg.max_speed_dps
        # The angles this servo can really reach: inside min_deg..max_deg AND
        # inside the min_us..max_us pulse window around this centre. Clamping to
        # this keeps the reported angle honest - a pulse silently cut short would
        # leave the head at +9 deg while angles() claimed +45.
        sign = -1.0 if cfg.invert else 1.0
        a = (cfg.min_us - cfg.center_us) / (sign * cfg.us_per_deg)
        b = (cfg.max_us - cfg.center_us) / (sign * cfg.us_per_deg)
        self.lo = min(0.0, max(cfg.min_deg, min(a, b)))
        self.hi = max(0.0, min(cfg.max_deg, max(a, b)))

    def clamp(self, deg: float) -> float:
        return max(self.lo, min(self.hi, deg))

    def pulse(self) -> float:
        sign = -1.0 if self.cfg.invert else 1.0
        us = self.cfg.center_us + sign * self.pos * self.cfg.us_per_deg
        return max(self.cfg.min_us, min(self.cfg.max_us, us))

    def step(self, dt: float) -> bool:
        """Advance one tick of the trapezoidal profile. True while moving."""
        err = self.target - self.pos
        if abs(err) < 1e-3 and abs(self.vel) < 1e-3:
            self.pos, self.vel = self.target, 0.0
            return False
        a = self.cfg.accel_dps2
        # Fastest speed from which we can still stop at the target.
        v_stop = (2.0 * a * abs(err)) ** 0.5
        v_want = min(self.speed, v_stop) * (1 if err > 0 else -1)
        dv = max(-a * dt, min(a * dt, v_want - self.vel))
        self.vel += dv
        move = self.vel * dt
        if abs(move) >= abs(err):
            self.pos, self.vel = self.target, 0.0
        else:
            self.pos += move
        return True


class Gimbal:
    """Two smooth, clamped servo axes driven from one 50 Hz thread."""

    def __init__(
        self,
        pan: AxisConfig = PAN_DEFAULT,
        tilt: AxisConfig = TILT_DEFAULT,
        backend: str = "hardware",        # "hardware" | "fake"
        chip: int = 0,
        hold_on_close: bool = True,       # keep the centre pulse running after close()
        state_path: Optional[Path] = None,  # where the last held pulses are kept (None: don't)
    ) -> None:
        make = (lambda c: HardwarePwm(c.channel, chip)) if backend == "hardware" else (lambda c: FakePwm())
        self.pan = _Axis(pan, make(pan))
        self.tilt = _Axis(tilt, make(tilt))
        self.hold_on_close = hold_on_close
        self._released = False
        self._state_path = Path(state_path) if state_path else None
        self._start_from_saved()
        for name, ax in (("pan", self.pan), ("tilt", self.tilt)):
            if ax.lo > ax.cfg.min_deg + 0.5 or ax.hi < ax.cfg.max_deg - 0.5:
                log.warning(
                    "gimbal: %s limited to %+.0f..%+.0f deg (wanted %+.0f..%+.0f) by the %.0f-%.0f us pulse "
                    "window around its %.0f us centre; re-fit the horn nearer the middle of its travel, "
                    "or widen servo_min_us / servo_max_us if the servo really goes further",
                    name, ax.lo, ax.hi, ax.cfg.min_deg, ax.cfg.max_deg,
                    ax.cfg.min_us, ax.cfg.max_us, ax.cfg.center_us)
        self._lock = threading.Lock()
        self._moving = False
        self._last_motion = time.monotonic()
        self._arrived = threading.Condition(self._lock)
        self._stop = threading.Event()
        # No pulse is sent here: opening the gimbal must not move the head. Each
        # servo gets its first pulse (at its assumed position) when it is first moved.
        self._thread = threading.Thread(target=self._loop, name="gimbal", daemon=True)
        self._thread.start()

    # ---------------------------------------------------------- soft start

    def _start_from_saved(self) -> None:
        """Assume each axis is at the pulse it was last holding, and stay there."""
        try:
            saved = json.loads(self._state_path.read_text()) if self._state_path else {}
        except (OSError, ValueError):
            return
        for name, ax in (("pan", self.pan), ("tilt", self.tilt)):
            us = saved.get(name + "_us")
            if not isinstance(us, (int, float)) or not ax.cfg.min_us <= us <= ax.cfg.max_us:
                continue
            sign = -1.0 if ax.cfg.invert else 1.0
            ax.pos = (float(us) - ax.cfg.center_us) / (sign * ax.cfg.us_per_deg)
            ax.target = ax.pos

    def _save_state(self) -> None:
        if not self._state_path:
            return
        try:
            with self._lock:
                data = {"pan_us": round(self.pan.pulse(), 1), "tilt_us": round(self.tilt.pulse(), 1)}
            self._state_path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._state_path.with_suffix(".tmp")
            tmp.write_text(json.dumps(data))
            os.replace(tmp, self._state_path)
        except OSError:
            log.exception("gimbal: could not save the head position")

    # ----------------------------------------------------------------- API

    def move_to(self, pan: Optional[float] = None, tilt: Optional[float] = None,
                speed_dps: Optional[float] = None, wait: bool = True,
                timeout: float = 10.0) -> tuple:
        """Slew to (pan, tilt) degrees, clamped to the limits. None = keep.
        Returns the clamped target actually used."""
        with self._lock:
            for ax, want in ((self.pan, pan), (self.tilt, tilt)):
                if want is not None:
                    if not ax.live:                    # first move: start the pulse where we think it is
                        ax.out.set_pulse_us(ax.pulse())
                        ax.live = True
                    ax.target = ax.clamp(want)
                    ax.speed = min(speed_dps or ax.cfg.max_speed_dps, ax.cfg.max_speed_dps * 2)
            self._moving = True
            target = (self.pan.target, self.tilt.target)
        if wait:
            self.wait(timeout)
        return target

    def wait(self, timeout: float = 10.0) -> bool:
        with self._arrived:
            return self._arrived.wait_for(lambda: not self._moving, timeout)

    def home(self, wait: bool = True) -> None:
        self.move_to(0.0, 0.0, wait=wait)

    def angles(self) -> tuple:
        """(pan_deg, tilt_deg) as commanded right now."""
        with self._lock:
            return self.pan.pos, self.tilt.pos

    def settled(self, settle_s: float = 0.15) -> bool:
        """Stationary for at least `settle_s` — the servo has caught up with
        the command and a frame or a range reading means what it says."""
        with self._lock:
            return not self._moving and time.monotonic() - self._last_motion >= settle_s

    def tilt_limits(self) -> Tuple[float, float]:
        return self.tilt.lo, self.tilt.hi

    def wait_settled(self, settle_s: float = 0.15, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.settled(settle_s):
                return True
            time.sleep(0.02)
        return False

    def close(self, release: Optional[bool] = None) -> None:
        """Park at the start position, then stop driving it.

        By default (`hold_on_close`) the centre pulse keeps running after this
        returns - the kernel PWM carries on without us - so the servos keep
        holding the head at centre instead of going limp and letting it flop.
        `release=True` stops the PWM instead (servos relax, no hum). It can also
        be called later on an already-closed gimbal, to let go.
        """
        release = (not self.hold_on_close) if release is None else release
        if not self._stop.is_set():
            try:
                live = [ax.live for ax in (self.pan, self.tilt)]
                if any(live):             # never touched: nothing to park, and no pulse to start now
                    self.move_to(0.0 if live[0] else None, 0.0 if live[1] else None, wait=True)
                    time.sleep(0.3)       # let the horn physically arrive
            finally:
                self._stop.set()
                self._thread.join(timeout=1.0)
                self._save_state()
        if release and not self._released:
            self._released = True
            for ax in (self.pan, self.tilt):
                try:
                    ax.out.disable()
                except Exception:                          # noqa: BLE001
                    log.exception("gimbal: disabling PWM failed")

    # -------------------------------------------------------------- engine

    def _loop(self) -> None:
        dt = 1.0 / FRAME_HZ
        nxt = time.monotonic()
        while not self._stop.is_set():
            arrived = False
            with self._lock:
                moving = False
                for ax in (self.pan, self.tilt):
                    if ax.step(dt):
                        moving = True
                        ax.out.set_pulse_us(ax.pulse())
                if moving:
                    self._last_motion = time.monotonic()
                elif self._moving:
                    self._moving = False
                    arrived = True
                    self._arrived.notify_all()
            if arrived:
                self._save_state()
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))


def from_params(p: dict, tilt_min: float = TILT_DEFAULT.min_deg, tilt_max: float = TILT_DEFAULT.max_deg,
                backend: str = "hardware") -> Gimbal:
    """A Gimbal from the `/explorer` parameters in robot.yaml (channels, centre
    trims, us-per-degree, inversion, speed). Pan is always ±90°; callers may narrow the tilt
    range — the face tasks pass tilt_min=0 so the head never looks below level."""
    speed = float(p.get("servo_speed_dps", 30.0))
    lo_us, hi_us = float(p.get("servo_min_us", 500.0)), float(p.get("servo_max_us", 2400.0))
    return Gimbal(
        pan=AxisConfig(channel=int(p.get("pan_channel", 1)), min_deg=PAN_DEFAULT.min_deg, max_deg=PAN_DEFAULT.max_deg,
                       center_us=float(p.get("pan_center_us", 1500.0)),
                       us_per_deg=float(p.get("pan_us_per_deg", PAN_DEFAULT.us_per_deg)),
                       invert=bool(p.get("pan_invert", False)), max_speed_dps=speed,
                       min_us=lo_us, max_us=hi_us),
        tilt=AxisConfig(channel=int(p.get("tilt_channel", 0)), min_deg=tilt_min, max_deg=tilt_max,
                        center_us=float(p.get("tilt_center_us", 1500.0)),
                        us_per_deg=float(p.get("tilt_us_per_deg", TILT_DEFAULT.us_per_deg)),
                        invert=bool(p.get("tilt_invert", False)), max_speed_dps=speed,
                        min_us=lo_us, max_us=hi_us),
        backend=backend, chip=int(p.get("pwm_chip", 0)),
        hold_on_close=bool(p.get("hold_on_close", True)),
        state_path=(Path(p.get("head_state_file") or Path.home() / ".cache" / "robot-head.json")
                    if backend == "hardware" else None))


ROBOT_YAML = Path(__file__).resolve().parents[2] / "ros2_ws/src/robot_bringup/config/robot.yaml"


def params_from_robot_yaml() -> dict:
    """The /explorer parameters from the repo's robot.yaml - the same channels,
    centres and inversion every node uses ({} with a warning if unreadable)."""
    try:
        import yaml
        return yaml.safe_load(ROBOT_YAML.read_text())["/explorer"]["ros__parameters"]
    except Exception as exc:                                        # noqa: BLE001
        log.warning("gimbal: cannot read %s (%s) - using default centres of 1500 us", ROBOT_YAML, exc)
        return {}


def main() -> None:
    '''`python -m robot_core.sensors.gimbal home | release | move PAN [TILT]`

    home     park at centre and keep holding it (default) - after a crash or power
             cut, before anything trusts the angles
    release  park, then stop the PWM so the servos relax
    move     go to PAN [TILT] degrees first, then park
    Channels and centres come from robot.yaml, like every node's.'''
    import sys
    logging.basicConfig(level=logging.INFO)
    cmd = sys.argv[1] if len(sys.argv) > 1 else "home"
    g = from_params(params_from_robot_yaml())
    try:
        if cmd == "move" and len(sys.argv) > 2:
            g.move_to(float(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else None)
            time.sleep(1.0)
    finally:
        g.close(release=True if cmd == "release" else None)


if __name__ == "__main__":
    main()
