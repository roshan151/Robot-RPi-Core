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
So the pulse each axis was last holding is saved to a tiny file (see
`state_path`) whenever the head comes to rest. The next open starts its pulses
from THAT value - where the head physically still is - and then walks to centre
at the normal slow speed, instead of snapping there. If nobody moved the head
meanwhile there is no jump at all. (First ever run, or a head moved by hand
while off: the first pulse is the centre, as before.)

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

Servo power
-----------
The servos have their own switch, and a servo with no power ignores the pulses it is
sent. Blind to that, the trajectory above keeps "moving" an unpowered head, and the
moment the switch goes on the servo is handed a pulse far from where it really is and
dashes there (it has torn the camera ribbon). So the supply is sensed: `servo_power_gpio`
in robot.yaml names a pin fed from the servo +5 V through a divider (high = powered).
While it reads low the head is FROZEN - nothing is stepped, queued moves are dropped,
new ones are refused - so the pulse the line holds is still the one the servo last
followed, switching on is a non-event, and motion resumes from there. Unset (0): the
supply is assumed to be on.

Fail-safe
---------
Anything wrong with the head's output STOPS it instead of letting it snap: both PWM channels are
disabled (the 10 k pull-downs on the signal lines then hold them low, so the servos go limp rather
than being driven), every move is refused, and the reason goes to the OLED (G03-G08, status.CODES).
A fault is latched in `<state_path>.fault`, so restarting the service does not quietly try again;
fix the cause, then `python -m robot_core.sensors.gimbal reset`. What trips it:
  pwm    the kernel channel no longer holds what we wrote (checked right after the first pulse and
         every 0.2 s): another process or driver reprogrammed it, or it was unexported
  pulse  one 20 ms frame moved a pulse further than the speed limit allows, or the angle is not a number
  loop   the 50 Hz thread hit an exception - it must not die with a pulse still running
  busy   another process or Gimbal already drives the PWM channels (opens dead; not latched)
A Gimbal that is latched or busy still constructs, but dead: it never touches the hardware.
"""

from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from pathlib import Path
from typing import Callable, Optional, Protocol, Tuple

from robot_core import status

log = logging.getLogger(__name__)

FRAME_HZ = 50.0
CHECK_EVERY_S = 0.2                 # how often the kernel PWM channel is compared with what we wrote
LOCK_PATH = "/dev/shm/robot-gimbal.lock"    # one owner of the PWM channels at a time (flock)
FAULT_CODES = {"pwm": "G04", "pulse": "G05", "loop": "G06", "latched": "G07", "busy": "G08"}
SAVE_EVERY_S = 0.5                  # while moving, refresh the saved head position this often
PERIOD_US = 1e6 / FRAME_HZ


class PwmOutput(Protocol):
    def set_pulse_us(self, us: float) -> None: ...
    def disable(self) -> None: ...
    def verify(self) -> Optional[str]: ...      # None while the output still is what we wrote


class HardwarePwm:
    """One Pi hardware-PWM channel (`pip install rpi-hardware-pwm`).

    Pi 4: chip 0, channel 0 = GPIO12, channel 1 = GPIO13, with
    `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4` in config.txt.
    """

    def __init__(self, channel: int, chip: int = 0) -> None:
        from rpi_hardware_pwm import HardwarePWM
        self._pwm = HardwarePWM(pwm_channel=channel, hz=FRAME_HZ, chip=chip)
        self._started = False
        self._dir, self._us = Path(f"/sys/class/pwm/pwmchip{chip}/pwm{channel}"), 0.0

    def set_pulse_us(self, us: float) -> None:
        self._us = us
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

    def verify(self) -> Optional[str]:
        """None while the kernel channel still holds exactly what we last wrote, else what differs."""
        if not self._started:
            return None
        try:
            en, per, duty = (int((self._dir / f).read_text()) for f in ("enable", "period", "duty_cycle"))
        except (OSError, ValueError) as e:
            return f"unreadable ({e})"
        if en != 1 or abs(per - PERIOD_US * 1000) > 1000 or abs(duty - self._us * 1000) > 2000:
            return f"enable={en} period={per} duty={duty} ns, but wrote {self._us:.0f} us"
        return None


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

    def verify(self) -> Optional[str]:
        return None


class GpioPowerSense:
    """Is the servo supply on? Reads BCM pin `pin`: high = powered (wire it from the servo +5 V
    through a 10k/20k divider). Unwired or broken reads low = "off", the safe answer."""

    def __init__(self, pin: int) -> None:
        import RPi.GPIO as GPIO                          # rpi-lgpio provides this on current Pi OS
        self._gpio, self.pin = GPIO, pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def __call__(self) -> bool:
        return bool(self._gpio.input(self.pin))

    def close(self) -> None:
        self._gpio.cleanup(self.pin)


def _claim_pwm() -> Optional[int]:
    '''Lock the PWM channels for this Gimbal: an fd; -1 if no lock file can be made (carry on unlocked rather
    than lose the head); None if another Gimbal, in this or any other process, holds it. Two writers on one
    channel fight over the pulse. The lock dies with its process.'''
    import fcntl
    try:
        try:
            fd = os.open(LOCK_PATH, os.O_RDONLY)            # O_CREAT on another user's file is refused (fs.protected_regular)
        except FileNotFoundError:
            fd = os.open(LOCK_PATH, os.O_RDONLY | os.O_CREAT, 0o666)
    except OSError:
        return -1
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        return None
    except OSError:
        os.close(fd)
        return -1
    return fd


class _Fault(Exception):
    """The head's output is not what we think it is. args = (kind, why); kind picks the OLED code."""


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
        self.vel = 0.0
        self.target = 0.0
        self.speed = cfg.max_speed_dps
        self.last_us = 0.0      # the pulse last written: one frame may not jump far from it
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
        power_sense: Optional[Callable[[], bool]] = None,   # () -> servo supply present? None: assume yes
    ) -> None:
        self.fault = self.fault_code = ""        # set = tripped: PWM off, moves refused (see "Fail-safe")
        self._fault_path = Path(state_path).with_suffix(".fault") if state_path else None
        self._lockfd: Optional[int] = -1
        if self._fault_path and self._fault_path.exists():
            was = self._fault_path.read_text().strip() if os.access(self._fault_path, os.R_OK) else "?"
            self._mark("latched", f"run gimbal reset (was {was})")
        elif backend == "hardware":
            self._lockfd = _claim_pwm()
            if self._lockfd is None:
                self._mark("busy", "another process drives the head")
        if self.fault:
            backend = "fake"                     # opens dead: nothing below touches the hardware
        make = (lambda c: HardwarePwm(c.channel, chip)) if backend == "hardware" else (lambda c: FakePwm())
        self.pan = _Axis(pan, make(pan))
        self.tilt = _Axis(tilt, make(tilt))
        self.hold_on_close = hold_on_close
        self._released = False
        self._state_path = Path(state_path) if state_path else None
        self._power_sense = power_sense
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
        self._moving = getattr(self, "_starting", False)
        self._last_motion = time.monotonic()
        self._arrived = threading.Condition(self._lock)
        self._stop = threading.Event()
        # The first pulse is where the head was last left (see "Soft start"), so
        # the servo has nothing to dash to; the loop then walks it to centre.
        for ax in (self.pan, self.tilt):
            ax.out.set_pulse_us(ax.pulse())
            ax.last_us = ax.pulse()
        for ax in (self.pan, self.tilt):         # the first pulse must really be on the line before anything moves
            bad = ax.out.verify()
            if bad and not self.fault:
                self._trip("pwm", f"ch{ax.cfg.channel} {bad}")
        self._thread = threading.Thread(target=self._loop, name="gimbal", daemon=True)
        self._thread.start()

    # ---------------------------------------------------------- soft start

    def _start_from_saved(self) -> None:
        """Begin each axis at the pulse it was last holding, with centre as the target."""
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
            ax.target = 0.0                       # then walk home, slowly
        self._starting = abs(self.pan.pos) > 1e-3 or abs(self.tilt.pos) > 1e-3

    def _save_state(self) -> None:
        # Supply off (or faulted): the servos are not following the pulses, so the
        # commanded position is not where the horn is. Saving it would make the next
        # start's first pulse dash there.
        if not self._state_path or not self.powered():
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

    def _forget_state(self) -> None:
        """No saved position: the next start's first pulse is the centre."""
        if self._state_path:
            try:
                self._state_path.unlink(missing_ok=True)
            except OSError:
                log.exception("gimbal: could not delete the saved head position")

    # ----------------------------------------------------------------- API

    def powered(self) -> bool:
        """May the head move? False once it has faulted, or while the servo supply is off (always True with
        no `power_sense`; a failing sensor reads as off)."""
        if self.fault:
            return False
        if self._power_sense is None:
            return True
        try:
            return bool(self._power_sense())
        except Exception:                                   # noqa: BLE001
            return False

    def refuse(self) -> str:
        """The head will not move: put the reason on the OLED and return it in words ("" if it will)."""
        if self.powered():
            return ""
        if self.fault:
            status.error(self.fault_code, self.fault.split(": ", 1)[-1])
            return f"head switched off after a fault ({self.fault}); it needs a reset"
        status.error("G03", "servo switch off?")
        return "head servos have no power - switch the servo supply on"

    def move_to(self, pan: Optional[float] = None, tilt: Optional[float] = None,
                speed_dps: Optional[float] = None, wait: bool = True,
                timeout: float = 10.0) -> tuple:
        """Slew to (pan, tilt) degrees, clamped to the limits. None = keep.
        Returns the clamped target actually used (the current angles, if the head is stopped or the
        request is not a usable number)."""
        why = self.refuse()
        if why:
            log.warning("gimbal: %s - move ignored", why)
            return self.angles()
        if any(v is not None and not math.isfinite(v) for v in (pan, tilt, speed_dps)) or (speed_dps or 0) < 0:
            log.warning("gimbal: refused pan=%s tilt=%s speed=%s", pan, tilt, speed_dps)    # clamp(nan) is the top limit
            return self.angles()
        with self._lock:
            for ax, want in ((self.pan, pan), (self.tilt, tilt)):
                if want is not None:
                    ax.target = ax.clamp(want)
                    ax.speed = min(speed_dps or ax.cfg.max_speed_dps, ax.cfg.max_speed_dps)    # servo_speed_dps is a hard ceiling
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

    def pan_limits(self) -> Tuple[float, float]:
        return self.pan.lo, self.pan.hi

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
                self.home(wait=True)
                time.sleep(0.3)           # let the horn physically arrive
            finally:
                self._stop.set()
                self._thread.join(timeout=1.0)
                if self.powered():
                    self._save_state()
                else:
                    self._forget_state()  # could not park: where the horn is is unknown
                getattr(self._power_sense, "close", lambda: None)()
                if self._lockfd is not None and self._lockfd >= 0:
                    os.close(self._lockfd)       # let the next Gimbal in
                self._lockfd = -1
        if release and not self._released:
            self._released = True
            for ax in (self.pan, self.tilt):
                try:
                    ax.out.disable()
                except Exception:                          # noqa: BLE001
                    log.exception("gimbal: disabling PWM failed")

    # ----------------------------------------------------------- fail-safe

    def _mark(self, kind: str, why: str) -> None:
        self.fault, self.fault_code = f"{kind}: {why}", FAULT_CODES[kind]
        log.warning("gimbal: FAULT %s - head stopped", self.fault)
        status.error(self.fault_code, why)

    def _trip(self, kind: str, why: str) -> None:
        """Stop driving the head: PWM off on both axes, moves refused, latched on disk (see "Fail-safe")."""
        if self.fault:
            return
        self._mark(kind, why)
        for ax in (self.pan, self.tilt):
            try:
                ax.out.disable()
            except Exception:                               # noqa: BLE001
                log.warning("gimbal: disabling PWM failed", exc_info=True)
        with self._lock:
            self._moving = False
            self._arrived.notify_all()                      # anyone in wait() returns at once
        if self._fault_path:
            try:
                self._fault_path.parent.mkdir(parents=True, exist_ok=True)
                self._fault_path.write_text(self.fault)
            except OSError:
                log.warning("gimbal: could not latch the fault on disk", exc_info=True)

    def _write(self, ax: _Axis) -> None:
        us = ax.pulse()
        step = 2 * ax.cfg.max_speed_dps * abs(ax.cfg.us_per_deg) / FRAME_HZ + 5.0       # full speed is half this
        if not math.isfinite(ax.pos) or abs(us - ax.last_us) > step:
            raise _Fault("pulse", f"ch{ax.cfg.channel} {ax.last_us:.0f} to {us:.0f} us in one frame")
        ax.out.set_pulse_us(us)
        ax.last_us = us

    # -------------------------------------------------------------- engine

    def _loop(self) -> None:
        try:
            self._run()
        except _Fault as f:
            self._trip(*f.args)
        except Exception as e:                              # noqa: BLE001 - a dead loop would leave its last pulse running
            self._trip("loop", f"{type(e).__name__}: {e}")

    def _run(self) -> None:
        dt = 1.0 / FRAME_HZ
        nxt = last_save = last_check = time.monotonic()
        rail = True
        while not self._stop.is_set() and not self.fault:
            arrived = False
            powered = self.powered()
            if powered != rail:
                rail = powered
                log.warning("gimbal: servo supply %s", "back on - resuming from the held pulse" if rail
                            else "OFF - head frozen, moves refused")
                if not rail:
                    status.error("G03", "servo switch off?")
            with self._lock:
                moving = False
                for ax in (self.pan, self.tilt):
                    if not powered:
                        ax.target, ax.vel = ax.pos, 0.0     # nothing is following the pulses: forget the order
                    elif ax.step(dt):
                        moving = True
                        self._write(ax)
                if moving:
                    self._last_motion = time.monotonic()
                elif self._moving:
                    self._moving = False
                    arrived = True
                    self._arrived.notify_all()
            if arrived:
                self._save_state()
            elif moving and time.monotonic() - last_save >= SAVE_EVERY_S:
                last_save = time.monotonic()
                self._save_state()                          # a reset mid-move must not leave a stale position
            if time.monotonic() - last_check >= CHECK_EVERY_S:
                last_check = time.monotonic()
                for ax in (self.pan, self.tilt):
                    bad = ax.out.verify()
                    if bad:
                        raise _Fault("pwm", f"ch{ax.cfg.channel} {bad}")
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))


def _state_file(p: dict) -> Path:
    return Path(p.get("head_state_file") or Path.home() / ".cache" / "robot-head.json")


def from_params(p: dict, tilt_min: float = TILT_DEFAULT.min_deg, tilt_max: float = TILT_DEFAULT.max_deg,
                backend: str = "hardware") -> Gimbal:
    """A Gimbal from the `/explorer` parameters in robot.yaml (channels, centre
    trims, us-per-degree, inversion, speed). Pan is always ±90°; callers may narrow the tilt
    range — the face tasks pass tilt_min=0 so the head never looks below level."""
    speed = float(p.get("servo_speed_dps", 30.0))
    lo_us, hi_us = float(p.get("servo_min_us", 500.0)), float(p.get("servo_max_us", 2400.0))
    pin = int(p.get("servo_power_gpio") or 0)
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
        state_path=_state_file(p) if backend == "hardware" else None,
        power_sense=GpioPowerSense(pin) if pin and backend == "hardware" else None)


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
    reset    clear a latched fault (see "Fail-safe") once its cause is fixed; touches no hardware
    Channels and centres come from robot.yaml, like every node's.'''
    import sys
    logging.basicConfig(level=logging.INFO)
    cmd = sys.argv[1] if len(sys.argv) > 1 else "home"
    if cmd == "reset":
        latch = _state_file(params_from_robot_yaml()).with_suffix(".fault")
        print(f"cleared: {latch.read_text().strip()}" if latch.exists() else "no fault was latched")
        latch.unlink(missing_ok=True)
        return
    g = from_params(params_from_robot_yaml())
    try:
        if cmd == "move" and len(sys.argv) > 2:
            g.move_to(float(sys.argv[2]), float(sys.argv[3]) if len(sys.argv) > 3 else None)
            time.sleep(1.0)
    finally:
        g.close(release=True if cmd == "release" else None)


if __name__ == "__main__":
    main()
