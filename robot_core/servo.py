"""Pan/tilt servos on the Pi's hardware PWM. PLAN Phase 4, VISION-DESIGN 4.5.

    tilt  GPIO12  PWM channel 0
    pan   GPIO13  PWM channel 1      both need  dtoverlay=pwm-2chan  in config.txt

Hardware PWM only: software PWM jitters under CPU load and the head twitches.
Power both servos from the buck converter's 5 V rail, NOT the Pi's 5 V pin,
with grounds shared — a loaded pan/tilt head browns the Pi out.

Angles: pan + = right, tilt + = up, 0/0 = straight ahead and level. Limits are
clamped here, never trusted from the caller. `trim` and `invert` absorb how the
horn happened to go on.

Needs `pip install rpi-hardware-pwm`. On a Pi 5 the PWM chip is 2, not 0.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

log = logging.getLogger(__name__)

PERIOD_US = 20_000     # 50 Hz


@dataclass
class ServoConfig:
    channel: int
    min_deg: float
    max_deg: float
    trim_deg: float = 0.0
    invert: bool = False
    us_per_deg: float = 11.11     # 1000 us over 90 deg: 500-2500 us spans 180 deg
    center_us: float = 1500.0


class Servo:
    def __init__(self, cfg: ServoConfig, chip: int = 0) -> None:
        from rpi_hardware_pwm import HardwarePWM   # lazy: Pi only

        self.cfg = cfg
        self._pwm = HardwarePWM(pwm_channel=cfg.channel, hz=50, chip=chip)
        self._started = False
        self.angle = 0.0

    def set(self, deg: float) -> float:
        c = self.cfg
        deg = max(c.min_deg, min(c.max_deg, float(deg)))
        signed = -deg if c.invert else deg
        us = c.center_us + (signed + c.trim_deg) * c.us_per_deg
        us = max(500.0, min(2500.0, us))
        duty = us / PERIOD_US * 100.0
        if not self._started:
            self._pwm.start(duty)
            self._started = True
        else:
            self._pwm.change_duty_cycle(duty)
        self.angle = deg
        return deg

    def relax(self) -> None:
        """Stop the pulse train. The servo goes limp — no holding current."""
        if self._started:
            self._pwm.stop()
            self._started = False


class PanTilt:
    def __init__(self, pan: ServoConfig, tilt: ServoConfig, chip: int = 0) -> None:
        self.pan = Servo(pan, chip)
        self.tilt = Servo(tilt, chip)

    def set(self, pan_deg: float, tilt_deg: float) -> tuple[float, float]:
        return self.pan.set(pan_deg), self.tilt.set(tilt_deg)

    def relax(self) -> None:
        self.pan.relax()
        self.tilt.relax()


class NullPanTilt:
    """No servos (bench, or rpi-hardware-pwm missing). Remembers angles only."""

    def __init__(self, *_a, **_k) -> None:
        self.angles = (0.0, 0.0)

    def set(self, pan_deg: float, tilt_deg: float) -> tuple[float, float]:
        self.angles = (pan_deg, tilt_deg)
        return self.angles

    def relax(self) -> None:
        pass
