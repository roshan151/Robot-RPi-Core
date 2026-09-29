"""Pan/tilt aim. VISION-DESIGN.md Part 4.4.

The controller is given the target's ABSOLUTE angle relative to the robot body
(pan at capture + bearing in the image), not the pixel error of the latest
frame. That matters: the frame is up to a second old, and the head has moved
since. Chasing "target is 10 deg right of centre" twice for the same stale
observation is how a gimbal overshoots. Chasing "target is at body bearing
+25 deg" converges no matter how old the observation is.

Hobby servos have no position feedback, so:
  * slew-rate limit, so the commanded angle stays close to the real one
  * clamp in here, never in the caller (PLAN standing rule #4)
  * report `settled` only after SETTLE_MS without a commanded change
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class GimbalConfig:
    pan_min: float = -90.0
    pan_max: float = 90.0
    tilt_min: float = -30.0
    tilt_max: float = 45.0
    gain: float = 0.35                # fraction of the remaining error closed per tick (20 Hz)
    max_rate_dps: float = 120.0       # slew limit, degrees per second
    deadband_deg: float = 1.5
    settle_s: float = 0.12            # SETTLE_MS
    sweep_rate_dps: float = 45.0
    sweep_amplitudes: tuple = (30.0, 60.0)
    home_tilt: float = 0.0


class GimbalController:
    def __init__(self, config: Optional[GimbalConfig] = None) -> None:
        self.cfg = config or GimbalConfig()
        self.pan = 0.0
        self.tilt = self.cfg.home_tilt
        self._last_change_t = -1e9
        self._sweep: Optional[_Sweep] = None
        self._goal = (self.pan, self.tilt)

    # ------------------------------------------------------------------ #

    def settled(self, now: float) -> bool:
        return now - self._last_change_t >= self.cfg.settle_s

    def track(self, target_pan: float, target_tilt: float) -> None:
        """Set the goal to an absolute body-frame angle (also used to aim by hand)."""
        self._sweep = None
        self._goal = (self._clamp_pan(target_pan), self._clamp_tilt(target_tilt))

    def home(self) -> None:
        self.track(0.0, self.cfg.home_tilt)

    def hold(self) -> None:
        self._sweep = None
        self._goal = (self.pan, self.tilt)

    def start_sweep(self, center_pan: float, now: float) -> None:
        self._sweep = _Sweep(self._clamp_pan(center_pan), now, self.cfg)

    @property
    def sweeping(self) -> bool:
        return self._sweep is not None

    def tick(self, now: float, dt: float) -> tuple[float, float]:
        """Advance one control step. Returns the new commanded (pan, tilt)."""
        if self._sweep is not None:
            goal = (self._sweep.pan_at(now), self.tilt)
        else:
            goal = self._goal
        step_max = self.cfg.max_rate_dps * max(dt, 0.0)
        new_pan = self._approach(self.pan, goal[0], step_max, sweeping=self._sweep is not None)
        new_tilt = self._approach(self.tilt, goal[1], step_max, sweeping=False)
        if abs(new_pan - self.pan) > 1e-3 or abs(new_tilt - self.tilt) > 1e-3:
            self._last_change_t = now
        self.pan, self.tilt = new_pan, new_tilt
        return self.pan, self.tilt

    # ------------------------------------------------------------------ #

    def _approach(self, cur: float, goal: float, step_max: float, sweeping: bool) -> float:
        err = goal - cur
        if abs(err) <= self.cfg.deadband_deg and not sweeping:
            return cur
        move = err if sweeping else err * self.cfg.gain
        move = max(-step_max, min(step_max, move))
        return cur + move

    def _clamp_pan(self, v: float) -> float:
        return max(self.cfg.pan_min, min(self.cfg.pan_max, v))

    def _clamp_tilt(self, v: float) -> float:
        return max(self.cfg.tilt_min, min(self.cfg.tilt_max, v))


class _Sweep:
    """center -> +a1 -> -a1 -> +a2 -> -a2 -> center, then repeat, at a fixed rate."""

    def __init__(self, center: float, t0: float, cfg: GimbalConfig) -> None:
        self.t0, self.cfg = t0, cfg
        pts = [center]
        for a in cfg.sweep_amplitudes:
            pts += [center + a, center - a]
        pts.append(center)
        self.points = [max(cfg.pan_min, min(cfg.pan_max, p)) for p in pts]
        self.durations = [abs(b - a) / cfg.sweep_rate_dps for a, b in zip(self.points, self.points[1:])]
        self.period = sum(self.durations) or 1.0

    def pan_at(self, now: float) -> float:
        t = (now - self.t0) % self.period
        for (a, b), d in zip(zip(self.points, self.points[1:]), self.durations):
            if t <= d and d > 0:
                return a + (b - a) * (t / d)
            t -= d
        return self.points[-1]
