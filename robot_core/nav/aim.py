"""Where the gimbal looks while the robot drives: active perception.

The ToF and the camera ride on the same head, so where it points is what the costmap
learns. Three wishes, blended:

  * look-ahead: pan into the turn, `k_w` seconds of the current turn rate, so the head scans
    the arc the robot is about to drive into;
  * idle sweep: driving straight with nothing near, sweep +-`sweep_deg` so the costmap stays
    filled on both flanks; nearer obstacles ahead shrink the sweep (look where it matters);
  * focus: following someone, the camera must keep them in view, so the head is centred on
    them and look-ahead and sweep together may move it no more than `focus_slack_deg`
    (the camera sees +-40 degrees; the person stays well inside).

  * glance: the way ahead not seen for a while (the head held on someone off to the side),
    the head looks along the direction of travel for one ToF grid (`glance()`), then goes back.

`look(pan, tilt)` hands the head over to someone else (a search sweep) until `release()`.
The gimbal's own 50 Hz thread does the slewing; this only moves its target, at the
controller's rate, and only when it has moved by more than half a degree.
"""

from __future__ import annotations

import math
from typing import Optional


class GimbalAim:
    def __init__(self, gimbal, tilt_deg: float, k_w: float = 0.8, sweep_deg: float = 45.0,
                 sweep_period_s: float = 4.0, focus_slack_deg: float = 22.0, max_pan_deg: float = 80.0,
                 speed_dps: Optional[float] = None) -> None:
        self.gimbal, self.tilt = gimbal, tilt_deg
        self.k_w, self.sweep, self.period = k_w, sweep_deg, sweep_period_s
        self.slack, self.max_pan, self.speed = focus_slack_deg, max_pan_deg, speed_dps
        self.manual: Optional[tuple] = None
        self._last: Optional[tuple] = None
        self._phase = 0.0
        self._t: Optional[float] = None
        self.glance_until, self.glance_pan = 0.0, 0.0

    def glance(self, t: float, pan: float, hold_s: float = 0.45) -> None:
        """Look at `pan` until t + hold_s (long enough to settle and take a grid), then resume."""
        self.glance_until, self.glance_pan = t + hold_s, pan

    def look(self, pan: float, tilt: float) -> None:
        self.manual = (pan, tilt)
        self._send(pan, tilt, wait=True)

    def release(self) -> None:
        self.manual = None

    def target(self, t: float, v: float, w: float, focus_pan: Optional[float] = None,
               near_ahead_m: Optional[float] = None) -> float:
        """The pan (deg, + left) to aim at now."""
        dt = 0.0 if self._t is None else max(0.0, min(0.2, t - self._t))
        self._t = t
        look = max(-45.0, min(45.0, math.degrees(self.k_w * w)))
        straight = abs(w) < 0.15 and v > 0.03
        amp = self.sweep if straight else 10.0
        if near_ahead_m is not None:                  # something near ahead: keep looking at it
            amp = min(amp, max(5.0, self.sweep * (near_ahead_m - 0.3) / 1.2))
        if focus_pan is not None:
            look = max(-self.slack, min(self.slack, look))
            amp = max(0.0, min(amp, self.slack - abs(look)))
            base = focus_pan + look
        else:
            base = look
        self._phase = (self._phase + 2.0 * math.pi * dt / self.period) % (2.0 * math.pi)
        return max(-self.max_pan, min(self.max_pan, base + amp * math.sin(self._phase)))

    def update(self, t: float, v: float, w: float, focus_pan: Optional[float] = None,
               near_ahead_m: Optional[float] = None) -> None:
        if self.manual is not None:
            return
        if t < self.glance_until:
            self._send(max(-self.max_pan, min(self.max_pan, self.glance_pan)), self.tilt, wait=False)
            return
        self._send(self.target(t, v, w, focus_pan, near_ahead_m), self.tilt, wait=False)

    def _send(self, pan: float, tilt: float, wait: bool) -> None:
        if self._last is not None and abs(pan - self._last[0]) < 0.5 and abs(tilt - self._last[1]) < 0.5 and not wait:
            return
        self._last = (pan, tilt)
        self.gimbal.move_to(pan, tilt, speed_dps=self.speed, wait=wait)
        if wait:
            self.gimbal.wait_settled()
