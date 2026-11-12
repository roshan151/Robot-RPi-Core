"""Look up / down / left / right for the voice tools: moves the head, remembers where it is.

The gimbal is opened on the first look and kept open, so the tilt persists for the
session. `release()` parks it and hands the head over (before a task that opens the head
itself, and on shutdown); the next look starts again from level.
"""

from __future__ import annotations

import threading
from typing import Any, Callable, Dict, Optional

DEFAULT_DEGREES = 30.0


class HeadLook:
    def __init__(self, open_gimbal: Callable[[], Any]) -> None:
        self._open = open_gimbal            # () -> Gimbal; may raise if there is no PWM
        self._gimbal: Optional[Any] = None
        self._pos = {"pan": 0.0, "tilt": 0.0}   # commanded angles, degrees: +pan left, +tilt up
        self._lock = threading.Lock()

    def look(self, degrees: float, axis: str = "tilt") -> Dict[str, Any]:
        """Move one axis ("tilt": + up; "pan": + left) by `degrees` from where it is
        now, clamped to the gimbal's limits. Returns at once; the head moves in the
        background."""
        with self._lock:
            if self._gimbal is None:
                self._gimbal = self._open()
                self._pos = {"pan": 0.0, "tilt": 0.0}
            g = self._gimbal
            why = g.refuse()
            if why:
                if g.fault:                      # tripped: let go of it, so the next look re-checks (a latched fault stays dead)
                    g.close()
                    self._gimbal = None
                return {"ok": False, "error": why}
            lo, hi = (g.tilt_limits if axis == "tilt" else g.pan_limits)()
            before, wanted = self._pos[axis], self._pos[axis] + degrees
            self._pos[axis] = g.move_to(**{axis: wanted}, wait=False)[1 if axis == "tilt" else 0]
            return {"ok": True, axis: round(self._pos[axis], 1), "clamped": not lo <= wanted <= hi,
                    "delta": self._pos[axis] - before}      # what actually moved (after clamping): used to retrace

    def center(self) -> Dict[str, Any]:
        """Back to straight ahead: both axes to the position the head started from."""
        deltas = {}
        for axis in ("tilt", "pan"):
            r = self.look(-self._pos[axis], axis)
            if not r.get("ok"):
                return r
            deltas[axis] = r["delta"]
        return {"ok": True, "pan": 0.0, "tilt": 0.0, "deltas": deltas}

    def gesture(self, axis: str, *degrees: float) -> bool:
        """Blocking: move `axis` by each of `degrees` in turn (each relative to the last),
        then back to where it started (a nod or a head shake). False if the head has no power."""
        with self._lock:
            if self._gimbal is None:
                self._gimbal = self._open()
                self._pos = {"pan": 0.0, "tilt": 0.0}
            g, start = self._gimbal, self._pos[axis]
        if not g.powered():
            return False
        pos = start
        for d in degrees:
            pos += d
            g.move_to(**{axis: pos}, wait=True)
        g.move_to(**{axis: start}, wait=True)
        return True

    def pan_to(self, degrees: float, wait: bool = False) -> bool:
        """Pan to `degrees` from where the head started (+ left), without changing the
        remembered position; `wait=False` returns at once. False if the head has no power."""
        with self._lock:
            if self._gimbal is None:
                self._gimbal = self._open()
                self._pos = {"pan": 0.0, "tilt": 0.0}
            g, start = self._gimbal, self._pos["pan"]
        if not g.powered():
            return False
        g.move_to(pan=start + degrees, wait=wait)
        return True

    def release(self, hold: bool = True) -> None:
        """Park the head. `hold=False` also stops the servo pulses afterwards (servos go limp):
        used before the Pi powers off, so the signal lines are never left driving a live pulse
        when the OS takes the pins away from a servo that is still powered."""
        with self._lock:
            g, self._gimbal, self._pos = self._gimbal, None, {"pan": 0.0, "tilt": 0.0}
        if g is None and not hold:
            try:
                g = self._open()           # an earlier close() left the PWM running: take it over to stop it
            except Exception:              # noqa: BLE001 - no PWM fitted, so nothing is running
                return
        if g is not None:
            g.close(release=None if hold else True)
