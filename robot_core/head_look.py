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
            if not g.powered():
                return {"ok": False, "error": "head servos have no power - switch the servo supply on"}
            lo, hi = (g.tilt_limits if axis == "tilt" else g.pan_limits)()
            wanted = self._pos[axis] + degrees
            self._pos[axis] = g.move_to(**{axis: wanted}, wait=False)[1 if axis == "tilt" else 0]
            return {"ok": True, axis: round(self._pos[axis], 1), "clamped": not lo <= wanted <= hi}

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
