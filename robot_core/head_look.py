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
            lo, hi = (g.tilt_limits if axis == "tilt" else g.pan_limits)()
            wanted = self._pos[axis] + degrees
            self._pos[axis] = g.move_to(**{axis: wanted}, wait=False)[1 if axis == "tilt" else 0]
            return {"ok": True, axis: round(self._pos[axis], 1), "clamped": not lo <= wanted <= hi}

    def release(self) -> None:
        with self._lock:
            g, self._gimbal, self._pos = self._gimbal, None, {"pan": 0.0, "tilt": 0.0}
        if g is not None:
            g.close()
