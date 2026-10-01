"""Look up / look down for the voice tools: moves the head's tilt, remembers where it is.

The gimbal is opened on the first look and kept open, so the tilt persists for the
session. `release()` parks it and frees the PWM (before a task that opens the head
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
        self._tilt = 0.0                    # commanded tilt, degrees, + is up
        self._lock = threading.Lock()

    def look(self, degrees: float) -> Dict[str, Any]:
        """Tilt by `degrees` from where it is now (+ up, - down), clamped to the
        gimbal's limits. Returns at once; the head moves in the background."""
        with self._lock:
            if self._gimbal is None:
                self._gimbal = self._open()
                self._tilt = 0.0
            lo, hi = self._gimbal.tilt_limits()
            wanted = self._tilt + degrees
            _, self._tilt = self._gimbal.move_to(tilt=wanted, wait=False)
            return {"ok": True, "tilt": round(self._tilt, 1), "clamped": not lo <= wanted <= hi}

    def release(self) -> None:
        with self._lock:
            g, self._gimbal, self._tilt = self._gimbal, None, 0.0
        if g is not None:
            g.close()
