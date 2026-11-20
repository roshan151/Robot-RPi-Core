"""Readings that carry their own age.

Nov 19, three bugs of one kind: a pose that froze when /odom stopped was steered by as if
current; a ToF grid was paired with a camera box from the frame before; any ToF range on a
person's box was taken as their distance. Each value travelled without when it was measured,
by what, or how sure it was. A `Reading` carries that, and `Latest` keeps the newest of
something that arrives by callback and hands it out only while it is fresh:

    odom = Latest("odom", frame="odom")
    odom.set((x, y, th))                  # from the /odom callback
    pose = odom.get(max_age_s=1.0)        # None once it is older than a second
    odom.age()                            # seconds since the last one (inf: never)

Plain Python, no ROS, no hardware: the nav stack, the simulator and the replay tests use it.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional


@dataclass(frozen=True)
class Reading:
    value: Any
    t: float                              # time.monotonic() when it was measured (or received)
    source: str = ""                      # what measured it: "odom", "tof", "mac:floor"...
    sigma: Optional[float] = None         # how far off it may be, in its own units
    frame: str = ""                       # the frame it is in: "odom", "base", "camera"...

    def age(self, now: Optional[float] = None) -> float:
        return (time.monotonic() if now is None else now) - self.t

    def fresh(self, max_age_s: float, now: Optional[float] = None) -> bool:
        return self.age(now) <= max_age_s


class Latest:
    """The newest Reading of one thing, set from a callback thread, read from any other."""

    def __init__(self, source: str, frame: str = "", clock: Callable[[], float] = time.monotonic) -> None:
        self.source, self.frame, self.clock = source, frame, clock
        self._r: Optional[Reading] = None
        self._lock = threading.Lock()

    def set(self, value: Any, t: Optional[float] = None, sigma: Optional[float] = None) -> None:
        r = Reading(value, self.clock() if t is None else t, self.source, sigma, self.frame)
        with self._lock:
            self._r = r

    def reading(self) -> Optional[Reading]:
        with self._lock:
            return self._r

    def get(self, max_age_s: float) -> Any:
        """The value while it is at most max_age_s old, else None (never guess with a stale one)."""
        r = self.reading()
        return r.value if r is not None and r.fresh(max_age_s, self.clock()) else None

    def age(self) -> float:
        r = self.reading()
        return math.inf if r is None else r.age(self.clock())
