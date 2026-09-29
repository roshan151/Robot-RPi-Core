"""Frame stamps: when a frame was captured and where the head was pointing.

A bounding box becomes a bearing only if you know the gimbal angles at the
instant of capture — the head keeps moving while a REST call is in flight.
Every frame gets a monotonic `seq`; every request carries it; every response
echoes it; the stamp is looked up on arrival. VISION-DESIGN.md Part 2.1.
"""

from __future__ import annotations

import threading
from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class FrameStamp:
    seq: int
    t_capture: float      # seconds, CLOCK_MONOTONIC (sensor timestamp when available)
    pan_deg: float        # commanded gimbal pan at capture, + = right
    tilt_deg: float       # commanded gimbal tilt at capture, + = up
    settled: bool         # head stationary for >= SETTLE_MS before capture


class StampRing:
    """The last `size` stamps, keyed by seq. Thread-safe."""

    def __init__(self, size: int = 64) -> None:
        self._size = size
        self._items: "OrderedDict[int, FrameStamp]" = OrderedDict()
        self._lock = threading.Lock()

    def add(self, stamp: FrameStamp) -> None:
        with self._lock:
            self._items[stamp.seq] = stamp
            while len(self._items) > self._size:
                self._items.popitem(last=False)

    def get(self, seq: int) -> Optional[FrameStamp]:
        with self._lock:
            return self._items.get(seq)

    def __len__(self) -> int:
        with self._lock:
            return len(self._items)
