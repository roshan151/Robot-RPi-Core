"""TF-Luna single-point ToF rangefinder on the Pi's UART.

Mounted on the head beside the camera, so it measures range *along wherever
the camera is looking*. That one fact is what turns a 1-D sensor into:

  * a range-at-bearing for anything the detector finds (aim, then read),
  * a slow 2-D lidar (sweep the pan servo, read continuously — see `sweep`),
  * a floor / cliff probe (tilt down to where the floor should be).

Wiring: TF-Luna TX -> Pi RXD (GPIO15), RX -> TXD (GPIO14), 5 V + GND from the
buck rail. Enable the UART and turn OFF the serial login console
(`raspi-config` -> Interface -> Serial Port: login shell No, hardware Yes);
the port is then `/dev/serial0` at 115200 baud.

Frame (9 bytes, 100 Hz by default):
    0x59 0x59 DistL DistH AmpL AmpH TempL TempH Checksum
Distance is cm; checksum is the low byte of the sum of the first 8 bytes.
Amp < 100 means too little signal to trust; 65535 means overexposed.
"""

from __future__ import annotations

import logging
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional

log = logging.getLogger(__name__)

HEADER = 0x59
MIN_AMP = 100
MAX_AMP = 65535
MIN_RANGE_M = 0.2
MAX_RANGE_M = 8.0


@dataclass(frozen=True)
class Reading:
    t: float                 # host monotonic seconds at receipt
    range_m: Optional[float] # None when the reading is not trustworthy
    amp: int


def parse(buf: bytearray) -> List[tuple]:
    """Consume complete frames from `buf` (in place) -> [(dist_cm, amp, temp_c)].

    Resynchronises on the 0x59 0x59 header, so a dropped byte costs one frame.
    """
    out = []
    i = 0
    n = len(buf)
    while i + 9 <= n:
        if buf[i] != HEADER or buf[i + 1] != HEADER:
            i += 1
            continue
        frame = buf[i:i + 9]
        if (sum(frame[:8]) & 0xFF) != frame[8]:
            i += 1
            continue
        dist = frame[2] | frame[3] << 8
        amp = frame[4] | frame[5] << 8
        temp = (frame[6] | frame[7] << 8) / 8.0 - 256.0
        out.append((dist, amp, temp))
        i += 9
    del buf[:i]
    return out


def to_reading(dist_cm: int, amp: int, t: float) -> Reading:
    r = dist_cm / 100.0
    ok = MIN_AMP <= amp < MAX_AMP and MIN_RANGE_M <= r <= MAX_RANGE_M
    return Reading(t, r if ok else None, amp)


class TFLuna:
    """Background reader keeping ~20 s of readings, so a sweep can be
    reconstructed afterwards by pairing each reading with the gimbal angle at
    the same instant."""

    def __init__(self, port: str = "/dev/serial0", baud: int = 115200,
                 history: int = 2000, source: Optional[Callable[[int], bytes]] = None) -> None:
        self._hist: deque = deque(maxlen=history)
        self._cond = threading.Condition()
        self._stop = threading.Event()
        if source is None:
            import serial
            self._ser = serial.Serial(port, baud, timeout=0.05)
            source = self._ser.read
        else:
            self._ser = None
        self._read = source
        self._thread = threading.Thread(target=self._loop, name="tfluna", daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                data = self._read(64)
            except Exception:                              # noqa: BLE001
                log.exception("TF-Luna read failed; retrying")
                time.sleep(0.5)
                continue
            if not data:
                continue
            buf.extend(data)
            frames = parse(buf)
            if frames:
                now = time.monotonic()
                with self._cond:
                    for dist, amp, _ in frames:
                        self._hist.append(to_reading(dist, amp, now))
                    self._cond.notify_all()

    def latest(self) -> Optional[Reading]:
        with self._cond:
            return self._hist[-1] if self._hist else None

    def since(self, t0: float, t1: Optional[float] = None) -> List[Reading]:
        with self._cond:
            return [r for r in self._hist if r.t >= t0 and (t1 is None or r.t <= t1)]

    def read(self, n: int = 10, timeout: float = 1.0) -> Optional[float]:
        """Median of the next `n` *valid* readings, or None if they don't come.
        Call with the head settled; at 100 Hz, n=10 costs 0.1 s."""
        t0 = time.monotonic()
        deadline = t0 + timeout
        with self._cond:
            while True:
                good = [r.range_m for r in self._hist if r.t > t0 and r.range_m is not None]
                if len(good) >= n:
                    return statistics.median(good[:n])
                left = deadline - time.monotonic()
                if left <= 0:
                    return statistics.median(good) if len(good) >= max(3, n // 3) else None
                self._cond.wait(left)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self._ser is not None:
            self._ser.close()
