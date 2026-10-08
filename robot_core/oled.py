"""Draws status.BOARD on the 2.42" SSD1309 OLED (128x64, 7-pin SPI).

Wiring (BCM / physical pin):  VCC 3.3V (1)  GND (25)  SCK GPIO11 (23)
SDA = data in = GPIO10 (19)  CS GPIO8 (24)  DC GPIO25 (22)  RES GPIO17 (11)
and `dtparam=spi=on` in /boot/firmware/config.txt. Needs: pip install ".[oled]".

8 rows x 21 characters, 6x8 pixels each. No screen or no libraries is not an
error for the robot: `start()` logs once and returns False.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, List, Optional

from robot_core import status

log = logging.getLogger(__name__)
CELL_W, CELL_H = 6, 8


def open_device(dc: int = 25, rst: int = 17, bus: int = 0, cs: int = 0, speed_hz: int = 8_000_000):
    from luma.core.interface.serial import spi
    from luma.oled.device import ssd1309
    return ssd1309(spi(port=bus, device=cs, gpio_DC=dc, gpio_RST=rst, bus_speed_hz=speed_hz))


def draw(device, rows: List[str]) -> None:
    from luma.core.legacy.font import LCD_FONT           # 5x7 glyphs in an 8-column cell
    from luma.core.render import canvas
    with canvas(device) as d:
        for r, line in enumerate(rows):
            for c, ch in enumerate(line):
                glyph = LCD_FONT[ord(ch) if ord(ch) < 128 else 63]
                for x, col in enumerate(glyph[:5]):
                    for y in range(CELL_H):
                        if col >> y & 1:
                            d.point((c * CELL_W + x, r * CELL_H + y), fill="white")


class Oled:
    def __init__(self, board: status.Board = status.BOARD, device=None,
                 opener: Callable = open_device, hz: float = 2.0) -> None:
        self.board, self.device, self._open, self.period = board, device, opener, 1.0 / hz
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        try:
            if self.device is None:
                self.device = self._open()
        except Exception as e:                              # noqa: BLE001 — no screen is not fatal
            log.warning("oled: not available (%s: %s)", type(e).__name__, e)
            return False
        self._thread = threading.Thread(target=self._loop, name="oled", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.device is not None:
            try:
                self.device.clear()
            except Exception:                               # noqa: BLE001
                pass

    def _loop(self) -> None:
        last: Optional[List[str]] = None
        failing = False
        while not self._stop.is_set():
            rows = self.board.lines()
            if rows != last:                                # SPI traffic only when something changed
                try:
                    draw(self.device, rows)
                    last, failing = rows, False
                except Exception as e:                      # noqa: BLE001 — screen unplugged: keep going, log once
                    if not failing:
                        log.warning("oled: draw failed (%s: %s)", type(e).__name__, e)
                    last, failing = None, True
            self._stop.wait(self.period)
