"""Draws status.BOARD on the 2.42" SSD1309 OLED (128x64, 7-pin SPI).

Wiring (BCM / physical pin):  VCC 3.3V (1)  GND (25)  SCK GPIO11 (23)
SDA = data in = GPIO10 (19)  CS GPIO8 (24)  DC GPIO25 (22)  RES GPIO17 (11)
and `dtparam=spi=on` in /boot/firmware/config.txt. Needs: pip install ".[oled]".

A header in small print (4x6) with the battery icon, then 7 rows x 21 characters,
6x8 pixels each; an error is a box of small print at the bottom, and whatever space the text leaves free
plays an animation. To add one: a painter and a line in ANIMS, then
`status.anim("name")`. No screen or no libraries is not an error for the robot:
`start()` logs once and returns False.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional

from robot_core import status

log = logging.getLogger(__name__)
CELL_W, CELL_H = 6, 8
SMALL_W, SMALL_H = 4, 6
ANIM_HZ = 8.0


def open_device(dc: int = 25, rst: int = 17, bus: int = 0, cs: int = 0, speed_hz: int = 8_000_000):
    from luma.core.interface.serial import spi
    from luma.oled.device import ssd1309
    return ssd1309(spi(port=bus, device=cs, gpio_DC=dc, gpio_RST=rst, bus_speed_hz=speed_hz))


def _text(d, x0: int, y0: int, line: str, font, cell_w: int) -> None:
    for c, ch in enumerate(line):
        glyph = font[ord(ch) if ord(ch) < 128 else 63]
        for x, col in enumerate(glyph[:cell_w - 1]):
            for y in range(CELL_H):
                if col >> y & 1:
                    d.point((x0 + c * cell_w + x, y0 + y), fill="white")


def _battery(d, x: int, y: int, percent: float) -> None:
    d.rectangle((x, y, x + 17, y + 6), outline="white")
    d.rectangle((x + 18, y + 2, x + 19, y + 4), fill="white")                  # the + terminal
    w = round(14 * max(0.0, min(100.0, percent)) / 100)
    if w:
        d.rectangle((x + 2, y + 2, x + 1 + w, y + 4), fill="white")


def _eyes(d, box, pose: str) -> None:
    x0, y0, x1, y1 = box
    fh = min(y1 - y0 - 4, 40)                                                  # the face, an outline around the eyes
    fw, cx, cy = fh * 3 // 2, (x0 + x1) // 2, (y0 + y1) // 2
    d.rounded_rectangle((cx - fw // 2, cy - fh // 2, cx + fw // 2, cy + fh // 2), radius=fh // 4, outline="white")
    h = fh * 2 // 5
    w, lid = h * 3 // 4, 2 if pose == "-" else h                               # "-" = blink
    cx += {"<": -fw // 8, ">": fw // 8}.get(pose, 0)
    for ex in (cx - w // 2 - w, cx + w // 2):
        d.rounded_rectangle((ex, cy - lid // 2, ex + w, cy + lid // 2), radius=min(w, lid) // 3, fill="white")


def _robot(d, box, pose: str) -> None:
    """A little box robot on treads, side on and driving right: the tread rolls, the head
    bobs, and puffs of dust drift away behind it."""
    x0, y0, x1, y1 = box
    step, s = int(pose), 2 if y1 - y0 >= 44 else 1                             # twice the size when there is room
    ox, oy = (x0 + x1) // 2 - 8 * s, y1 - 2 - 20 * s                           # robot is 22 x 20 units, dust to its left

    def rect(a, b, c, e, **kw):                                                # in robot units
        d.rectangle((ox + a * s, oy + b * s, ox + (c + 1) * s - 1, oy + (e + 1) * s - 1), **kw)

    bob = step % 2
    d.rounded_rectangle((ox + 11 * s, oy + bob * s, ox + 21 * s - 1, oy + (5 + bob) * s - 1), radius=2 * s, fill="white")
    rect(17, 1 + bob, 18, 3 + bob, fill="black")                               # the eye and its lens
    rect(12, 5 + bob, 13, 7, fill="white")                                     # neck
    rect(4, 8, 15, 13, outline="white", width=s)                               # body
    rect(16, 10, 19, 10, fill="white")                                         # arm
    rect(19, 9, 19, 11, fill="white")
    for (a, b, c, e), fill in (((7, 14, 12, 20), "white"), ((8, 13, 13, 19), "black")):   # the tread: a triangle
        d.polygon([(ox + a * s, oy + c * s), (ox + b * s - 1, oy + c * s),                 # on its side, outlined
                   (ox + (b + 5) * s - 1, oy + e * s - 1), (ox + (a - 5) * s, oy + e * s - 1)], fill=fill)
    for wx, wy in ((5, 17), (10, 17), (15, 17), (10, 14)):
        rect(wx, wy, wx + 1, wy + 1, fill="white")                             # wheels
    for i in range(6):
        rect(2 + (3 * i - step) % 18, 19, 2 + (3 * i - step) % 18, 19, fill="black")       # tread gaps run backwards
    for k in range(3):                                                         # dust: small and solid, then bigger and hollow
        age = (step + 2 * k) % 6
        cx, cy, r = ox - 2.4 * age * s, oy + (18.5 - 0.7 * age) * s, (0.8 + 0.35 * age) * s
        d.ellipse((cx - r, cy - r, cx + r, cy + r), outline="white", fill="white" if age < 2 else None)


# name -> (one pose per tick of ANIM_HZ, looping; painter(d, box, pose)). "" is the idle face.
ANIMS = {
    "": ("o" * 22 + "-" + "o" * 10 + "<" * 5 + "o" * 8 + ">" * 5 + "o" * 6 + "-", _eyes),   # open, blink, glance
    "tank": ("012345", _robot),                                                             # while driving
}


def pose(anim: str, t: float) -> str:
    poses = ANIMS.get(anim, ANIMS[""])[0]
    return poses[int(t * ANIM_HZ) % len(poses)]


def draw(device, frame: status.Frame, at: str = "o") -> None:
    from luma.core.legacy.font import LCD_FONT, TINY_FONT   # 5x7 and 3x5 glyphs
    from luma.core.render import canvas
    with canvas(device) as d:
        _text(d, 0, 0, frame.rows[0].upper(), TINY_FONT, SMALL_W)             # header; capitals read better at 3x5
        for r, line in enumerate(frame.rows[1:], 1):
            _text(d, 0, r * CELL_H, line, LCD_FONT, CELL_W)
        if frame.battery is not None:
            _battery(d, device.width - 20, 0, frame.battery)
        top = CELL_H * (max((r for r, line in enumerate(frame.rows) if line), default=-1) + 1)
        bottom = device.height
        if frame.error:                                                        # boxed, over the bottom of the screen
            bottom -= len(frame.error) * SMALL_H + 5
            d.rectangle((0, bottom + 1, device.width - 1, device.height - 1), outline="white", fill="black")
            for i, line in enumerate(frame.error):
                _text(d, 4, bottom + 3 + i * SMALL_H, line.upper(), TINY_FONT, SMALL_W)
        if bottom - top >= 22:                                                 # free space under the text
            ANIMS.get(frame.anim, ANIMS[""])[1](d, (0, top, device.width, bottom), at)


class Oled:
    def __init__(self, board: status.Board = status.BOARD, device=None,
                 opener: Callable = open_device, hz: float = ANIM_HZ) -> None:
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
        last: Optional[tuple] = None
        failing = False
        while not self._stop.is_set():
            frame = self.board.frame()
            view = (frame, pose(frame.anim, time.monotonic()))
            if view != last:                                # SPI traffic only when something changed
                try:
                    draw(self.device, *view)
                    last, failing = view, False
                except Exception as e:                      # noqa: BLE001 — screen unplugged: keep going, log once
                    if not failing:
                        log.warning("oled: draw failed (%s: %s)", type(e).__name__, e)
                    last, failing = None, True
            self._stop.wait(self.period)
