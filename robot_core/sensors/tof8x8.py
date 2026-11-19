"""DFRobot Gravity 8x8 matrix ToF sensor (SEN0628: VL53L7CX + RP2040), on USB or I2C.

64 ranges in an 8x8 grid across a 60 x 60 degree view, 2 cm to 3.5 m. On the head
beside the camera it is three things:

  * the range sensor the TF-Luna was: `read()`, `latest()` and `since()` answer
    with the middle four zones, so `Head` and follow/explore work unchanged;
  * a floor scanner: tilted down, the rows are strips of floor at growing
    distances (`Head.floor_profile`, `robot_core/terrain.py`);
  * a range for anything the camera boxes (`range_in_box`).

Wiring is in the README ("IMU and ToF sensors"). Two ways in:

  * USB (`port`, robot.yaml tof_port): a USB-C cable to the Pi, nothing on the GPIO
    pins. The module prints the grid as text on its USB serial port (115200 baud),
    on its own, at its own rate. DFRobot does not document the layout, so the reader
    takes any line of 8 numbers as a row (a 9th, leading number, a row label, is
    dropped), 8 such rows in a row as a grid, a line of 64 numbers as a grid, and
    anything else (a header, a blank line, a pause in the stream) as the start of
    a new grid. Millimetres; 4000 (invalid, firmware 1.3) and anything past 3.5 m
    is None, as on I2C.
  * I2C (no port): the board's switch on I2C, its address switches left at 0x33,
    powered from 3.3 V so its data lines match the Pi's.

The module's own protocol (from DFRobot_MatrixLidar): write register 0x55 then
[length_hi, length_lo, command, arguments...], with length = arguments + 1; read
the answer one byte at a time: 0x53 (ok) or 0x63 (failed), the command, length_lo,
length_hi, then the data. "All data" (command 2) returns 64 little-endian
millimetre values. The module answers about five times a second over I2C.

Row 0 is the TOP of the view and column 0 its LEFT, as the sensor looks out. Which
way up the board is mounted decides that. A board turned on its side is set with
`tilt_deg` (robot.yaml tof_tilt): 90 = turned a quarter-turn clockwise as seen from
behind the head looking out (the board's top edge points to the robot's right),
-90 = anticlockwise; the grid is turned back so row 0 is the top of the view again.
Mirroring after that: check with Monitoring-Service/calibrate.py tof and set `flip_rows` /
`flip_cols` (robot.yaml tof_flip_rows / tof_flip_cols). The view is square, so a
turned board sees the same 60 x 60 degrees.
"""

from __future__ import annotations

import logging
import math
import re
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

from .. import debug_upload
from .tfluna import Reading

log = logging.getLogger(__name__)

SIZE = 8
FOV_DEG = 60.0
ZONE_DEG = FOV_DEG / SIZE
MIN_RANGE_M, MAX_RANGE_M = 0.02, 3.5
REG, CMD_SET_MODE, CMD_ALL_DATA = 0x55, 1, 2
OK, FAILED = 0x53, 0x63
CENTRE = [(r, c) for r in (3, 4) for c in (3, 4)]


@dataclass(frozen=True)
class Grid:
    t: float                                   # host monotonic seconds at receipt
    rows: Tuple[Tuple[Optional[float], ...], ...]   # [row][col] metres, None = no trustworthy range

    def centre(self) -> Optional[float]:
        good = [self.rows[r][c] for r, c in CENTRE if self.rows[r][c] is not None]
        return statistics.median(good) if good else None


def zone_angles(row: int, col: int) -> Tuple[float, float]:
    """(up, left) in degrees of a zone's centre from the sensor's axis."""
    mid = (SIZE - 1) / 2.0
    return (mid - row) * ZONE_DEG, (mid - col) * ZONE_DEG


def request(cmd: int, args: Sequence[int] = ()) -> List[int]:
    """The bytes written after register 0x55."""
    n = len(args) + 1
    return [(n >> 8) & 0xFF, n & 0xFF, cmd, *args]


def read_answer(read_byte: Callable[[], int], cmd: int, timeout: float = 1.0) -> Optional[bytes]:
    """The data of the answer to `cmd`, or None (failed, wrong command, or nothing in time)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = read_byte()
        if status not in (OK, FAILED):          # 0xFF or 0: not ready yet
            time.sleep(0.005)
            continue
        if read_byte() != cmd:
            return None
        n = read_byte() | read_byte() << 8
        if n > 2 * SIZE * SIZE:
            return None
        data = bytes(read_byte() for _ in range(n))
        return data if status == OK else None
    return None


def decode(data: bytes, flip_rows: bool = False, flip_cols: bool = False,
           tilt_deg: int = 0) -> Optional[Tuple[Tuple[Optional[float], ...], ...]]:
    """128 bytes -> 8 rows of 8 ranges in metres. None if it is not an 8x8 frame.
    tilt_deg (a multiple of 90): how far the board is turned clockwise, seen from behind; undone first."""
    if len(data) != 2 * SIZE * SIZE:
        return None
    return shape([data[i] | data[i + 1] << 8 for i in range(0, len(data), 2)], flip_rows, flip_cols, tilt_deg)


def shape(mm: Sequence[int], flip_rows: bool = False, flip_cols: bool = False,
          tilt_deg: int = 0) -> Tuple[Tuple[Optional[float], ...], ...]:
    """64 millimetre values, row by row as the module sends them -> the grid, turned and flipped."""
    m = [v / 1000.0 if MIN_RANGE_M <= v / 1000.0 <= MAX_RANGE_M else None for v in mm]
    rows = [m[r * SIZE:(r + 1) * SIZE] for r in range(SIZE)]
    for _ in range((int(tilt_deg) // 90) % 4):
        # Board a quarter-turn clockwise: its left edge looks up, its top edge right.
        rows = [[rows[SIZE - 1 - c][r] for c in range(SIZE)] for r in range(SIZE)]
    if flip_rows:
        rows.reverse()
    if flip_cols:
        rows = [row[::-1] for row in rows]
    return tuple(tuple(row) for row in rows)


class TextFrames:
    """The USB stream, line by line -> 64 millimetre values once a whole grid has come.
    See the module docstring for what is taken as a row; `gap_s`: a pause this long
    between lines starts a new grid (the module sends a grid in one burst)."""

    def __init__(self, gap_s: float = 0.03) -> None:
        self.gap_s, self._rows, self._last = gap_s, [], None

    def feed(self, line: str, t: float) -> Optional[List[int]]:
        if self._last is not None and t - self._last > self.gap_s:
            self._rows = []
        self._last = t
        nums = [int(v) for v in re.findall(r"\d+", line)]
        if len(nums) == SIZE * SIZE:
            self._rows = []
            return nums
        if len(nums) == SIZE + 1:
            nums = nums[1:]
        if len(nums) != SIZE:
            self._rows = []
            return None
        self._rows.append(nums)
        if len(self._rows) < SIZE:
            return None
        rows, self._rows = self._rows, []
        return [v for r in rows for v in r]


class _I2C:
    def __init__(self, bus: int, addr: int) -> None:
        from smbus2 import SMBus
        self._bus, self._addr = SMBus(bus), addr

    def write(self, pkt: Sequence[int]) -> None:
        self._bus.write_i2c_block_data(self._addr, REG, list(pkt))

    def read_byte(self) -> int:
        return self._bus.read_byte(self._addr)

    def close(self) -> None:
        self._bus.close()


class Tof8x8:
    """Background reader. On I2C `link` needs write(pkt), read_byte() and close(); with `port`
    (USB) it needs readline() and close(). Tests pass a fake."""

    def __init__(self, bus: int = 1, addr: int = 0x33, hz: float = 5.0,
                 flip_rows: bool = False, flip_cols: bool = False,
                 history: int = 200, link=None, tilt_deg: int = 0, port: Optional[str] = None,
                 range_scale: float = 1.0, range_offset_m: float = 0.0) -> None:
        if int(tilt_deg) % 90:
            raise ValueError(f"tof_tilt must be a multiple of 90 degrees, not {tilt_deg}")
        self._port = port
        if port:
            if link is None:
                import serial
                link = serial.Serial(port, 115200, timeout=0.5)
            self._link = link
        else:
            self._link = link if link is not None else _I2C(bus, addr)
        self._flip = (flip_rows, flip_cols, int(tilt_deg))
        self._fix = (float(range_scale), float(range_offset_m))   # tape-measured: true = scale * read + offset
        self._period = 1.0 / hz
        self._hist: deque = deque(maxlen=history)          # Grid
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._usb_loop if port else self._loop, name="tof8x8", daemon=True)
        self._thread.start()

    def _add(self, rows) -> None:
        scale, off = self._fix
        if (scale, off) != (1.0, 0.0):
            rows = tuple(tuple(None if v is None else round(scale * v + off, 4) for v in row) for row in rows)
        with self._cond:
            g = Grid(time.monotonic(), rows)
            self._hist.append(g)
            self._cond.notify_all()
        debug_upload.offer_tof(g.t, g.rows)             # debug mode only; a reference, no copy

    def _usb_loop(self) -> None:
        frames, failing = TextFrames(), False
        while not self._stop.is_set():
            try:
                line = self._link.readline()
            except Exception as e:                          # noqa: BLE001 — unplugged: say so once, keep trying
                if not failing:
                    log.warning("tof: USB read failed (%s: %s); retrying", type(e).__name__, e)
                failing = True
                self._stop.wait(1.0)
                continue
            if not line:
                continue
            mm = frames.feed(line.decode("ascii", "replace") if isinstance(line, bytes) else line, time.monotonic())
            if mm is not None:
                failing = False
                self._add(shape(mm, *self._flip))

    def _frame(self) -> Optional[bytes]:
        self._link.write(request(CMD_ALL_DATA))
        time.sleep(min(0.1, self._period / 2))             # the module needs a moment to answer
        return read_answer(self._link.read_byte, CMD_ALL_DATA)

    def _loop(self) -> None:
        failing = mode_set = False
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                data = self._frame()
                rows = decode(data, *self._flip) if data else None
                if data and rows is None and not mode_set:  # answering, but not 8x8 (it was left in 4x4)
                    mode_set = True
                    log.warning("tof: not in 8x8 mode; switching (takes 5 s)")
                    self._link.write(request(CMD_SET_MODE, [0, 0, 0, SIZE]))
                    self._stop.wait(5.0)
                    continue
            except Exception as e:                          # noqa: BLE001
                if not failing:
                    log.warning("tof: read failed (%s: %s); retrying", type(e).__name__, e)
                failing = True
                self._stop.wait(0.5)
                continue
            if rows is not None:
                failing = False
                self._add(rows)
            self._stop.wait(max(0.0, self._period - (time.monotonic() - t0)))

    # ------------------------------------------------------------ the grid

    def grid(self, newer_than: Optional[float] = None, timeout: float = 1.0) -> Optional[Grid]:
        """The latest grid; with `newer_than`, wait for one received after that
        moment (take the time once the head has settled). None if none comes."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                g = self._hist[-1] if self._hist else None
                if g is not None and (newer_than is None or g.t > newer_than):
                    return g
                left = deadline - time.monotonic()
                if newer_than is None or left <= 0:
                    return None
                self._cond.wait(left)

    # ------------------------------------- the TF-Luna's interface (centre zones)

    def latest(self) -> Optional[Reading]:
        g = self.grid()
        return None if g is None else Reading(g.t, g.centre(), 0)

    def since(self, t0: float, t1: Optional[float] = None) -> List[Reading]:
        with self._cond:
            return [Reading(g.t, g.centre(), 0) for g in self._hist
                    if g.t >= t0 and (t1 is None or g.t <= t1)]

    def read(self, n: int = 10, timeout: float = 1.0) -> Optional[float]:
        """Median range straight ahead from the grids that arrive from now on.
        At five grids a second `n` is capped at 3; one grid is already four zones."""
        n = max(1, min(n, 3))
        t0 = time.monotonic()
        deadline = t0 + timeout
        with self._cond:
            while True:
                good = [c for c in (g.centre() for g in self._hist if g.t > t0) if c is not None]
                left = deadline - time.monotonic()
                if len(good) >= n or left <= 0:
                    return statistics.median(good[:n]) if good else None
                self._cond.wait(left)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.5)
        try:
            self._link.close()
        except Exception:                                   # noqa: BLE001
            pass


def range_in_box(grid: Optional[Grid], box: Sequence[float], width: int, height: int,
                 hfov_deg: float, tof_pan_offset_deg: float = 0.0,
                 tof_tilt_offset_deg: float = 0.0) -> Optional[float]:
    """Median range of the zones whose centres fall inside a camera box
    (x1, y1, x2, y2 in pixels of a width x height frame). The sensor sits beside
    the camera and looks the same way. A box too small to hold any zone's centre (a
    pot across the room) takes the NEAREST of the zones it touches: the thing boxed
    is in front of whatever is behind it. None when no zone on it has a range."""
    if grid is None or not box:
        return None
    if tof_pan_offset_deg is None:
        tof_pan_offset_deg = 0.0
    if tof_tilt_offset_deg is None:
        tof_tilt_offset_deg = 0.0
    f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    left = math.degrees(math.atan2(width / 2.0 - box[0], f))
    right = math.degrees(math.atan2(width / 2.0 - box[2], f))
    top = math.degrees(math.atan2(height / 2.0 - box[1], f))
    bottom = math.degrees(math.atan2(height / 2.0 - box[3], f))
    good, touched = [], []
    half = ZONE_DEG / 2.0
    for r in range(SIZE):
        for c in range(SIZE):
            if grid.rows[r][c] is None:
                continue
            up, lf = zone_angles(r, c)
            up += tof_tilt_offset_deg
            lf += tof_pan_offset_deg
            if right <= lf <= left and bottom <= up <= top:
                good.append(grid.rows[r][c])
            elif right - half < lf < left + half and bottom - half < up < top + half:
                touched.append(grid.rows[r][c])
    if good:
        return statistics.median(good)
    return min(touched) if touched else None


def from_params(p: dict) -> Tof8x8:
    """The sensor as robot.yaml's /explorer section describes it: on USB if tof_port is set, else I2C."""
    return Tof8x8(port=str(p.get("tof_port") or "") or None,
                  bus=int(p.get("tof_bus", 1)), addr=int(p.get("tof_addr", 0x33)),
                  flip_rows=bool(p.get("tof_flip_rows", False)), flip_cols=bool(p.get("tof_flip_cols", False)),
                  tilt_deg=int(p.get("tof_tilt", 0)),
                  range_scale=float(p.get("tof_range_scale", 1.0)),
                  range_offset_m=float(p.get("tof_range_offset_m", 0.0)))
