"""Pi power-dip watcher: spots under-voltage, says what the robot was doing, and
leaves one small note so the next boot can show it even if the dip reset the Pi.

The firmware keeps a live flag for "supply below ~4.63 V right now" (bit 0 of
get_throttled). We poll it ~5x a second: each rise is one dip. The sticky "has
happened since boot" bit cannot count repeats, so it is used only once, at start.

The cause is a rule-of-thumb guess from what the robot was doing, not a measurement.
"""

from __future__ import annotations

import json
import logging
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Callable, Optional

from robot_core import robot_log, status

log = logging.getLogger(__name__)

SYSFS = "/sys/devices/platform/soc/soc:firmware/get_throttled"
UV_NOW, CAPPED_NOW, THROTTLED_NOW, HOT_NOW, UV_EVER = 1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 16
LAST_FILE = Path.home() / ".cache" / "robot-lastdip.json"
WRITE_EVERY_S = 10.0                                         # at most one SD write per 10 s


def read_flags() -> Optional[int]:
    try:
        return int(Path(SYSFS).read_text().strip(), 0)
    except (OSError, ValueError):
        pass
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=1).stdout
        return int(out.split("=")[1].strip(), 0)
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def cpu_temp_c() -> Optional[float]:
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000.0
    except (OSError, ValueError):
        return None


def classify(task: str, tool: str, detail: str, uptime_s: float,
             battery: Optional[float] = None, temp_c: Optional[float] = None) -> str:
    """Short guess at why the supply dipped (at most 6 words)."""
    doing = f"{tool} {detail}".lower()
    first = tool.split(" ")[0] if tool else ""
    if uptime_s < 90:
        return "right after boot"
    if battery is not None and battery < 20:
        return "low battery"
    if temp_c is not None and temp_c >= 80:
        return "cpu hot"
    if first in ("drive", "turn", "stop") or "driv" in doing or "moving" in doing:
        return "wheels drawing current"
    if tool.startswith("look") or "head" in doing:
        return "head servo moving"
    if task:
        return "camera and vision load"
    return "idle, check supply"


class PowerMonitor:
    def __init__(self, board: status.Board = status.BOARD, read: Callable[[], Optional[int]] = read_flags,
                 battery_read: Optional[Callable[[], object]] = None,        # -> BatteryState (percent, volts, amps)
                 last_file: Path = LAST_FILE, poll_s: float = 0.2,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.board, self._read, self._battery, self.last_file = board, read, battery_read, Path(last_file)
        self.poll_s, self._clock = poll_s, clock
        self.dips, self._flags, self._started, self._dip_at = 0, 0, False, 0.0
        self._warned = False
        self._last_write = float("-inf")
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def show_last_boot(self) -> None:
        """Show (once) the note the previous run left behind, then remove it."""
        try:
            d = json.loads(self.last_file.read_text())
            self.last_file.unlink()
            status.note(f"last dip: {d['cause']}", 60)
            robot_log.event("power.lastdip", logging.WARNING, **d)
        except (OSError, ValueError, KeyError):
            pass

    def poll_once(self) -> None:
        flags = self._read()
        if flags is None:
            return
        new = flags & ~self._flags if self._started else flags & (UV_NOW | CAPPED_NOW | THROTTLED_NOW | HOT_NOW)
        if not self._started and flags & UV_EVER and not flags & UV_NOW:
            status.error("P02")                              # a dip happened before we were watching
            robot_log.event("power.earlier", logging.WARNING, flags=hex(flags), temp_c=cpu_temp_c())
        self._started, old = True, self._flags
        self._flags = flags
        if new & UV_NOW:
            self.dips += 1
            self._on_dip()
        elif old & UV_NOW and not flags & UV_NOW:
            status.power("OK", self.dips)
            robot_log.event("power.clear", dips=self.dips, secs=round(self._clock() - self._dip_at, 1))
        for bit, code in ((CAPPED_NOW, "P04"), (THROTTLED_NOW, "P03"), (HOT_NOW, "P05")):
            if new & bit:
                status.error(code)
                robot_log.event("power.flag", logging.WARNING, code=code, flags=hex(flags), temp_c=cpu_temp_c())

    def _on_dip(self) -> None:
        b = self.board
        battery = b.battery                                   # last PiSugar reading, never waits on it
        cause = classify(b.task, b.tool, b.detail, self._clock() - b.t0, battery, cpu_temp_c())
        status.error("P01", cause)
        status.power("DIP", self.dips)
        now = self._dip_at = self._clock()
        robot_log.event("power.dip", logging.WARNING, n=self.dips, cause=cause, task=b.task, tool=b.tool,
                        detail=b.detail, battery=battery, volts=b.volts, amps=b.amps, temp_c=cpu_temp_c(), flags=hex(self._flags),
                        uptime_s=round(now - b.t0))
        if now - self._last_write >= WRITE_EVERY_S:
            self._last_write = now
            try:                                              # one small overwrite, never a growing log
                self.last_file.parent.mkdir(parents=True, exist_ok=True)
                tmp = self.last_file.with_suffix(".tmp")
                tmp.write_text(json.dumps({"cause": cause, "task": b.task, "tool": b.tool}))
                os.replace(tmp, self.last_file)
            except OSError:
                log.exception("power: could not save the dip note")

    def start(self) -> None:
        self.show_last_boot()
        self._thread = threading.Thread(target=self._loop, name="power", daemon=True)
        self._thread.start()
        if self._battery:
            threading.Thread(target=self._battery_loop, name="pisugar", daemon=True).start()

    def stop(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.is_set():
            self.poll_once()
            self._stop.wait(self.poll_s)

    def _battery_loop(self) -> None:
        """PiSugar volts / amps / percent every 2 s, on its own thread so a slow
        socket can never delay dip detection. Unreadable shows as --."""
        while not self._stop.is_set():
            try:
                s = self._battery()
                status.battery(s.percent, s.volts, s.amps)
                if not getattr(s, "ok", True) and not self._warned:
                    self._warned = True
                    log.warning("power: PiSugar server not reachable, showing -- for volts and amps")
            except Exception:                                  # noqa: BLE001
                status.battery()
            self._stop.wait(2.0)
