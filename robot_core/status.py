"""What the robot is doing, in a few words, for the OLED (see robot_core/oled.py).

Anything can call these from any thread; nothing is stored on disk. Every
message is cut to 10 words, and the screen shows 8 rows of 21 characters.

    status.task("explore")            what the robot is working on
    status.tool("drive 0.5m")         the voice agent's current tool call
    status.detail("plant_03 photo 4/8")
    status.timer("enroll Sam", 30)    counts down on screen
    status.result("Sam enrolled")     shown for a few seconds
    status.error("P01")               E-P01 plus its short text from CODES

One process owns the screen. The others call `set_sink()` with something that
forwards the message to it (the ROS nodes publish /robot/status); until then
messages land on the local BOARD, which is what the display node draws.
"""

from __future__ import annotations

import logging
import re
import textwrap
import threading
import time
from typing import Callable, Dict, List, Optional

WIDTH, ROWS, MAX_WORDS = 21, 8, 10

# Error codes: letter = area, number = what. Texts are kept short for the screen.
CODES: Dict[str, str] = {
    "P01": "undervolt now", "P02": "undervolt earlier", "P03": "throttled",
    "P04": "cpu freq capped", "P05": "cpu too hot",
    "S01": "arduino link lost", "S02": "wheel stuck", "S03": "emergency stop",
    "S04": "motor cmd timeout",
    "C01": "camera open failed", "C02": "camera capture failed",
    "V01": "vision unreachable", "V02": "vision timeout",
    "G01": "head unavailable", "G02": "servo range limited",
    "L01": "voice session lost", "L02": "tool call failed", "L03": "unknown tool",
    "A01": "audio error", "B01": "battery low",
    "F01": "no face found", "F02": "several faces", "F03": "too few good frames",
    "F04": "face task failed",
    "N01": "drive stuck", "N02": "viewpoint unreachable", "N03": "explore aborted",
    "X01": "unexpected error", "X02": "fatal error",
}


def words(text, limit: int = MAX_WORDS) -> str:
    return " ".join(str(text).split()[:limit])


def _mmss(seconds: float) -> str:
    s = max(0, int(seconds + 0.999))
    return f"{s // 60}:{s % 60:02d}"


def _up(seconds: float) -> str:
    m = int(seconds // 60)
    return f"{m}m" if m < 60 else f"{m // 60}h{m % 60:02d}" if m < 600 else f"{m // 60}h"


class Board:
    """The screen's state. `apply()` takes messages; `lines()` is what to draw."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self._clock, self._lock = clock, threading.Lock()
        self.t0 = clock()
        self.task = self.tool = self.detail = ""
        self.code = self.cause = self.note = ""
        self.power, self.dips, self.battery = "OK", 0, None
        self.volts = self.amps = None             # from the PiSugar
        self._timer: Optional[tuple] = None       # (label, started, seconds)
        self._result: Optional[tuple] = None      # (text, until)
        self._note_until = 0.0

    def apply(self, msg: dict) -> None:
        k, v, now = msg.get("k"), words(msg.get("v", "")), self._clock()
        with self._lock:
            if k == "task":
                self.task, self.tool, self.detail = v, "", ""
                self._timer = None                    # a result stays up its few seconds
            elif k == "tool":
                self.tool = v
            elif k == "detail":
                self.detail = v
            elif k == "error":
                self.code, self.cause = str(msg.get("v", ""))[:4], words(msg.get("t", ""), 6)
            elif k == "timer":
                self._timer, self._result = (v, now, float(msg.get("n", 0))), None
            elif k == "timer_stop":
                self._timer = None
            elif k == "result":
                self._result, self._timer = (v, now + float(msg.get("n", 6))), None
            elif k == "power":
                self.power, self.dips = v or "OK", int(msg.get("n", 0))
            elif k == "battery":
                self.battery, self.volts, self.amps = msg.get("n"), msg.get("volts"), msg.get("amps")
            elif k == "note":
                self.note, self._note_until = v, now + float(msg.get("n", 30))

    def lines(self) -> List[str]:
        now = self._clock()
        with self._lock:
            pwr = self.power + (f" x{self.dips}" if self.dips else "")
            bat = "--" if self.battery is None else f"{self.battery:.0f}%"
            volts = "--V" if self.volts is None else f"{self.volts:.2f}V"      # top right, always
            amps = "--A" if self.amps is None else f"{self.amps:+.2f}A"
            rows = [f"{pwr:<9.9}{volts + ' ' + amps:>12}",
                    f"{self.task or 'idle':<12.12}{bat:>4} {_up(now - self.t0):>4}",
                    f"TOOL {self.tool}" if self.tool else ""]
            rows += (textwrap.wrap(self.detail, WIDTH) + ["", ""])[:2]
            if self._result and now < self._result[1]:
                rows.append(self._result[0])
            elif self._timer:
                label, t, total = self._timer
                rows.append(f"{label[:13]} {_mmss(total - (now - t))}")
            else:
                rows.append("")
            rows.append(f"E-{self.code} {CODES.get(self.code, '')}" if self.code else "")
            rows.append(self.cause if self.code else (self.note if now < self._note_until else ""))
        return [r[:WIDTH] for r in rows[:ROWS]] + [""] * (ROWS - len(rows))


BOARD = Board()
_sink: Callable[[dict], None] = BOARD.apply


def set_sink(fn: Optional[Callable[[dict], None]]) -> None:
    """Send messages somewhere else (None: back to the local BOARD)."""
    global _sink
    _sink = fn or BOARD.apply


def _post(**msg) -> None:
    try:
        _sink(msg)
    except Exception:                                  # noqa: BLE001 — a screen must never break the robot
        pass


def task(name: str) -> None: _post(k="task", v=words(name, 3))
def tool(text: str) -> None: _post(k="tool", v=words(text))
def detail(text: str) -> None: _post(k="detail", v=words(text))
def timer(label: str, seconds: float) -> None: _post(k="timer", v=words(label, 3), n=seconds)
def timer_stop() -> None: _post(k="timer_stop")
def result(text: str, hold: float = 6.0) -> None: _post(k="result", v=words(text), n=hold)
def error(code: str, text: str = "") -> None: _post(k="error", v=code, t=text)
def power(state: str, dips: int = 0) -> None: _post(k="power", v=state, n=dips)
def battery(percent=None, volts=None, amps=None) -> None: _post(k="battery", n=percent, volts=volts, amps=amps)
def note(text: str, seconds: float = 30.0) -> None: _post(k="note", v=text, n=seconds)


# ---------------------------------------------------------------- log -> code

# (event name or logger-name part, regex on the message, code). First match wins.
# Only WARNING and above are looked at.
RULES = (
    ("fatal", None, "X02"), ("estop", None, "S03"), ("move.failed", None, "S02"),
    ("voice.drop", None, "L01"), ("audio.error", None, "A01"),
    ("task.error", r"head", "G01"), ("voice.tool", r"unknown tool", "L03"), ("voice.tool", None, "L02"),
    ("gimbal", r"limited", "G02"),
    ("face_tasks", r"faces in view", "F02"), ("face_tasks", r"usable frames", "F03"),
    ("face_tasks", None, "F04"),
    ("vision", r"time", "V02"), ("vision", None, "V01"),
    ("drivetrain", r"timeout|timed out", "S04"), ("arduino", None, "S01"), ("serial", None, "S01"),
    ("camera", r"open|start", "C01"), ("camera", None, "C02"),
    ("explore", r"unreachable", "N02"), ("explore", r"vision service", "N03"),
    ("explore", r"stuck", "N01"), ("battery", None, "B01"),
)


def code_for(name: str, message: str) -> Optional[str]:
    for key, pattern, code in RULES:
        if key in name and (pattern is None or re.search(pattern, message, re.I)):
            return code
    return None


class LogHandler(logging.Handler):
    """Turns warnings and errors from any logger into an E-code on the screen."""

    def __init__(self) -> None:
        super().__init__(level=logging.WARNING)

    def emit(self, record: logging.LogRecord) -> None:
        evt = getattr(record, "evt", None)
        fields = getattr(record, "fields", None) or {}
        cause = str(fields.get("err") or fields.get("cause") or "")
        message = f"{record.getMessage()} {cause}"
        code = (evt and code_for(evt, message)) or code_for(record.name, message) \
            or ("X01" if record.levelno >= logging.ERROR else None)
        if code:
            error(code, cause)
