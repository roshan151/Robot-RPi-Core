"""
PiSugar battery state, and the startup announcement.

Why announce it out loud
------------------------
On a headless robot the battery is invisible. Discovering it was at 8% only
when the Pi browns out mid-drive — which on this board also resets the Arduino
— is a bad way to find out. One spoken line at startup, before the microphone
is ever opened, costs nothing and prevents that.

This is a deliberate exception to "the robot is silent during conversation",
and it is safe for the same reason the connected announcement is: nothing is
listening yet. It happens strictly before the first capture stream opens.

Protocol: the PiSugar server accepts line commands on a unix socket (and on
TCP 8423), replying `key: value`.
"""

from __future__ import annotations

import logging
import socket
import time
from dataclasses import dataclass
from typing import Optional

from robot_core import settings
from robot_core import robot_log
from robot_core import speech

logger = logging.getLogger(__name__)


class PiSugarUnavailable(RuntimeError):
    """The power manager is not reachable."""


@dataclass
class BatteryState:
    percent: Optional[float] = None
    volts: Optional[float] = None
    charging: Optional[bool] = None

    @property
    def ok(self) -> bool:
        return self.percent is not None or self.volts is not None

    def phrase(self) -> str:
        """Spoken form. Numbers are rounded — nobody needs two decimals.

        Written as plain digits: Gemini TTS reads "4.1 volts" correctly, so the
        old "4 point 1" spelling that espeak needed is gone. Keeping it would
        now make the robot enunciate the workaround.
        """
        parts = []
        if self.percent is not None:
            parts.append(f"battery {self.percent:.0f} percent")
        if self.volts is not None:
            parts.append(f"{self.volts:.1f} volts")
        if self.charging:
            parts.append("charging")
        return ", ".join(parts) if parts else "battery unknown"

    def as_dict(self) -> dict:
        return {
            "pct": None if self.percent is None else round(self.percent, 1),
            "volts": None if self.volts is None else round(self.volts, 2),
            "charging": self.charging,
        }


def _query(command: str, timeout: float = 1.0) -> str:
    """One request/response against the PiSugar server."""
    for path in settings.PISUGAR_SOCKETS:
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.settimeout(timeout)
                s.connect(path)
                s.sendall((command + "\n").encode())
                return s.recv(1024).decode(errors="replace").strip()
        except (FileNotFoundError, ConnectionRefusedError, OSError):
            continue

    host, port = settings.PISUGAR_TCP
    if host:
        try:
            with socket.create_connection((host, port), timeout=timeout) as s:
                s.sendall((command + "\n").encode())
                return s.recv(1024).decode(errors="replace").strip()
        except OSError:
            pass

    raise PiSugarUnavailable("PiSugar server not reachable")


def _value(reply: str) -> str:
    """Replies look like 'battery: 87.5' — take the right-hand side."""
    return reply.split(":", 1)[1].strip() if ":" in reply else reply.strip()


def read(retries: int = 3, delay: float = 1.0) -> BatteryState:
    """Read battery state, retrying while the server comes up.

    At boot this runs seconds after the socket's owner starts, so a first
    failure is expected rather than exceptional. Never raises: an unknown
    battery must not stop the robot.
    """
    state = BatteryState()
    for attempt in range(1, retries + 1):
        try:
            try:
                state.percent = float(_value(_query("get battery")))
            except (ValueError, IndexError):
                pass
            try:
                state.volts = float(_value(_query("get battery_v")))
            except (ValueError, IndexError):
                pass
            try:
                state.charging = _value(_query("get battery_charging")).lower() == "true"
            except (ValueError, IndexError):
                pass
            if state.ok:
                return state
        except PiSugarUnavailable:
            if attempt < retries:
                time.sleep(delay)
            continue
    return state


def announce(state: Optional[BatteryState] = None) -> BatteryState:
    """Read and speak the battery state. Call BEFORE opening the microphone.

    Returns the state so the caller can act on a low reading.
    """
    st = read() if state is None else state

    if not st.ok:
        robot_log.event("battery", logging.WARNING, ok=False,
                        reason="PiSugar unreachable")
        if settings.BATTERY_ANNOUNCE:
            # A primed static phrase, so this still speaks when the PiSugar is
            # unreachable because the whole board came up without a network.
            speech.say(speech.STATIC_PHRASES["battery_unknown"], event="battery.say")
        return st

    low = st.percent is not None and st.percent <= settings.BATTERY_LOW_PCT
    robot_log.event("battery", logging.WARNING if low else logging.INFO,
                    **st.as_dict(), low=low)

    if settings.BATTERY_ANNOUNCE:
        phrase = st.phrase()
        if low and not st.charging:
            phrase += ". Battery low."
        # The reading is different almost every boot, so this text misses the
        # cache and is synthesized live. The fallback keeps a flat battery from
        # being announced as silence when the robot booted with no network.
        speech.say(phrase.capitalize(), event="battery.say",
                fallback_text=speech.STATIC_PHRASES["battery_unknown"],
                **st.as_dict())

    return st


if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="Read / announce PiSugar battery.")
    ap.add_argument("--quiet", action="store_true", help="read only, do not speak")
    args = ap.parse_args()

    robot_log.setup()
    st = read()
    print(f"percent : {st.percent}")
    print(f"volts   : {st.volts}")
    print(f"charging: {st.charging}")
    print(f"phrase  : {st.phrase()}")
    if not args.quiet:
        print(f"voice   : {settings.TTS_VOICE} ({settings.TTS_MODEL})")
        print(f"playback: {speech.backend()}")
        announce(st)
