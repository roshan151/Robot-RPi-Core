"""Which microphone the voice sessions use: the buds when their mic is up, else
the camera's (docs/USB-CAMERA-MIC.md).

Chosen through PipeWire's default input with `pactl`, so PortAudio's default
device (ROBOT_AUDIO_INPUT_DEVICE blank) follows it and PipeWire converts the
camera mic to the 16 kHz mono the Live API wants. "buds" vs "room" also picks
the main prompt (settings.live_prompt). Without pactl (a laptop, the tests) it
is room mode and nothing is changed.
"""

from __future__ import annotations

import subprocess
import time
from dataclasses import dataclass
from typing import List, Optional

from robot_core import robot_log, settings

_T0 = time.monotonic()          # the voice node started: the fast check phase runs from here


@dataclass(frozen=True)
class Route:
    mode: str                        # "buds" | "room"
    source: Optional[str] = None     # PipeWire input to make the default
    sink: Optional[str] = None       # buds: their output, for the robot's voice


def _pactl(*args: str) -> str:
    try:
        return subprocess.run(["pactl", *args], capture_output=True, text=True, timeout=3).stdout
    except Exception:                                   # noqa: BLE001 — no pactl, no PipeWire
        return ""


def _names(kind: str) -> List[str]:
    """Device names from `pactl list short sources|sinks`, monitors left out."""
    rows = (line.split("\t") for line in _pactl("list", "short", kind).splitlines())
    return [r[1] for r in rows if len(r) > 1 and not r[1].endswith(".monitor")]


def _bluez(names: List[str]) -> List[str]:
    """Bluetooth devices (PipeWire or PulseAudio naming), BT_MAC's first."""
    bt = [n for n in names if n.startswith(("bluez_input.", "bluez_source.", "bluez_output.", "bluez_sink."))]
    mac = settings.BT_MAC.replace(":", "_").upper()
    return sorted(bt, key=lambda n: mac not in n.upper()) if mac else bt


def current() -> Route:
    """Buds if their MIC is up (buds on music only count as room), else room."""
    sources = _names("sources")
    buds = _bluez(sources)
    if settings.MIC_MODE == "buds" or (settings.MIC_MODE == "auto" and buds):
        return Route("buds", buds[0] if buds else None, next(iter(_bluez(_names("sinks"))), None))
    room = [n for n in sources if settings.ROOM_MIC and settings.ROOM_MIC in n]
    return Route("room", room[0] if room else None)


def use() -> Route:
    """The current route, made PipeWire's default input (and output, with buds)."""
    route = current()
    for kind, name in (("source", route.source), ("sink", route.sink)):
        if name:
            _pactl(f"set-default-{kind}", name)
    robot_log.event("audio.route", mode=route.mode, source=route.source)
    return route


def poll_interval() -> float:
    """How long until the next buds check: fast for the first minutes after start."""
    fast = time.monotonic() - _T0 < settings.ROUTE_FAST_FOR_S
    return settings.ROUTE_POLL_FAST_S if fast else settings.ROUTE_POLL_S
