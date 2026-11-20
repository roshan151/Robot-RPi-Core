"""Health: what must work before a task starts, and what still works while it runs.

Nov 19: follow started with the head servos unpowered (the head froze; it stalled into things
5 times in 12 s), and twice with odometry dead after an Arduino reset (it stood, then quit after
3 s with no reason the user could see). Hardware faults showed up as "follow not working".

    preflight(...)    every check at once, before anything moves: what failed and how to fix it
    Watchdog          the same inputs while running -> a mode, decided here and not in each task:

        mode        when                                          the task does
        normal      everything fresh
        degraded    no ToF grid for TOF_DEGRADED_S                stand still (no driving blind)
                    odometry older than POSE_DEGRADED_S          nav stops itself ("no pose")
                    the Mac slower than MAC_SLOW_S                keep the last goal
        stop        odometry older than POSE_STOP_S               end the task, saying why
                    servo supply off                              end the task, saying why

Mode changes are logged ("health.mode"); preflight results too ("health.preflight").
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from robot_core import robot_log

TOF_FRESH_S = 1.0           # preflight: a grid at least this new
TOF_DEGRADED_S = 1.0
POSE_DEGRADED_S = 1.0
POSE_STOP_S = 3.0
MAC_SLOW_S = 2.0
MAC_PREFLIGHT_S = 2.0       # the Mac must answer /healthz within this


@dataclass
class Check:
    name: str
    ok: bool
    detail: str = ""


def preflight(head=None, motion=None, pose: Optional[Callable] = None, vision=None,
              clock: Callable[[], float] = time.monotonic) -> List[Check]:
    """Each check that applies (a None argument skips its checks). Never raises."""
    out: List[Check] = []
    if head is not None:
        powered = getattr(head.gimbal, "powered", None)
        if powered is not None:
            on = _safe(powered, True)
            out.append(Check("head servo power", on, "" if on else "switch the servo supply on"))
        grid_fn = getattr(head.lidar, "grid", None)
        if grid_fn is not None:
            def fresh_grid():                           # the head (and the ToF thread) may have opened just now:
                try:                                    # wait up to 1.5 s for a grid, as robot_core's Tof8x8.grid can
                    return grid_fn(newer_than=time.monotonic() - TOF_FRESH_S, timeout=1.5)
                except TypeError:
                    return grid_fn()
            g = _safe(fresh_grid, None)
            age = math.inf if g is None else clock() - g.t
            out.append(Check("ToF grids arriving", age <= TOF_FRESH_S,
                             f"newest {age:.1f} s old" if g is not None else "no grid yet: check the ToF on I2C"))
    if motion is not None:
        vel = _safe(getattr(motion, "can_glide", lambda: False), False)
        out.append(Check("velocity firmware", vel, "" if vel else "flash the -vel firmware"))
    if pose is not None:
        p = _safe(pose, None)
        out.append(Check("odometry pose fresh", p is not None,
                         "" if p is not None else "no /odom: the Arduino reset, hung or its serial link dropped"))
    if vision is not None:
        t0 = clock()
        try:
            h = getattr(vision, "health")
            try:
                h(timeout=MAC_PREFLIGHT_S)
            except TypeError:
                h()
            dt = clock() - t0
            out.append(Check("Mac answering", dt <= MAC_PREFLIGHT_S, f"{dt * 1000:.0f} ms"))
        except Exception as exc:                        # noqa: BLE001
            out.append(Check("Mac answering", False, f"{type(exc).__name__}: {exc}"[:120]))
    robot_log.event("health.preflight", logging.INFO if all(c.ok for c in out) else logging.WARNING,
                    checks={c.name: (c.ok, c.detail) for c in out})
    return out


def failed(checks: List[Check]) -> str:
    """'' if all passed, else the failures in one line, for status and the voice agent."""
    return "; ".join(f"{c.name}: {c.detail}" if c.detail else c.name for c in checks if not c.ok)


class Watchdog:
    """Call update() once per task step; it says the mode and why."""

    def __init__(self, head=None, pose: Optional[Callable] = None, clock: Callable[[], float] = time.monotonic) -> None:
        self.head, self.pose, self.clock = head, pose, clock
        self.pose_ok_t = clock()
        self.mode, self.reasons = "normal", []

    def update(self, mac_latency_s: Optional[float] = None) -> Tuple[str, List[str]]:
        now = self.clock()
        stop, degraded = [], []
        if self.pose is not None:
            if _safe(self.pose, None) is not None:
                self.pose_ok_t = now
            quiet = now - self.pose_ok_t
            if quiet > POSE_STOP_S:
                stop.append(f"no odometry for {quiet:.0f} s (drivetrain telemetry stopped)")
            elif quiet > POSE_DEGRADED_S:
                degraded.append(f"odometry {quiet:.1f} s old")
        if self.head is not None:
            powered = getattr(self.head.gimbal, "powered", None)
            if powered is not None and not _safe(powered, True):
                stop.append("head servo supply off")
            grid_fn = getattr(self.head.lidar, "grid", None)
            if grid_fn is not None:
                g = _safe(grid_fn, None)
                age = math.inf if g is None else now - g.t
                if age > TOF_DEGRADED_S:
                    degraded.append(f"no ToF grid for {age:.1f} s")
        if mac_latency_s is not None and mac_latency_s > MAC_SLOW_S:
            degraded.append(f"Mac took {mac_latency_s:.1f} s")
        mode = "stop" if stop else "degraded" if degraded else "normal"
        reasons = stop or degraded
        if mode != self.mode:
            robot_log.event("health.mode", logging.WARNING if mode != "normal" else logging.INFO,
                            mode=mode, was=self.mode, why=reasons)
        self.mode, self.reasons = mode, reasons
        return mode, reasons


def _safe(fn, default):
    try:
        return fn()
    except Exception:                                   # noqa: BLE001 — a broken sensor is a failed check, not a crash
        return default
