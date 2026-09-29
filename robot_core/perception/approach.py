"""Drive up to a detected object, one short step at a time.

The design doc's approach planner (Part 7.1) scores a ring of viewpoints in the
odom frame using the lidar scan. Neither the lidar nor odometry exists yet, so
this is the version that works today: face the object, check the floor between
you is clear, step closer, look again. Each step re-detects, so drift and
estimate errors are corrected as the object gets bigger in frame.

Bounded like the doc's planner: at most MAX_APPROACH_M of travel in total, then
stop and report rather than wander.

If the straight line is blocked, it takes one detour step toward the clearest
open direction the free-space check found, then re-plans.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .motion_types import Move


@dataclass
class ApproachConfig:
    standoff_m: float = 0.6           # stop this far from the object (bbox estimate)
    tolerance_m: float = 0.15
    align_deg: float = 8.0
    max_turn_deg: float = 60.0
    max_step_m: float = 0.5
    creep_step_m: float = 0.2         # when no distance estimate exists for the class
    max_total_m: float = 3.0          # MAX_APPROACH_M
    clear_max_age_s: float = 1.2
    detour_step_m: float = 0.3
    max_detours: int = 2


@dataclass(frozen=True)
class ApproachInput:
    bearing_deg: Optional[float]      # robot-body frame; None = not currently seen
    distance_m: Optional[float]
    travelled_m: float                # so far in this approach
    detours: int
    clear_ahead: Optional[bool]
    clear_age_s: float
    best_open_bearing_deg: Optional[float] = None   # from the last full free-space check


@dataclass(frozen=True)
class ApproachStep:
    move: Move
    done: bool = False
    failed: bool = False
    detour: bool = False


def step(inp: ApproachInput, cfg: ApproachConfig) -> ApproachStep:
    if inp.bearing_deg is None:
        return ApproachStep(Move.hold("object not in view"))

    if inp.distance_m is not None and inp.distance_m <= cfg.standoff_m + cfg.tolerance_m:
        return ApproachStep(Move.hold("arrived"), done=True)
    if inp.travelled_m >= cfg.max_total_m:
        return ApproachStep(Move.hold(f"travel limit {cfg.max_total_m:g} m reached"), failed=True)

    if abs(inp.bearing_deg) > cfg.align_deg:
        a = max(-cfg.max_turn_deg, min(cfg.max_turn_deg, inp.bearing_deg))
        return ApproachStep(Move("turn", round(a, 1), "face object"))

    if inp.clear_ahead is None or inp.clear_age_s > cfg.clear_max_age_s:
        return ApproachStep(Move.hold("need clearance check"))

    remaining = cfg.max_total_m - inp.travelled_m
    if not inp.clear_ahead:
        if inp.detours >= cfg.max_detours or inp.best_open_bearing_deg is None:
            return ApproachStep(Move.hold("path blocked"), failed=True)
        return ApproachStep(Move("turn", round(inp.best_open_bearing_deg, 1), "detour"), detour=True)

    if inp.distance_m is None:
        d = min(cfg.creep_step_m, remaining)
        return ApproachStep(Move("drive", round(d, 2), "creep (no distance estimate)"))
    d = min(cfg.max_step_m, inp.distance_m - cfg.standoff_m, remaining)
    return ApproachStep(Move("drive", round(d, 2), "close distance"))
