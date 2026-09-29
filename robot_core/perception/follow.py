"""Person following as short chained moves. VISION-DESIGN.md Part 7.2, adapted.

The drivetrain takes encoder-counted Drive(metres) / Turn(degrees) goals, not
velocities, so following is a loop of small steps: look, decide one short move,
let it finish, look again. At ~0.3 m per step that is jerky but predictable,
and every step goes through the drivetrain's own limits and e-stop.

The distance invariant from the design doc still holds; what changes without
the lidar is WHO enforces it. Today it is this controller alone, from a
bbox-size distance estimate — so the margins are wider and forward motion
needs a fresh "floor between us is clear" check from the depth model.

    FOLLOW_STOP_M      below this, never step forward. Hold, don't reverse
    FOLLOW_STANDOFF_M  target distance
    forward steps      only with a corridor check younger than CLEAR_MAX_AGE_S

Recovery from too-close is HOLD, not reverse: there is no rear sensor.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .motion_types import Move


@dataclass
class FollowConfig:
    stop_m: float = 0.9               # wider than the doc's 0.8: no lidar veto behind this
    standoff_m: float = 1.3
    band_m: float = 0.25              # dead band around standoff
    turn_deadband_deg: float = 10.0   # inside this the gimbal handles it; the base stays put
    max_turn_deg: float = 45.0
    turn_gain: float = 0.9            # turn slightly less than measured; the next look corrects
    max_step_m: float = 0.30
    min_step_m: float = 0.08
    clear_max_age_s: float = 1.2


@dataclass(frozen=True)
class FollowInput:
    bearing_deg: Optional[float]      # target, robot-body frame, + = right. None = not seen
    distance_m: Optional[float]
    clear_ahead: Optional[bool]       # corridor check; None = unknown
    clear_age_s: float                # how old that check is


def decide(inp: FollowInput, cfg: FollowConfig) -> Move:
    if inp.bearing_deg is None:
        return Move.hold("target not in view")

    # Face them first. Turning in place never closes distance, so it needs no
    # clearance check — and it keeps the target centred, which keeps the
    # distance estimate honest (off-centre boxes are more often cut by the edge).
    if abs(inp.bearing_deg) > cfg.turn_deadband_deg:
        angle = max(-cfg.max_turn_deg, min(cfg.max_turn_deg, inp.bearing_deg * cfg.turn_gain))
        return Move("turn", round(angle, 1), "face target")

    d = inp.distance_m
    if d is None:
        return Move.hold("no distance estimate")
    if d < cfg.stop_m:
        return Move.hold("too close")
    if d <= cfg.standoff_m + cfg.band_m:
        return Move.hold("at standoff")

    if inp.clear_ahead is None or inp.clear_age_s > cfg.clear_max_age_s:
        return Move.hold("need clearance check")
    if not inp.clear_ahead:
        return Move.hold("something between us")

    step = min(cfg.max_step_m, d - cfg.standoff_m)
    if step < cfg.min_step_m:
        return Move.hold("at standoff")
    return Move("drive", round(step, 2), "close distance")
