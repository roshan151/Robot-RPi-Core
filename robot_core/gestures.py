"""
The robot's output channel.

It never speaks during normal operation — audio is reserved for the failure path
and only plays once the voice session is already torn down, which is what keeps
the microphone free of the robot's own voice.  So everything the robot has to
say, it says by moving.

Vocabulary
----------
    YES      forward 0.1 m, back 0.1 m          (a nod)
    NO       left 30°, right 60°, left 30°      (a head shake, net zero)
    UNCLEAR  same as NO — "no" and "I didn't understand you" are deliberately
             the same gesture
    DANCE    a ~20 s celebration ending in a full spin. Net zero on paper:
             the four 0.1 m reverses are undone by the closing 0.4 m forward,
             and the turns sum to exactly -360°. It is the one gesture that
             outlasts the sentence that prompted it, which is why `play()`
             refuses to start a second one on top of it.

Steps are **signed**, in the same units the model's tools use: positive metres
drive forward, positive degrees turn right.  That means a gesture is literally a
short script of ordinary `drive` / `turn` calls, with no separate vocabulary for
the motion layer to translate.

VOCABULARY below is the single source of truth for what the model may ask for:
live/tools.py builds both the function declaration and its validation from it. A
gesture added here needs no change there — and a gesture added here that ISN'T
picked up there is exactly how `dance` came to be advertised in the system
prompt while every call to it was refused.

Thinking has no gesture, by design.  Note the consequence: absence of motion is
ambiguous between *thinking*, *no*, and *the session is dead* — a silent robot
cannot be told apart from a broken one without reading logs.json.

Gestures are net-zero on paper and not in practice — wheel slip on three
consecutive turns accumulates heading error.  That is accepted; this build does
not depend on odometry.
"""

from __future__ import annotations

import logging
from typing import List, Optional, Tuple

from robot_core import robot_log
from robot_core.motion import MotionBackend

logger = logging.getLogger(__name__)

# ("drive", signed metres) or ("turn", signed degrees)
Step = Tuple[str, float]

GESTURE_YES_METERS = 0.1
GESTURE_NO_DEGREES = 30.0

DRIVE = "drive"
TURN = "turn"

YES: List[Step] = [
    (DRIVE, +GESTURE_YES_METERS),
    (DRIVE, -GESTURE_YES_METERS),
]

NO: List[Step] = [
    (TURN, -GESTURE_NO_DEGREES),
    (TURN, +GESTURE_NO_DEGREES * 2),
    (TURN, -GESTURE_NO_DEGREES),
]

DANCE: List[Step] = [
    (TURN,  -GESTURE_NO_DEGREES),
    (DRIVE, -GESTURE_YES_METERS),
    (TURN,  +GESTURE_NO_DEGREES * 2),
    (DRIVE, -GESTURE_YES_METERS),
    (TURN,  -GESTURE_NO_DEGREES * 2),
    (DRIVE, -GESTURE_YES_METERS),
    (TURN,  +GESTURE_NO_DEGREES * 2),
    (DRIVE, -GESTURE_YES_METERS),
    (TURN,  -(360 + GESTURE_NO_DEGREES)),
    (DRIVE, +GESTURE_YES_METERS * 4),
]

UNCLEAR: List[Step] = NO

VOCABULARY = {
    "yes": YES,
    "no": NO,
    "unclear": UNCLEAR,
    "dance": DANCE,
}


class Gesturer:
    """Plays gestures without ever blocking the caller."""

    def __init__(self, motion: MotionBackend) -> None:
        self._motion = motion

    def play(self, name: str) -> Optional[List[int]]:
        """Queue a gesture. Returns job ids, or None if it was suppressed.

        Suppressed whenever the robot is already moving or has work queued, for
        two reasons that amount to the same thing: queueing turns a burst of
        answers into a twenty-second dance that outlives the conversation that
        prompted it, and the robot should not nod while driving somewhere.

        The check is delegated to the backend rather than done here, so the ROS
        build can answer it authoritatively — there, the drivetrain node
        *rejects* a busy gesture goal, which closes the check-then-act race this
        method would otherwise have.
        """
        key = str(name).lower()
        steps = VOCABULARY.get(key)
        if steps is None:
            raise ValueError(
                f"unknown gesture {name!r}; expected one of {sorted(VOCABULARY)}"
            )

        if self._motion.busy():
            robot_log.event("gesture.skip", name=key, why="busy")
            return None

        jobs: List[int] = []
        for op, value in steps:
            result = (self._motion.drive(value, gesture=True) if op == DRIVE
                      else self._motion.turn(value, gesture=True))
            if not result.get("ok"):
                robot_log.event("gesture.skip", name=key, why="backend refused",
                                detail=result.get("error", ""), done=len(jobs))
                break
            jobs.append(result.get("queued"))

        robot_log.event("gesture", name=key, jobs=jobs)
        return jobs
