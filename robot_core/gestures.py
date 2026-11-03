"""
The robot's output channel.

It never speaks during normal operation — audio is reserved for the failure path
and only plays once the voice session is already torn down, which is what keeps
the microphone free of the robot's own voice.  So everything the robot has to
say, it says by moving.

Vocabulary
----------
    YES      head tilts up, down past level, then back   (a nod - gimbal only, wheels still)
    NO       head pans left, right past centre, then back (a shake - gimbal only, wheels still)
    UNCLEAR  same as NO — "no" and "I didn't understand you" are deliberately
             the same gesture
    DANCE    a ~20 s celebration ending in a full spin. Net zero on paper:
             the four 0.1 m reverses are undone by the closing 0.4 m forward,
             and the turns sum to exactly -360°. It is the one gesture that
             outlasts the sentence that prompted it, which is why `play()`
             refuses to start a second one on top of it. With a head fitted,
             the head counter-pans every turn so it keeps facing the same way.

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
import threading
import time
from typing import List, Optional, Tuple

from robot_core import robot_log
from robot_core.motion import MotionBackend

logger = logging.getLogger(__name__)

# ("drive", signed metres), ("turn", signed degrees) or ("tilt"/"pan", degrees, degrees, ...)
Step = Tuple

GESTURE_YES_METERS = 0.1
GESTURE_NO_DEGREES = 30.0
GESTURE_NOD_DEGREES = 30.0      # head tilt for YES
GESTURE_SHAKE_DEGREES = 45.0    # head pan for NO

DRIVE = "drive"
TURN = "turn"
TILT = "tilt"                   # head steps: (axis, move, move, ...), each move relative to the last, then back to the start; + tilt up, + pan left
PAN = "pan"

YES: List[Step] = [(TILT, +GESTURE_NOD_DEGREES, -2 * GESTURE_NOD_DEGREES)]        # up 30, down 60, back

NO: List[Step] = [(PAN, +GESTURE_SHAKE_DEGREES, -2 * GESTURE_SHAKE_DEGREES)]      # left 45, right 90, back

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

    def __init__(self, motion: MotionBackend, head=None) -> None:
        self._motion = motion
        self._head = head                       # HeadLook for the yes/no gestures; None = no head
        self._head_thread: Optional[threading.Thread] = None

    def play(self, name: str, wait: bool = False) -> Optional[List[int]]:
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

        head_busy = self._head_thread is not None and self._head_thread.is_alive()
        if self._motion.busy() or head_busy:
            robot_log.event("gesture.skip", name=key, why="busy")
            return None

        if all(op in (TILT, PAN) for op, *_ in steps):
            return self._play_head(key, steps, wait)
        if self._head is not None:
            return self._play_body_with_head(key, steps, wait)

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

    def _play_body_with_head(self, key: str, steps: List[Step], wait: bool) -> Optional[List[int]]:
        """Body steps one at a time, the head counter-panning each turn in sync so it keeps
        facing the same way (+turn is right, +pan is left: the same sign is the opposite
        direction). Drives leave the head where it is; the full spin brings it back to centre."""
        def run() -> None:
            heading = 0.0
            try:
                for op, value in steps:
                    if op == TURN:
                        heading += value
                        self._head.pan_to(((heading + 180) % 360) - 180)    # starts now, doesn't wait
                        result = self._motion.turn(value, gesture=True)
                    else:
                        result = self._motion.drive(value, gesture=True)
                    if not result.get("ok"):
                        robot_log.event("gesture.skip", name=key, why="backend refused",
                                        detail=result.get("error", ""))
                        break
                    while self._motion.busy():                                # this step done...
                        time.sleep(0.02)
                    if op == TURN:
                        self._head.pan_to(((heading + 180) % 360) - 180, wait=True)  # ...and the head too
            except Exception as e:                       # noqa: BLE001 - a gesture must never take the session down
                robot_log.event("gesture.skip", logging.WARNING, name=key, why=f"{type(e).__name__}: {e}")
            finally:
                try:
                    self._head.pan_to(0.0, wait=True)    # always end facing forward
                except Exception:                        # noqa: BLE001
                    pass

        robot_log.event("gesture", name=key, head=True, body=True)
        if wait:
            run()
        else:
            self._head_thread = threading.Thread(target=run, name=f"gesture-{key}", daemon=True)
            self._head_thread.start()
        return []

    def _play_head(self, key: str, steps: List[Step], wait: bool) -> Optional[List[int]]:
        """Run a head gesture on the gimbal, in the background unless `wait`."""
        if self._head is None:
            robot_log.event("gesture.skip", name=key, why="no head")
            return None

        def run() -> None:
            try:
                for axis, *degrees in steps:
                    if not self._head.gesture(axis, *degrees):
                        robot_log.event("gesture.skip", name=key, why="head has no power")
                        return
            except Exception as e:                       # noqa: BLE001 - a gesture must never take the session down
                robot_log.event("gesture.skip", logging.WARNING, name=key, why=f"{type(e).__name__}: {e}")

        robot_log.event("gesture", name=key, head=True)
        if wait:
            run()
        else:
            self._head_thread = threading.Thread(target=run, name=f"gesture-{key}", daemon=True)
            self._head_thread.start()
        return []
