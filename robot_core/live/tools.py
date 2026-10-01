"""
The robot's functions, as Gemini Live tools.

Every tool returns IMMEDIATELY
------------------------------
This is the load-bearing property, and it is a correctness requirement rather
than a nicety.

Tool calls are dispatched from inside `async for response in session.receive()`.
Awaiting a 1.4 s drive inside that loop stops the websocket being read for 1.4 s.
The receive side stops draining, back-pressure reaches the send side, the
microphone uplink stalls, audio starts being dropped, and the session dies —
which is exactly the failure we saw: two turns, `audio.error stage=mic-queue`,
then `voice.drop`, then a reconnect that survives one more command.

So no tool blocks. Every one of them hands off to a MotionBackend (see
robot_core/motion.py) that queues the work and answers "queued" straight away.
The event loop is then free to keep reading the socket and pumping the
microphone, which is what "always listening" actually requires.

`stop` is the exception, deliberately: it runs inline, empties the queue, and
fires the out-of-band brake. A stop that waits its turn behind the moves it is
meant to cancel is not a stop.

Nothing here knows whether the backend is a local MotionExecutor or a set of ROS
action clients. That is the entire point of the seam.
"""

from __future__ import annotations

import logging
import re
from typing import Any, Callable, Dict, Optional, Tuple

from robot_core import robot_log, settings
from robot_core.gestures import VOCABULARY, Gesturer
from robot_core.head_look import DEFAULT_DEGREES, HeadLook
from robot_core.motion import MotionBackend

logger = logging.getLogger(__name__)

TASKS = ("enroll_face", "match_face", "explore")


def declarations() -> list:
    """FunctionDeclarations for the Live session config.

    Deliberately small. Every tool is context the model re-reads on every turn,
    and a robot that can do four things reliably beats one that can nominally
    do ten.
    """
    from google.genai import types  # type: ignore

    def schema(**props):
        required = [k for k, v in props.items() if v.pop("_required", False)]
        return types.Schema(
            type="OBJECT",
            properties={k: types.Schema(**v) for k, v in props.items()},
            required=required,
        )

    return [
        types.FunctionDeclaration(
            name="drive",
            description=(
                "Queue a straight move. Positive metres go forward, negative "
                "go backward. Returns as soon as the move is queued, not when "
                "it finishes — you can keep listening and queue more."
            ),
            parameters=schema(meters={
                "type": "NUMBER",
                "description": "Distance in metres. Negative to reverse.",
                "_required": True,
            }),
        ),
        types.FunctionDeclaration(
            name="turn",
            description=(
                "Queue a turn on the spot. Positive degrees turn RIGHT, "
                "negative turn LEFT. Returns as soon as it is queued."
            ),
            parameters=schema(degrees={
                "type": "NUMBER",
                "description": "Degrees to turn. Negative for left.",
                "_required": True,
            }),
        ),
        types.FunctionDeclaration(
            name="look_up",
            description=(
                "Tilt the head up from where it is now. Default 30 degrees; the "
                "head stops at its limit if asked for more. Returns at once."
            ),
            parameters=schema(degrees={"type": "NUMBER", "description": "Degrees up, default 30."}),
        ),
        types.FunctionDeclaration(
            name="look_down",
            description=(
                "Tilt the head down from where it is now. Default 30 degrees; the "
                "head stops at its limit if asked for more. Returns at once."
            ),
            parameters=schema(degrees={"type": "NUMBER", "description": "Degrees down, default 30."}),
        ),
        types.FunctionDeclaration(
            name="stop",
            description=(
                "Stop immediately: cancels everything queued AND interrupts "
                "the move in progress. Call this the instant the operator says "
                "stop, and whenever you are unsure whether it is safe to keep "
                "moving."
            ),
            parameters=schema(),
        ),
        types.FunctionDeclaration(
            name="answer",
            description=(
                "Reply by moving, since the robot does not speak during a "
                "conversation. 'yes' nods; 'no' shakes; 'unclear' uses the "
                "same shake as 'no' and means you could not make out the "
                "speech; 'dance' is a celebration that takes about twenty "
                "seconds and returns to the starting position — use it only "
                "when asked, never as an answer to a question. Returns as "
                "soon as it is queued, and 'stop' cancels it."
            ),
            # Built from the gesture vocabulary rather than written out, so a
            # gesture added to gestures.py is offered to the model instead of
            # being rejected at dispatch by a list nobody remembered to update.
            parameters=schema(value={
                "type": "STRING",
                "description": "One of: " + ", ".join(sorted(VOCABULARY)),
                "_required": True,
            }),
        ),
        types.FunctionDeclaration(
            name="run_task",
            description=(
                "Start a long job. Your session closes now, so you cannot hear "
                "'stop' until it ends; it reopens afterwards and you answer by "
                "gesture (yes = worked / face known, no = failed / face unknown). "
                "'enroll_face' learns the face in front of the camera under "
                "`name` (required). 'match_face' checks whether that face is "
                "known. 'explore' maps the house and photographs every plant."
            ),
            parameters=schema(
                task={"type": "STRING", "description": "One of: " + ", ".join(TASKS),
                      "_required": True},
                name={"type": "STRING",
                      "description": "The name the operator said to enrol, e.g. 'register Roshan' "
                                     "-> 'Roshan'. Required for enroll_face. Never guess or make "
                                     "one up: if none was said, leave it empty."},
            ),
        ),
    ]


class RobotTools:
    """Executes the model's tool calls. Nothing here ever blocks the loop.

    Does NOT own the backend's lifetime — whoever constructed the backend
    closes it. That keeps the ownership obvious when a ROS node holds the
    action clients and this object is just a user of them.
    """

    def __init__(self, motion: MotionBackend,
                 task_runner: Optional[Callable[[str, str], bool]] = None,
                 head: Optional[HeadLook] = None) -> None:
        self._motion = motion
        self._head = head                    # look_up / look_down; None = no head
        self._gestures = Gesturer(motion)
        # (task, name) -> success; blocking, run by the agent with the session
        # closed. `pending` is how run_task hands a job to the agent.
        self.task_runner = task_runner
        self.pending: Optional[Tuple[str, str]] = None

    @property
    def motion(self) -> MotionBackend:
        return self._motion

    # ------------------------------------------------------------------ #

    async def dispatch(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Run one tool call. Never raises — a tool error is a result.

        Not actually async work: every handler is synchronous and fast. The
        coroutine signature is kept so the caller can await it uniformly.
        """
        handler: Optional[Callable[..., Dict[str, Any]]] = {
            "drive": self._drive,
            "turn": self._turn,
            "look_up": lambda degrees=DEFAULT_DEGREES: self._look(degrees),
            "look_down": lambda degrees=DEFAULT_DEGREES: self._look(-float(degrees)),
            "stop": self._stop,
            "answer": self._answer,
            "run_task": self._run_task,
        }.get(name)

        if handler is None:
            robot_log.event("voice.tool", logging.WARNING, name=name,
                            ok=False, err="unknown tool")
            return {"ok": False, "error": f"unknown tool {name!r}"}

        try:
            result = handler(**args)
            robot_log.event("voice.tool", name=name, args=args, **result)
            return result
        except TypeError as e:
            robot_log.event("voice.tool", logging.WARNING, name=name,
                            args=args, ok=False, err=str(e))
            return {"ok": False, "error": f"bad arguments: {e}"}
        except Exception as e:
            robot_log.event("voice.tool", logging.ERROR, name=name, args=args,
                            ok=False, err=f"{type(e).__name__}: {e}")
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ------------------------------------------------------------------ #

    def _drive(self, meters: float) -> Dict[str, Any]:
        m = float(meters)
        if abs(m) > settings.MAX_DRIVE_METERS:
            # Belt and braces. In the ROS build the drivetrain node rejects
            # over-limit goals too, and that is the limit that actually counts:
            # a limit the model can talk itself out of is not a limit.
            return {"ok": False,
                    "error": f"refused: {m} m exceeds the "
                             f"{settings.MAX_DRIVE_METERS} m limit"}
        if abs(m) < 1e-3:
            return {"ok": True, "note": "zero distance, nothing to do"}
        return self._motion.drive(m)

    def _turn(self, degrees: float) -> Dict[str, Any]:
        d = float(degrees)
        if abs(d) < 0.5:
            return {"ok": True, "note": "zero angle, nothing to do"}
        return self._motion.turn(d)

    def _look(self, degrees: float) -> Dict[str, Any]:
        if self._head is None:
            return {"ok": False, "error": "no head fitted"}
        d = float(degrees)
        sign = 1.0 if d >= 0 else -1.0
        return self._head.look(sign * min(abs(d), 90.0))    # limits are enforced by the gimbal

    def _stop(self) -> Dict[str, Any]:
        """Empty the queue and interrupt the move already running.

        Inline and out of band, all the way down: the backend clears every
        pending job and writes the brake straight to the serial port, bypassing
        the command lock the running move is holding. Anything that waited its
        turn here would arrive after the move it was meant to cancel had
        already finished.
        """
        return self._motion.stop()

    def _run_task(self, task: str, name: str = "") -> Dict[str, Any]:
        """Queue a job for the agent. Does not run it: the agent closes the Live
        session first, so the job never holds the receive loop or the microphone."""
        task = str(task).lower().strip()
        if task not in TASKS:
            return {"ok": False, "error": f"unknown task {task!r}; expected one of {', '.join(TASKS)}"}
        if self.task_runner is None:
            return {"ok": False, "error": "tasks are not available in this build"}
        if self.pending is not None:
            return {"ok": False, "error": "a task is already starting"}
        # The name becomes a folder and a gallery label on the Mac: first word, letters only.
        name = re.sub(r"[^A-Za-z0-9_-]", "", (str(name).split() or [""])[0])[:32]
        if task == "enroll_face" and not name:
            # No name was said: nothing to enrol under. Shake the head, start nothing.
            jobs = self._gestures.play("no")
            return {"ok": False, "gestured": jobs is not None,
                    "error": "enroll_face needs the person's name and none was given; task not started"}
        self.pending = (task, name)
        return {"ok": True, "starting": task, "note": "session closing; it reopens when the task ends"}

    def _answer(self, value: str) -> Dict[str, Any]:
        v = str(value).lower().strip()
        if v not in VOCABULARY:
            # Checked against the gesture table itself. Hardcoding the three
            # original names here is what made `dance` unreachable: it existed
            # in gestures.py and was advertised in the system prompt, and every
            # call the model made was refused by this line.
            return {"ok": False,
                    "error": f"unknown answer {value!r}; expected one of "
                             f"{', '.join(sorted(VOCABULARY))}"}
        # Gesturer drops the gesture if the robot is already moving: motion is
        # both the actuator and the display, and the actuator wins.
        jobs = self._gestures.play(v)
        if jobs is None:
            return {"ok": True, "gestured": False, "note": "busy moving"}
        return {"ok": True, "gestured": True, "value": v, "jobs": jobs}
