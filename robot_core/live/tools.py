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
from typing import Any, Callable, Dict, Optional

from robot_core import robot_log, settings
from robot_core.gestures import VOCABULARY, Gesturer
from robot_core.motion import MotionBackend
from robot_core.perception.backend import VisionBackend

logger = logging.getLogger(__name__)


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
    ]


def vision_declarations() -> list:
    """The camera tools. Offered only when a VisionBackend is connected.

    Every one returns at once; what the robot saw or did arrives a moment later
    as a "[robot] ..." line of context.
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
            name="look",
            description=(
                "Report what the camera currently sees: objects, people you "
                "recognise, roughly how far and which direction. The answer "
                "arrives shortly as a [robot] line."
            ),
            parameters=schema(),
        ),
        types.FunctionDeclaration(
            name="who_is_there",
            description="Report which people are in view and who they are, if known.",
            parameters=schema(),
        ),
        types.FunctionDeclaration(
            name="follow_person",
            description=(
                "Start following a person, keeping about a metre and a half "
                "away. Give their name to follow someone specific, or leave it "
                "empty to follow whoever is in front. Keeps going until "
                "stop_following or stop."
            ),
            parameters=schema(name={
                "type": "STRING",
                "description": "Name of the person to follow, or empty for whoever is there.",
            }),
        ),
        types.FunctionDeclaration(
            name="stop_following",
            description="Stop following / approaching / tracking. The robot stays where it is.",
            parameters=schema(),
        ),
        types.FunctionDeclaration(
            name="approach",
            description=(
                "Drive up to an object you can see, stopping a little short of "
                "it, at most 3 m in total. Use everyday object names "
                "(chair, couch, bottle, cup, potted plant, tv, laptop, dog, person...)."
            ),
            parameters=schema(object={
                "type": "STRING",
                "description": "The kind of object, e.g. 'chair'.",
                "_required": True,
            }),
        ),
        types.FunctionDeclaration(
            name="find_clear_path",
            description=(
                "Check which direction in front of the robot looks open, from the "
                "camera. A rough hint, not a guarantee: call it before driving "
                "somewhere you were not told is clear."
            ),
            parameters=schema(),
        ),
        types.FunctionDeclaration(
            name="remember_face",
            description=(
                "Learn the face of the person standing in front of the camera "
                "under the given name, so you recognise them later. Ask them to "
                "face the robot for a few seconds first."
            ),
            parameters=schema(name={
                "type": "STRING",
                "description": "Their name.",
                "_required": True,
            }),
        ),
    ]


class RobotTools:
    """Executes the model's tool calls. Nothing here ever blocks the loop.

    Does NOT own the backend's lifetime — whoever constructed the backend
    closes it. That keeps the ownership obvious when a ROS node holds the
    action clients and this object is just a user of them.
    """

    def __init__(self, motion: MotionBackend, vision: Optional[VisionBackend] = None) -> None:
        self._motion = motion
        self._vision = vision
        self._gestures = Gesturer(motion)

    @property
    def motion(self) -> MotionBackend:
        return self._motion

    @property
    def vision(self) -> Optional[VisionBackend]:
        return self._vision

    def declarations(self) -> list:
        return declarations() + (vision_declarations() if self._vision is not None else [])

    # ------------------------------------------------------------------ #

    async def dispatch(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        """Run one tool call. Never raises — a tool error is a result.

        Not actually async work: every handler is synchronous and fast. The
        coroutine signature is kept so the caller can await it uniformly.
        """
        handlers: Dict[str, Callable[..., Dict[str, Any]]] = {
            "drive": self._drive,
            "turn": self._turn,
            "stop": self._stop,
            "answer": self._answer,
        }
        if self._vision is not None:
            handlers.update({
                "look": self._look,
                "who_is_there": self._who,
                "follow_person": self._follow,
                "stop_following": self._stop_following,
                "approach": self._approach,
                "find_clear_path": self._clear_path,
                "remember_face": self._remember,
            })
        handler: Optional[Callable[..., Dict[str, Any]]] = handlers.get(name)

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

    def _stop(self) -> Dict[str, Any]:
        """Empty the queue and interrupt the move already running.

        Inline and out of band, all the way down: the backend clears every
        pending job and writes the brake straight to the serial port, bypassing
        the command lock the running move is holding. Anything that waited its
        turn here would arrive after the move it was meant to cancel had
        already finished.
        """
        # A stop must also end following/approaching — otherwise the behavior
        # would send its next step a moment later.
        if self._vision is not None:
            try:
                self._vision.stop_following()
            except Exception:                   # noqa: BLE001 — the brake matters more
                logger.exception("stop_following failed during stop")
        return self._motion.stop()

    # -------------------------------------------------------------- vision

    def _look(self) -> Dict[str, Any]:
        return self._vision.look()

    def _who(self) -> Dict[str, Any]:
        return self._vision.who()

    def _follow(self, name: str = "") -> Dict[str, Any]:
        n = str(name or "").strip().lower()
        if n in ("me", "him", "her", "them", "anyone", "someone", "whoever"):
            n = ""
        return self._vision.follow(n or None)

    def _stop_following(self) -> Dict[str, Any]:
        return self._vision.stop_following()

    def _approach(self, object: str) -> Dict[str, Any]:           # noqa: A002 — tool arg name
        what = str(object).strip().lower()
        if not what:
            return {"ok": False, "error": "say what to approach, e.g. 'chair'"}
        return self._vision.approach(what)

    def _clear_path(self) -> Dict[str, Any]:
        return self._vision.clear_path()

    def _remember(self, name: str) -> Dict[str, Any]:
        n = str(name).strip().lower()
        if not n or len(n) > 64:
            return {"ok": False, "error": "need a name of 1-64 characters"}
        return self._vision.remember_face(n)

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
