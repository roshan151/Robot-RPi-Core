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
import queue
import re
import subprocess
import threading
from collections import deque
from typing import Any, Callable, Dict, Optional, Tuple

from robot_core import robot_log, settings, status
from robot_core.gestures import VOCABULARY, Gesturer
from robot_core.head_look import DEFAULT_DEGREES, HeadLook
from robot_core.motion import MotionBackend

logger = logging.getLogger(__name__)

TASKS = ("enroll_face", "match_face", "explore", "follow")
SHUTDOWN_DELAY_S = 4.0
MAX_COMMAND_QUEUE = 20


def _system_poweroff() -> None:
    """Needs passwordless sudo for this one command (install_pi.sh adds it)."""
    try:
        subprocess.run(["sudo", "-n", "systemctl", "poweroff"], check=True, timeout=20)
    except Exception as exc:                                  # noqa: BLE001
        robot_log.event("power.shutdown", logging.ERROR, state="failed", err=f"{type(exc).__name__}: {exc}")
        status.task("voice")
        status.error("X01", "shutdown failed")


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
            name="look_left",
            description=(
                "Turn the head left from where it is now. Default 30 degrees; the "
                "head stops at its limit if asked for more. Returns at once."
            ),
            parameters=schema(degrees={"type": "NUMBER", "description": "Degrees left, default 30."}),
        ),
        types.FunctionDeclaration(
            name="look_right",
            description=(
                "Turn the head right from where it is now. Default 30 degrees; the "
                "head stops at its limit if asked for more. Returns at once."
            ),
            parameters=schema(degrees={"type": "NUMBER", "description": "Degrees right, default 30."}),
        ),
        types.FunctionDeclaration(
            name="look_straight",
            description=(
                "Bring the head back to straight ahead (level, facing forward). "
                "For 'look straight', 'look ahead', 'center your head'. Returns at once."
            ),
            parameters=schema(),
        ),
        types.FunctionDeclaration(
            name="retrace",
            description=(
                "Undo the last `steps` drive / turn / look commands, newest first: "
                "'go back', 'retrace your steps', 'undo the last 3 moves'. Default 1. "
                "Returns as soon as the reverse moves are queued; 'stop' cancels them."
            ),
            parameters=schema(steps={"type": "INTEGER", "description": "How many commands to undo, default 1."}),
        ),
        types.FunctionDeclaration(
            name="shutdown",
            description=(
                "Shut the whole robot down cleanly (power off the Pi). Call it ONLY when "
                "the operator clearly asks to shut down, power off or turn off the robot. "
                "Never for background speech or a doubtful phrase; if unsure, answer 'unclear'. "
                "It powers off after about 4 seconds, and 'stop' cancels it in that time."
            ),
            parameters=schema(),
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
                "Queue a long job. If drives/turns are already queued or running, "
                "the job waits in order until they finish (your session stays open "
                "until then so 'stop' or more commands still work). Once the job "
                "starts, your session closes until it ends; it reopens afterwards "
                "and you answer by gesture (yes = worked / face known, no = failed / "
                "face unknown). 'enroll_face' learns the face in front of the camera "
                "under `name` (required). 'match_face' checks whether that face is "
                "known. 'explore' maps the house and photographs every plant "
                "(`mode` 'continue' when told to continue or finish the map). "
                "'follow' follows an enrolled person: `name` if one was said, "
                "empty for 'follow me'."
            ),
            parameters=schema(
                task={"type": "STRING", "description": "One of: " + ", ".join(TASKS),
                      "_required": True},
                name={"type": "STRING",
                      "description": "The name the operator said to enrol, e.g. 'register Roshan' "
                                     "-> 'Roshan', or to follow, e.g. 'follow Roshan' -> 'Roshan'. "
                                     "Required for enroll_face. Never guess or make "
                                     "one up: if none was said, leave it empty."},
                mode={"type": "STRING",
                      "description": "explore only: 'continue' when the operator says to continue or "
                                     "finish the map (stop if the robot is not on a known map); "
                                     "otherwise leave it empty."},
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
                 head: Optional[HeadLook] = None,
                 power_off: Optional[Callable[[], None]] = None) -> None:
        self._motion = motion
        self._power_off = power_off or _system_poweroff
        self._shutdown_timer: Optional[threading.Timer] = None
        self._head = head                    # look_up / look_down; None = no head
        self._gestures = Gesturer(motion, head)
        self._history: list = []             # (kind, signed amount) of what was commanded: drive/turn/tilt/pan
        # Single FIFO command queue so chained voice commands ("go straight, turn left,
        # then follow me") execute strictly in order — moves, head looks, retraces, and
        # long tasks alike — while every tool call still returns immediately.
        self._lock = threading.RLock()
        self._cmd_q: deque = deque()
        self._active_move: Optional[Tuple[str, float]] = None
        self._active_id: Optional[Any] = None
        # (task, name) -> success; blocking, run by the agent with the session
        # closed. `pending` is how run_task hands a job to the agent.
        self.task_runner = task_runner
        self.pending: Optional[Tuple[str, str]] = None

    @property
    def motion(self) -> MotionBackend:
        return self._motion

    def has_queued(self) -> bool:
        """True while a queued move, task, or pending follow-up command is active."""
        with self._lock:
            return bool(self._active_move is not None or self.pending is not None or self._cmd_q)

    def clear_queue(self, reason: str = "", *, reset_active: bool = False) -> int:
        """Drop every unstarted command from the FIFO queue (e.g. after stop or failure)."""
        with self._lock:
            dropped = len(self._cmd_q)
            self._cmd_q.clear()
            if reset_active:
                self._active_move = None
                self._active_id = None
            if dropped:
                robot_log.event("voice.queue_clear", dropped=dropped, reason=reason or "cleared")
            return dropped

    def pop_next_task(self) -> bool:
        """If the very next queued command is a long task, promote it straight to `pending`
        so back-to-back tasks run without a pointless Live reconnect in between."""
        with self._lock:
            if self._active_move is None and self.pending is None and self._cmd_q and self._cmd_q[0][0] == "task":
                _, task, name = self._cmd_q.popleft()
                self.pending = (task, name)
                self._history.clear()
                return True
            return False

    def advance(self) -> None:
        """Start queued commands in FIFO order until a blocking move or task takes the floor."""
        with self._lock:
            while self._active_move is None and self.pending is None and self._cmd_q:
                item = self._cmd_q.popleft()
                kind = item[0]
                if kind == "drive":
                    _, meters, record = item
                    r = self._motion.drive(meters)
                    if not r.get("ok"):
                        self.clear_queue(f"drive {meters:g} refused: {r.get('error', 'error')}")
                        return
                    if record:
                        self._record("drive", meters)
                    self._active_move = ("drive", meters)
                    q_id = r.get("queued")
                    self._active_id = q_id if not isinstance(q_id, bool) else None
                    return
                if kind == "turn":
                    _, degrees, record = item
                    r = self._motion.turn(degrees)
                    if not r.get("ok"):
                        self.clear_queue(f"turn {degrees:g} refused: {r.get('error', 'error')}")
                        return
                    if record:
                        self._record("turn", degrees)
                    self._active_move = ("turn", degrees)
                    q_id = r.get("queued")
                    self._active_id = q_id if not isinstance(q_id, bool) else None
                    return
                if kind == "look":
                    _, degrees, axis, record = item
                    r = self._exec_look(degrees, axis, record=record)
                    if not r.get("ok"):
                        self.clear_queue(f"look {axis} failed: {r.get('error', 'error')}")
                        return
                    continue
                if kind == "look_straight":
                    r = self._exec_look_straight()
                    if not r.get("ok"):
                        self.clear_queue(f"look_straight failed: {r.get('error', 'error')}")
                        return
                    continue
                if kind == "retrace":
                    _, steps = item
                    self._expand_retrace_front(steps)
                    continue
                if kind == "task":
                    _, task, name = item
                    self.pending = (task, name)
                    self._history.clear()
                    return

    def on_motion_result(self, event: Dict[str, Any]) -> str:
        """Handle a completed motion event from the backend and advance or clear the queue."""
        with self._lock:
            if self._active_move is None:
                return ""
            ev_id = event.get("id")
            if self._active_id is not None and ev_id is not None and ev_id != self._active_id:
                return ""
            self._active_move = None
            self._active_id = None
            st = str(event.get("status", "")).lower()
            if st in ("done", "finished", "ok"):
                self.advance()
                return ""
            dropped = self.clear_queue(f"move {st or 'failed'}")
            return f"cancelled {dropped} queued command(s)" if dropped else ""

    def poll_queue(self) -> None:
        """Keep the queue moving for backends without an async `results` queue (LocalMotion / mocks)
        and trigger RosMotion's lost-result timeout watchdog."""
        if getattr(self._motion, "results", None) is not None:
            # RosMotion: busy() pushes a 'timeout' event onto results if a goal ever went silent;
            # on_motion_result() from _pump_results is the single race-free advancement path.
            self._motion.busy()
            return
        executor = getattr(self._motion, "executor", None)
        if executor is not None:
            while True:
                try:
                    ev = executor.events.get_nowait()
                except queue.Empty:
                    break
                self.on_motion_result({"id": ev.job_id, "op": ev.op, "value": ev.value, "status": ev.status})
            return
        with self._lock:
            if self._active_move is not None and not self._motion.busy():
                self._active_move = None
                self._active_id = None
                self.advance()

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
            "look_left": lambda degrees=DEFAULT_DEGREES: self._look(degrees, "pan"),
            "look_right": lambda degrees=DEFAULT_DEGREES: self._look(-float(degrees), "pan"),
            "look_straight": self._look_straight,
            "retrace": self._retrace,
            "shutdown": self._shutdown,
            "stop": self._stop,
            "answer": self._answer,
            "run_task": self._run_task,
        }.get(name)

        if handler is None:
            robot_log.event("voice.tool", logging.WARNING, name=name,
                            ok=False, err="unknown tool")
            return {"ok": False, "error": f"unknown tool {name!r}"}

        status.tool(f"{name} {' '.join(str(v) for v in args.values())}")
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
        with self._lock:
            if not self.has_queued():
                r = self._motion.drive(m)
                if r.get("ok"):
                    self._record("drive", m)
                    self._active_move = ("drive", m)
                    q_id = r.get("queued")
                    self._active_id = q_id if not isinstance(q_id, bool) else None
                return r
            if len(self._cmd_q) >= MAX_COMMAND_QUEUE:
                return {"ok": False, "error": f"command queue full ({MAX_COMMAND_QUEUE} pending)"}
            self._cmd_q.append(("drive", m, True))
            return {"ok": True, "queued": True, "op": "drive", "value": m,
                    "queue_depth": len(self._cmd_q)}

    def _turn(self, degrees: float) -> Dict[str, Any]:
        d = float(degrees)
        if abs(d) < 0.5:
            return {"ok": True, "note": "zero angle, nothing to do"}
        with self._lock:
            if not self.has_queued():
                r = self._motion.turn(d)
                if r.get("ok"):
                    self._record("turn", d)
                    self._active_move = ("turn", d)
                    q_id = r.get("queued")
                    self._active_id = q_id if not isinstance(q_id, bool) else None
                return r
            if len(self._cmd_q) >= MAX_COMMAND_QUEUE:
                return {"ok": False, "error": f"command queue full ({MAX_COMMAND_QUEUE} pending)"}
            self._cmd_q.append(("turn", d, True))
            return {"ok": True, "queued": True, "op": "turn", "value": d,
                    "queue_depth": len(self._cmd_q)}

    def _exec_look(self, degrees: float, axis: str = "tilt", *, record: bool = True) -> Dict[str, Any]:
        if self._head is None:
            return {"ok": False, "error": "no head fitted"}
        d = float(degrees)
        sign = 1.0 if d >= 0 else -1.0
        r = self._head.look(sign * min(abs(d), 90.0), axis)    # limits are enforced by the gimbal
        if r.get("ok") and record:
            self._record(axis, r["delta"])
        return r

    def _look(self, degrees: float, axis: str = "tilt") -> Dict[str, Any]:
        if self._head is None:
            return {"ok": False, "error": "no head fitted"}
        d = float(degrees)
        with self._lock:
            if not self.has_queued():
                return self._exec_look(d, axis, record=True)
            if len(self._cmd_q) >= MAX_COMMAND_QUEUE:
                return {"ok": False, "error": f"command queue full ({MAX_COMMAND_QUEUE} pending)"}
            self._cmd_q.append(("look", d, axis, True))
            return {"ok": True, "queued": True, "op": f"look_{axis}", "degrees": d,
                    "queue_depth": len(self._cmd_q)}

    def _exec_look_straight(self) -> Dict[str, Any]:
        if self._head is None:
            return {"ok": False, "error": "no head fitted"}
        r = self._head.center()
        if r.get("ok"):
            for axis, delta in r["deltas"].items():
                self._record(axis, delta)
        return r

    def _look_straight(self) -> Dict[str, Any]:
        if self._head is None:
            return {"ok": False, "error": "no head fitted"}
        with self._lock:
            if not self.has_queued():
                return self._exec_look_straight()
            if len(self._cmd_q) >= MAX_COMMAND_QUEUE:
                return {"ok": False, "error": f"command queue full ({MAX_COMMAND_QUEUE} pending)"}
            self._cmd_q.append(("look_straight",))
            return {"ok": True, "queued": True, "op": "look_straight",
                    "queue_depth": len(self._cmd_q)}

    def _record(self, kind: str, amount: float) -> None:
        if abs(amount) > 1e-3:
            self._history.append((kind, amount))
            del self._history[:-50]                         # keep the last 50

    def _expand_retrace_front(self, steps: int) -> int:
        n = min(max(int(steps), 1), len(self._history))
        if not n:
            return 0
        rev = []
        for _ in range(n):
            kind, amount = self._history.pop()
            if kind == "drive":
                rev.append(("drive", -amount, False))
            elif kind == "turn":
                rev.append(("turn", -amount, False))
            elif self._head is not None:
                rev.append(("look", -amount, kind, False))
        for item in reversed(rev):
            self._cmd_q.appendleft(item)
        return len(rev)

    def _retrace(self, steps: int = 1) -> Dict[str, Any]:
        """Undo the newest `steps` commands, newest first, by commanding the opposite.
        Open loop, like the original moves. If commands are already queued ahead of it,
        retrace waits its turn so chained commands ('go straight, turn left, retrace 2')
        work in order."""
        s = max(int(steps), 1)
        with self._lock:
            if self.has_queued():
                if len(self._cmd_q) >= MAX_COMMAND_QUEUE:
                    return {"ok": False, "error": f"command queue full ({MAX_COMMAND_QUEUE} pending)"}
                self._cmd_q.append(("retrace", s))
                return {"ok": True, "queued": True, "op": "retrace", "steps": s,
                        "queue_depth": len(self._cmd_q)}
            n = self._expand_retrace_front(s)
            if not n:
                return {"ok": True, "retraced": 0, "note": "nothing to retrace"}
            self.advance()
            return {"ok": True, "retraced": n, "remaining": len(self._history)}

    def _stop(self) -> Dict[str, Any]:
        """Empty the queue and interrupt the move already running.

        Inline and out of band, all the way down: clears every queued command
        (moves, looks, and tasks) and writes the brake straight to the serial
        port, bypassing the command lock the running move is holding.
        """
        if self._shutdown_timer is not None:                # "stop" also cancels a pending power-off
            self._shutdown_timer.cancel()
            self._shutdown_timer = None
            status.task("voice")
            robot_log.event("power.shutdown", logging.WARNING, state="cancelled")
        self._gestures.cancel()                              # a dance sends its steps one by one: no more
        with self._lock:
            dropped = len(self._cmd_q) + (1 if self.pending is not None else 0)
            self._cmd_q.clear()
            self._active_move = None
            self._active_id = None
            self.pending = None
        r = self._motion.stop()
        if dropped:
            robot_log.event("voice.queue_clear", dropped=dropped, reason="stop")
        return {**r, "dropped_queued": dropped}

    def _shutdown(self) -> Dict[str, Any]:
        """Brake, park the head, then power the Pi off after a short delay so
        the operator can still say "stop". The OS stops the services cleanly, so the
        SD card is closed properly instead of being cut off."""
        if self._shutdown_timer is not None:
            return {"ok": True, "note": "already shutting down"}
        self._stop()
        if self._head is not None:
            self._head.release(hold=False)       # park, then stop the servo pulses: the Pi is about to drop the pins
        status.task("shutting down")
        status.detail("say stop to cancel")
        robot_log.event("power.shutdown", logging.WARNING, state="requested")
        self._shutdown_timer = threading.Timer(SHUTDOWN_DELAY_S, self._power_off)
        self._shutdown_timer.start()
        return {"ok": True, "note": f"powering off in {SHUTDOWN_DELAY_S:.0f} seconds; stop cancels"}

    def _run_task(self, task: str, name: str = "", mode: str = "") -> Dict[str, Any]:
        """Queue a job for the agent. If moves are running or queued ahead of it,
        the task waits in FIFO order until they finish (the Live session stays open
        so 'stop' or more commands still work). Otherwise it starts immediately."""
        task = str(task).lower().strip()
        if task not in TASKS:
            return {"ok": False, "error": f"unknown task {task!r}; expected one of {', '.join(TASKS)}"}
        if self.task_runner is None:
            return {"ok": False, "error": "tasks are not available in this build"}
        # The name becomes a folder and a gallery label on the Mac: first word, letters only.
        name = re.sub(r"[^A-Za-z0-9_-]", "", (str(name).split() or [""])[0])[:32]
        if task == "enroll_face" and not name:
            # No name was said: nothing to enrol under. Shake the head, start nothing.
            jobs = self._gestures.play("no")
            return {"ok": False, "gestured": jobs is not None,
                    "error": "enroll_face needs the person's name and none was given; task not started"}
        if task == "explore":                # the explorer's mode rides in the name slot
            name = "continue" if str(mode).lower().strip() == "continue" else "auto"
        with self._lock:
            if not self.has_queued():
                self.pending = (task, name)
                self._history.clear()        # the task moves the robot: the old path no longer leads back
                return {"ok": True, "starting": task, "note": "session closing; it reopens when the task ends"}
            if len(self._cmd_q) >= MAX_COMMAND_QUEUE:
                return {"ok": False, "error": f"command queue full ({MAX_COMMAND_QUEUE} pending)"}
            ahead = len(self._cmd_q) + (1 if self._active_move is not None else 0) + (1 if self.pending is not None else 0)
            self._cmd_q.append(("task", task, name))
            return {"ok": True, "queued": True, "task": task, "queue_depth": len(self._cmd_q),
                    "note": f"queued after {ahead} pending command(s); session stays open until moves finish"}

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
        if self.has_queued() and not settings.GESTURE_TEXT_ONLY:
            return {"ok": True, "gestured": False, "note": "busy with queued commands"}
        # Gesturer drops the gesture if the robot is already moving: motion is
        # both the actuator and the display, and the actuator wins.
        jobs = self._gestures.play(v)
        if jobs is None:
            return {"ok": True, "gestured": False, "note": "busy moving"}
        return {"ok": True, "gestured": True, "value": v, "jobs": jobs}
