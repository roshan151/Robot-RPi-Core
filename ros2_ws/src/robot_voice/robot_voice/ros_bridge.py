"""MotionBackend implemented over ROS actions.

Two event loops, one process
----------------------------
rclpy has its own executor loop; the Gemini Live session has an asyncio loop.
They run on different threads and must not block each other:

  * rclpy spins on a daemon thread (see voice_node.py).
  * asyncio owns the main thread.
  * **`_push()` is the only place either one touches the other**, and it does so
    through `call_soon_threadsafe`, which is the one asyncio call that is legal
    from a foreign thread.

The rule that falls out of that: never call `spin_until_future_complete` from
inside a tool handler. It blocks the asyncio loop, the websocket stops being
read, the microphone uplink stalls and the session dies — the same failure as
awaiting a drive, arriving through a different door. Everything here is
`send_goal_async` / `call_async` with callbacks.

Feedback
--------
Tools answer "queued" immediately, so the model would otherwise never learn that
its 3 m drive was halted at 0.4 m. Completed goals land on `self.results`, an
asyncio queue the agent drains and feeds back into the session as context. That
queue is what makes the agent closed-loop.
"""

from __future__ import annotations

import asyncio
import threading
from typing import Any, Dict, Optional

import time

from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_interfaces.action import Drive, Turn
from robot_interfaces.srv import ClearPath, Enroll, Look, StartBehavior

class RosMotion:
    """MotionBackend over /drive, /turn and /estop."""

    def __init__(self, node: Node, loop: asyncio.AbstractEventLoop) -> None:
        self._node = node
        self._loop = loop
        self._drive = ActionClient(node, Drive, "drive")
        self._turn = ActionClient(node, Turn, "turn")
        self._estop = node.create_client(Trigger, "estop")

        # Completed goals, for the agent to relay to the model.
        self.results: asyncio.Queue = asyncio.Queue()

        # Goals sent but not yet finished. This is what `busy()` reports, and
        # it is why gesture bursts get dropped rather than queued: the second
        # "yes" arrives while the first still has steps outstanding.
        self._outstanding = 0
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ #
    # Setup
    # ------------------------------------------------------------------ #

    def wait_for_servers(self, timeout_s: float = 30.0) -> bool:
        """Block until the drivetrain node is up.

        Called from the main thread before the session opens, never from inside
        a tool handler. Without it the first few commands would be sent into the
        void and silently dropped.
        """
        ok = self._drive.wait_for_server(timeout_sec=timeout_s)
        ok = self._turn.wait_for_server(timeout_sec=timeout_s) and ok
        ok = self._estop.wait_for_service(timeout_sec=timeout_s) and ok
        return ok

    def close(self) -> None:
        self._drive.destroy()
        self._turn.destroy()

    # ------------------------------------------------------------------ #
    # MotionBackend
    # ------------------------------------------------------------------ #

    def busy(self) -> bool:
        with self._lock:
            return self._outstanding > 0

    def drive(self, meters: float, *, gesture: bool = False) -> Dict[str, Any]:
        goal = Drive.Goal(meters=float(meters), gesture=gesture)
        return self._send(self._drive, goal, "drive", float(meters))

    def turn(self, degrees: float, *, gesture: bool = False) -> Dict[str, Any]:
        goal = Turn.Goal(degrees=float(degrees), gesture=gesture)
        return self._send(self._turn, goal, "turn", float(degrees))

    def stop(self) -> Dict[str, Any]:
        """Fire the e-stop and return. Deliberately does not await the reply.

        The service call reaches the drivetrain node's reentrant callback group
        and writes the brake to the serial port out of band. Waiting for the
        acknowledgement here would block the asyncio loop for the round trip,
        and there is nothing useful to do with the answer anyway — a stop that
        needs confirmation before it counts is not a stop.
        """
        self._estop.call_async(Trigger.Request())
        return {"ok": True, "stopped": True}

    # ------------------------------------------------------------------ #
    # Internals — everything below runs on the rclpy thread
    # ------------------------------------------------------------------ #

    def _send(self, client: ActionClient, goal: Any,
              op: str, value: float) -> Dict[str, Any]:
        if not client.server_is_ready():
            return {"ok": False, "error": f"{op}: drivetrain is not running"}
        with self._lock:
            self._outstanding += 1
        future = client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_response(f, op, value))
        return {"ok": True, "queued": True, "op": op, "value": value}

    def _on_response(self, future, op: str, value: float) -> None:
        try:
            handle = future.result()
        except Exception as e:                                # noqa: BLE001
            self._finish(op, value, f"send failed: {type(e).__name__}: {e}")
            return

        if not handle.accepted:
            # The drivetrain node refused it — over the distance limit, or a
            # gesture arriving while the robot was already busy. Either way the
            # model should hear about it rather than assume it is moving.
            self._finish(op, value, "rejected")
            return

        handle.get_result_async().add_done_callback(
            lambda f: self._on_result(f, op, value))

    def _on_result(self, future, op: str, value: float) -> None:
        try:
            status = future.result().result.status or "finished"
        except Exception as e:                                # noqa: BLE001
            status = f"error: {type(e).__name__}: {e}"
        self._finish(op, value, status)

    def _finish(self, op: str, value: float, status: str) -> None:
        with self._lock:
            self._outstanding = max(0, self._outstanding - 1)
        self._push({"op": op, "value": value, "status": status})

    def _push(self, event: Dict[str, Any]) -> None:
        """rclpy thread -> asyncio loop. The ONLY crossing point in the file."""
        try:
            self._loop.call_soon_threadsafe(self.results.put_nowait, event)
        except RuntimeError:
            pass            # loop already closed; the session is going down


class RosVision:
    """VisionBackend over the perception and behavior services.

    Same rules as RosMotion: every call is `call_async` with a callback, the
    tool returns at once, and the answer is pushed onto the SAME results queue
    the agent already drains — as {"text": ...}, one line of context. Behavior
    events (/vision/events: recognised, lost, reacquired, arrived, gave up)
    arrive the same way, unasked, which is what lets the model follow along
    without polling.
    """

    EVENT_REPEAT_S = 30.0      # "recognised roshan" once per 30 s, not per new track

    def __init__(self, node: Node, loop: asyncio.AbstractEventLoop,
                 results: asyncio.Queue) -> None:
        self._loop = loop
        self.results = results
        self._look = node.create_client(Look, "vision/look")
        self._clear = node.create_client(ClearPath, "vision/clear_path")
        self._enroll = node.create_client(Enroll, "vision/enroll")
        self._start = node.create_client(StartBehavior, "vision/start")
        self._stop = node.create_client(Trigger, "vision/stop_behavior")
        self._recent: Dict[str, float] = {}
        node.create_subscription(String, "vision/events", self._on_event, 10)

    # ---------------------------------------------------------- VisionBackend

    def look(self) -> Dict[str, Any]:
        return self._call(self._look, Look.Request(filter=""), "look",
                          lambda r: r.summary)

    def who(self) -> Dict[str, Any]:
        return self._call(self._look, Look.Request(filter="person"), "who",
                          lambda r: r.summary)

    def follow(self, name: Optional[str] = None) -> Dict[str, Any]:
        req = StartBehavior.Request(behavior="follow", target=name or "")
        return self._call(self._start, req, "follow", lambda r: r.message)

    def approach(self, what: str) -> Dict[str, Any]:
        req = StartBehavior.Request(behavior="approach", target=what)
        return self._call(self._start, req, "approach", lambda r: r.message)

    def look_at(self, what: str) -> Dict[str, Any]:
        req = StartBehavior.Request(behavior="look", target=what)
        return self._call(self._start, req, "look at", lambda r: r.message)

    def stop_following(self) -> Dict[str, Any]:
        if self._stop.service_is_ready():
            self._stop.call_async(Trigger.Request())
        return {"ok": True, "stopped": True}

    def clear_path(self) -> Dict[str, Any]:
        def fmt(r) -> str:
            if not r.ok or not r.known:
                return f"can't tell which way is clear: {r.summary}"
            text = r.summary
            if r.has_best and abs(r.best_bearing_deg) >= 5:
                text += f" (turn({r.best_bearing_deg:.0f}) would face it)"
            return text
        return self._call(self._clear, ClearPath.Request(mode="full"), "path", fmt)

    def remember_face(self, name: str) -> Dict[str, Any]:
        req = Enroll.Request(label=name, images=0)
        return self._call(self._enroll, req, "remember", lambda r: r.message)

    def close(self) -> None:
        pass

    # ------------------------------------------------------------- internals

    def _call(self, client, req, label: str, fmt) -> Dict[str, Any]:
        if not client.service_is_ready():
            return {"ok": False, "error": f"{label}: vision is not running"}
        fut = client.call_async(req)
        fut.add_done_callback(lambda f: self._on_done(f, label, fmt))
        return {"ok": True, "pending": True, "note": "the answer follows as a [robot] line"}

    def _on_done(self, fut, label: str, fmt) -> None:
        try:
            text = fmt(fut.result())
        except Exception as e:                                # noqa: BLE001
            text = f"failed: {type(e).__name__}: {e}"
        self._push(f"{label}: {text}")

    def _on_event(self, msg: String) -> None:
        now = time.monotonic()
        if now - self._recent.get(msg.data, -1e9) < self.EVENT_REPEAT_S:
            return
        self._recent[msg.data] = now
        self._push(msg.data)

    def _push(self, text: str) -> None:
        try:
            self._loop.call_soon_threadsafe(self.results.put_nowait, {"text": text})
        except RuntimeError:
            pass
