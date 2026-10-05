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

from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from robot_interfaces.action import Drive, Turn

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
