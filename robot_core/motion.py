"""What the tools call to make the robot move.

RobotTools must not know whether a move goes straight to a MotionExecutor in
this process or out over ROS actions to another one.  Both backends obey the
same two rules, and both are correctness requirements rather than style:

  * **Nothing blocks.**  Tool calls are dispatched from inside the Live
    session's receive loop.  Awaiting a 1.4 s drive there stops the websocket
    being read for 1.4 s: the receive side stops draining, back-pressure
    reaches the send side, the microphone uplink stalls, audio is dropped, and
    the session dies.
  * **stop() is immediate and out of band.**  A stop that waits its turn behind
    the moves it is meant to cancel is not a stop.

`busy()` exists so gestures can be *dropped* rather than queued — the robot
should not nod while it is driving somewhere.  Motion is both the actuator and
the display, and the actuator wins.

Distances and angles are signed here, matching the tool vocabulary the model
sees: positive metres drive forward, positive degrees turn right.  The backend
converts to the executor's unsigned op names.
"""

from __future__ import annotations

from typing import Any, Dict, Protocol

from robot_core.motion_executor import MotionExecutor


class MotionBackend(Protocol):
    """The five things RobotTools needs. Implemented by LocalMotion and, in
    the ROS build, by robot_voice.ros_bridge.RosMotion."""

    def drive(self, meters: float, *, gesture: bool = False) -> Dict[str, Any]: ...
    def turn(self, degrees: float, *, gesture: bool = False) -> Dict[str, Any]: ...
    def stop(self) -> Dict[str, Any]: ...
    def busy(self) -> bool: ...
    def close(self) -> None: ...


class LocalMotion:
    """MotionBackend over a MotionExecutor in this process. No ROS involved.

    This is what you get when running the agent without the ROS graph — useful
    for bench testing, and it keeps robot_core independently runnable.
    """

    def __init__(self, drivetrain: Any) -> None:
        self._exec = MotionExecutor(drivetrain).start()

    def close(self) -> None:
        self._exec.close()

    @property
    def executor(self) -> MotionExecutor:
        return self._exec

    def busy(self) -> bool:
        status = self._exec.status()
        return bool(status["moving"] or status["queue_depth"])

    def drive(self, meters: float, *, gesture: bool = False) -> Dict[str, Any]:
        op = "straight" if meters > 0 else "reverse"
        job = self._exec.submit(op, abs(meters), gesture=gesture)
        return self._queued(job, meters=meters)

    def turn(self, degrees: float, *, gesture: bool = False) -> Dict[str, Any]:
        op = "right" if degrees > 0 else "left"
        job = self._exec.submit(op, abs(degrees), gesture=gesture)
        return self._queued(job, degrees=degrees)

    def stop(self) -> Dict[str, Any]:
        # cancel_all() empties the queue AND writes the brake straight to the
        # serial port, bypassing the command lock the running move is holding.
        return {"ok": True, "stopped": True,
                "cancelled": self._exec.cancel_all("voice: stop")}

    def _queued(self, job_id: int, **extra: Any) -> Dict[str, Any]:
        return {"ok": True, "queued": job_id,
                "queue_depth": self._exec.status()["queue_depth"], **extra}
