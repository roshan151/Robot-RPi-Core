"""The cancel listener: a second, cancel-only Gemini Live session for while a task runs.

Every task (enroll, match, follow, explore) runs with the main Live session
closed, so nobody is listening. This session opens beside the task and has one
tool, `cancel_task`: no motion, head, gesture or task tools, so it cannot fight
the task for the wheels or the gimbal, and no move results are fed to it.

It reuses LiveAgent (microphone, uplink, server-side VAD, tool calls) with its
own prompt and without the "connected" chime. Its failures are its own: it
reconnects with backoff and never brakes or speaks, unlike the main session's
supervisor. The task/stop service is the fallback when the network is down.
"""

from __future__ import annotations

import asyncio
import logging
from typing import Any, Callable, Dict

from robot_core import robot_log, settings, task_cancel


def declarations() -> list:
    from google.genai import types  # type: ignore

    return [types.FunctionDeclaration(
        name="cancel_task",
        description=(
            "Stop the task the robot is doing (following someone, learning a face, "
            "exploring) and brake now. Call it the moment the operator wants that."
        ),
    )]


class CancelTools:
    """The listener's only tool. Shaped like RobotTools as far as LiveAgent looks:
    no pending job, and no motion backend, so no move results reach the model."""

    pending = None
    motion = None

    def __init__(self, stop: Callable[[], Any]) -> None:
        self._stop = stop

    async def dispatch(self, name: str, args: Dict[str, Any]) -> Dict[str, Any]:
        if name != "cancel_task":
            return {"ok": False, "error": "only cancel_task exists"}
        task_cancel.request("voice")
        robot_log.event("task.cancel", by="voice")
        try:
            self._stop()
        except Exception as e:                          # noqa: BLE001
            robot_log.event("task.error", logging.WARNING, err=f"cancel brake failed: {e}")
        return {"ok": True, "note": "task cancelled"}


async def watch(motion) -> None:
    """Listen until cancelled (by the agent, when the task ends)."""
    from robot_core.live.agent import LiveAgent, LiveConfigError   # agent imports us lazily

    tools = CancelTools(motion.stop)
    backoff = settings.LIVE_RECONNECT_BACKOFF_S
    while True:
        agent = LiveAgent(tools, system_prompt=settings.CANCEL_PROMPT,
                          tool_declarations=declarations, announce=False, role="cancel")
        try:
            await agent.run()
        except asyncio.CancelledError:
            raise
        except Exception as e:                          # noqa: BLE001
            robot_log.event("task.error", logging.WARNING,
                            err=f"cancel listener: {type(e).__name__}: {e}"[:300])
            if isinstance(e, LiveConfigError):
                return                                  # same config, same refusal: don't loop
        await asyncio.sleep(backoff)
        backoff = min(backoff * 2, settings.LIVE_RECONNECT_MAX_S)
