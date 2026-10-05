"""Run the robot without ROS.

    python -m robot_core.run

This is the bench path and the fallback: everything except the node graph. It
is what makes the "Layer A is independently runnable" claim true rather than
aspirational, and it is worth keeping working — when the ROS build misbehaves,
being able to fall back one layer tells you which layer broke.

The ROS entry point is `ros2 launch robot_bringup command.launch.py`.
"""

from __future__ import annotations

import argparse
import logging
import os

from robot_core import battery, robot_log, settings, speech
from robot_core.drivetrain import SerialDrivetrain
from robot_core.live.agent import run_live_agent
from robot_core.motion import LocalMotion

# Registered once the backend exists, so the fatal path can brake.
_MOTION = None


def _emergency_brake(cause: str) -> None:
    """Last action before the process dies: get the motors off.

    Runs from sys.excepthook / threading.excepthook / the signal handler, so it
    must never raise — a failure here would mask the fault we are trying to
    report. The firmware's link watchdog is the backstop for SIGKILL, which no
    handler can intercept.
    """
    if _MOTION is None:
        return
    try:
        _MOTION.stop()
        robot_log.event("estop", logging.CRITICAL, reason=f"process dying: {cause}")
    except Exception as e:
        robot_log.event("estop", logging.CRITICAL, ok=False,
                        reason=f"process dying: {cause}",
                        err=f"{type(e).__name__}: {e}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Voice + drivetrain, without ROS")
    parser.add_argument("--log", default=None,
                        help="JSON Lines event log (default: settings.LOG_PATH)")
    parser.add_argument("--no-battery-announce", action="store_true",
                        help="Skip the spoken battery report at startup.")
    args = parser.parse_args()

    log_path = robot_log.setup(args.log)
    robot_log.install_crash_handlers(on_fatal=_emergency_brake)
    robot_log.event("session.start", mode="local", pid=os.getpid(),
                    port=settings.SERIAL_PORT,
                    speed_pct=settings.DEFAULT_SPEED_PERCENT, log=str(log_path))

    # Prime the static phrases before anything can need them. On a cache hit
    # this is three stat() calls; on a cold cache it is what makes the robot
    # able to announce its own failure later with the network down — which is
    # exactly the situation where it will be asked to.
    primed = speech.prime()
    if not all(primed.values()):
        robot_log.event("audio.error", logging.WARNING, stage="tts-prime",
                        primed=primed,
                        err="some static phrases are not cached",
                        fix="check GEMINI_API_KEY and network, then run "
                            "`python -m robot_core.speech --prime`")

    # Battery next, and deliberately before anything opens the microphone: this
    # is the one moment speech is unambiguously safe, because no capture stream
    # exists yet. It also gives the operator an audible "I booted".
    if not args.no_battery_announce:
        battery.announce()

    global _MOTION
    drivetrain = SerialDrivetrain()
    motion = LocalMotion(drivetrain)
    _MOTION = motion                      # arms the crash handler's brake

    try:
        run_live_agent(motion)
        robot_log.event("session.stop", reason="clean exit")
    finally:
        motion.close()
        drivetrain.close()


if __name__ == "__main__":
    main()
