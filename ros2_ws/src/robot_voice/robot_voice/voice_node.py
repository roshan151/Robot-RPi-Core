"""The Gemini Live session, as a ROS node.

Runs in its own process, deliberately. It owns an asyncio loop and an always-open
microphone, and it is the biggest CPU user in the system; isolating it means a
stall here cannot delay a stop, and `respawn=True` in the launch file can restart
it without taking the drivetrain down.

Thread layout — three things, and getting them the wrong way round is the
classic failure:

    main thread     asyncio: the Live session, the microphone, the tools
    daemon thread   rclpy: the executor, action clients, service clients
    worker threads  rclpy callbacks (MultiThreadedExecutor)

The agent never learns any of this. It is handed a MotionBackend (RosMotion) and
calls drive/turn/stop on it exactly as it would call a local executor. That seam
is what let the whole Live session move onto ROS without editing a line of the
conversation logic.
"""

from __future__ import annotations

import asyncio
import logging
import os
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_core import robot_log, settings, speech
from robot_core.live.agent import run_live_agent
from robot_voice.ros_bridge import RosMotion, RosVision


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("voice")

    log_path = robot_log.setup(None)
    robot_log.event("session.start", mode="ros", pid=os.getpid(),
                    log=str(log_path))

    # rclpy on its own thread so the asyncio loop below never waits on it.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True,
                                   name="rclpy-spin")
    spin_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    motion = RosMotion(node, loop)
    # Camera tools share the motion results queue: one feedback channel.
    vision = RosVision(node, loop, motion.results)

    try:
        # Blocking, and correctly so: this runs before the session opens, on
        # the main thread, while the spin thread services the discovery. If the
        # drivetrain node is not up, the first commands would vanish silently.
        if not motion.wait_for_servers(timeout_s=30.0):
            robot_log.event("fatal", logging.CRITICAL,
                            cause="drivetrain node did not appear",
                            fix="start robot_bringup first, or check that the "
                                "Arduino serial port is correct in robot.yaml")
            node.get_logger().error("drivetrain not available; giving up")
            return

        # Prime the cached phrases before the microphone opens. On a cache hit
        # this is a few stat() calls; on a cold one it is what lets the robot
        # announce its own failure later with the network down — which is
        # exactly when it will be asked to.
        primed = speech.prime()
        if not all(primed.values()):
            robot_log.event("audio.error", logging.WARNING, stage="tts-prime",
                            primed=primed,
                            err="some static phrases are not cached")
            
        # strating session, stop all motion if queued via previous start
        motion.stop()
        run_live_agent(motion, vision)
        robot_log.event("session.stop", reason="clean exit")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            motion.stop()          # brake before anything else goes away
        except Exception:          # noqa: BLE001
            pass
        motion.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    main()
