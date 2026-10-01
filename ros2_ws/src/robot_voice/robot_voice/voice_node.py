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
import subprocess
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_core import robot_log, settings, speech
from robot_core.live.agent import run_live_agent
from robot_core.head_look import HeadLook
from robot_voice.ros_bridge import RosMotion


def _explore() -> bool:
    """Run explore mode as a child process and wait for it. The drivetrain is
    already up (command mode), so this is just the explorer node; it exits by
    itself when the mission ends, and non-zero if it could not run."""
    params = os.path.join(get_package_share_directory("robot_bringup"), "config", "robot.yaml")
    proc = subprocess.Popen(["ros2", "run", "robot_explore", "explorer_node",
                             "--ros-args", "--params-file", params])
    try:
        return proc.wait() == 0
    finally:
        if proc.poll() is None:              # we are being torn down mid-mission
            proc.terminate()
            proc.wait(timeout=20)


def _open_head():
    """The head for look_up / look_down: same channels and trims as the explorer's."""
    import yaml
    from robot_core.sensors.gimbal import from_params
    with open(os.path.join(get_package_share_directory("robot_bringup"), "config", "robot.yaml")) as f:
        return from_params(yaml.safe_load(f)["/explorer"]["ros__parameters"])


head_look = HeadLook(_open_head)


def run_task(task: str, name: str) -> bool:
    """What `run_task` asks for, run by the agent while the Live session is closed.

    The camera is opened for the job and released afterwards, so the explorer
    (which owns it during a mission) never finds it taken."""
    head_look.release()                      # the task opens the head itself
    if task == "explore":
        return _explore()
    import yaml
    from robot_core import face_tasks
    from robot_core.sensors.camera import Camera
    from robot_core.sensors.gimbal import from_params
    from robot_core.sensors.head import Head, HeadGeometry
    from robot_core.vision_client import VisionClient
    camera = Camera(main_size=(640, 480))
    gimbal = head = None
    try:
        # The head is the explorer's hardware: same channels and trims, read from
        # robot.yaml. Tilt is limited to 0..+45 here, so it only ever looks up from
        # its base position. No PWM (overlay missing)? Carry on with a fixed camera.
        try:
            with open(os.path.join(get_package_share_directory("robot_bringup"), "config", "robot.yaml")) as f:
                p = yaml.safe_load(f)["/explorer"]["ros__parameters"]
            gimbal = from_params(p, tilt_min=0.0)
            head = Head(gimbal, None, camera, HeadGeometry(hfov_deg=float(p.get("hfov_deg", 66.0))))
        except Exception as e:                                  # noqa: BLE001
            robot_log.event("task.error", logging.WARNING, err=f"head unavailable: {type(e).__name__}: {e}")
        vision = VisionClient()
        if task == "enroll_face":
            return face_tasks.enroll(camera, vision, name, head=head)
        return face_tasks.match(camera, vision, head=head) is not None
    finally:
        if gimbal:
            gimbal.close()                   # parks the head at its start position
        camera.close()


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
        run_live_agent(motion, run_task, head_look)
        robot_log.event("session.stop", reason="clean exit")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            motion.stop()          # brake before anything else goes away
        except Exception:          # noqa: BLE001
            pass
        head_look.release()        # parks the head
        motion.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    main()
