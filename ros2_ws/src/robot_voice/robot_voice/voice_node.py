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
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_core import robot_log, settings, speech, status, task_cancel
from robot_core.live.agent import run_live_agent
from robot_core.head_look import HeadLook
from robot_display.link import connect
from robot_voice.ros_bridge import RosMotion


def _explore() -> bool:
    """Run explore mode as a child process and wait for it. The drivetrain is
    already up (command mode), so this is just the explorer node; it exits by
    itself when the mission ends, and non-zero if it could not run. A cancelled
    task asks it to stop (explore/stop: after the current step, map saved)."""
    params = os.path.join(get_package_share_directory("robot_bringup"), "config", "robot.yaml")
    proc = subprocess.Popen(["ros2", "run", "robot_explore", "explorer_node",
                             "--ros-args", "--params-file", params])
    try:
        asked = False
        while proc.poll() is None:
            if task_cancel.requested() and not asked and _explore_stop is not None:
                from std_srvs.srv import Trigger
                _explore_stop.call_async(Trigger.Request())
                asked = True
            time.sleep(0.2)
        return proc.returncode == 0
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
_motion = None                               # RosMotion, set in main(); follow drives with it
_explore_stop = None                         # explore/stop client, set in main()


def run_task(task: str, name: str) -> bool:
    status.task(task)
    try:
        return _run_task(task, name)
    finally:
        status.task("voice")


def _run_task(task: str, name: str) -> bool:
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
    # The head is the explorer's hardware: same camera, channels and trims, read
    # from robot.yaml.
    with open(os.path.join(get_package_share_directory("robot_bringup"), "config", "robot.yaml")) as f:
        p = yaml.safe_load(f)["/explorer"]["ros__parameters"]
    camera = Camera(main_size=tuple(p.get("face_camera_size", (640, 480))),
                    inverted=bool(p.get("camera_inverted", True)),
                    backend=p.get("camera_backend", "auto"), index=int(p.get("camera_index", 0)))
    gimbal = head = lidar = None
    try:
        # Tilt is limited to 0..+45 here, so it only ever looks up from its base
        # position. No PWM (overlay missing)? Carry on with a fixed camera.
        try:
            gimbal = from_params(p, tilt_min=0.0)
            if task == "follow":             # TF-Luna gives follow a range; it works without one
                try:
                    from robot_core.sensors.tfluna import TFLuna
                    lidar = TFLuna(p["tfluna_port"])
                except Exception as e:                          # noqa: BLE001
                    robot_log.event("task.error", logging.WARNING, err=f"tfluna unavailable: {e}")
            head = Head(gimbal, lidar, camera, HeadGeometry(
                height_m=float(p.get("head_height_m", 0.25)), hfov_deg=float(p.get("hfov_deg", 66.0))))
        except Exception as e:                                  # noqa: BLE001
            robot_log.event("task.error", logging.WARNING, err=f"head unavailable: {type(e).__name__}: {e}")
        vision = VisionClient()
        if task == "enroll_face":
            return face_tasks.enroll(camera, vision, name, head=head)
        if task == "follow":
            from robot_core import follow
            return follow.follow(camera, vision, _motion, name, head=head)
        return face_tasks.match(camera, vision, head=head) is not None
    finally:
        if gimbal:
            gimbal.close()                   # parks the head at its start position
        if lidar:
            lidar.close()
        camera.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("voice")

    log_path = robot_log.setup(None)
    robot_log.event("session.start", mode="ros", pid=os.getpid(),
                    log=str(log_path))
    connect(node)                        # after setup(): status + warnings go to the OLED
    status.task("voice")

    # rclpy on its own thread so the asyncio loop below never waits on it.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True,
                                   name="rclpy-spin")
    spin_thread.start()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    motion = RosMotion(node, loop)
    global _motion, _explore_stop
    _motion = motion

    # Cancel the running task without the voice session (no internet needed):
    #   ros2 service call /task/stop std_srvs/srv/Trigger
    # The cancel listener does the same when you say "stop" during a task.
    from std_srvs.srv import Trigger

    def _on_task_stop(_req, resp):
        task_cancel.request("service")
        robot_log.event("task.cancel", by="service")
        motion.stop()
        resp.success, resp.message = True, "cancelling the running task (if any) and braking"
        return resp

    node.create_service(Trigger, "task/stop", _on_task_stop)
    _explore_stop = node.create_client(Trigger, "explore/stop")

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
