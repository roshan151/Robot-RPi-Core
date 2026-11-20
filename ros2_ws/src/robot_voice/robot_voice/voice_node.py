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
import math
import os
import re
import subprocess
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from robot_core import robot_log, settings, speech, status, task_cancel
from robot_core.reading import Latest
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
    import yaml
    with open(params) as f:
        ex = yaml.safe_load(f)["/explorer"]["ros__parameters"]
        if not (ex.get("tof_fitted", False) or ex.get("tfluna_fitted", True)):
            # No range sensor: no map, no plant ranging, no floor checks (it would drive blind).
            robot_log.event("task.error", logging.WARNING,
                            err="explore needs a range sensor (robot.yaml tof_fitted / tfluna_fitted are both false)")
            status.result("explore needs ToF")
            return False
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
_pitch = [None]                              # the robot's nose-up angle from /motion_health; None = no IMU
_odom = Latest("odom", frame="odom")         # (x, y, heading) from /odom, stamped: get() is None once stale
_bump_t = [0.0]                              # time.monotonic() of the last "bumped" in /motion_health
_twist = [None]                              # (v m/s, w rad/s) from /odom; w is the gyro's with the IMU
POSE_STALE_S = 1.0                           # /odom comes at 10 Hz: older than this, the pose is not known
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
    from robot_core.sensors.head import Head, HeadGeometry, parse_self_zones
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
        # position; follow with the ToF may also look down to the drive pose, to
        # check the floor. No PWM (overlay missing)? Carry on with a fixed camera.
        tof = task == "follow" and bool(p.get("tof_fitted", False))
        drive_tilt = float(p.get("drive_tilt_deg", -30.0))
        try:
            gimbal = from_params(p, tilt_min=min(0.0, drive_tilt) if tof else 0.0)
            if tof:
                try:
                    from robot_core.sensors import tof8x8
                    lidar = tof8x8.from_params(p)
                except Exception as e:                          # noqa: BLE001
                    robot_log.event("task.error", logging.WARNING, err=f"tof unavailable: {type(e).__name__}: {e}")
            elif task == "follow" and p.get("tfluna_fitted", True):   # gives follow a range; works without
                try:
                    from robot_core.sensors.tfluna import TFLuna
                    lidar = TFLuna(p["tfluna_port"])
                except Exception as e:                          # noqa: BLE001
                    robot_log.event("task.error", logging.WARNING, err=f"tfluna unavailable: {e}")
            from robot_core import terrain
            head = Head(gimbal, lidar, camera, HeadGeometry(
                height_m=float(p.get("head_height_m", 0.25)), hfov_deg=float(p.get("hfov_deg", 66.0)),
                pan_axis_x_m=float(p.get("pan_axis_x_m", 0.05)), drive_tilt_deg=drive_tilt,
                tof_perpendicular=bool(p.get("tof_perpendicular", True)),
                tilt_offset_deg=float(p.get("head_tilt_offset_deg", 0.0)),
                tof_pan_offset_deg=float(p.get("tof_pan_offset_deg", 0.0)),
                tof_tilt_offset_deg=float(p.get("tof_tilt_offset_deg", 0.0)),
                pan_axis_y_m=float(p.get("tof_from_right_m", 0.125)) - float(p.get("robot_width_m", 0.25)) / 2.0,
                self_zones=parse_self_zones(p.get("tof_self_zones"))),
                pitch=lambda: _pitch[0], terrain_params=terrain.params_from(p))
        except Exception as e:                                  # noqa: BLE001
            robot_log.event("task.error", logging.WARNING, err=f"head unavailable: {type(e).__name__}: {e}")
        # The same Mac as the explorer's (robot.yaml /explorer vision_url), else /etc/robot.env.
        vision = VisionClient(p["vision_url"]) if p.get("vision_url") else VisionClient()
        if task == "enroll_face":
            return face_tasks.enroll(camera, vision, name, head=head)
        if task == "follow":
            from robot_core import follow
            return follow.follow(camera, vision, _motion, name, head=head,
                                 pose=lambda: _odom.get(POSE_STALE_S),
                                 bumped=lambda: _bump_t[0], twist=lambda: _twist[0])
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
    # OpenCV takes ~4.5 s to import on the Pi: do it now, in the background, so the
    # first face task doesn't wait for it.
    threading.Thread(target=lambda: __import__("cv2"), name="cv2-import", daemon=True).start()
    if not node.declare_parameter("buds_check", True).value:   # robot.yaml /voice
        settings.MIC_MODE = "room"                            # camera mic only, no buds checks
    if not node.declare_parameter("live_resume", True).value:  # robot.yaml /voice
        settings.LIVE_RESUME = False
    if node.declare_parameter("gesture_text_only", False).value:  # robot.yaml /voice
        settings.GESTURE_TEXT_ONLY = True

    if not node.declare_parameter("tts", True).value:          # robot.yaml /voice
        settings.TTS_ENABLED = False

    debug = bool(node.declare_parameter("debug_mode", False).value)   # robot.yaml /voice
    log_path = robot_log.setup(None, debug=debug)
    robot_log.event("session.start", mode="ros", pid=os.getpid(),
                    log=str(log_path), debug=debug)
    if debug:                            # camera frames + ToF grids to the Mac (robot_core/debug_upload.py)
        import yaml
        from robot_core import debug_upload
        with open(os.path.join(get_package_share_directory("robot_bringup"), "config", "robot.yaml")) as f:
            ex = yaml.safe_load(f)["/explorer"]["ros__parameters"]
        debug_upload.configure(
            True, ex.get("vision_url") or settings.VISION_SERVICE_BASE_URL,
            geometry={"camera_hfov_deg": float(ex.get("hfov_deg", 66.0)), "tof_fov_deg": 60.0,
                      "tof_on": "gimbal"},
            extras=lambda: {"pose": _odom.get(POSE_STALE_S), "pitch_deg": _pitch[0]})
    connect(node)                        # after setup(): status + warnings go to the OLED
    status.task("voice")

    # A thread that dies (the rclpy spin thread included) says so in logs.json, not only the journal.
    def _thread_died(a) -> None:
        robot_log.event("fatal", logging.CRITICAL, cause="thread died",
                        thread=a.thread.name if a.thread else "?",
                        err=f"{a.exc_type.__name__}: {a.exc_value}", tb=robot_log._short_tb(a.exc_traceback))
    threading.excepthook = _thread_died

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

    # The robot's pitch (IMU, via the drivetrain node): ToF projection and climb detection in follow.
    from robot_interfaces.msg import MotionHealth

    bump_g = float(node.declare_parameter("follow_bump_g", 1.5).value)   # robot.yaml /voice
    _bump_log = [0.0]

    def _on_health(m) -> None:
        _pitch[0] = m.pitch_deg if m.imu_known else None
        # A drop, or the drivetrain braking a stuck move (stalled, slipping, climbing: "tilted"): follow
        # treats them as hitting something it did not see. A jolt ("bumped") only when it is well above
        # floor vibration: the tracks shake past the drivetrain's 0.8 g at full speed, which braked
        # follow every 2 s (Nov 19; the drivetrain itself reports jolts and does not stop on them).
        hit = m.state in ("drop", "stalled", "slipping", "tilted")
        if m.state == "bumped":
            g = re.search(r"jolt of ([0-9.]+) g", m.reason or "")
            hit = g is not None and float(g.group(1)) >= bump_g
        if m.state in ("bumped", "drop", "stalled", "slipping", "tilted"):
            now = time.monotonic()
            if hit:
                _bump_t[0] = now
            if now - _bump_log[0] > 1.0:                    # what fired, for the logs (not every tick)
                _bump_log[0] = now
                robot_log.event("follow.health", state=m.state, reason=m.reason, braked=hit)

    from rclpy.callback_groups import ReentrantCallbackGroup
    topics = ReentrantCallbackGroup()        # never held back by a slow callback elsewhere
    node.create_subscription(MotionHealth, "motion_health", _on_health, 10, callback_group=topics)

    # Where the robot is (encoders, gyro heading with the IMU): follow's floor memory and search hint.
    from nav_msgs.msg import Odometry

    _odom_warned, _odom_t0 = [False], time.monotonic()      # warned already? (and since when we have waited)

    def _on_odom(m) -> None:
        q = m.pose.pose.orientation
        _odom.set((m.pose.pose.position.x, m.pose.pose.position.y, 2.0 * math.atan2(q.z, q.w)))
        _twist[0] = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    node.create_subscription(Odometry, "odom", _on_odom, 10, callback_group=topics)

    def _odom_watch() -> None:               # follow and explore steer by this pose: say when it stops
        quiet = min(_odom.age(), time.monotonic() - _odom_t0)
        if quiet > 5.0 and not _odom_warned[0]:
            _odom_warned[0] = True
            robot_log.event("voice.diag", logging.WARNING,
                            err=f"no /odom for {quiet:.0f} s: follow has no pose (drivetrain telemetry?)")
        elif quiet <= 5.0:
            _odom_warned[0] = False

    node.create_timer(5.0, _odom_watch, callback_group=topics)
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
