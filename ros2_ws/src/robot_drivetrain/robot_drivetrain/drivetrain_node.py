"""The only thing in the system that talks to the Arduino.

What this node is, and is not
-----------------------------
It is a translation layer. ROS goals in, `robot_core` calls out. It holds no
serial logic, no protocol knowledge and no threading of its own — all of that
lives one layer down in `robot_core/`, where it can be tested without ROS.

Two callback lanes, and the split is load-bearing
-------------------------------------------------
`/drive` and `/turn` execute on a MutuallyExclusive group: one move at a time,
which is what a single serial link and a single drivetrain actually allow.

`/estop` sits on a **Reentrant** group so it can run *while* a move callback is
blocked inside `wait_for()`. Under the default SingleThreadedExecutor the stop
would queue behind the move it is meant to interrupt and arrive after it had
already finished — which is exactly the bug the old VisionGuardian comment
documents, one layer up and wearing a different hat. The `MultiThreadedExecutor`
in robot_bringup is the other half of this; neither is optional.

Enforcement lives here
----------------------
The distance limit is checked in the goal callback, not in the agent's tool
handler. A limit the model can talk itself out of is not a limit — the actuator
has to be the one that says no.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from types import SimpleNamespace
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from robot_core import robot_log, settings, status, velocity
from robot_core.drivetrain import SerialDrivetrain
from robot_core.motion_executor import CANCELLED, DONE, FAILED, MotionExecutor
from robot_core.odometry import DiffDriveOdometry, MotionHealth, effective_wheelbase_m, turn_correction
from sensor_msgs.msg import Imu as ImuMsg
from robot_interfaces.action import Drive, Turn
from robot_interfaces.msg import Encoders, MotionHealth as MotionHealthMsg, VisualMotion

# How often a blocked move callback wakes to check whether it has been
# cancelled. Small enough to be responsive, large enough not to spin.
_POLL_S = 0.1


class _PyLog:
    """The node's logger, written through Python logging so its lines land in logs.json
    (robot_log). Anything but a plain message (once=, throttle_duration_sec=) stays on ROS's."""

    def __init__(self, ros) -> None:
        self._ros, self._py = ros, logging.getLogger("robot_drivetrain")

    def __getattr__(self, name):
        level = {"debug": logging.DEBUG, "info": logging.INFO, "warn": logging.WARNING,
                 "warning": logging.WARNING, "error": logging.ERROR, "fatal": logging.CRITICAL}.get(name)
        ros = getattr(self._ros, name)
        if level is None:
            return ros
        return lambda msg, **kw: ros(msg, **kw) if kw else self._py.log(level, msg)


class DrivetrainNode(Node):
    def get_logger(self):
        return _PyLog(super().get_logger())

    def _drop_note(self) -> None:
        """Only the TF-Luna watches for drops (not the ToF): without a fresh reading, or with the
        guard off, say so with the OLED's no-drop icon (renewed every 10 s while it lasts)."""
        cm, t = self._drivetrain.tfluna()
        if self._drop is None or cm is None or time.monotonic() - t > 2.0:
            status.no_drop(15.0)

    def __init__(self) -> None:
        super().__init__("drivetrain")

        self.declare_parameter("serial_port", settings.SERIAL_PORT)
        self.declare_parameter("baud", settings.BAUD_RATE)
        self.declare_parameter("max_drive_m", settings.MAX_DRIVE_METERS)
        self.declare_parameter("move_timeout_s", settings.MOVE_TIMEOUT_S)
        self.declare_parameter("ticks_per_cm", settings.TICKS_PER_CM)
        self.declare_parameter("ticks_per_degree", settings.TICKS_PER_DEGREE)
        self.declare_parameter("encoder_publish_hz", 10.0)
        # Odometry. wheelbase_m <= 0 derives the *effective* track width from
        # the two calibrations (see robot_core.odometry.effective_wheelbase_m).
        self.declare_parameter("wheelbase_m", 0.0)
        self.declare_parameter("odom_swap_sides", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        # Brake as soon as motion health says stalled/slipping, instead of
        # letting the firmware push a blocked wheel for its full 15 s timeout.
        self.declare_parameter("stop_on_stuck", True)
        # IMU (robot_core/sensors/imu.py; wiring in the README). With it the heading
        # comes from the gyro, turns are trimmed to the asked angle, and a tilt past
        # max_pitch_deg / max_roll_deg stops the move (stop_on_stuck).
        self.declare_parameter("imu_fitted", False)
        self.declare_parameter("imu_bus", 1)
        self.declare_parameter("imu_gyro_addr", 0x69)           # the BMI323
        self.declare_parameter("imu_axes", ["y", "x", "-z"])   # board axis along robot forward, left, up
        self.declare_parameter("imu_pitch_offset_deg", 0.0)     # pitch read on flat floor: taken off (calibrate.py imu)
        self.declare_parameter("turn_tolerance_deg", 2.0)
        self.declare_parameter("turn_trim", True)               # false: no correcting moves after a turn
        # Gyro-held moves (velocity mode + IMU): straight drives steered back to their start heading,
        # and a turn's last few degrees finished slowly against the gyro (see _held).
        self.declare_parameter("drive_heading_hold", True)
        self.declare_parameter("drive_held_mps", 0.18)          # straight-drive speed, held
        self.declare_parameter("drive_heading_gain", 3.0)       # rad/s of turn per rad of heading error
        # TF-Luna drop guard (firmware v7+, robot_core/drop_guard.py): brake a forward drive on a drop
        self.declare_parameter("drop_guard", False)
        self.declare_parameter("tfluna_floor_cm", 0.0)          # the floor's range at the mount angle; 0 = learn
        self.declare_parameter("drop_jump_cm", 6.0)
        self.declare_parameter("drop_low_cm", 6.0)
        self.declare_parameter("drop_misses", 3)
        self.declare_parameter("max_pitch_deg", 25.0)
        self.declare_parameter("max_roll_deg", 20.0)
        self.declare_parameter("max_dip_deg", 8.0)                # nose falling this far on a drive: stop
        # Velocity driving (/cmd_vel), firmware v6 only: robot_core/velocity.py.
        self.declare_parameter("velocity_timeout_s", 0.6)         # no /cmd_vel for this long: stop
        for k, v in vars(velocity.VelocityParams()).items():
            self.declare_parameter(f"velocity_{k}", float(v))

        self._apply_calibration()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._drivetrain = SerialDrivetrain(
            port=self.get_parameter("serial_port").value,
            baud=self.get_parameter("baud").value,
        )
        self._exec = MotionExecutor(self._drivetrain).start()

        self._odo = DiffDriveOdometry(
            ticks_per_m=settings.TICKS_PER_CM * 100.0,
            wheelbase_m=self._wheelbase(),
            swap_sides=bool(self.get_parameter("odom_swap_sides").value),
        )
        self._health = MotionHealth(
            max_pitch_deg=float(self.get_parameter("max_pitch_deg").value),
            max_roll_deg=float(self.get_parameter("max_roll_deg").value),
            max_dip_deg=float(self.get_parameter("max_dip_deg").value))
        self._imu = None
        if self.get_parameter("imu_fitted").value:
            try:
                from robot_core.sensors.imu import Imu
                self._imu = Imu(bus=int(self.get_parameter("imu_bus").value),
                                axes=list(self.get_parameter("imu_axes").value),
                                gyro_addr=int(self.get_parameter("imu_gyro_addr").value),
                                pitch_offset_deg=float(self.get_parameter("imu_pitch_offset_deg").value))
                if not self._imu.wait_calibrated(5.0):
                    self.get_logger().warn("imu: gyro not zeroed yet (was the robot moving?); "
                                           "it zeroes itself the next time it stands still")
            except Exception as e:                          # noqa: BLE001 — drive without it
                self.get_logger().error(f"imu unavailable ({type(e).__name__}: {e}); "
                                        "heading from the encoders, no turn trim, no tilt stop")
                self._imu = None
        self._drivetrain.add_odometry_listener(
            lambda l, r, ms: self._odo.update(l, r, ms, time.monotonic(), yaw=self._yaw_rad()))
        self._drop = self._drop_why = None
        self._drop_t = 0.0
        if self.get_parameter("drop_guard").value:
            from robot_core.drop_guard import DropGuard
            self._drop = DropGuard(float(self.get_parameter("tfluna_floor_cm").value),
                                   float(self.get_parameter("drop_jump_cm").value),
                                   float(self.get_parameter("drop_low_cm").value),
                                   int(self.get_parameter("drop_misses").value))
        self._wheel_mps = None                       # (left, right, host time) measured, for /cmd_vel
        self._last_totals = None
        self._drivetrain.add_odometry_listener(self._on_wheel_totals)

        moves = MutuallyExclusiveCallbackGroup()
        safety = ReentrantCallbackGroup()

        self._drive_server = ActionServer(
            self, Drive, "drive",
            goal_callback=self._accept_drive,
            cancel_callback=self._accept_cancel,
            execute_callback=self._execute_drive,
            callback_group=moves,
        )
        self._turn_server = ActionServer(
            self, Turn, "turn",
            goal_callback=self._accept_turn,
            cancel_callback=self._accept_cancel,
            execute_callback=self._execute_turn,
            callback_group=moves,
        )
        self.create_service(Trigger, "estop", self._on_estop,
                            callback_group=safety)
        self.create_timer(10.0, self._drop_note, callback_group=safety)

        self._encoders = self.create_publisher(Encoders, "encoders", 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._health_pub = self.create_publisher(MotionHealthMsg, "motion_health", 10)
        self._imu_pub = self.create_publisher(ImuMsg, "imu", 10)
        self._tf = TransformBroadcaster(self)
        self.create_subscription(
            VisualMotion, "visual_motion",
            lambda m: self._health.on_visual(m.moving, m.t_monotonic), 10,
            callback_group=safety)
        self.create_service(Trigger, "odom/relocalized", self._on_relocalized,
                            callback_group=safety)
        hz = max(0.1, float(self.get_parameter("encoder_publish_hz").value))
        self.create_timer(1.0 / hz, self._publish_encoders,
                          callback_group=safety)

        # Velocity driving: only with firmware that has the T frame.
        self._vel_cmd = (0.0, 0.0, 0.0)              # (v, w, host time)
        self._vel_on = False                         # the speed loop has the wheels
        self._vel_hold = False                       # stopped by /estop or motion health: wait for a 0 command
        self._vel_ctrl = None
        self._held_on = False                        # _held has the wheels (and the speed loop)
        if getattr(self._drivetrain, "supports_velocity", False):
            p = {n: self.get_parameter(f"velocity_{n}").value for n in vars(velocity.VelocityParams())}
            self._vel_ctrl = velocity.WheelSpeedController(
                self._wheelbase(), velocity.params_from({f"velocity_{k}": v for k, v in p.items()}))
            self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10, callback_group=safety)
            self.create_timer(0.1, self._vel_tick, callback_group=MutuallyExclusiveCallbackGroup())
        else:
            self.get_logger().info("firmware has no velocity mode (needs drv8871-v6-vel): /cmd_vel off")

        self.get_logger().info(
            f"drivetrain ready on {self.get_parameter('serial_port').value} "
            f"(limit {self.get_parameter('max_drive_m').value} m, "
            f"velocity {'on' if self._vel_ctrl else 'off'})")

    # ------------------------------------------------------------------ #
    # Calibration
    # ------------------------------------------------------------------ #

    def _apply_calibration(self) -> None:
        """Point robot_core's calibration at this node's parameters.

        settings.py resolves these from environment variables at import time.
        Re-pointing the two module globals makes robot.yaml the single place to
        edit them, and makes `ros2 param set` a real calibration workflow —
        drive, measure, adjust, drive again, no restart.
        """
        settings.TICKS_PER_CM = float(self.get_parameter("ticks_per_cm").value)
        settings.TICKS_PER_DEGREE = float(self.get_parameter("ticks_per_degree").value)
        if hasattr(self, "_odo"):
            self._odo.ticks_per_m = settings.TICKS_PER_CM * 100.0
            self._odo.wheelbase_m = self._wheelbase()

    def _wheelbase(self) -> float:
        wb = float(self.get_parameter("wheelbase_m").value)
        return wb if wb > 0 else effective_wheelbase_m(
            settings.TICKS_PER_CM, settings.TICKS_PER_DEGREE)

    def _on_set_parameters(self, params) -> SetParametersResult:
        # Refused mid-move: rescaling the tick target of a move already in
        # flight would silently change how far it goes.
        names = {p.name for p in params}
        if names & {"ticks_per_cm", "ticks_per_degree"} and self._exec.status()["moving"]:
            return SetParametersResult(
                successful=False,
                reason="cannot recalibrate while a move is running")
        for p in params:
            if p.name == "ticks_per_cm":
                settings.TICKS_PER_CM = float(p.value)
            elif p.name == "ticks_per_degree":
                settings.TICKS_PER_DEGREE = float(p.value)
        # Odometry scales follow the calibration. (Reading wheelbase_m here
        # would still see the old value mid-set, so it is applied from p.)
        self._odo.ticks_per_m = settings.TICKS_PER_CM * 100.0
        wb = next((float(p.value) for p in params if p.name == "wheelbase_m"),
                  float(self.get_parameter("wheelbase_m").value))
        self._odo.wheelbase_m = wb if wb > 0 else effective_wheelbase_m(
            settings.TICKS_PER_CM, settings.TICKS_PER_DEGREE)
        return SetParametersResult(successful=True)

    def _yaw_deg(self) -> Optional[float]:
        st = self._imu.state() if self._imu is not None else None
        return st.yaw_deg if st is not None else None

    def _yaw_rad(self) -> Optional[float]:
        yaw = self._yaw_deg()
        return None if yaw is None else math.radians(yaw)

    # ------------------------------------------------------------------ #
    # Goal acceptance — where the limits are actually enforced
    # ------------------------------------------------------------------ #

    def _busy(self) -> bool:
        status = self._exec.status()
        return bool(status["moving"] or status["queue_depth"] or self._held_on)

    def _reject_gesture_if_busy(self, goal) -> bool:
        """Gestures are dropped, never queued behind real motion.

        Queueing them turns a burst of answers into a twenty-second dance that
        outlives the conversation that prompted it, and the robot should not nod
        while it is driving somewhere. Deciding it *here* removes the
        check-then-act race the client-side version had.

        The subtlety: a gesture is several goals in a row, and the first one
        makes the robot busy. So "busy" alone would reject a gesture's own
        remaining steps. Accept when what is already running is *also* a
        gesture; reject only when real motion is in the way. Suppressing a
        second gesture on top of the first is the client's job — it can see that
        it has steps outstanding, which this node cannot.
        """
        if not goal.gesture:
            return False
        status = self._exec.status()
        if not (status["moving"] or status["queue_depth"]):
            return False                    # idle: start it
        return not status["gesturing"]      # busy: only yield to real motion

    def _accept_drive(self, goal) -> GoalResponse:
        limit = float(self.get_parameter("max_drive_m").value)
        if abs(goal.meters) > limit:
            self.get_logger().warn(
                f"rejected drive {goal.meters} m: over the {limit} m limit")
            return GoalResponse.REJECT
        if self._reject_gesture_if_busy(goal):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _accept_turn(self, goal) -> GoalResponse:
        if self._reject_gesture_if_busy(goal):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _accept_cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ #
    # Execution
    # ------------------------------------------------------------------ #

    def _execute_drive(self, goal_handle):
        request = goal_handle.request
        op = "straight" if request.meters > 0 else "reverse"
        return self._run(goal_handle, op, abs(request.meters), Drive)

    def _execute_turn(self, goal_handle):
        request = goal_handle.request
        op = "right" if request.degrees > 0 else "left"
        return self._run(goal_handle, op, abs(request.degrees), Turn)

    def _run(self, goal_handle, op: str, value: float, action_type):
        """Submit one move and wait it out, honouring cancellation.

        The wait is a poll rather than a single blocking call so the goal's own
        cancel request is noticed. It occupies one executor thread for the
        duration of the move — which is fine, and precisely why /estop lives on
        a different callback group.
        """
        yaw0 = self._yaw_deg()
        timeout = float(self.get_parameter("move_timeout_s").value)
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout + 1.0
        status.anim("tank", timeout + 2.0)        # OLED: the tank runs while the wheels do

        gesture = goal_handle.request.gesture
        if op in ("straight", "reverse") and not gesture and self._can_hold() \
                and self.get_parameter("drive_heading_hold").value:
            event = self._held(goal_handle, deadline, meters=value if op == "straight" else -value)
        else:
            event = self._wait(goal_handle, self._exec.submit(op, value, gesture=gesture), action_type, deadline)
        if (event is not None and event.status == DONE and yaw0 is not None
                and op in ("left", "right") and not gesture
                and self.get_parameter("turn_trim").value):
            goal = value if op == "left" else -value
            if self._can_hold():                 # the last degrees, slowly, against the gyro
                time.sleep(0.3)                  # let it stop rocking before reading
                if turn_correction(goal, self._yaw_deg() - yaw0,
                                   float(self.get_parameter("turn_tolerance_deg").value)) is not None:
                    fix = self._held(goal_handle, deadline, yaw_goal=yaw0 + goal)
                    event = event if fix.status == DONE else fix
            else:
                event = self._trim_turn(goal_handle, action_type, goal, yaw0, deadline, event)

        status.anim("")
        result = action_type.Result()
        counts = self._drivetrain.get_encoder_status()
        result.ticks_left = int(counts["motor1_count"])
        result.ticks_right = int(counts["motor2_count"])

        if event is None:
            result.status = "timeout"
            goal_handle.abort()
            return result

        result.duration_s = float(event.duration_s)

        if event.status == DONE:
            result.status = "done"
            goal_handle.succeed()
        elif event.status == CANCELLED:
            result.status = "cancelled"
            # canceled() is only legal when the cancellation came in through the
            # action interface. An /estop from the guardian or the operator
            # stops the same move without ever touching this goal handle, so
            # that path has to abort instead — calling canceled() there raises.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
        else:
            result.status = event.detail or "failed"
            goal_handle.abort()
        return result

    def _wait(self, goal_handle, job_id, action_type, deadline: float):
        """The job's final event, or None if it is still running at `deadline`."""
        while True:
            if goal_handle.is_cancel_requested:
                self._exec.cancel_all("action cancel")
            event = self._exec.wait_for(job_id, timeout=_POLL_S)
            if event is not None:
                return event
            self._publish_feedback(goal_handle, action_type)
            if self.get_clock().now().nanoseconds / 1e9 > deadline:
                return None

    def _trim_turn(self, goal_handle, action_type, goal_ccw_deg: float, yaw0: float,
                   deadline: float, event):
        """Tracks slip, so a turn of N encoder-degrees is not N degrees. The gyro
        knows how far the robot really turned: make up the difference with a short
        move, twice at most. Returns the event to report (the turn's own if all
        went well, else the failed trim's)."""
        tol = float(self.get_parameter("turn_tolerance_deg").value)
        for _ in range(2):
            time.sleep(0.3)                                  # let it stop rocking before reading
            fix = turn_correction(goal_ccw_deg, self._yaw_deg() - yaw0, tol)
            if fix is None:
                break
            self.get_logger().info(f"turn trim: {fix[0]} {fix[1]:.1f} deg (asked {goal_ccw_deg:+.0f} ccw)")
            trim = self._wait(goal_handle, self._exec.submit(fix[0], fix[1]), action_type, deadline)
            if trim is None or trim.status != DONE:
                return trim
        return event

    def _can_hold(self) -> bool:
        """Gyro-held moves need the speed loop (firmware -vel) and a gyro heading."""
        return self._vel_ctrl is not None and self._yaw_deg() is not None

    def _held(self, goal_handle, deadline: float, meters: float = 0.0, yaw_goal: Optional[float] = None):
        """Drive `meters` (signed) holding the heading it starts on, or (no meters) turn on the
        spot to `yaw_goal` (deg, CCW +), in velocity mode. Ten times a second the gyro's heading
        error becomes a turn rate, and the speed loop holds each track to its share: a track that
        slips or a weaker motor is steered out as it happens (counted moves only keep the tick
        counts equal). Distance comes from the encoders; it slows over the last few cm. Stops on a
        cancel, /estop, motion health or the drop guard (all set _vel_hold). Returns an event
        like the executor's."""
        ctrl, tpm = self._vel_ctrl, settings.TICKS_PER_CM * 100.0
        gain = float(self.get_parameter("drive_heading_gain").value)
        speed = min(float(self.get_parameter("drive_held_mps").value), ctrl.p.max_mps)
        tol = float(self.get_parameter("turn_tolerance_deg").value)
        target = self._yaw_deg() if yaw_goal is None else yaw_goal
        start, t0, err = self._last_totals, time.monotonic(), 0.0
        result, detail = DONE, ""
        ctrl.reset()
        self._vel_hold, self._held_on, self._vel_on = False, True, True
        try:
            while True:
                if goal_handle.is_cancel_requested:
                    result, detail = CANCELLED, "action cancel"
                    break
                if self._vel_hold:                   # /estop, motion health or the drop guard braked
                    result, detail = CANCELLED, "interrupted by stop"
                    break
                if self.get_clock().now().nanoseconds / 1e9 > deadline:
                    result, detail = FAILED, "timeout"
                    break
                yaw = self._yaw_deg()
                if yaw is None:
                    result, detail = FAILED, "gyro lost"
                    break
                err = target - yaw
                if meters:
                    tot = self._last_totals
                    gone = 0.0 if start is None or tot is None else \
                        (abs(tot[0] - start[0]) + abs(tot[1] - start[1])) / 2.0 / tpm
                    left = abs(meters) - gone
                    if left <= 0.005:
                        break
                    v = math.copysign(min(speed, max(0.05, 1.5 * left)), meters)
                    w = max(-0.6, min(0.6, gain * math.radians(err)))
                else:
                    if abs(err) <= tol:
                        break
                    v = 0.0
                    w = math.copysign(min(0.6, max(0.25, gain * math.radians(abs(err)))), err)
                meas = self._wheel_mps
                meas = meas[:2] if meas is not None and time.monotonic() - meas[2] < 0.3 else None
                self._drivetrain.set_wheels(*ctrl.step(v, w, meas, 0.1))
                time.sleep(0.1)
        except Exception as e:                       # noqa: BLE001 — T refused or link down
            result, detail = FAILED, f"{type(e).__name__}: {e}"
        finally:
            try:
                self._drivetrain.set_wheels(0, 0)
            except Exception:                        # noqa: BLE001
                pass
            ctrl.reset()
            self._held_on = self._vel_on = False
        dur = time.monotonic() - t0
        robot_log.event({DONE: "move.done", CANCELLED: "move.cancelled"}.get(result, "move.failed"),
                        op="held", val=round(meters, 3) if meters else round(target, 1), dur=round(dur, 3),
                        heading_err=round(err, 1), detail=detail or None)
        return SimpleNamespace(status=result, duration_s=dur, detail=detail)

    def _publish_feedback(self, goal_handle, action_type) -> None:
        counts = self._drivetrain.get_encoder_status()
        feedback = action_type.Feedback()
        feedback.ticks_left = int(counts["motor1_count"])
        feedback.ticks_right = int(counts["motor2_count"])
        goal_handle.publish_feedback(feedback)

    # ------------------------------------------------------------------ #
    # Safety and telemetry
    # ------------------------------------------------------------------ #

    def _on_estop(self, _request, response):
        """Empty the queue and interrupt the move already running.

        `cancel_all()` writes the brake straight to the serial port, bypassing
        the command lock the running move holds. This is the only stop that
        interrupts a move rather than following it.
        """
        self._vel_hold = self._vel_on or self._vel_hold   # a stop overrides /cmd_vel until it says 0
        dropped = self._exec.cancel_all("estop service")
        response.success = True
        response.message = f"stopped; cancelled {len(dropped)} job(s)"
        return response

    def _publish_encoders(self) -> None:
        counts = self._drivetrain.get_encoder_status()
        self._encoders.publish(Encoders(
            left=int(counts["motor1_count"]),
            right=int(counts["motor2_count"]),
            sync_error=int(counts["sync_error"]),
        ))
        self._publish_odometry()

    def _publish_odometry(self) -> None:
        st = self._odo.state()
        now = time.monotonic()
        if st.samples == 0 or now - st.t > 1.0:
            if not getattr(self, "_warned_no_odo", False):
                self._warned_no_odo = True
                self.get_logger().warn(
                    "no odometry telemetry — flash firmware v5 (drv8871-v5-odo); "
                    "/odom and /motion_health are not published until it arrives")
            return
        self._warned_no_odo = False
        stamp = self.get_clock().now().to_msg()
        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        qz, qw = math.sin(st.pose.theta / 2), math.cos(st.pose.theta / 2)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = st.pose.x
        msg.pose.pose.position.y = st.pose.y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = cov[7] = st.sigma_xy ** 2
        cov[35] = st.sigma_theta ** 2
        msg.pose.covariance = cov
        msg.twist.twist.linear.x = st.v
        msg.twist.twist.angular.z = st.w
        self._odom_pub.publish(msg)

        if self.get_parameter("publish_tf").value:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = odom_frame
            tf.child_frame_id = base_frame
            tf.transform.translation.x = st.pose.x
            tf.transform.translation.y = st.pose.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf.sendTransform(tf)

        status = self._exec.status()
        op = status["current"]["op"] if status["current"] else None
        if op is None and self._vel_on and self._vel_ctrl is not None:   # the speed loop is driving
            v, w = self._vel_ctrl.v, self._vel_ctrl.w
            op = ("straight" if v > 0.02 else "reverse" if v < -0.02 else
                  "left" if w > 0.05 else "right" if w < -0.05 else None)
        ims = self._imu.state() if self._imu is not None else None
        if self._imu is not None:
            self._imu.set_moving(op is not None)             # the gyro zeroes itself only at rest
        if ims is not None:
            self._publish_imu(ims, stamp, base_frame)
        rep = self._health.update(st, op, now, imu=ims)
        state, reason = rep.state, rep.reason
        if self._drop is not None:                         # TF-Luna drop guard, every tick (10 Hz)
            cm, t_cm = self._drivetrain.tfluna()
            cur = status["current"]
            forward = (op == "straight" and (cur is None or float(cur["value"]) > 0)) if op else False
            why = self._drop.update(cm, forward) if now - t_cm < 0.5 else None
            if why:
                self._drop_why, self._drop_t = why, now
                self.get_logger().warn(f"drop guard: {why}")
                self._vel_hold = self._vel_on or self._vel_hold
                self._exec.cancel_all(f"drop guard: {why}")
            if self._drop_why and now - self._drop_t < 2.0:   # say so for 2 s: follow/explore remember it
                state, reason = "drop", self._drop_why
        self._health_pub.publish(MotionHealthMsg(
            state=state, reason=reason, v=rep.v, w=rep.w,
            sigma_xy=rep.sigma_xy, sigma_theta=rep.sigma_theta,
            drift_deg_per_m=rep.drift_deg_per_m,
            visual_known=rep.visual_moving is not None,
            visual_moving=bool(rep.visual_moving), since_s=rep.since_s,
            imu_known=ims is not None, pitch_deg=rep.pitch_deg, roll_deg=rep.roll_deg))
        if rep.state != getattr(self, "_last_health", None):
            self._last_health = rep.state
            if rep.stuck:
                self.get_logger().warn(f"motion health: {rep.state} — {rep.reason}")
                if self.get_parameter("stop_on_stuck").value:
                    self._vel_hold = self._vel_on or self._vel_hold
                    self._exec.cancel_all(f"motion health: {rep.state}")

    def _publish_imu(self, ims, stamp, frame: str) -> None:
        """/imu in REP-103 axes: roll about forward, pitch about left (nose DOWN is
        positive there, the opposite of ImuState), yaw about up."""
        r, p, y = math.radians(ims.roll_deg), math.radians(-ims.pitch_deg), math.radians(ims.yaw_deg)
        cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2),
                                  math.cos(y / 2), math.sin(y / 2))
        msg = ImuMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.angular_velocity.z = math.radians(ims.yaw_rate_dps)
        msg.angular_velocity_covariance[0] = -1.0            # roll and pitch rates are not published
        msg.linear_acceleration_covariance[0] = -1.0         # nor is acceleration
        self._imu_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # Velocity driving (/cmd_vel -> T frames)
    # ------------------------------------------------------------------ #

    def _on_wheel_totals(self, total_left: int, total_right: int, fw_ms: int) -> None:
        """Each track's speed (m/s, physical left/right) from the never-reset totals."""
        if self.get_parameter("odom_swap_sides").value:
            total_left, total_right = total_right, total_left
        last, self._last_totals = self._last_totals, (total_left, total_right, fw_ms)
        if last is None or fw_ms <= last[2]:
            return
        dt, tpm = (fw_ms - last[2]) / 1000.0, settings.TICKS_PER_CM * 100.0
        self._wheel_mps = ((total_left - last[0]) / tpm / dt, (total_right - last[1]) / tpm / dt,
                           time.monotonic())

    def _on_cmd_vel(self, msg: Twist) -> None:
        v, w = float(msg.linear.x), float(msg.angular.z)
        if v == 0.0 and w == 0.0:
            self._vel_hold = False                   # a zero command clears a stop
        self._vel_cmd = (v, w, time.monotonic())

    def _vel_tick(self) -> None:
        """10 Hz: drive the tracks at the latest /cmd_vel, held by the speed loop. Not
        while a Drive/Turn goal runs, after a stop until /cmd_vel says 0, nor once
        /cmd_vel has gone quiet for velocity_timeout_s."""
        if self._held_on:                            # a gyro-held move is using the speed loop
            return
        ctrl, now = self._vel_ctrl, time.monotonic()
        v, w, t = self._vel_cmd
        if self._busy() or self._vel_hold:           # a goal has the wheels, or a stop braked them:
            ctrl.reset()                             # never ramp back out of it
            self._vel_on = False
            return
        if now - t > float(self.get_parameter("velocity_timeout_s").value):
            v = w = 0.0                              # gone quiet: ramp down and stop
        meas = self._wheel_mps
        meas = meas[:2] if meas is not None and now - meas[2] < 0.3 else None
        pl, pr = ctrl.step(v, w, meas, 0.1)
        if pl == 0 and pr == 0 and not self._vel_on:
            return
        try:
            self._drivetrain.set_wheels(pl, pr)
        except Exception as e:                       # noqa: BLE001 — refused (a move started) or link down
            self.get_logger().warn(f"velocity: T refused ({type(e).__name__}: {e})")
            ctrl.reset()
            self._vel_on = False
            return
        self._vel_on = not (pl == 0 and pr == 0)

    def _on_relocalized(self, _request, response):
        """SLAM fixed our pose: restart the drift budget."""
        self._odo.correct()
        response.success = True
        response.message = "odometry drift budget reset"
        return response

    def destroy_node(self) -> bool:
        try:
            self._exec.close()
            self._drivetrain.close()
            if self._imu is not None:
                self._imu.close()
        except Exception as e:                              # noqa: BLE001
            self.get_logger().error(f"shutdown: {type(e).__name__}: {e}")
        return super().destroy_node()


def main(args=None) -> None:
    """Standalone entry point. robot_bringup runs this node alongside the
    others in one process instead; this exists for `ros2 run` during Phase 1
    bring-up and for debugging one node in isolation."""
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init(args=args)
    node = DrivetrainNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
