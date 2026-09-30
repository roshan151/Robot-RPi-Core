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

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from robot_core import settings
from robot_core.drivetrain import SerialDrivetrain
from robot_core.motion_executor import CANCELLED, DONE, MotionExecutor
from robot_core.odometry import DiffDriveOdometry, MotionHealth, effective_wheelbase_m
from robot_interfaces.action import Drive, Turn
from robot_interfaces.msg import Encoders, MotionHealth as MotionHealthMsg, VisualMotion

# How often a blocked move callback wakes to check whether it has been
# cancelled. Small enough to be responsive, large enough not to spin.
_POLL_S = 0.1


class DrivetrainNode(Node):
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
        self._health = MotionHealth()
        self._drivetrain.add_odometry_listener(
            lambda l, r, ms: self._odo.update(l, r, ms, time.monotonic()))

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

        self._encoders = self.create_publisher(Encoders, "encoders", 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._health_pub = self.create_publisher(MotionHealthMsg, "motion_health", 10)
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

        self.get_logger().info(
            f"drivetrain ready on {self.get_parameter('serial_port').value} "
            f"(limit {self.get_parameter('max_drive_m').value} m)")

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

    # ------------------------------------------------------------------ #
    # Goal acceptance — where the limits are actually enforced
    # ------------------------------------------------------------------ #

    def _busy(self) -> bool:
        status = self._exec.status()
        return bool(status["moving"] or status["queue_depth"])

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
        job_id = self._exec.submit(op, value, gesture=goal_handle.request.gesture)
        timeout = float(self.get_parameter("move_timeout_s").value)
        deadline = self.get_clock().now().nanoseconds / 1e9 + timeout + 1.0

        event = None
        while event is None:
            if goal_handle.is_cancel_requested:
                self._exec.cancel_all("action cancel")
            event = self._exec.wait_for(job_id, timeout=_POLL_S)
            if event is None:
                self._publish_feedback(goal_handle, action_type)
                if self.get_clock().now().nanoseconds / 1e9 > deadline:
                    break

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
        rep = self._health.update(st, op, now)
        self._health_pub.publish(MotionHealthMsg(
            state=rep.state, reason=rep.reason, v=rep.v, w=rep.w,
            sigma_xy=rep.sigma_xy, sigma_theta=rep.sigma_theta,
            drift_deg_per_m=rep.drift_deg_per_m,
            visual_known=rep.visual_moving is not None,
            visual_moving=bool(rep.visual_moving), since_s=rep.since_s))
        if rep.state != getattr(self, "_last_health", None):
            self._last_health = rep.state
            if rep.stuck:
                self.get_logger().warn(f"motion health: {rep.state} — {rep.reason}")
                if self.get_parameter("stop_on_stuck").value:
                    self._exec.cancel_all(f"motion health: {rep.state}")

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
