"""The drivetrain node's decisions, tested without ROS or a robot.

Goal acceptance and terminal-status mapping are the two places in the node where
a mistake is both easy and expensive, and neither can be exercised by any other
test: the hardware scripts need a robot, and the robot_core tests never see the
node. So we stub out enough of rclpy to import the module and then call the
decision methods directly.

Worth the stubbing for the third case in particular — a gesture is several goals
in a row, and the first one makes the robot busy. A naive "reject when busy"
rule silently truncates every gesture to its first step, and the symptom (the
robot nods once and stops shaking its head) looks like a firmware fault.
"""

from __future__ import annotations

import ast
import sys
import types
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "ros2_ws" / "src" / "robot_drivetrain"))


def _stub_ros() -> None:
    """Enough of rclpy and the generated interfaces to import the node."""
    if "rclpy" in sys.modules:
        return

    class _Enum:
        ACCEPT = "accept"
        REJECT = "reject"

    def module(name: str, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    module("rclpy", init=lambda **kw: None, shutdown=lambda: None)
    module("rclpy.node", Node=type("Node", (), {"__init__": lambda self, *a, **k: None}))
    module("rclpy.action", ActionServer=object, CancelResponse=_Enum,
           GoalResponse=_Enum)
    module("rclpy.callback_groups",
           MutuallyExclusiveCallbackGroup=object, ReentrantCallbackGroup=object)
    module("rclpy.executors", MultiThreadedExecutor=object)
    module("rcl_interfaces")
    module("rcl_interfaces.msg", SetParametersResult=object)
    module("std_srvs")
    module("std_srvs.srv", Trigger=object)
    module("robot_interfaces")
    module("robot_interfaces.action", Drive=object, Turn=object)
    module("robot_interfaces.msg", Encoders=object, MotionHealth=object, VisualMotion=object)
    module("geometry_msgs")
    module("geometry_msgs.msg", TransformStamped=object)
    module("nav_msgs")
    module("nav_msgs.msg", Odometry=object)
    module("tf2_ros", TransformBroadcaster=object)


_stub_ros()

from robot_drivetrain.drivetrain_node import DrivetrainNode  # noqa: E402
from rclpy.action import GoalResponse  # noqa: E402


class FakeExecutor:
    def __init__(self, moving=False, queue_depth=0, gesturing=False):
        self._status = {"moving": moving, "queue_depth": queue_depth,
                        "gesturing": gesturing}

    def status(self):
        return self._status


class FakeNode:
    """Just the two attributes the decision methods actually touch."""

    def __init__(self, executor, max_drive_m=5.0):
        self._exec = executor
        self._max = max_drive_m

    def get_parameter(self, name):
        assert name == "max_drive_m", name
        return types.SimpleNamespace(value=self._max)

    def get_logger(self):
        return types.SimpleNamespace(warn=lambda *_a, **_k: None,
                                     info=lambda *_a, **_k: None)

    # Bind the real implementations onto this stand-in.
    _busy = DrivetrainNode._busy
    _reject_gesture_if_busy = DrivetrainNode._reject_gesture_if_busy
    _accept_drive = DrivetrainNode._accept_drive
    _accept_turn = DrivetrainNode._accept_turn


def goal(meters=0.0, degrees=0.0, gesture=False):
    return types.SimpleNamespace(meters=meters, degrees=degrees, gesture=gesture)


# --------------------------------------------------------------------------- #
# The distance limit belongs to the actuator, not the caller
# --------------------------------------------------------------------------- #

def test_over_limit_drive_is_rejected() -> None:
    node = FakeNode(FakeExecutor(), max_drive_m=5.0)
    assert node._accept_drive(goal(meters=6.0)) == GoalResponse.REJECT
    assert node._accept_drive(goal(meters=-6.0)) == GoalResponse.REJECT, \
        "the limit must apply to reverse too"


def test_within_limit_drive_is_accepted() -> None:
    node = FakeNode(FakeExecutor(), max_drive_m=5.0)
    assert node._accept_drive(goal(meters=5.0)) == GoalResponse.ACCEPT
    assert node._accept_drive(goal(meters=-2.0)) == GoalResponse.ACCEPT


# --------------------------------------------------------------------------- #
# Gestures: dropped for real motion, but never truncated mid-sequence
# --------------------------------------------------------------------------- #

def test_gesture_rejected_while_really_moving() -> None:
    node = FakeNode(FakeExecutor(moving=True, gesturing=False))
    assert node._accept_drive(goal(meters=0.1, gesture=True)) == GoalResponse.REJECT
    assert node._accept_turn(goal(degrees=30, gesture=True)) == GoalResponse.REJECT


def test_gesture_continues_over_its_own_steps() -> None:
    """The subtle one. A gesture's first step makes the robot busy; its
    remaining steps must still get through, or every gesture is one step long."""
    node = FakeNode(FakeExecutor(moving=True, gesturing=True))
    assert node._accept_turn(goal(degrees=60, gesture=True)) == GoalResponse.ACCEPT
    assert node._accept_drive(goal(meters=0.1, gesture=True)) == GoalResponse.ACCEPT


def test_gesture_accepted_when_idle() -> None:
    node = FakeNode(FakeExecutor())
    assert node._accept_drive(goal(meters=0.1, gesture=True)) == GoalResponse.ACCEPT


def test_real_move_is_never_rejected_for_being_busy() -> None:
    """Real moves queue; only gestures are dropped. Losing a commanded move
    silently would be far worse than a late one."""
    node = FakeNode(FakeExecutor(moving=True, gesturing=False))
    assert node._accept_drive(goal(meters=1.0)) == GoalResponse.ACCEPT
    assert node._accept_turn(goal(degrees=90)) == GoalResponse.ACCEPT


def test_queued_work_counts_as_busy() -> None:
    node = FakeNode(FakeExecutor(moving=False, queue_depth=3, gesturing=False))
    assert node._busy() is True
    assert node._accept_drive(goal(meters=0.1, gesture=True)) == GoalResponse.REJECT


# --------------------------------------------------------------------------- #
# Source-level guards for the two things a refactor would quietly undo
# --------------------------------------------------------------------------- #

NODE_SRC = (REPO / "ros2_ws" / "src" / "robot_drivetrain"
            / "robot_drivetrain" / "drivetrain_node.py").read_text()


def test_estop_is_on_a_reentrant_callback_group() -> None:
    """Under a mutually-exclusive group the stop would queue behind the move it
    is meant to interrupt and arrive after it had already finished."""
    assert "ReentrantCallbackGroup" in NODE_SRC
    assert 'callback_group=safety' in NODE_SRC
    assert NODE_SRC.index("safety = ReentrantCallbackGroup()") < NODE_SRC.index(
        'self.create_service(Trigger, "estop"')


def test_external_stop_aborts_rather_than_cancels() -> None:
    """goal_handle.canceled() is only legal when the cancellation arrived
    through the action interface. An /estop never touches the goal handle, so
    that path has to abort — calling canceled() there raises."""
    assert "if goal_handle.is_cancel_requested:" in NODE_SRC
    assert "goal_handle.canceled()" in NODE_SRC
    assert "goal_handle.abort()" in NODE_SRC


@pytest.mark.parametrize("forbidden", ["spin_until_future_complete",
                                       "spin_once", "run_until_complete"])
def test_bridge_never_blocks_the_asyncio_loop(forbidden: str) -> None:
    """Checked against the AST, not the text — the file's own docstring warns
    about these by name, and a substring search would flag the warning."""
    src = (REPO / "ros2_ws" / "src" / "robot_voice"
           / "robot_voice" / "ros_bridge.py").read_text()
    called = {
        node.func.attr
        for node in ast.walk(ast.parse(src))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert forbidden not in called, (
        f"{forbidden}() in the bridge blocks the event loop; the microphone "
        "uplink stalls and the session dies")
