"""Layer A — the robot, as plain Python.

Nothing in this package imports ROS. That is the invariant the whole rebuild
rests on, and it is checked in CI:

    grep -r "import rclpy" robot_core/     # must return nothing

Three things follow from it. The tests run on a laptop with no ROS installed.
"Lightweight" is enforced structurally, because a ROS node that grew real logic
would have to import from here anyway. And the migration stays reversible —
delete ros2_ws/ and there is still a working robot.
"""
