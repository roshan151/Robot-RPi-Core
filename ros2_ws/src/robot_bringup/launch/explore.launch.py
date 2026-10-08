"""Explore mode: map the house, find plants, photograph each from all sides.

    ros2 launch robot_bringup explore.launch.py
    ./start_robot.sh explore                      (same thing, sources ROS first)

Two processes, like command mode, minus the voice:

  robot     drivetrain (actions, e-stop, /odom, /motion_health)
  explorer  head (camera + TF-Luna + pan/tilt), the mission, map → odom

The Mac vision service must be running: SLAM, plant identity and the
image+text model all live there. The mission ends by itself when nothing
reachable is left unexplored (or after max_minutes), and when the explorer
exits, this whole launch shuts down — explore mode is a job, not a daemon.

Stop early (map is still saved):  ros2 service call /explore/stop std_srvs/srv/Trigger
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "robot.yaml")
    params = LaunchConfiguration("params")

    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params,
                              description="Parameter file (robot.yaml)."),
        Node(package="robot_display", executable="display_node", name="display",
             output="screen", emulate_tty=True, respawn=True, respawn_delay=5.0),
        Node(package="robot_bringup", executable="bringup", name="robot",
             parameters=[params], output="screen", emulate_tty=True),
        Node(package="robot_explore", executable="explorer_node", name="explorer",
             parameters=[params], output="screen", emulate_tty=True,
             on_exit=Shutdown(reason="explore mission finished")),
    ])
