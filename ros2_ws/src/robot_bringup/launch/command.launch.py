"""Command mode: the robot listens and does what it is told.

    ros2 launch robot_bringup command.launch.py

Two processes, and the split is a safety property rather than tidiness. The
voice node holds an always-open microphone and an asyncio loop; the robot
process holds the serial link and the e-stop. Separate processes means separate
GILs, so a stall in the conversation cannot delay a stop.

`respawn=True` on the voice node gives supervision for free — a dropped session
restarts without taking the drivetrain with it.

Explore mode (SLAM + plants, no audio) is explore.launch.py. Neither mode
should know the other exists.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("robot_bringup"), "config", "robot.yaml")

    params = LaunchConfiguration("params")
    voice = LaunchConfiguration("voice")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params", default_value=default_params,
            description="Parameter file. Copy robot.yaml to tune without "
                        "editing the installed one."),
        DeclareLaunchArgument(
            "voice", default_value="true",
            description="Set false to bring up the drivetrain alone — useful "
                        "for `ros2 action send_goal` testing without a "
                        "microphone or an API key."),

        Node(
            package="robot_bringup",
            executable="bringup",
            name="robot",
            parameters=[params],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="robot_voice",
            executable="voice_node",
            name="voice",
            parameters=[params],
            output="screen",
            emulate_tty=True,
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(voice),
        ),
    ])
