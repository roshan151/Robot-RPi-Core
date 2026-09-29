"""Command mode: the robot listens and does what it is told.

    ros2 launch robot_bringup command.launch.py

Three processes, and the split is a safety property rather than tidiness. The
voice node holds an always-open microphone and an asyncio loop; the robot
process holds the serial link and the e-stop. Separate processes means separate
GILs, so a stall in the conversation cannot delay a stop.

The third, `perception`, owns the camera and the REST calls to the vision
service on the Mac mini; it is isolated for the same reason.

`respawn=True` on the voice node gives supervision for free — a dropped session
restarts without taking the drivetrain with it.

Scan mode (SLAM, no audio) gets its own launch file at Phase 7. Neither mode
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
    vision = LaunchConfiguration("vision")

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

        DeclareLaunchArgument(
            "vision", default_value="true",
            description="Set false to run without the camera / vision service. "
                        "The gimbal and behavior nodes still start (they idle)."),

        Node(
            package="robot_bringup",
            executable="bringup",
            # No `name=` here: it becomes a __node remap that renames EVERY node
            # in this process to the same name (and stops robot.yaml's
            # /drivetrain, /gimbal and /vision_behavior sections from applying).
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
        # Its own process: it owns the camera and makes REST calls to the
        # vision service, and network calls hang for seconds (PLAN 4.3).
        Node(
            package="robot_vision",
            executable="perception_node",
            name="perception",
            parameters=[params],
            output="screen",
            emulate_tty=True,
            respawn=True,
            respawn_delay=3.0,
            condition=IfCondition(vision),
        ),
    ])
