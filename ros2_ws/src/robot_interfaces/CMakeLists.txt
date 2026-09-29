cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# Interface packages have to be ament_cmake — message generation is a CMake
# step. This is the ONLY package in the tree that actually compiles, and the
# only one you need to `colcon build` after an edit; the Python packages are
# symlinked by --symlink-install.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Drive.action"
  "action/Turn.action"
  "msg/Encoders.msg"
  # vision (VISION-DESIGN.md)
  "msg/Detection.msg"
  "msg/Detections.msg"
  "msg/TrackedObject.msg"
  "msg/Tracks.msg"
  "msg/PerceptionStatus.msg"
  "msg/GimbalCommand.msg"
  "msg/GimbalState.msg"
  "msg/BehaviorStatus.msg"
  "srv/Look.srv"
  "srv/ClearPath.srv"
  "srv/Enroll.srv"
  "srv/StartBehavior.srv"
  "srv/Aim.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
