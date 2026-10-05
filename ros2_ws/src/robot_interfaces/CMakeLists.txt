cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# Interface packages have to be ament_cmake — message generation is a CMake
# step. This is the ONLY package in the tree that actually compiles, and the
# only one you need to `colcon build` after an edit; the Python packages are
# symlinked by --symlink-install.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Drive.action"
  "action/Turn.action"
  "msg/Encoders.msg"
  "msg/MotionHealth.msg"
  "msg/VisualMotion.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
