// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from robot_interfaces:msg/VisualMotion.idl
// generated code does not contain a copyright notice
#ifndef ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_interfaces/msg/detail/visual_motion__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
bool cdr_serialize_robot_interfaces__msg__VisualMotion(
  const robot_interfaces__msg__VisualMotion * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
bool cdr_deserialize_robot_interfaces__msg__VisualMotion(
  eprosima::fastcdr::Cdr &,
  robot_interfaces__msg__VisualMotion * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t get_serialized_size_robot_interfaces__msg__VisualMotion(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t max_serialized_size_robot_interfaces__msg__VisualMotion(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
bool cdr_serialize_key_robot_interfaces__msg__VisualMotion(
  const robot_interfaces__msg__VisualMotion * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t get_serialized_size_key_robot_interfaces__msg__VisualMotion(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
size_t max_serialized_size_key_robot_interfaces__msg__VisualMotion(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, msg, VisualMotion)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
