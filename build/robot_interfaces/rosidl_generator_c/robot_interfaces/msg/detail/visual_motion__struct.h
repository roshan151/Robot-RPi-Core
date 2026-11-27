// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/VisualMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/visual_motion.h"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/VisualMotion in the package robot_interfaces.
/**
  * Camera-based "are we moving" — the third opinion MotionHealth uses to tell a
  * stall from a wheel spin. Only published while the head is settled, because a
  * slewing head moves the image without the robot moving.
 */
typedef struct robot_interfaces__msg__VisualMotion
{
  /// CLOCK_MONOTONIC seconds at capture (same host)
  double t_monotonic;
  bool moving;
  /// global image translation between frames (lores px)
  float shift_px;
  /// share of image cells that changed (0..1)
  float changed_fraction;
} robot_interfaces__msg__VisualMotion;

// Struct for a sequence of robot_interfaces__msg__VisualMotion.
typedef struct robot_interfaces__msg__VisualMotion__Sequence
{
  robot_interfaces__msg__VisualMotion * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__VisualMotion__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__STRUCT_H_
