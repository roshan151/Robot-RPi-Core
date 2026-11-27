// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/MotionHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/motion_health.h"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'state'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/MotionHealth in the package robot_interfaces.
/**
  * Is the robot actually doing what it was told? Published by drivetrain_node at
  * the encoder rate. See robot_core/odometry.py:MotionHealth for the logic.
 */
typedef struct robot_interfaces__msg__MotionHealth
{
  /// idle | moving | stalled | slipping | pushed | wrong_direction | tilted | bumped
  rosidl_runtime_c__String state;
  /// human-readable, empty when all is well
  rosidl_runtime_c__String reason;
  /// m/s from the encoders
  double v;
  /// rad/s from the encoders, CCW positive
  double w;
  /// 1-sigma position drift budget since the last SLAM fix, m
  double sigma_xy;
  /// 1-sigma heading drift budget, rad
  double sigma_theta;
  /// heading change per metre on straight moves (EWMA)
  double drift_deg_per_m;
  /// a fresh /visual_motion sample was available
  bool visual_known;
  /// ...and what it said
  bool visual_moving;
  /// how long `state` has held
  double since_s;
  /// an IMU is fitted and answering; the two angles below mean something
  bool imu_known;
  /// nose up positive
  double pitch_deg;
  /// right side down positive
  double roll_deg;
} robot_interfaces__msg__MotionHealth;

// Struct for a sequence of robot_interfaces__msg__MotionHealth.
typedef struct robot_interfaces__msg__MotionHealth__Sequence
{
  robot_interfaces__msg__MotionHealth * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__MotionHealth__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__STRUCT_H_
