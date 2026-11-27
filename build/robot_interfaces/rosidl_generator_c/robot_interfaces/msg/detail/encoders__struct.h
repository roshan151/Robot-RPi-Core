// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/encoders.h"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
#define ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Encoders in the package robot_interfaces.
/**
  * Live quadrature counts. Signed and hardware-decoded: a wheel rolled backwards
  * down a slope counts DOWN. Direction is measured, not inferred from the
  * commanded move.
 */
typedef struct robot_interfaces__msg__Encoders
{
  int32_t left;
  int32_t right;
  /// left - right; a value pinned to one side across
  /// every move means the motor<->encoder pairing is
  /// crossed, not that the gain needs tuning.
  int32_t sync_error;
} robot_interfaces__msg__Encoders;

// Struct for a sequence of robot_interfaces__msg__Encoders.
typedef struct robot_interfaces__msg__Encoders__Sequence
{
  robot_interfaces__msg__Encoders * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__msg__Encoders__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
