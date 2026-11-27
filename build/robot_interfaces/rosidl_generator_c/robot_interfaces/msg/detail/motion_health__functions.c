// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/MotionHealth.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/motion_health__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `state`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_interfaces__msg__MotionHealth__init(robot_interfaces__msg__MotionHealth * msg)
{
  if (!msg) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__init(&msg->state)) {
    robot_interfaces__msg__MotionHealth__fini(msg);
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    robot_interfaces__msg__MotionHealth__fini(msg);
    return false;
  }
  // v
  // w
  // sigma_xy
  // sigma_theta
  // drift_deg_per_m
  // visual_known
  // visual_moving
  // since_s
  // imu_known
  // pitch_deg
  // roll_deg
  return true;
}

void
robot_interfaces__msg__MotionHealth__fini(robot_interfaces__msg__MotionHealth * msg)
{
  if (!msg) {
    return;
  }
  // state
  rosidl_runtime_c__String__fini(&msg->state);
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
  // v
  // w
  // sigma_xy
  // sigma_theta
  // drift_deg_per_m
  // visual_known
  // visual_moving
  // since_s
  // imu_known
  // pitch_deg
  // roll_deg
}

bool
robot_interfaces__msg__MotionHealth__are_equal(const robot_interfaces__msg__MotionHealth * lhs, const robot_interfaces__msg__MotionHealth * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->state), &(rhs->state)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  // v
  if (lhs->v != rhs->v) {
    return false;
  }
  // w
  if (lhs->w != rhs->w) {
    return false;
  }
  // sigma_xy
  if (lhs->sigma_xy != rhs->sigma_xy) {
    return false;
  }
  // sigma_theta
  if (lhs->sigma_theta != rhs->sigma_theta) {
    return false;
  }
  // drift_deg_per_m
  if (lhs->drift_deg_per_m != rhs->drift_deg_per_m) {
    return false;
  }
  // visual_known
  if (lhs->visual_known != rhs->visual_known) {
    return false;
  }
  // visual_moving
  if (lhs->visual_moving != rhs->visual_moving) {
    return false;
  }
  // since_s
  if (lhs->since_s != rhs->since_s) {
    return false;
  }
  // imu_known
  if (lhs->imu_known != rhs->imu_known) {
    return false;
  }
  // pitch_deg
  if (lhs->pitch_deg != rhs->pitch_deg) {
    return false;
  }
  // roll_deg
  if (lhs->roll_deg != rhs->roll_deg) {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__MotionHealth__copy(
  const robot_interfaces__msg__MotionHealth * input,
  robot_interfaces__msg__MotionHealth * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  if (!rosidl_runtime_c__String__copy(
      &(input->state), &(output->state)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  // v
  output->v = input->v;
  // w
  output->w = input->w;
  // sigma_xy
  output->sigma_xy = input->sigma_xy;
  // sigma_theta
  output->sigma_theta = input->sigma_theta;
  // drift_deg_per_m
  output->drift_deg_per_m = input->drift_deg_per_m;
  // visual_known
  output->visual_known = input->visual_known;
  // visual_moving
  output->visual_moving = input->visual_moving;
  // since_s
  output->since_s = input->since_s;
  // imu_known
  output->imu_known = input->imu_known;
  // pitch_deg
  output->pitch_deg = input->pitch_deg;
  // roll_deg
  output->roll_deg = input->roll_deg;
  return true;
}

robot_interfaces__msg__MotionHealth *
robot_interfaces__msg__MotionHealth__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__MotionHealth * msg = (robot_interfaces__msg__MotionHealth *)allocator.allocate(sizeof(robot_interfaces__msg__MotionHealth), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__MotionHealth));
  bool success = robot_interfaces__msg__MotionHealth__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__MotionHealth__destroy(robot_interfaces__msg__MotionHealth * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__MotionHealth__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__MotionHealth__Sequence__init(robot_interfaces__msg__MotionHealth__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__MotionHealth * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__MotionHealth *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__MotionHealth), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__MotionHealth__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__MotionHealth__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_interfaces__msg__MotionHealth__Sequence__fini(robot_interfaces__msg__MotionHealth__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_interfaces__msg__MotionHealth__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_interfaces__msg__MotionHealth__Sequence *
robot_interfaces__msg__MotionHealth__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__MotionHealth__Sequence * array = (robot_interfaces__msg__MotionHealth__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__MotionHealth__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__MotionHealth__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__MotionHealth__Sequence__destroy(robot_interfaces__msg__MotionHealth__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__MotionHealth__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__MotionHealth__Sequence__are_equal(const robot_interfaces__msg__MotionHealth__Sequence * lhs, const robot_interfaces__msg__MotionHealth__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__MotionHealth__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__MotionHealth__Sequence__copy(
  const robot_interfaces__msg__MotionHealth__Sequence * input,
  robot_interfaces__msg__MotionHealth__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__MotionHealth);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__MotionHealth * data =
      (robot_interfaces__msg__MotionHealth *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__MotionHealth__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__MotionHealth__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__MotionHealth__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
