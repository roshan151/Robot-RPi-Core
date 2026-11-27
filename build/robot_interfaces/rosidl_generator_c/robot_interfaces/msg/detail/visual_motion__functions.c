// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_interfaces:msg/VisualMotion.idl
// generated code does not contain a copyright notice
#include "robot_interfaces/msg/detail/visual_motion__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_interfaces__msg__VisualMotion__init(robot_interfaces__msg__VisualMotion * msg)
{
  if (!msg) {
    return false;
  }
  // t_monotonic
  // moving
  // shift_px
  // changed_fraction
  return true;
}

void
robot_interfaces__msg__VisualMotion__fini(robot_interfaces__msg__VisualMotion * msg)
{
  if (!msg) {
    return;
  }
  // t_monotonic
  // moving
  // shift_px
  // changed_fraction
}

bool
robot_interfaces__msg__VisualMotion__are_equal(const robot_interfaces__msg__VisualMotion * lhs, const robot_interfaces__msg__VisualMotion * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // t_monotonic
  if (lhs->t_monotonic != rhs->t_monotonic) {
    return false;
  }
  // moving
  if (lhs->moving != rhs->moving) {
    return false;
  }
  // shift_px
  if (lhs->shift_px != rhs->shift_px) {
    return false;
  }
  // changed_fraction
  if (lhs->changed_fraction != rhs->changed_fraction) {
    return false;
  }
  return true;
}

bool
robot_interfaces__msg__VisualMotion__copy(
  const robot_interfaces__msg__VisualMotion * input,
  robot_interfaces__msg__VisualMotion * output)
{
  if (!input || !output) {
    return false;
  }
  // t_monotonic
  output->t_monotonic = input->t_monotonic;
  // moving
  output->moving = input->moving;
  // shift_px
  output->shift_px = input->shift_px;
  // changed_fraction
  output->changed_fraction = input->changed_fraction;
  return true;
}

robot_interfaces__msg__VisualMotion *
robot_interfaces__msg__VisualMotion__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__VisualMotion * msg = (robot_interfaces__msg__VisualMotion *)allocator.allocate(sizeof(robot_interfaces__msg__VisualMotion), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_interfaces__msg__VisualMotion));
  bool success = robot_interfaces__msg__VisualMotion__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_interfaces__msg__VisualMotion__destroy(robot_interfaces__msg__VisualMotion * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_interfaces__msg__VisualMotion__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_interfaces__msg__VisualMotion__Sequence__init(robot_interfaces__msg__VisualMotion__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__VisualMotion * data = NULL;

  if (size) {
    data = (robot_interfaces__msg__VisualMotion *)allocator.zero_allocate(size, sizeof(robot_interfaces__msg__VisualMotion), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_interfaces__msg__VisualMotion__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_interfaces__msg__VisualMotion__fini(&data[i - 1]);
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
robot_interfaces__msg__VisualMotion__Sequence__fini(robot_interfaces__msg__VisualMotion__Sequence * array)
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
      robot_interfaces__msg__VisualMotion__fini(&array->data[i]);
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

robot_interfaces__msg__VisualMotion__Sequence *
robot_interfaces__msg__VisualMotion__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_interfaces__msg__VisualMotion__Sequence * array = (robot_interfaces__msg__VisualMotion__Sequence *)allocator.allocate(sizeof(robot_interfaces__msg__VisualMotion__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_interfaces__msg__VisualMotion__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_interfaces__msg__VisualMotion__Sequence__destroy(robot_interfaces__msg__VisualMotion__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_interfaces__msg__VisualMotion__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_interfaces__msg__VisualMotion__Sequence__are_equal(const robot_interfaces__msg__VisualMotion__Sequence * lhs, const robot_interfaces__msg__VisualMotion__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_interfaces__msg__VisualMotion__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_interfaces__msg__VisualMotion__Sequence__copy(
  const robot_interfaces__msg__VisualMotion__Sequence * input,
  robot_interfaces__msg__VisualMotion__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_interfaces__msg__VisualMotion);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_interfaces__msg__VisualMotion * data =
      (robot_interfaces__msg__VisualMotion *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_interfaces__msg__VisualMotion__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_interfaces__msg__VisualMotion__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_interfaces__msg__VisualMotion__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
