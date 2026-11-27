// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

#include "robot_interfaces/msg/detail/encoders__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_interfaces
const rosidl_type_hash_t *
robot_interfaces__msg__Encoders__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7b, 0x72, 0xfb, 0x7b, 0x5e, 0x2e, 0xde, 0xd7,
      0xee, 0x07, 0xd9, 0x08, 0xc8, 0xde, 0xda, 0xb2,
      0x43, 0x60, 0x1b, 0xdf, 0x40, 0x2b, 0x99, 0x83,
      0x1b, 0xac, 0xa7, 0xd4, 0x78, 0xc1, 0x3f, 0xca,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_interfaces__msg__Encoders__TYPE_NAME[] = "robot_interfaces/msg/Encoders";

// Define type names, field names, and default values
static char robot_interfaces__msg__Encoders__FIELD_NAME__left[] = "left";
static char robot_interfaces__msg__Encoders__FIELD_NAME__right[] = "right";
static char robot_interfaces__msg__Encoders__FIELD_NAME__sync_error[] = "sync_error";

static rosidl_runtime_c__type_description__Field robot_interfaces__msg__Encoders__FIELDS[] = {
  {
    {robot_interfaces__msg__Encoders__FIELD_NAME__left, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__Encoders__FIELD_NAME__right, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__Encoders__FIELD_NAME__sync_error, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_interfaces__msg__Encoders__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_interfaces__msg__Encoders__TYPE_NAME, 29, 29},
      {robot_interfaces__msg__Encoders__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Live quadrature counts. Signed and hardware-decoded: a wheel rolled backwards\n"
  "# down a slope counts DOWN. Direction is measured, not inferred from the\n"
  "# commanded move.\n"
  "\n"
  "int32 left\n"
  "int32 right\n"
  "int32 sync_error            # left - right; a value pinned to one side across\n"
  "                            # every move means the motor<->encoder pairing is\n"
  "                            # crossed, not that the gain needs tuning.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_interfaces__msg__Encoders__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_interfaces__msg__Encoders__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 422, 422},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_interfaces__msg__Encoders__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_interfaces__msg__Encoders__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
