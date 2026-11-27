// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_interfaces:msg/VisualMotion.idl
// generated code does not contain a copyright notice

#include "robot_interfaces/msg/detail/visual_motion__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_interfaces
const rosidl_type_hash_t *
robot_interfaces__msg__VisualMotion__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x00, 0x86, 0xf1, 0x46, 0x74, 0xef, 0x77, 0xee,
      0xe5, 0x48, 0xc9, 0x59, 0x3d, 0xc7, 0x61, 0xea,
      0x79, 0x2d, 0x25, 0x4c, 0x71, 0xca, 0x41, 0x6d,
      0x9c, 0x06, 0xac, 0xa2, 0xe1, 0x07, 0x8b, 0x48,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_interfaces__msg__VisualMotion__TYPE_NAME[] = "robot_interfaces/msg/VisualMotion";

// Define type names, field names, and default values
static char robot_interfaces__msg__VisualMotion__FIELD_NAME__t_monotonic[] = "t_monotonic";
static char robot_interfaces__msg__VisualMotion__FIELD_NAME__moving[] = "moving";
static char robot_interfaces__msg__VisualMotion__FIELD_NAME__shift_px[] = "shift_px";
static char robot_interfaces__msg__VisualMotion__FIELD_NAME__changed_fraction[] = "changed_fraction";

static rosidl_runtime_c__type_description__Field robot_interfaces__msg__VisualMotion__FIELDS[] = {
  {
    {robot_interfaces__msg__VisualMotion__FIELD_NAME__t_monotonic, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__VisualMotion__FIELD_NAME__moving, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__VisualMotion__FIELD_NAME__shift_px, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__VisualMotion__FIELD_NAME__changed_fraction, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_interfaces__msg__VisualMotion__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_interfaces__msg__VisualMotion__TYPE_NAME, 33, 33},
      {robot_interfaces__msg__VisualMotion__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Camera-based \"are we moving\" \\xe2\\x80\\x94 the third opinion MotionHealth uses to tell a\n"
  "# stall from a wheel spin. Only published while the head is settled, because a\n"
  "# slewing head moves the image without the robot moving.\n"
  "\n"
  "float64 t_monotonic         # CLOCK_MONOTONIC seconds at capture (same host)\n"
  "bool    moving\n"
  "float32 shift_px            # global image translation between frames (lores px)\n"
  "float32 changed_fraction    # share of image cells that changed (0..1)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_interfaces__msg__VisualMotion__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_interfaces__msg__VisualMotion__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 460, 460},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_interfaces__msg__VisualMotion__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_interfaces__msg__VisualMotion__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
