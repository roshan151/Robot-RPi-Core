// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_interfaces:msg/MotionHealth.idl
// generated code does not contain a copyright notice

#include "robot_interfaces/msg/detail/motion_health__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_interfaces
const rosidl_type_hash_t *
robot_interfaces__msg__MotionHealth__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x43, 0x72, 0x1f, 0x35, 0x30, 0x5a, 0x07, 0xd2,
      0x3f, 0x3d, 0xa1, 0x00, 0x9d, 0x0f, 0x16, 0x69,
      0x83, 0x24, 0xfc, 0xb2, 0x06, 0xa3, 0x79, 0x63,
      0x77, 0xe8, 0xa4, 0x50, 0xfa, 0xfb, 0xfa, 0x44,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_interfaces__msg__MotionHealth__TYPE_NAME[] = "robot_interfaces/msg/MotionHealth";

// Define type names, field names, and default values
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__state[] = "state";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__reason[] = "reason";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__v[] = "v";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__w[] = "w";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__sigma_xy[] = "sigma_xy";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__sigma_theta[] = "sigma_theta";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__drift_deg_per_m[] = "drift_deg_per_m";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__visual_known[] = "visual_known";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__visual_moving[] = "visual_moving";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__since_s[] = "since_s";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__imu_known[] = "imu_known";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__pitch_deg[] = "pitch_deg";
static char robot_interfaces__msg__MotionHealth__FIELD_NAME__roll_deg[] = "roll_deg";

static rosidl_runtime_c__type_description__Field robot_interfaces__msg__MotionHealth__FIELDS[] = {
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__state, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__reason, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__v, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__w, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__sigma_xy, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__sigma_theta, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__drift_deg_per_m, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__visual_known, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__visual_moving, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__since_s, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__imu_known, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__pitch_deg, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_interfaces__msg__MotionHealth__FIELD_NAME__roll_deg, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_interfaces__msg__MotionHealth__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_interfaces__msg__MotionHealth__TYPE_NAME, 33, 33},
      {robot_interfaces__msg__MotionHealth__FIELDS, 13, 13},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Is the robot actually doing what it was told? Published by drivetrain_node at\n"
  "# the encoder rate. See robot_core/odometry.py:MotionHealth for the logic.\n"
  "\n"
  "string  state               # idle | moving | stalled | slipping | pushed | wrong_direction | tilted | bumped\n"
  "string  reason              # human-readable, empty when all is well\n"
  "float64 v                   # m/s from the encoders\n"
  "float64 w                   # rad/s from the encoders, CCW positive\n"
  "float64 sigma_xy            # 1-sigma position drift budget since the last SLAM fix, m\n"
  "float64 sigma_theta         # 1-sigma heading drift budget, rad\n"
  "float64 drift_deg_per_m     # heading change per metre on straight moves (EWMA)\n"
  "bool    visual_known        # a fresh /visual_motion sample was available\n"
  "bool    visual_moving       # ...and what it said\n"
  "float64 since_s             # how long `state` has held\n"
  "bool    imu_known           # an IMU is fitted and answering; the two angles below mean something\n"
  "float64 pitch_deg           # nose up positive\n"
  "float64 roll_deg            # right side down positive";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_interfaces__msg__MotionHealth__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_interfaces__msg__MotionHealth__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1066, 1066},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_interfaces__msg__MotionHealth__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_interfaces__msg__MotionHealth__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
