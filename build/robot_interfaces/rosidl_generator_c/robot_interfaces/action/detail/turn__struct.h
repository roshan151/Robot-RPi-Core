// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_interfaces:action/Turn.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/action/turn.h"


#ifndef ROBOT_INTERFACES__ACTION__DETAIL__TURN__STRUCT_H_
#define ROBOT_INTERFACES__ACTION__DETAIL__TURN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_Goal
{
  /// positive turns RIGHT, negative LEFT
  double degrees;
  /// gesture goals are REJECTED when busy, never queued
  bool gesture;
  /// 0 = use the node's configured default
  double speed_pct;
} robot_interfaces__action__Turn_Goal;

// Struct for a sequence of robot_interfaces__action__Turn_Goal.
typedef struct robot_interfaces__action__Turn_Goal__Sequence
{
  robot_interfaces__action__Turn_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_Result
{
  /// done | cancelled | failed | timeout
  rosidl_runtime_c__String status;
  int32_t ticks_left;
  int32_t ticks_right;
  double duration_s;
} robot_interfaces__action__Turn_Result;

// Struct for a sequence of robot_interfaces__action__Turn_Result.
typedef struct robot_interfaces__action__Turn_Result__Sequence
{
  robot_interfaces__action__Turn_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_Feedback
{
  int32_t ticks_left;
  int32_t ticks_right;
} robot_interfaces__action__Turn_Feedback;

// Struct for a sequence of robot_interfaces__action__Turn_Feedback.
typedef struct robot_interfaces__action__Turn_Feedback__Sequence
{
  robot_interfaces__action__Turn_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "robot_interfaces/action/detail/turn__struct.h"

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_interfaces__action__Turn_Goal goal;
} robot_interfaces__action__Turn_SendGoal_Request;

// Struct for a sequence of robot_interfaces__action__Turn_SendGoal_Request.
typedef struct robot_interfaces__action__Turn_SendGoal_Request__Sequence
{
  robot_interfaces__action__Turn_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} robot_interfaces__action__Turn_SendGoal_Response;

// Struct for a sequence of robot_interfaces__action__Turn_SendGoal_Response.
typedef struct robot_interfaces__action__Turn_SendGoal_Response__Sequence
{
  robot_interfaces__action__Turn_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robot_interfaces__action__Turn_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robot_interfaces__action__Turn_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robot_interfaces__action__Turn_SendGoal_Request__Sequence request;
  robot_interfaces__action__Turn_SendGoal_Response__Sequence response;
} robot_interfaces__action__Turn_SendGoal_Event;

// Struct for a sequence of robot_interfaces__action__Turn_SendGoal_Event.
typedef struct robot_interfaces__action__Turn_SendGoal_Event__Sequence
{
  robot_interfaces__action__Turn_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} robot_interfaces__action__Turn_GetResult_Request;

// Struct for a sequence of robot_interfaces__action__Turn_GetResult_Request.
typedef struct robot_interfaces__action__Turn_GetResult_Request__Sequence
{
  robot_interfaces__action__Turn_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_GetResult_Response
{
  int8_t status;
  robot_interfaces__action__Turn_Result result;
} robot_interfaces__action__Turn_GetResult_Response;

// Struct for a sequence of robot_interfaces__action__Turn_GetResult_Response.
typedef struct robot_interfaces__action__Turn_GetResult_Response__Sequence
{
  robot_interfaces__action__Turn_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  robot_interfaces__action__Turn_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  robot_interfaces__action__Turn_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  robot_interfaces__action__Turn_GetResult_Request__Sequence request;
  robot_interfaces__action__Turn_GetResult_Response__Sequence response;
} robot_interfaces__action__Turn_GetResult_Event;

// Struct for a sequence of robot_interfaces__action__Turn_GetResult_Event.
typedef struct robot_interfaces__action__Turn_GetResult_Event__Sequence
{
  robot_interfaces__action__Turn_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"

/// Struct defined in action/Turn in the package robot_interfaces.
typedef struct robot_interfaces__action__Turn_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  robot_interfaces__action__Turn_Feedback feedback;
} robot_interfaces__action__Turn_FeedbackMessage;

// Struct for a sequence of robot_interfaces__action__Turn_FeedbackMessage.
typedef struct robot_interfaces__action__Turn_FeedbackMessage__Sequence
{
  robot_interfaces__action__Turn_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_interfaces__action__Turn_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_INTERFACES__ACTION__DETAIL__TURN__STRUCT_H_
