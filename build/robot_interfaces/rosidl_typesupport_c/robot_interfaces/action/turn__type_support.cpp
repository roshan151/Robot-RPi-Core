// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from robot_interfaces:action/Turn.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_interfaces/action/detail/turn__struct.h"
#include "robot_interfaces/action/detail/turn__type_support.h"
#include "robot_interfaces/action/detail/turn__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Goal_type_support_ids_t;

static const _Turn_Goal_type_support_ids_t _Turn_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_Goal_type_support_symbol_names_t _Turn_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_Goal)),
  }
};

typedef struct _Turn_Goal_type_support_data_t
{
  void * data[2];
} _Turn_Goal_type_support_data_t;

static _Turn_Goal_type_support_data_t _Turn_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_Goal_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_Goal__get_type_hash,
  &robot_interfaces__action__Turn_Goal__get_type_description,
  &robot_interfaces__action__Turn_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_Goal)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Result_type_support_ids_t;

static const _Turn_Result_type_support_ids_t _Turn_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_Result_type_support_symbol_names_t _Turn_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_Result)),
  }
};

typedef struct _Turn_Result_type_support_data_t
{
  void * data[2];
} _Turn_Result_type_support_data_t;

static _Turn_Result_type_support_data_t _Turn_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_Result_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_Result_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_Result_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_Result__get_type_hash,
  &robot_interfaces__action__Turn_Result__get_type_description,
  &robot_interfaces__action__Turn_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_Result)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Feedback_type_support_ids_t;

static const _Turn_Feedback_type_support_ids_t _Turn_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_Feedback_type_support_symbol_names_t _Turn_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_Feedback)),
  }
};

typedef struct _Turn_Feedback_type_support_data_t
{
  void * data[2];
} _Turn_Feedback_type_support_data_t;

static _Turn_Feedback_type_support_data_t _Turn_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_Feedback_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_Feedback__get_type_hash,
  &robot_interfaces__action__Turn_Feedback__get_type_description,
  &robot_interfaces__action__Turn_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_Feedback)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_Request_type_support_ids_t;

static const _Turn_SendGoal_Request_type_support_ids_t _Turn_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_SendGoal_Request_type_support_symbol_names_t _Turn_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_SendGoal_Request)),
  }
};

typedef struct _Turn_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _Turn_SendGoal_Request_type_support_data_t;

static _Turn_SendGoal_Request_type_support_data_t _Turn_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_SendGoal_Request_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_SendGoal_Request__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal_Request__get_type_description,
  &robot_interfaces__action__Turn_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_SendGoal_Request)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_Response_type_support_ids_t;

static const _Turn_SendGoal_Response_type_support_ids_t _Turn_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_SendGoal_Response_type_support_symbol_names_t _Turn_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_SendGoal_Response)),
  }
};

typedef struct _Turn_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _Turn_SendGoal_Response_type_support_data_t;

static _Turn_SendGoal_Response_type_support_data_t _Turn_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_SendGoal_Response_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_SendGoal_Response__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal_Response__get_type_description,
  &robot_interfaces__action__Turn_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_SendGoal_Response)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_Event_type_support_ids_t;

static const _Turn_SendGoal_Event_type_support_ids_t _Turn_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_SendGoal_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_SendGoal_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_SendGoal_Event_type_support_symbol_names_t _Turn_SendGoal_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_SendGoal_Event)),
  }
};

typedef struct _Turn_SendGoal_Event_type_support_data_t
{
  void * data[2];
} _Turn_SendGoal_Event_type_support_data_t;

static _Turn_SendGoal_Event_type_support_data_t _Turn_SendGoal_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_SendGoal_Event_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_SendGoal_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_SendGoal_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_SendGoal_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_SendGoal_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_SendGoal_Event__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal_Event__get_type_description,
  &robot_interfaces__action__Turn_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_SendGoal_Event)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{
typedef struct _Turn_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_type_support_ids_t;

static const _Turn_SendGoal_type_support_ids_t _Turn_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_SendGoal_type_support_symbol_names_t _Turn_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_SendGoal)),
  }
};

typedef struct _Turn_SendGoal_type_support_data_t
{
  void * data[2];
} _Turn_SendGoal_type_support_data_t;

static _Turn_SendGoal_type_support_data_t _Turn_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_SendGoal_service_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_Turn_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_Turn_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Turn_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &Turn_SendGoal_Request_message_type_support_handle,
  &Turn_SendGoal_Response_message_type_support_handle,
  &Turn_SendGoal_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_interfaces,
    action,
    Turn_SendGoal
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_interfaces,
    action,
    Turn_SendGoal
  ),
  &robot_interfaces__action__Turn_SendGoal__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal__get_type_description,
  &robot_interfaces__action__Turn_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_SendGoal)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_Request_type_support_ids_t;

static const _Turn_GetResult_Request_type_support_ids_t _Turn_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_GetResult_Request_type_support_symbol_names_t _Turn_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_GetResult_Request)),
  }
};

typedef struct _Turn_GetResult_Request_type_support_data_t
{
  void * data[2];
} _Turn_GetResult_Request_type_support_data_t;

static _Turn_GetResult_Request_type_support_data_t _Turn_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_GetResult_Request_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_GetResult_Request__get_type_hash,
  &robot_interfaces__action__Turn_GetResult_Request__get_type_description,
  &robot_interfaces__action__Turn_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_GetResult_Request)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_Response_type_support_ids_t;

static const _Turn_GetResult_Response_type_support_ids_t _Turn_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_GetResult_Response_type_support_symbol_names_t _Turn_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_GetResult_Response)),
  }
};

typedef struct _Turn_GetResult_Response_type_support_data_t
{
  void * data[2];
} _Turn_GetResult_Response_type_support_data_t;

static _Turn_GetResult_Response_type_support_data_t _Turn_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_GetResult_Response_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_GetResult_Response__get_type_hash,
  &robot_interfaces__action__Turn_GetResult_Response__get_type_description,
  &robot_interfaces__action__Turn_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_GetResult_Response)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_Event_type_support_ids_t;

static const _Turn_GetResult_Event_type_support_ids_t _Turn_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_GetResult_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_GetResult_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_GetResult_Event_type_support_symbol_names_t _Turn_GetResult_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_GetResult_Event)),
  }
};

typedef struct _Turn_GetResult_Event_type_support_data_t
{
  void * data[2];
} _Turn_GetResult_Event_type_support_data_t;

static _Turn_GetResult_Event_type_support_data_t _Turn_GetResult_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_GetResult_Event_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_GetResult_Event_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_GetResult_Event_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_GetResult_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_GetResult_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_GetResult_Event__get_type_hash,
  &robot_interfaces__action__Turn_GetResult_Event__get_type_description,
  &robot_interfaces__action__Turn_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_GetResult_Event)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "service_msgs/msg/service_event_info.h"
// already included above
// #include "builtin_interfaces/msg/time.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{
typedef struct _Turn_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_type_support_ids_t;

static const _Turn_GetResult_type_support_ids_t _Turn_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_GetResult_type_support_symbol_names_t _Turn_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_GetResult)),
  }
};

typedef struct _Turn_GetResult_type_support_data_t
{
  void * data[2];
} _Turn_GetResult_type_support_data_t;

static _Turn_GetResult_type_support_data_t _Turn_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_GetResult_service_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_Turn_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_Turn_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t Turn_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &Turn_GetResult_Request_message_type_support_handle,
  &Turn_GetResult_Response_message_type_support_handle,
  &Turn_GetResult_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_interfaces,
    action,
    Turn_GetResult
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    robot_interfaces,
    action,
    Turn_GetResult
  ),
  &robot_interfaces__action__Turn_GetResult__get_type_hash,
  &robot_interfaces__action__Turn_GetResult__get_type_description,
  &robot_interfaces__action__Turn_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_GetResult)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _Turn_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_FeedbackMessage_type_support_ids_t;

static const _Turn_FeedbackMessage_type_support_ids_t _Turn_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _Turn_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Turn_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Turn_FeedbackMessage_type_support_symbol_names_t _Turn_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_interfaces, action, Turn_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_interfaces, action, Turn_FeedbackMessage)),
  }
};

typedef struct _Turn_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _Turn_FeedbackMessage_type_support_data_t;

static _Turn_FeedbackMessage_type_support_data_t _Turn_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Turn_FeedbackMessage_message_typesupport_map = {
  2,
  "robot_interfaces",
  &_Turn_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_Turn_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_Turn_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Turn_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_FeedbackMessage__get_type_hash,
  &robot_interfaces__action__Turn_FeedbackMessage__get_type_description,
  &robot_interfaces__action__Turn_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, robot_interfaces, action, Turn_FeedbackMessage)() {
  return &::robot_interfaces::action::rosidl_typesupport_c::Turn_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "robot_interfaces/action/turn.h"
// already included above
// #include "robot_interfaces/action/detail/turn__type_support.h"

static rosidl_action_type_support_t _robot_interfaces__action__Turn__typesupport_c = {
  NULL, NULL, NULL, NULL, NULL,
  &robot_interfaces__action__Turn__get_type_hash,
  &robot_interfaces__action__Turn__get_type_description,
  &robot_interfaces__action__Turn__get_type_description_sources,
};

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, robot_interfaces, action, Turn)()
{
  // Thread-safe by always writing the same values to the static struct
  _robot_interfaces__action__Turn__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, robot_interfaces, action, Turn_SendGoal)();
  _robot_interfaces__action__Turn__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, robot_interfaces, action, Turn_GetResult)();
  _robot_interfaces__action__Turn__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _robot_interfaces__action__Turn__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, robot_interfaces, action, Turn_FeedbackMessage)();
  _robot_interfaces__action__Turn__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_robot_interfaces__action__Turn__typesupport_c;
}

#ifdef __cplusplus
}
#endif
