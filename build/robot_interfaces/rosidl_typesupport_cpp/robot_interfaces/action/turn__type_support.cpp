// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from robot_interfaces:action/Turn.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "robot_interfaces/action/detail/turn__functions.h"
#include "robot_interfaces/action/detail/turn__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Goal_type_support_ids_t;

static const _Turn_Goal_type_support_ids_t _Turn_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_Goal)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_Goal__get_type_hash,
  &robot_interfaces__action__Turn_Goal__get_type_description,
  &robot_interfaces__action__Turn_Goal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_Goal>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_Goal)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Result_type_support_ids_t;

static const _Turn_Result_type_support_ids_t _Turn_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_Result)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_Result__get_type_hash,
  &robot_interfaces__action__Turn_Result__get_type_description,
  &robot_interfaces__action__Turn_Result__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_Result>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_Result)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_Feedback_type_support_ids_t;

static const _Turn_Feedback_type_support_ids_t _Turn_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_Feedback)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_Feedback__get_type_hash,
  &robot_interfaces__action__Turn_Feedback__get_type_description,
  &robot_interfaces__action__Turn_Feedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_Feedback>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_Feedback)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_Request_type_support_ids_t;

static const _Turn_SendGoal_Request_type_support_ids_t _Turn_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_SendGoal_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_SendGoal_Request__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal_Request__get_type_description,
  &robot_interfaces__action__Turn_SendGoal_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Request>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_SendGoal_Request)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_Response_type_support_ids_t;

static const _Turn_SendGoal_Response_type_support_ids_t _Turn_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_SendGoal_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_SendGoal_Response__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal_Response__get_type_description,
  &robot_interfaces__action__Turn_SendGoal_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Response>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_SendGoal_Response)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_SendGoal_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_Event_type_support_ids_t;

static const _Turn_SendGoal_Event_type_support_ids_t _Turn_SendGoal_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_SendGoal_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_SendGoal_Event)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_SendGoal_Event__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal_Event__get_type_description,
  &robot_interfaces__action__Turn_SendGoal_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Event>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_SendGoal_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_SendGoal_Event)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_SendGoal_type_support_ids_t;

static const _Turn_SendGoal_type_support_ids_t _Turn_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_SendGoal)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_interfaces::action::Turn_SendGoal_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<robot_interfaces::action::Turn_SendGoal>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<robot_interfaces::action::Turn_SendGoal>,
  &robot_interfaces__action__Turn_SendGoal__get_type_hash,
  &robot_interfaces__action__Turn_SendGoal__get_type_description,
  &robot_interfaces__action__Turn_SendGoal__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robot_interfaces::action::Turn_SendGoal>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<robot_interfaces::action::Turn_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_Request_type_support_ids_t;

static const _Turn_GetResult_Request_type_support_ids_t _Turn_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_GetResult_Request)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_GetResult_Request__get_type_hash,
  &robot_interfaces__action__Turn_GetResult_Request__get_type_description,
  &robot_interfaces__action__Turn_GetResult_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Request>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_GetResult_Request)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_Response_type_support_ids_t;

static const _Turn_GetResult_Response_type_support_ids_t _Turn_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_GetResult_Response)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_GetResult_Response__get_type_hash,
  &robot_interfaces__action__Turn_GetResult_Response__get_type_description,
  &robot_interfaces__action__Turn_GetResult_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Response>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_GetResult_Response)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_GetResult_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_Event_type_support_ids_t;

static const _Turn_GetResult_Event_type_support_ids_t _Turn_GetResult_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_GetResult_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_GetResult_Event)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_Event_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_GetResult_Event__get_type_hash,
  &robot_interfaces__action__Turn_GetResult_Event__get_type_description,
  &robot_interfaces__action__Turn_GetResult_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Event>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_GetResult_Event_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_GetResult_Event)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Event>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_GetResult_type_support_ids_t;

static const _Turn_GetResult_type_support_ids_t _Turn_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_GetResult)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Request>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Response>(),
  ::rosidl_typesupport_cpp::get_message_type_support_handle<robot_interfaces::action::Turn_GetResult_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<robot_interfaces::action::Turn_GetResult>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<robot_interfaces::action::Turn_GetResult>,
  &robot_interfaces__action__Turn_GetResult__get_type_hash,
  &robot_interfaces__action__Turn_GetResult__get_type_description,
  &robot_interfaces__action__Turn_GetResult__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<robot_interfaces::action::Turn_GetResult>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<robot_interfaces::action::Turn_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "robot_interfaces/action/detail/turn__functions.h"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _Turn_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Turn_FeedbackMessage_type_support_ids_t;

static const _Turn_FeedbackMessage_type_support_ids_t _Turn_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
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
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_interfaces, action, Turn_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_interfaces, action, Turn_FeedbackMessage)),
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
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Turn_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &robot_interfaces__action__Turn_FeedbackMessage__get_type_hash,
  &robot_interfaces__action__Turn_FeedbackMessage__get_type_description,
  &robot_interfaces__action__Turn_FeedbackMessage__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_interfaces::action::Turn_FeedbackMessage>()
{
  return &::robot_interfaces::action::rosidl_typesupport_cpp::Turn_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn_FeedbackMessage)() {
  return get_message_type_support_handle<robot_interfaces::action::Turn_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "robot_interfaces/action/detail/turn__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace robot_interfaces
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t Turn_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL,
  &robot_interfaces__action__Turn__get_type_hash,
  &robot_interfaces__action__Turn__get_type_description,
  &robot_interfaces__action__Turn__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace robot_interfaces

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<robot_interfaces::action::Turn>()
{
  using ::robot_interfaces::action::rosidl_typesupport_cpp::Turn_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  Turn_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::robot_interfaces::action::Turn::Impl::SendGoalService>();
  Turn_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::robot_interfaces::action::Turn::Impl::GetResultService>();
  Turn_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::robot_interfaces::action::Turn::Impl::CancelGoalService>();
  Turn_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::robot_interfaces::action::Turn::Impl::FeedbackMessage>();
  Turn_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::robot_interfaces::action::Turn::Impl::GoalStatusMessage>();
  return &Turn_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, robot_interfaces, action, Turn)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<robot_interfaces::action::Turn>();
}

#ifdef __cplusplus
}
#endif
