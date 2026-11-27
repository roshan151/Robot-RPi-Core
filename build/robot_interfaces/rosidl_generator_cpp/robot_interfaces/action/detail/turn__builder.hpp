// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:action/Turn.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/action/turn.hpp"


#ifndef ROBOT_INTERFACES__ACTION__DETAIL__TURN__BUILDER_HPP_
#define ROBOT_INTERFACES__ACTION__DETAIL__TURN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/action/detail/turn__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_Goal_speed_pct
{
public:
  explicit Init_Turn_Goal_speed_pct(::robot_interfaces::action::Turn_Goal & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_Goal speed_pct(::robot_interfaces::action::Turn_Goal::_speed_pct_type arg)
  {
    msg_.speed_pct = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Goal msg_;
};

class Init_Turn_Goal_gesture
{
public:
  explicit Init_Turn_Goal_gesture(::robot_interfaces::action::Turn_Goal & msg)
  : msg_(msg)
  {}
  Init_Turn_Goal_speed_pct gesture(::robot_interfaces::action::Turn_Goal::_gesture_type arg)
  {
    msg_.gesture = std::move(arg);
    return Init_Turn_Goal_speed_pct(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Goal msg_;
};

class Init_Turn_Goal_degrees
{
public:
  Init_Turn_Goal_degrees()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_Goal_gesture degrees(::robot_interfaces::action::Turn_Goal::_degrees_type arg)
  {
    msg_.degrees = std::move(arg);
    return Init_Turn_Goal_gesture(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_Goal>()
{
  return robot_interfaces::action::builder::Init_Turn_Goal_degrees();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_Result_duration_s
{
public:
  explicit Init_Turn_Result_duration_s(::robot_interfaces::action::Turn_Result & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_Result duration_s(::robot_interfaces::action::Turn_Result::_duration_s_type arg)
  {
    msg_.duration_s = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Result msg_;
};

class Init_Turn_Result_ticks_right
{
public:
  explicit Init_Turn_Result_ticks_right(::robot_interfaces::action::Turn_Result & msg)
  : msg_(msg)
  {}
  Init_Turn_Result_duration_s ticks_right(::robot_interfaces::action::Turn_Result::_ticks_right_type arg)
  {
    msg_.ticks_right = std::move(arg);
    return Init_Turn_Result_duration_s(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Result msg_;
};

class Init_Turn_Result_ticks_left
{
public:
  explicit Init_Turn_Result_ticks_left(::robot_interfaces::action::Turn_Result & msg)
  : msg_(msg)
  {}
  Init_Turn_Result_ticks_right ticks_left(::robot_interfaces::action::Turn_Result::_ticks_left_type arg)
  {
    msg_.ticks_left = std::move(arg);
    return Init_Turn_Result_ticks_right(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Result msg_;
};

class Init_Turn_Result_status
{
public:
  Init_Turn_Result_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_Result_ticks_left status(::robot_interfaces::action::Turn_Result::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Turn_Result_ticks_left(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_Result>()
{
  return robot_interfaces::action::builder::Init_Turn_Result_status();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_Feedback_ticks_right
{
public:
  explicit Init_Turn_Feedback_ticks_right(::robot_interfaces::action::Turn_Feedback & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_Feedback ticks_right(::robot_interfaces::action::Turn_Feedback::_ticks_right_type arg)
  {
    msg_.ticks_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Feedback msg_;
};

class Init_Turn_Feedback_ticks_left
{
public:
  Init_Turn_Feedback_ticks_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_Feedback_ticks_right ticks_left(::robot_interfaces::action::Turn_Feedback::_ticks_left_type arg)
  {
    msg_.ticks_left = std::move(arg);
    return Init_Turn_Feedback_ticks_right(msg_);
  }

private:
  ::robot_interfaces::action::Turn_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_Feedback>()
{
  return robot_interfaces::action::builder::Init_Turn_Feedback_ticks_left();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_SendGoal_Request_goal
{
public:
  explicit Init_Turn_SendGoal_Request_goal(::robot_interfaces::action::Turn_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_SendGoal_Request goal(::robot_interfaces::action::Turn_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_SendGoal_Request msg_;
};

class Init_Turn_SendGoal_Request_goal_id
{
public:
  Init_Turn_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_SendGoal_Request_goal goal_id(::robot_interfaces::action::Turn_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Turn_SendGoal_Request_goal(msg_);
  }

private:
  ::robot_interfaces::action::Turn_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_SendGoal_Request>()
{
  return robot_interfaces::action::builder::Init_Turn_SendGoal_Request_goal_id();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_SendGoal_Response_stamp
{
public:
  explicit Init_Turn_SendGoal_Response_stamp(::robot_interfaces::action::Turn_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_SendGoal_Response stamp(::robot_interfaces::action::Turn_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_SendGoal_Response msg_;
};

class Init_Turn_SendGoal_Response_accepted
{
public:
  Init_Turn_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_SendGoal_Response_stamp accepted(::robot_interfaces::action::Turn_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Turn_SendGoal_Response_stamp(msg_);
  }

private:
  ::robot_interfaces::action::Turn_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_SendGoal_Response>()
{
  return robot_interfaces::action::builder::Init_Turn_SendGoal_Response_accepted();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_SendGoal_Event_response
{
public:
  explicit Init_Turn_SendGoal_Event_response(::robot_interfaces::action::Turn_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_SendGoal_Event response(::robot_interfaces::action::Turn_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_SendGoal_Event msg_;
};

class Init_Turn_SendGoal_Event_request
{
public:
  explicit Init_Turn_SendGoal_Event_request(::robot_interfaces::action::Turn_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_Turn_SendGoal_Event_response request(::robot_interfaces::action::Turn_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Turn_SendGoal_Event_response(msg_);
  }

private:
  ::robot_interfaces::action::Turn_SendGoal_Event msg_;
};

class Init_Turn_SendGoal_Event_info
{
public:
  Init_Turn_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_SendGoal_Event_request info(::robot_interfaces::action::Turn_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Turn_SendGoal_Event_request(msg_);
  }

private:
  ::robot_interfaces::action::Turn_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_SendGoal_Event>()
{
  return robot_interfaces::action::builder::Init_Turn_SendGoal_Event_info();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_GetResult_Request_goal_id
{
public:
  Init_Turn_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_interfaces::action::Turn_GetResult_Request goal_id(::robot_interfaces::action::Turn_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_GetResult_Request>()
{
  return robot_interfaces::action::builder::Init_Turn_GetResult_Request_goal_id();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_GetResult_Response_result
{
public:
  explicit Init_Turn_GetResult_Response_result(::robot_interfaces::action::Turn_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_GetResult_Response result(::robot_interfaces::action::Turn_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_GetResult_Response msg_;
};

class Init_Turn_GetResult_Response_status
{
public:
  Init_Turn_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_GetResult_Response_result status(::robot_interfaces::action::Turn_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Turn_GetResult_Response_result(msg_);
  }

private:
  ::robot_interfaces::action::Turn_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_GetResult_Response>()
{
  return robot_interfaces::action::builder::Init_Turn_GetResult_Response_status();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_GetResult_Event_response
{
public:
  explicit Init_Turn_GetResult_Event_response(::robot_interfaces::action::Turn_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_GetResult_Event response(::robot_interfaces::action::Turn_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_GetResult_Event msg_;
};

class Init_Turn_GetResult_Event_request
{
public:
  explicit Init_Turn_GetResult_Event_request(::robot_interfaces::action::Turn_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_Turn_GetResult_Event_response request(::robot_interfaces::action::Turn_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Turn_GetResult_Event_response(msg_);
  }

private:
  ::robot_interfaces::action::Turn_GetResult_Event msg_;
};

class Init_Turn_GetResult_Event_info
{
public:
  Init_Turn_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_GetResult_Event_request info(::robot_interfaces::action::Turn_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Turn_GetResult_Event_request(msg_);
  }

private:
  ::robot_interfaces::action::Turn_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_GetResult_Event>()
{
  return robot_interfaces::action::builder::Init_Turn_GetResult_Event_info();
}

}  // namespace robot_interfaces


namespace robot_interfaces
{

namespace action
{

namespace builder
{

class Init_Turn_FeedbackMessage_feedback
{
public:
  explicit Init_Turn_FeedbackMessage_feedback(::robot_interfaces::action::Turn_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::action::Turn_FeedbackMessage feedback(::robot_interfaces::action::Turn_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::action::Turn_FeedbackMessage msg_;
};

class Init_Turn_FeedbackMessage_goal_id
{
public:
  Init_Turn_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Turn_FeedbackMessage_feedback goal_id(::robot_interfaces::action::Turn_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Turn_FeedbackMessage_feedback(msg_);
  }

private:
  ::robot_interfaces::action::Turn_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::action::Turn_FeedbackMessage>()
{
  return robot_interfaces::action::builder::Init_Turn_FeedbackMessage_goal_id();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__ACTION__DETAIL__TURN__BUILDER_HPP_
