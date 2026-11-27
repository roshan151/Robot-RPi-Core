// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/encoders.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/encoders__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_Encoders_sync_error
{
public:
  explicit Init_Encoders_sync_error(::robot_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::Encoders sync_error(::robot_interfaces::msg::Encoders::_sync_error_type arg)
  {
    msg_.sync_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::Encoders msg_;
};

class Init_Encoders_right
{
public:
  explicit Init_Encoders_right(::robot_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  Init_Encoders_sync_error right(::robot_interfaces::msg::Encoders::_right_type arg)
  {
    msg_.right = std::move(arg);
    return Init_Encoders_sync_error(msg_);
  }

private:
  ::robot_interfaces::msg::Encoders msg_;
};

class Init_Encoders_left
{
public:
  Init_Encoders_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Encoders_right left(::robot_interfaces::msg::Encoders::_left_type arg)
  {
    msg_.left = std::move(arg);
    return Init_Encoders_right(msg_);
  }

private:
  ::robot_interfaces::msg::Encoders msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::Encoders>()
{
  return robot_interfaces::msg::builder::Init_Encoders_left();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_
