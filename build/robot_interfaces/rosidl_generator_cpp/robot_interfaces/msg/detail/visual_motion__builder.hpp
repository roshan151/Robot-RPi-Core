// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/VisualMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/visual_motion.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/visual_motion__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_VisualMotion_changed_fraction
{
public:
  explicit Init_VisualMotion_changed_fraction(::robot_interfaces::msg::VisualMotion & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::VisualMotion changed_fraction(::robot_interfaces::msg::VisualMotion::_changed_fraction_type arg)
  {
    msg_.changed_fraction = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::VisualMotion msg_;
};

class Init_VisualMotion_shift_px
{
public:
  explicit Init_VisualMotion_shift_px(::robot_interfaces::msg::VisualMotion & msg)
  : msg_(msg)
  {}
  Init_VisualMotion_changed_fraction shift_px(::robot_interfaces::msg::VisualMotion::_shift_px_type arg)
  {
    msg_.shift_px = std::move(arg);
    return Init_VisualMotion_changed_fraction(msg_);
  }

private:
  ::robot_interfaces::msg::VisualMotion msg_;
};

class Init_VisualMotion_moving
{
public:
  explicit Init_VisualMotion_moving(::robot_interfaces::msg::VisualMotion & msg)
  : msg_(msg)
  {}
  Init_VisualMotion_shift_px moving(::robot_interfaces::msg::VisualMotion::_moving_type arg)
  {
    msg_.moving = std::move(arg);
    return Init_VisualMotion_shift_px(msg_);
  }

private:
  ::robot_interfaces::msg::VisualMotion msg_;
};

class Init_VisualMotion_t_monotonic
{
public:
  Init_VisualMotion_t_monotonic()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VisualMotion_moving t_monotonic(::robot_interfaces::msg::VisualMotion::_t_monotonic_type arg)
  {
    msg_.t_monotonic = std::move(arg);
    return Init_VisualMotion_moving(msg_);
  }

private:
  ::robot_interfaces::msg::VisualMotion msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::VisualMotion>()
{
  return robot_interfaces::msg::builder::Init_VisualMotion_t_monotonic();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__BUILDER_HPP_
