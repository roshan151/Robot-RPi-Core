// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_interfaces:msg/MotionHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/motion_health.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__BUILDER_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_interfaces/msg/detail/motion_health__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MotionHealth_roll_deg
{
public:
  explicit Init_MotionHealth_roll_deg(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  ::robot_interfaces::msg::MotionHealth roll_deg(::robot_interfaces::msg::MotionHealth::_roll_deg_type arg)
  {
    msg_.roll_deg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_pitch_deg
{
public:
  explicit Init_MotionHealth_pitch_deg(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_roll_deg pitch_deg(::robot_interfaces::msg::MotionHealth::_pitch_deg_type arg)
  {
    msg_.pitch_deg = std::move(arg);
    return Init_MotionHealth_roll_deg(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_imu_known
{
public:
  explicit Init_MotionHealth_imu_known(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_pitch_deg imu_known(::robot_interfaces::msg::MotionHealth::_imu_known_type arg)
  {
    msg_.imu_known = std::move(arg);
    return Init_MotionHealth_pitch_deg(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_since_s
{
public:
  explicit Init_MotionHealth_since_s(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_imu_known since_s(::robot_interfaces::msg::MotionHealth::_since_s_type arg)
  {
    msg_.since_s = std::move(arg);
    return Init_MotionHealth_imu_known(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_visual_moving
{
public:
  explicit Init_MotionHealth_visual_moving(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_since_s visual_moving(::robot_interfaces::msg::MotionHealth::_visual_moving_type arg)
  {
    msg_.visual_moving = std::move(arg);
    return Init_MotionHealth_since_s(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_visual_known
{
public:
  explicit Init_MotionHealth_visual_known(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_visual_moving visual_known(::robot_interfaces::msg::MotionHealth::_visual_known_type arg)
  {
    msg_.visual_known = std::move(arg);
    return Init_MotionHealth_visual_moving(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_drift_deg_per_m
{
public:
  explicit Init_MotionHealth_drift_deg_per_m(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_visual_known drift_deg_per_m(::robot_interfaces::msg::MotionHealth::_drift_deg_per_m_type arg)
  {
    msg_.drift_deg_per_m = std::move(arg);
    return Init_MotionHealth_visual_known(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_sigma_theta
{
public:
  explicit Init_MotionHealth_sigma_theta(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_drift_deg_per_m sigma_theta(::robot_interfaces::msg::MotionHealth::_sigma_theta_type arg)
  {
    msg_.sigma_theta = std::move(arg);
    return Init_MotionHealth_drift_deg_per_m(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_sigma_xy
{
public:
  explicit Init_MotionHealth_sigma_xy(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_sigma_theta sigma_xy(::robot_interfaces::msg::MotionHealth::_sigma_xy_type arg)
  {
    msg_.sigma_xy = std::move(arg);
    return Init_MotionHealth_sigma_theta(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_w
{
public:
  explicit Init_MotionHealth_w(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_sigma_xy w(::robot_interfaces::msg::MotionHealth::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_MotionHealth_sigma_xy(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_v
{
public:
  explicit Init_MotionHealth_v(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_w v(::robot_interfaces::msg::MotionHealth::_v_type arg)
  {
    msg_.v = std::move(arg);
    return Init_MotionHealth_w(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_reason
{
public:
  explicit Init_MotionHealth_reason(::robot_interfaces::msg::MotionHealth & msg)
  : msg_(msg)
  {}
  Init_MotionHealth_v reason(::robot_interfaces::msg::MotionHealth::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_MotionHealth_v(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

class Init_MotionHealth_state
{
public:
  Init_MotionHealth_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotionHealth_reason state(::robot_interfaces::msg::MotionHealth::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MotionHealth_reason(msg_);
  }

private:
  ::robot_interfaces::msg::MotionHealth msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_interfaces::msg::MotionHealth>()
{
  return robot_interfaces::msg::builder::Init_MotionHealth_state();
}

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__BUILDER_HPP_
