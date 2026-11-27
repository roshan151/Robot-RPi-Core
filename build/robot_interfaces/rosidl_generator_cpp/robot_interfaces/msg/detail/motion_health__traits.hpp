// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/MotionHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/motion_health.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/motion_health__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotionHealth & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: v
  {
    out << "v: ";
    rosidl_generator_traits::value_to_yaml(msg.v, out);
    out << ", ";
  }

  // member: w
  {
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << ", ";
  }

  // member: sigma_xy
  {
    out << "sigma_xy: ";
    rosidl_generator_traits::value_to_yaml(msg.sigma_xy, out);
    out << ", ";
  }

  // member: sigma_theta
  {
    out << "sigma_theta: ";
    rosidl_generator_traits::value_to_yaml(msg.sigma_theta, out);
    out << ", ";
  }

  // member: drift_deg_per_m
  {
    out << "drift_deg_per_m: ";
    rosidl_generator_traits::value_to_yaml(msg.drift_deg_per_m, out);
    out << ", ";
  }

  // member: visual_known
  {
    out << "visual_known: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_known, out);
    out << ", ";
  }

  // member: visual_moving
  {
    out << "visual_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_moving, out);
    out << ", ";
  }

  // member: since_s
  {
    out << "since_s: ";
    rosidl_generator_traits::value_to_yaml(msg.since_s, out);
    out << ", ";
  }

  // member: imu_known
  {
    out << "imu_known: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_known, out);
    out << ", ";
  }

  // member: pitch_deg
  {
    out << "pitch_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_deg, out);
    out << ", ";
  }

  // member: roll_deg
  {
    out << "roll_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_deg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotionHealth & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: v
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v: ";
    rosidl_generator_traits::value_to_yaml(msg.v, out);
    out << "\n";
  }

  // member: w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << "\n";
  }

  // member: sigma_xy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sigma_xy: ";
    rosidl_generator_traits::value_to_yaml(msg.sigma_xy, out);
    out << "\n";
  }

  // member: sigma_theta
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sigma_theta: ";
    rosidl_generator_traits::value_to_yaml(msg.sigma_theta, out);
    out << "\n";
  }

  // member: drift_deg_per_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "drift_deg_per_m: ";
    rosidl_generator_traits::value_to_yaml(msg.drift_deg_per_m, out);
    out << "\n";
  }

  // member: visual_known
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_known: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_known, out);
    out << "\n";
  }

  // member: visual_moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "visual_moving: ";
    rosidl_generator_traits::value_to_yaml(msg.visual_moving, out);
    out << "\n";
  }

  // member: since_s
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "since_s: ";
    rosidl_generator_traits::value_to_yaml(msg.since_s, out);
    out << "\n";
  }

  // member: imu_known
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu_known: ";
    rosidl_generator_traits::value_to_yaml(msg.imu_known, out);
    out << "\n";
  }

  // member: pitch_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_deg, out);
    out << "\n";
  }

  // member: roll_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_deg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotionHealth & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robot_interfaces::msg::MotionHealth & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::MotionHealth & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::MotionHealth>()
{
  return "robot_interfaces::msg::MotionHealth";
}

template<>
inline const char * name<robot_interfaces::msg::MotionHealth>()
{
  return "robot_interfaces/msg/MotionHealth";
}

template<>
struct has_fixed_size<robot_interfaces::msg::MotionHealth>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_interfaces::msg::MotionHealth>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_interfaces::msg::MotionHealth>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__TRAITS_HPP_
