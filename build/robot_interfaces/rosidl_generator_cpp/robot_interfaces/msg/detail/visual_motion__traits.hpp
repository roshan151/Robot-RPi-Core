// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/VisualMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/visual_motion.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/visual_motion__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const VisualMotion & msg,
  std::ostream & out)
{
  out << "{";
  // member: t_monotonic
  {
    out << "t_monotonic: ";
    rosidl_generator_traits::value_to_yaml(msg.t_monotonic, out);
    out << ", ";
  }

  // member: moving
  {
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << ", ";
  }

  // member: shift_px
  {
    out << "shift_px: ";
    rosidl_generator_traits::value_to_yaml(msg.shift_px, out);
    out << ", ";
  }

  // member: changed_fraction
  {
    out << "changed_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.changed_fraction, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VisualMotion & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: t_monotonic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_monotonic: ";
    rosidl_generator_traits::value_to_yaml(msg.t_monotonic, out);
    out << "\n";
  }

  // member: moving
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "moving: ";
    rosidl_generator_traits::value_to_yaml(msg.moving, out);
    out << "\n";
  }

  // member: shift_px
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shift_px: ";
    rosidl_generator_traits::value_to_yaml(msg.shift_px, out);
    out << "\n";
  }

  // member: changed_fraction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "changed_fraction: ";
    rosidl_generator_traits::value_to_yaml(msg.changed_fraction, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VisualMotion & msg, bool use_flow_style = false)
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
  const robot_interfaces::msg::VisualMotion & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::VisualMotion & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::VisualMotion>()
{
  return "robot_interfaces::msg::VisualMotion";
}

template<>
inline const char * name<robot_interfaces::msg::VisualMotion>()
{
  return "robot_interfaces/msg/VisualMotion";
}

template<>
struct has_fixed_size<robot_interfaces::msg::VisualMotion>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_interfaces::msg::VisualMotion>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_interfaces::msg::VisualMotion>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__TRAITS_HPP_
