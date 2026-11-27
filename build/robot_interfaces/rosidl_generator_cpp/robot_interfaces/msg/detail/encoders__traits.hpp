// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/encoders.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__TRAITS_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robot_interfaces/msg/detail/encoders__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Encoders & msg,
  std::ostream & out)
{
  out << "{";
  // member: left
  {
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << ", ";
  }

  // member: right
  {
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << ", ";
  }

  // member: sync_error
  {
    out << "sync_error: ";
    rosidl_generator_traits::value_to_yaml(msg.sync_error, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Encoders & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left: ";
    rosidl_generator_traits::value_to_yaml(msg.left, out);
    out << "\n";
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right: ";
    rosidl_generator_traits::value_to_yaml(msg.right, out);
    out << "\n";
  }

  // member: sync_error
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sync_error: ";
    rosidl_generator_traits::value_to_yaml(msg.sync_error, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Encoders & msg, bool use_flow_style = false)
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
  const robot_interfaces::msg::Encoders & msg,
  std::ostream & out, size_t indentation = 0)
{
  robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const robot_interfaces::msg::Encoders & msg)
{
  return robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robot_interfaces::msg::Encoders>()
{
  return "robot_interfaces::msg::Encoders";
}

template<>
inline const char * name<robot_interfaces::msg::Encoders>()
{
  return "robot_interfaces/msg/Encoders";
}

template<>
struct has_fixed_size<robot_interfaces::msg::Encoders>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<robot_interfaces::msg::Encoders>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<robot_interfaces::msg::Encoders>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__TRAITS_HPP_
