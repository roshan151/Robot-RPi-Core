// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/VisualMotion.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/visual_motion.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__VisualMotion __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__VisualMotion __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VisualMotion_
{
  using Type = VisualMotion_<ContainerAllocator>;

  explicit VisualMotion_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t_monotonic = 0.0;
      this->moving = false;
      this->shift_px = 0.0f;
      this->changed_fraction = 0.0f;
    }
  }

  explicit VisualMotion_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t_monotonic = 0.0;
      this->moving = false;
      this->shift_px = 0.0f;
      this->changed_fraction = 0.0f;
    }
  }

  // field types and members
  using _t_monotonic_type =
    double;
  _t_monotonic_type t_monotonic;
  using _moving_type =
    bool;
  _moving_type moving;
  using _shift_px_type =
    float;
  _shift_px_type shift_px;
  using _changed_fraction_type =
    float;
  _changed_fraction_type changed_fraction;

  // setters for named parameter idiom
  Type & set__t_monotonic(
    const double & _arg)
  {
    this->t_monotonic = _arg;
    return *this;
  }
  Type & set__moving(
    const bool & _arg)
  {
    this->moving = _arg;
    return *this;
  }
  Type & set__shift_px(
    const float & _arg)
  {
    this->shift_px = _arg;
    return *this;
  }
  Type & set__changed_fraction(
    const float & _arg)
  {
    this->changed_fraction = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::VisualMotion_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::VisualMotion_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::VisualMotion_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::VisualMotion_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__VisualMotion
    std::shared_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__VisualMotion
    std::shared_ptr<robot_interfaces::msg::VisualMotion_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VisualMotion_ & other) const
  {
    if (this->t_monotonic != other.t_monotonic) {
      return false;
    }
    if (this->moving != other.moving) {
      return false;
    }
    if (this->shift_px != other.shift_px) {
      return false;
    }
    if (this->changed_fraction != other.changed_fraction) {
      return false;
    }
    return true;
  }
  bool operator!=(const VisualMotion_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VisualMotion_

// alias to use template instance with default allocator
using VisualMotion =
  robot_interfaces::msg::VisualMotion_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__VISUAL_MOTION__STRUCT_HPP_
