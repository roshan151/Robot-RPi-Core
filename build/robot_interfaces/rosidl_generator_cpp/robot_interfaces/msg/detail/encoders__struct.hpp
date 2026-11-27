// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/encoders.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__Encoders __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__Encoders __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Encoders_
{
  using Type = Encoders_<ContainerAllocator>;

  explicit Encoders_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left = 0l;
      this->right = 0l;
      this->sync_error = 0l;
    }
  }

  explicit Encoders_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->left = 0l;
      this->right = 0l;
      this->sync_error = 0l;
    }
  }

  // field types and members
  using _left_type =
    int32_t;
  _left_type left;
  using _right_type =
    int32_t;
  _right_type right;
  using _sync_error_type =
    int32_t;
  _sync_error_type sync_error;

  // setters for named parameter idiom
  Type & set__left(
    const int32_t & _arg)
  {
    this->left = _arg;
    return *this;
  }
  Type & set__right(
    const int32_t & _arg)
  {
    this->right = _arg;
    return *this;
  }
  Type & set__sync_error(
    const int32_t & _arg)
  {
    this->sync_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::Encoders_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::Encoders_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::Encoders_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::Encoders_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__Encoders
    std::shared_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__Encoders
    std::shared_ptr<robot_interfaces::msg::Encoders_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Encoders_ & other) const
  {
    if (this->left != other.left) {
      return false;
    }
    if (this->right != other.right) {
      return false;
    }
    if (this->sync_error != other.sync_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const Encoders_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Encoders_

// alias to use template instance with default allocator
using Encoders =
  robot_interfaces::msg::Encoders_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_
