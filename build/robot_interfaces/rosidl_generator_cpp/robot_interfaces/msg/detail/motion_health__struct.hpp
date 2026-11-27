// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:msg/MotionHealth.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/msg/motion_health.hpp"


#ifndef ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__STRUCT_HPP_
#define ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__msg__MotionHealth __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__msg__MotionHealth __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotionHealth_
{
  using Type = MotionHealth_<ContainerAllocator>;

  explicit MotionHealth_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->reason = "";
      this->v = 0.0;
      this->w = 0.0;
      this->sigma_xy = 0.0;
      this->sigma_theta = 0.0;
      this->drift_deg_per_m = 0.0;
      this->visual_known = false;
      this->visual_moving = false;
      this->since_s = 0.0;
      this->imu_known = false;
      this->pitch_deg = 0.0;
      this->roll_deg = 0.0;
    }
  }

  explicit MotionHealth_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc),
    reason(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = "";
      this->reason = "";
      this->v = 0.0;
      this->w = 0.0;
      this->sigma_xy = 0.0;
      this->sigma_theta = 0.0;
      this->drift_deg_per_m = 0.0;
      this->visual_known = false;
      this->visual_moving = false;
      this->since_s = 0.0;
      this->imu_known = false;
      this->pitch_deg = 0.0;
      this->roll_deg = 0.0;
    }
  }

  // field types and members
  using _state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _state_type state;
  using _reason_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _reason_type reason;
  using _v_type =
    double;
  _v_type v;
  using _w_type =
    double;
  _w_type w;
  using _sigma_xy_type =
    double;
  _sigma_xy_type sigma_xy;
  using _sigma_theta_type =
    double;
  _sigma_theta_type sigma_theta;
  using _drift_deg_per_m_type =
    double;
  _drift_deg_per_m_type drift_deg_per_m;
  using _visual_known_type =
    bool;
  _visual_known_type visual_known;
  using _visual_moving_type =
    bool;
  _visual_moving_type visual_moving;
  using _since_s_type =
    double;
  _since_s_type since_s;
  using _imu_known_type =
    bool;
  _imu_known_type imu_known;
  using _pitch_deg_type =
    double;
  _pitch_deg_type pitch_deg;
  using _roll_deg_type =
    double;
  _roll_deg_type roll_deg;

  // setters for named parameter idiom
  Type & set__state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__reason(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->reason = _arg;
    return *this;
  }
  Type & set__v(
    const double & _arg)
  {
    this->v = _arg;
    return *this;
  }
  Type & set__w(
    const double & _arg)
  {
    this->w = _arg;
    return *this;
  }
  Type & set__sigma_xy(
    const double & _arg)
  {
    this->sigma_xy = _arg;
    return *this;
  }
  Type & set__sigma_theta(
    const double & _arg)
  {
    this->sigma_theta = _arg;
    return *this;
  }
  Type & set__drift_deg_per_m(
    const double & _arg)
  {
    this->drift_deg_per_m = _arg;
    return *this;
  }
  Type & set__visual_known(
    const bool & _arg)
  {
    this->visual_known = _arg;
    return *this;
  }
  Type & set__visual_moving(
    const bool & _arg)
  {
    this->visual_moving = _arg;
    return *this;
  }
  Type & set__since_s(
    const double & _arg)
  {
    this->since_s = _arg;
    return *this;
  }
  Type & set__imu_known(
    const bool & _arg)
  {
    this->imu_known = _arg;
    return *this;
  }
  Type & set__pitch_deg(
    const double & _arg)
  {
    this->pitch_deg = _arg;
    return *this;
  }
  Type & set__roll_deg(
    const double & _arg)
  {
    this->roll_deg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::msg::MotionHealth_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::msg::MotionHealth_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::MotionHealth_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::msg::MotionHealth_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__msg__MotionHealth
    std::shared_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__msg__MotionHealth
    std::shared_ptr<robot_interfaces::msg::MotionHealth_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotionHealth_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->reason != other.reason) {
      return false;
    }
    if (this->v != other.v) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    if (this->sigma_xy != other.sigma_xy) {
      return false;
    }
    if (this->sigma_theta != other.sigma_theta) {
      return false;
    }
    if (this->drift_deg_per_m != other.drift_deg_per_m) {
      return false;
    }
    if (this->visual_known != other.visual_known) {
      return false;
    }
    if (this->visual_moving != other.visual_moving) {
      return false;
    }
    if (this->since_s != other.since_s) {
      return false;
    }
    if (this->imu_known != other.imu_known) {
      return false;
    }
    if (this->pitch_deg != other.pitch_deg) {
      return false;
    }
    if (this->roll_deg != other.roll_deg) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotionHealth_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotionHealth_

// alias to use template instance with default allocator
using MotionHealth =
  robot_interfaces::msg::MotionHealth_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__MSG__DETAIL__MOTION_HEALTH__STRUCT_HPP_
