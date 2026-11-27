// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_interfaces:action/Drive.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_interfaces/action/drive.hpp"


#ifndef ROBOT_INTERFACES__ACTION__DETAIL__DRIVE__STRUCT_HPP_
#define ROBOT_INTERFACES__ACTION__DETAIL__DRIVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_Goal __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_Goal __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_Goal_
{
  using Type = Drive_Goal_<ContainerAllocator>;

  explicit Drive_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->gesture = false;
      this->speed_pct = 0.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->meters = 0.0;
      this->gesture = false;
      this->speed_pct = 0.0;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->meters = 0.0;
    }
  }

  explicit Drive_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->gesture = false;
      this->speed_pct = 0.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->meters = 0.0;
      this->gesture = false;
      this->speed_pct = 0.0;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->meters = 0.0;
    }
  }

  // field types and members
  using _meters_type =
    double;
  _meters_type meters;
  using _gesture_type =
    bool;
  _gesture_type gesture;
  using _speed_pct_type =
    double;
  _speed_pct_type speed_pct;

  // setters for named parameter idiom
  Type & set__meters(
    const double & _arg)
  {
    this->meters = _arg;
    return *this;
  }
  Type & set__gesture(
    const bool & _arg)
  {
    this->gesture = _arg;
    return *this;
  }
  Type & set__speed_pct(
    const double & _arg)
  {
    this->speed_pct = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_Goal
    std::shared_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_Goal
    std::shared_ptr<robot_interfaces::action::Drive_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_Goal_ & other) const
  {
    if (this->meters != other.meters) {
      return false;
    }
    if (this->gesture != other.gesture) {
      return false;
    }
    if (this->speed_pct != other.speed_pct) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_Goal_

// alias to use template instance with default allocator
using Drive_Goal =
  robot_interfaces::action::Drive_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_Result __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_Result __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_Result_
{
  using Type = Drive_Result_<ContainerAllocator>;

  explicit Drive_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
      this->ticks_left = 0l;
      this->ticks_right = 0l;
      this->duration_s = 0.0;
    }
  }

  explicit Drive_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = "";
      this->ticks_left = 0l;
      this->ticks_right = 0l;
      this->duration_s = 0.0;
    }
  }

  // field types and members
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;
  using _ticks_left_type =
    int32_t;
  _ticks_left_type ticks_left;
  using _ticks_right_type =
    int32_t;
  _ticks_right_type ticks_right;
  using _duration_s_type =
    double;
  _duration_s_type duration_s;

  // setters for named parameter idiom
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__ticks_left(
    const int32_t & _arg)
  {
    this->ticks_left = _arg;
    return *this;
  }
  Type & set__ticks_right(
    const int32_t & _arg)
  {
    this->ticks_right = _arg;
    return *this;
  }
  Type & set__duration_s(
    const double & _arg)
  {
    this->duration_s = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_Result
    std::shared_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_Result
    std::shared_ptr<robot_interfaces::action::Drive_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_Result_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->ticks_left != other.ticks_left) {
      return false;
    }
    if (this->ticks_right != other.ticks_right) {
      return false;
    }
    if (this->duration_s != other.duration_s) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_Result_

// alias to use template instance with default allocator
using Drive_Result =
  robot_interfaces::action::Drive_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces


#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_Feedback __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_Feedback_
{
  using Type = Drive_Feedback_<ContainerAllocator>;

  explicit Drive_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ticks_left = 0l;
      this->ticks_right = 0l;
    }
  }

  explicit Drive_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ticks_left = 0l;
      this->ticks_right = 0l;
    }
  }

  // field types and members
  using _ticks_left_type =
    int32_t;
  _ticks_left_type ticks_left;
  using _ticks_right_type =
    int32_t;
  _ticks_right_type ticks_right;

  // setters for named parameter idiom
  Type & set__ticks_left(
    const int32_t & _arg)
  {
    this->ticks_left = _arg;
    return *this;
  }
  Type & set__ticks_right(
    const int32_t & _arg)
  {
    this->ticks_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_Feedback
    std::shared_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_Feedback
    std::shared_ptr<robot_interfaces::action::Drive_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_Feedback_ & other) const
  {
    if (this->ticks_left != other.ticks_left) {
      return false;
    }
    if (this->ticks_right != other.ticks_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_Feedback_

// alias to use template instance with default allocator
using Drive_Feedback =
  robot_interfaces::action::Drive_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "robot_interfaces/action/detail/drive__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_SendGoal_Request __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_SendGoal_Request_
{
  using Type = Drive_SendGoal_Request_<ContainerAllocator>;

  explicit Drive_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit Drive_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    robot_interfaces::action::Drive_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const robot_interfaces::action::Drive_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_SendGoal_Request
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_SendGoal_Request
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_SendGoal_Request_

// alias to use template instance with default allocator
using Drive_SendGoal_Request =
  robot_interfaces::action::Drive_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_SendGoal_Response __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_SendGoal_Response_
{
  using Type = Drive_SendGoal_Response_<ContainerAllocator>;

  explicit Drive_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit Drive_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_SendGoal_Response
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_SendGoal_Response
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_SendGoal_Response_

// alias to use template instance with default allocator
using Drive_SendGoal_Response =
  robot_interfaces::action::Drive_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_SendGoal_Event __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_SendGoal_Event __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_SendGoal_Event_
{
  using Type = Drive_SendGoal_Event_<ContainerAllocator>;

  explicit Drive_SendGoal_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Drive_SendGoal_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_SendGoal_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_SendGoal_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_SendGoal_Event
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_SendGoal_Event
    std::shared_ptr<robot_interfaces::action::Drive_SendGoal_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_SendGoal_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_SendGoal_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_SendGoal_Event_

// alias to use template instance with default allocator
using Drive_SendGoal_Event =
  robot_interfaces::action::Drive_SendGoal_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces

namespace robot_interfaces
{

namespace action
{

struct Drive_SendGoal
{
  using Request = robot_interfaces::action::Drive_SendGoal_Request;
  using Response = robot_interfaces::action::Drive_SendGoal_Response;
  using Event = robot_interfaces::action::Drive_SendGoal_Event;
};

}  // namespace action

}  // namespace robot_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_GetResult_Request __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_GetResult_Request_
{
  using Type = Drive_GetResult_Request_<ContainerAllocator>;

  explicit Drive_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit Drive_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_GetResult_Request
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_GetResult_Request
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_GetResult_Request_

// alias to use template instance with default allocator
using Drive_GetResult_Request =
  robot_interfaces::action::Drive_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "robot_interfaces/action/detail/drive__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_GetResult_Response __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_GetResult_Response_
{
  using Type = Drive_GetResult_Response_<ContainerAllocator>;

  explicit Drive_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit Drive_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    robot_interfaces::action::Drive_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const robot_interfaces::action::Drive_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_GetResult_Response
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_GetResult_Response
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_GetResult_Response_

// alias to use template instance with default allocator
using Drive_GetResult_Response =
  robot_interfaces::action::Drive_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces


// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_GetResult_Event __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_GetResult_Event __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_GetResult_Event_
{
  using Type = Drive_GetResult_Event_<ContainerAllocator>;

  explicit Drive_GetResult_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit Drive_GetResult_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_GetResult_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<robot_interfaces::action::Drive_GetResult_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_GetResult_Event
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_GetResult_Event
    std::shared_ptr<robot_interfaces::action::Drive_GetResult_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_GetResult_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_GetResult_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_GetResult_Event_

// alias to use template instance with default allocator
using Drive_GetResult_Event =
  robot_interfaces::action::Drive_GetResult_Event_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces

namespace robot_interfaces
{

namespace action
{

struct Drive_GetResult
{
  using Request = robot_interfaces::action::Drive_GetResult_Request;
  using Response = robot_interfaces::action::Drive_GetResult_Response;
  using Event = robot_interfaces::action::Drive_GetResult_Event;
};

}  // namespace action

}  // namespace robot_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "robot_interfaces/action/detail/drive__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_interfaces__action__Drive_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__robot_interfaces__action__Drive_FeedbackMessage __declspec(deprecated)
#endif

namespace robot_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct Drive_FeedbackMessage_
{
  using Type = Drive_FeedbackMessage_<ContainerAllocator>;

  explicit Drive_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit Drive_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    robot_interfaces::action::Drive_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const robot_interfaces::action::Drive_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_interfaces__action__Drive_FeedbackMessage
    std::shared_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_interfaces__action__Drive_FeedbackMessage
    std::shared_ptr<robot_interfaces::action::Drive_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Drive_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const Drive_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Drive_FeedbackMessage_

// alias to use template instance with default allocator
using Drive_FeedbackMessage =
  robot_interfaces::action::Drive_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace robot_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace robot_interfaces
{

namespace action
{

struct Drive
{
  /// The goal message defined in the action definition.
  using Goal = robot_interfaces::action::Drive_Goal;
  /// The result message defined in the action definition.
  using Result = robot_interfaces::action::Drive_Result;
  /// The feedback message defined in the action definition.
  using Feedback = robot_interfaces::action::Drive_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = robot_interfaces::action::Drive_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = robot_interfaces::action::Drive_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = robot_interfaces::action::Drive_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct Drive Drive;

}  // namespace action

}  // namespace robot_interfaces

#endif  // ROBOT_INTERFACES__ACTION__DETAIL__DRIVE__STRUCT_HPP_
