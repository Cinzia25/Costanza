// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vicon_receiver/msg/position_list.hpp"


#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__STRUCT_HPP_
#define VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'positions'
#include "vicon_receiver/msg/detail/position__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__vicon_receiver__msg__PositionList __attribute__((deprecated))
#else
# define DEPRECATED__vicon_receiver__msg__PositionList __declspec(deprecated)
#endif

namespace vicon_receiver
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PositionList_
{
  using Type = PositionList_<ContainerAllocator>;

  explicit PositionList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0l;
    }
  }

  explicit PositionList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->n = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _n_type =
    int32_t;
  _n_type n;
  using _positions_type =
    std::vector<vicon_receiver::msg::Position_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vicon_receiver::msg::Position_<ContainerAllocator>>>;
  _positions_type positions;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__n(
    const int32_t & _arg)
  {
    this->n = _arg;
    return *this;
  }
  Type & set__positions(
    const std::vector<vicon_receiver::msg::Position_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<vicon_receiver::msg::Position_<ContainerAllocator>>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    vicon_receiver::msg::PositionList_<ContainerAllocator> *;
  using ConstRawPtr =
    const vicon_receiver::msg::PositionList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      vicon_receiver::msg::PositionList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      vicon_receiver::msg::PositionList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__vicon_receiver__msg__PositionList
    std::shared_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__vicon_receiver__msg__PositionList
    std::shared_ptr<vicon_receiver::msg::PositionList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PositionList_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->n != other.n) {
      return false;
    }
    if (this->positions != other.positions) {
      return false;
    }
    return true;
  }
  bool operator!=(const PositionList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PositionList_

// alias to use template instance with default allocator
using PositionList =
  vicon_receiver::msg::PositionList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace vicon_receiver

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__STRUCT_HPP_
