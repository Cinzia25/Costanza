// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vicon_receiver/msg/position_list.hpp"


#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__BUILDER_HPP_
#define VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vicon_receiver/msg/detail/position_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vicon_receiver
{

namespace msg
{

namespace builder
{

class Init_PositionList_positions
{
public:
  explicit Init_PositionList_positions(::vicon_receiver::msg::PositionList & msg)
  : msg_(msg)
  {}
  ::vicon_receiver::msg::PositionList positions(::vicon_receiver::msg::PositionList::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vicon_receiver::msg::PositionList msg_;
};

class Init_PositionList_n
{
public:
  explicit Init_PositionList_n(::vicon_receiver::msg::PositionList & msg)
  : msg_(msg)
  {}
  Init_PositionList_positions n(::vicon_receiver::msg::PositionList::_n_type arg)
  {
    msg_.n = std::move(arg);
    return Init_PositionList_positions(msg_);
  }

private:
  ::vicon_receiver::msg::PositionList msg_;
};

class Init_PositionList_header
{
public:
  Init_PositionList_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionList_n header(::vicon_receiver::msg::PositionList::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PositionList_n(msg_);
  }

private:
  ::vicon_receiver::msg::PositionList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vicon_receiver::msg::PositionList>()
{
  return vicon_receiver::msg::builder::Init_PositionList_header();
}

}  // namespace vicon_receiver

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__BUILDER_HPP_
