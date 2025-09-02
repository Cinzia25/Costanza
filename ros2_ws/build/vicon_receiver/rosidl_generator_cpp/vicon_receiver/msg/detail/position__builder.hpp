// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from vicon_receiver:msg/Position.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vicon_receiver/msg/position.hpp"


#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION__BUILDER_HPP_
#define VICON_RECEIVER__MSG__DETAIL__POSITION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "vicon_receiver/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace vicon_receiver
{

namespace msg
{

namespace builder
{

class Init_Position_translation_type
{
public:
  explicit Init_Position_translation_type(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  ::vicon_receiver::msg::Position translation_type(::vicon_receiver::msg::Position::_translation_type_type arg)
  {
    msg_.translation_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_frame_number
{
public:
  explicit Init_Position_frame_number(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_translation_type frame_number(::vicon_receiver::msg::Position::_frame_number_type arg)
  {
    msg_.frame_number = std::move(arg);
    return Init_Position_translation_type(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_subject_name
{
public:
  explicit Init_Position_subject_name(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_frame_number subject_name(::vicon_receiver::msg::Position::_subject_name_type arg)
  {
    msg_.subject_name = std::move(arg);
    return Init_Position_frame_number(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_segment_name
{
public:
  explicit Init_Position_segment_name(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_subject_name segment_name(::vicon_receiver::msg::Position::_segment_name_type arg)
  {
    msg_.segment_name = std::move(arg);
    return Init_Position_subject_name(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_z_rot_euler
{
public:
  explicit Init_Position_z_rot_euler(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_segment_name z_rot_euler(::vicon_receiver::msg::Position::_z_rot_euler_type arg)
  {
    msg_.z_rot_euler = std::move(arg);
    return Init_Position_segment_name(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_y_rot_euler
{
public:
  explicit Init_Position_y_rot_euler(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_z_rot_euler y_rot_euler(::vicon_receiver::msg::Position::_y_rot_euler_type arg)
  {
    msg_.y_rot_euler = std::move(arg);
    return Init_Position_z_rot_euler(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_x_rot_euler
{
public:
  explicit Init_Position_x_rot_euler(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_y_rot_euler x_rot_euler(::vicon_receiver::msg::Position::_x_rot_euler_type arg)
  {
    msg_.x_rot_euler = std::move(arg);
    return Init_Position_y_rot_euler(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_w
{
public:
  explicit Init_Position_w(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_x_rot_euler w(::vicon_receiver::msg::Position::_w_type arg)
  {
    msg_.w = std::move(arg);
    return Init_Position_x_rot_euler(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_z_rot
{
public:
  explicit Init_Position_z_rot(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_w z_rot(::vicon_receiver::msg::Position::_z_rot_type arg)
  {
    msg_.z_rot = std::move(arg);
    return Init_Position_w(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_y_rot
{
public:
  explicit Init_Position_y_rot(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_z_rot y_rot(::vicon_receiver::msg::Position::_y_rot_type arg)
  {
    msg_.y_rot = std::move(arg);
    return Init_Position_z_rot(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_x_rot
{
public:
  explicit Init_Position_x_rot(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_y_rot x_rot(::vicon_receiver::msg::Position::_x_rot_type arg)
  {
    msg_.x_rot = std::move(arg);
    return Init_Position_y_rot(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_z_trans
{
public:
  explicit Init_Position_z_trans(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_x_rot z_trans(::vicon_receiver::msg::Position::_z_trans_type arg)
  {
    msg_.z_trans = std::move(arg);
    return Init_Position_x_rot(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_y_trans
{
public:
  explicit Init_Position_y_trans(::vicon_receiver::msg::Position & msg)
  : msg_(msg)
  {}
  Init_Position_z_trans y_trans(::vicon_receiver::msg::Position::_y_trans_type arg)
  {
    msg_.y_trans = std::move(arg);
    return Init_Position_z_trans(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

class Init_Position_x_trans
{
public:
  Init_Position_x_trans()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Position_y_trans x_trans(::vicon_receiver::msg::Position::_x_trans_type arg)
  {
    msg_.x_trans = std::move(arg);
    return Init_Position_y_trans(msg_);
  }

private:
  ::vicon_receiver::msg::Position msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::vicon_receiver::msg::Position>()
{
  return vicon_receiver::msg::builder::Init_Position_x_trans();
}

}  // namespace vicon_receiver

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION__BUILDER_HPP_
