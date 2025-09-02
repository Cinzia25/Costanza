// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vicon_receiver/msg/position_list.hpp"


#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__TRAITS_HPP_
#define VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vicon_receiver/msg/detail/position_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'positions'
#include "vicon_receiver/msg/detail/position__traits.hpp"

namespace vicon_receiver
{

namespace msg
{

inline void to_flow_style_yaml(
  const PositionList & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: n
  {
    out << "n: ";
    rosidl_generator_traits::value_to_yaml(msg.n, out);
    out << ", ";
  }

  // member: positions
  {
    if (msg.positions.size() == 0) {
      out << "positions: []";
    } else {
      out << "positions: [";
      size_t pending_items = msg.positions.size();
      for (auto item : msg.positions) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PositionList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: n
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "n: ";
    rosidl_generator_traits::value_to_yaml(msg.n, out);
    out << "\n";
  }

  // member: positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positions.size() == 0) {
      out << "positions: []\n";
    } else {
      out << "positions:\n";
      for (auto item : msg.positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PositionList & msg, bool use_flow_style = false)
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

}  // namespace vicon_receiver

namespace rosidl_generator_traits
{

[[deprecated("use vicon_receiver::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vicon_receiver::msg::PositionList & msg,
  std::ostream & out, size_t indentation = 0)
{
  vicon_receiver::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vicon_receiver::msg::to_yaml() instead")]]
inline std::string to_yaml(const vicon_receiver::msg::PositionList & msg)
{
  return vicon_receiver::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vicon_receiver::msg::PositionList>()
{
  return "vicon_receiver::msg::PositionList";
}

template<>
inline const char * name<vicon_receiver::msg::PositionList>()
{
  return "vicon_receiver/msg/PositionList";
}

template<>
struct has_fixed_size<vicon_receiver::msg::PositionList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vicon_receiver::msg::PositionList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vicon_receiver::msg::PositionList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__TRAITS_HPP_
