// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vicon_receiver:msg/Position.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vicon_receiver/msg/position.hpp"


#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION__TRAITS_HPP_
#define VICON_RECEIVER__MSG__DETAIL__POSITION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vicon_receiver/msg/detail/position__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vicon_receiver
{

namespace msg
{

inline void to_flow_style_yaml(
  const Position & msg,
  std::ostream & out)
{
  out << "{";
  // member: x_trans
  {
    out << "x_trans: ";
    rosidl_generator_traits::value_to_yaml(msg.x_trans, out);
    out << ", ";
  }

  // member: y_trans
  {
    out << "y_trans: ";
    rosidl_generator_traits::value_to_yaml(msg.y_trans, out);
    out << ", ";
  }

  // member: z_trans
  {
    out << "z_trans: ";
    rosidl_generator_traits::value_to_yaml(msg.z_trans, out);
    out << ", ";
  }

  // member: x_rot
  {
    out << "x_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.x_rot, out);
    out << ", ";
  }

  // member: y_rot
  {
    out << "y_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.y_rot, out);
    out << ", ";
  }

  // member: z_rot
  {
    out << "z_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.z_rot, out);
    out << ", ";
  }

  // member: w
  {
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << ", ";
  }

  // member: x_rot_euler
  {
    out << "x_rot_euler: ";
    rosidl_generator_traits::value_to_yaml(msg.x_rot_euler, out);
    out << ", ";
  }

  // member: y_rot_euler
  {
    out << "y_rot_euler: ";
    rosidl_generator_traits::value_to_yaml(msg.y_rot_euler, out);
    out << ", ";
  }

  // member: z_rot_euler
  {
    out << "z_rot_euler: ";
    rosidl_generator_traits::value_to_yaml(msg.z_rot_euler, out);
    out << ", ";
  }

  // member: segment_name
  {
    out << "segment_name: ";
    rosidl_generator_traits::value_to_yaml(msg.segment_name, out);
    out << ", ";
  }

  // member: subject_name
  {
    out << "subject_name: ";
    rosidl_generator_traits::value_to_yaml(msg.subject_name, out);
    out << ", ";
  }

  // member: frame_number
  {
    out << "frame_number: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_number, out);
    out << ", ";
  }

  // member: translation_type
  {
    out << "translation_type: ";
    rosidl_generator_traits::value_to_yaml(msg.translation_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Position & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x_trans
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_trans: ";
    rosidl_generator_traits::value_to_yaml(msg.x_trans, out);
    out << "\n";
  }

  // member: y_trans
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_trans: ";
    rosidl_generator_traits::value_to_yaml(msg.y_trans, out);
    out << "\n";
  }

  // member: z_trans
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_trans: ";
    rosidl_generator_traits::value_to_yaml(msg.z_trans, out);
    out << "\n";
  }

  // member: x_rot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.x_rot, out);
    out << "\n";
  }

  // member: y_rot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.y_rot, out);
    out << "\n";
  }

  // member: z_rot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_rot: ";
    rosidl_generator_traits::value_to_yaml(msg.z_rot, out);
    out << "\n";
  }

  // member: w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "w: ";
    rosidl_generator_traits::value_to_yaml(msg.w, out);
    out << "\n";
  }

  // member: x_rot_euler
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_rot_euler: ";
    rosidl_generator_traits::value_to_yaml(msg.x_rot_euler, out);
    out << "\n";
  }

  // member: y_rot_euler
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_rot_euler: ";
    rosidl_generator_traits::value_to_yaml(msg.y_rot_euler, out);
    out << "\n";
  }

  // member: z_rot_euler
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z_rot_euler: ";
    rosidl_generator_traits::value_to_yaml(msg.z_rot_euler, out);
    out << "\n";
  }

  // member: segment_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "segment_name: ";
    rosidl_generator_traits::value_to_yaml(msg.segment_name, out);
    out << "\n";
  }

  // member: subject_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "subject_name: ";
    rosidl_generator_traits::value_to_yaml(msg.subject_name, out);
    out << "\n";
  }

  // member: frame_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_number: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_number, out);
    out << "\n";
  }

  // member: translation_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "translation_type: ";
    rosidl_generator_traits::value_to_yaml(msg.translation_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Position & msg, bool use_flow_style = false)
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
  const vicon_receiver::msg::Position & msg,
  std::ostream & out, size_t indentation = 0)
{
  vicon_receiver::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vicon_receiver::msg::to_yaml() instead")]]
inline std::string to_yaml(const vicon_receiver::msg::Position & msg)
{
  return vicon_receiver::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vicon_receiver::msg::Position>()
{
  return "vicon_receiver::msg::Position";
}

template<>
inline const char * name<vicon_receiver::msg::Position>()
{
  return "vicon_receiver/msg/Position";
}

template<>
struct has_fixed_size<vicon_receiver::msg::Position>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<vicon_receiver::msg::Position>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<vicon_receiver::msg::Position>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION__TRAITS_HPP_
