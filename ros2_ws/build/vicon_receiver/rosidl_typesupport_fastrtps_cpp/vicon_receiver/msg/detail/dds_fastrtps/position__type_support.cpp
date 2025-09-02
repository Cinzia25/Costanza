// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vicon_receiver:msg/Position.idl
// generated code does not contain a copyright notice
#include "vicon_receiver/msg/detail/position__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vicon_receiver/msg/detail/position__functions.h"
#include "vicon_receiver/msg/detail/position__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vicon_receiver
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vicon_receiver
cdr_serialize(
  const vicon_receiver::msg::Position & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: x_trans
  cdr << ros_message.x_trans;

  // Member: y_trans
  cdr << ros_message.y_trans;

  // Member: z_trans
  cdr << ros_message.z_trans;

  // Member: x_rot
  cdr << ros_message.x_rot;

  // Member: y_rot
  cdr << ros_message.y_rot;

  // Member: z_rot
  cdr << ros_message.z_rot;

  // Member: w
  cdr << ros_message.w;

  // Member: x_rot_euler
  cdr << ros_message.x_rot_euler;

  // Member: y_rot_euler
  cdr << ros_message.y_rot_euler;

  // Member: z_rot_euler
  cdr << ros_message.z_rot_euler;

  // Member: segment_name
  cdr << ros_message.segment_name;

  // Member: subject_name
  cdr << ros_message.subject_name;

  // Member: frame_number
  cdr << ros_message.frame_number;

  // Member: translation_type
  cdr << ros_message.translation_type;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vicon_receiver
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vicon_receiver::msg::Position & ros_message)
{
  // Member: x_trans
  cdr >> ros_message.x_trans;

  // Member: y_trans
  cdr >> ros_message.y_trans;

  // Member: z_trans
  cdr >> ros_message.z_trans;

  // Member: x_rot
  cdr >> ros_message.x_rot;

  // Member: y_rot
  cdr >> ros_message.y_rot;

  // Member: z_rot
  cdr >> ros_message.z_rot;

  // Member: w
  cdr >> ros_message.w;

  // Member: x_rot_euler
  cdr >> ros_message.x_rot_euler;

  // Member: y_rot_euler
  cdr >> ros_message.y_rot_euler;

  // Member: z_rot_euler
  cdr >> ros_message.z_rot_euler;

  // Member: segment_name
  cdr >> ros_message.segment_name;

  // Member: subject_name
  cdr >> ros_message.subject_name;

  // Member: frame_number
  cdr >> ros_message.frame_number;

  // Member: translation_type
  cdr >> ros_message.translation_type;

  return true;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vicon_receiver
get_serialized_size(
  const vicon_receiver::msg::Position & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: x_trans
  {
    size_t item_size = sizeof(ros_message.x_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: y_trans
  {
    size_t item_size = sizeof(ros_message.y_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: z_trans
  {
    size_t item_size = sizeof(ros_message.z_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: x_rot
  {
    size_t item_size = sizeof(ros_message.x_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: y_rot
  {
    size_t item_size = sizeof(ros_message.y_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: z_rot
  {
    size_t item_size = sizeof(ros_message.z_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: w
  {
    size_t item_size = sizeof(ros_message.w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: x_rot_euler
  {
    size_t item_size = sizeof(ros_message.x_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: y_rot_euler
  {
    size_t item_size = sizeof(ros_message.y_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: z_rot_euler
  {
    size_t item_size = sizeof(ros_message.z_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: segment_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.segment_name.size() + 1);

  // Member: subject_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.subject_name.size() + 1);

  // Member: frame_number
  {
    size_t item_size = sizeof(ros_message.frame_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: translation_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.translation_type.size() + 1);

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vicon_receiver
max_serialized_size_Position(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: x_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: y_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: z_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: x_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: y_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: z_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: w
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: x_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: y_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: z_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: segment_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // Member: subject_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // Member: frame_number
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // Member: translation_type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vicon_receiver::msg::Position;
    is_plain =
      (
      offsetof(DataType, translation_type) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vicon_receiver
cdr_serialize_key(
  const vicon_receiver::msg::Position & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: x_trans
  cdr << ros_message.x_trans;

  // Member: y_trans
  cdr << ros_message.y_trans;

  // Member: z_trans
  cdr << ros_message.z_trans;

  // Member: x_rot
  cdr << ros_message.x_rot;

  // Member: y_rot
  cdr << ros_message.y_rot;

  // Member: z_rot
  cdr << ros_message.z_rot;

  // Member: w
  cdr << ros_message.w;

  // Member: x_rot_euler
  cdr << ros_message.x_rot_euler;

  // Member: y_rot_euler
  cdr << ros_message.y_rot_euler;

  // Member: z_rot_euler
  cdr << ros_message.z_rot_euler;

  // Member: segment_name
  cdr << ros_message.segment_name;

  // Member: subject_name
  cdr << ros_message.subject_name;

  // Member: frame_number
  cdr << ros_message.frame_number;

  // Member: translation_type
  cdr << ros_message.translation_type;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vicon_receiver
get_serialized_size_key(
  const vicon_receiver::msg::Position & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: x_trans
  {
    size_t item_size = sizeof(ros_message.x_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: y_trans
  {
    size_t item_size = sizeof(ros_message.y_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: z_trans
  {
    size_t item_size = sizeof(ros_message.z_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: x_rot
  {
    size_t item_size = sizeof(ros_message.x_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: y_rot
  {
    size_t item_size = sizeof(ros_message.y_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: z_rot
  {
    size_t item_size = sizeof(ros_message.z_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: w
  {
    size_t item_size = sizeof(ros_message.w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: x_rot_euler
  {
    size_t item_size = sizeof(ros_message.x_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: y_rot_euler
  {
    size_t item_size = sizeof(ros_message.y_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: z_rot_euler
  {
    size_t item_size = sizeof(ros_message.z_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: segment_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.segment_name.size() + 1);

  // Member: subject_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.subject_name.size() + 1);

  // Member: frame_number
  {
    size_t item_size = sizeof(ros_message.frame_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: translation_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.translation_type.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vicon_receiver
max_serialized_size_key_Position(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Member: x_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: y_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: z_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: x_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: y_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: z_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: w
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: x_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: y_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: z_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: segment_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: subject_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: frame_number
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: translation_type
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vicon_receiver::msg::Position;
    is_plain =
      (
      offsetof(DataType, translation_type) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _Position__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vicon_receiver::msg::Position *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Position__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vicon_receiver::msg::Position *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Position__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vicon_receiver::msg::Position *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Position__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Position(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Position__callbacks = {
  "vicon_receiver::msg",
  "Position",
  _Position__cdr_serialize,
  _Position__cdr_deserialize,
  _Position__get_serialized_size,
  _Position__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Position__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Position__callbacks,
  get_message_typesupport_handle_function,
  &vicon_receiver__msg__Position__get_type_hash,
  &vicon_receiver__msg__Position__get_type_description,
  &vicon_receiver__msg__Position__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vicon_receiver

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vicon_receiver
const rosidl_message_type_support_t *
get_message_type_support_handle<vicon_receiver::msg::Position>()
{
  return &vicon_receiver::msg::typesupport_fastrtps_cpp::_Position__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vicon_receiver, msg, Position)() {
  return &vicon_receiver::msg::typesupport_fastrtps_cpp::_Position__handle;
}

#ifdef __cplusplus
}
#endif
