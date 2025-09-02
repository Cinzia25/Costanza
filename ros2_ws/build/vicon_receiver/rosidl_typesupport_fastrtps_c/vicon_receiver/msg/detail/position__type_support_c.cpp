// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vicon_receiver:msg/Position.idl
// generated code does not contain a copyright notice
#include "vicon_receiver/msg/detail/position__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vicon_receiver/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vicon_receiver/msg/detail/position__struct.h"
#include "vicon_receiver/msg/detail/position__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // segment_name, subject_name, translation_type
#include "rosidl_runtime_c/string_functions.h"  // segment_name, subject_name, translation_type

// forward declare type support functions


using _Position__ros_msg_type = vicon_receiver__msg__Position;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
bool cdr_serialize_vicon_receiver__msg__Position(
  const vicon_receiver__msg__Position * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: x_trans
  {
    cdr << ros_message->x_trans;
  }

  // Field name: y_trans
  {
    cdr << ros_message->y_trans;
  }

  // Field name: z_trans
  {
    cdr << ros_message->z_trans;
  }

  // Field name: x_rot
  {
    cdr << ros_message->x_rot;
  }

  // Field name: y_rot
  {
    cdr << ros_message->y_rot;
  }

  // Field name: z_rot
  {
    cdr << ros_message->z_rot;
  }

  // Field name: w
  {
    cdr << ros_message->w;
  }

  // Field name: x_rot_euler
  {
    cdr << ros_message->x_rot_euler;
  }

  // Field name: y_rot_euler
  {
    cdr << ros_message->y_rot_euler;
  }

  // Field name: z_rot_euler
  {
    cdr << ros_message->z_rot_euler;
  }

  // Field name: segment_name
  {
    const rosidl_runtime_c__String * str = &ros_message->segment_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: subject_name
  {
    const rosidl_runtime_c__String * str = &ros_message->subject_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: frame_number
  {
    cdr << ros_message->frame_number;
  }

  // Field name: translation_type
  {
    const rosidl_runtime_c__String * str = &ros_message->translation_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
bool cdr_deserialize_vicon_receiver__msg__Position(
  eprosima::fastcdr::Cdr & cdr,
  vicon_receiver__msg__Position * ros_message)
{
  // Field name: x_trans
  {
    cdr >> ros_message->x_trans;
  }

  // Field name: y_trans
  {
    cdr >> ros_message->y_trans;
  }

  // Field name: z_trans
  {
    cdr >> ros_message->z_trans;
  }

  // Field name: x_rot
  {
    cdr >> ros_message->x_rot;
  }

  // Field name: y_rot
  {
    cdr >> ros_message->y_rot;
  }

  // Field name: z_rot
  {
    cdr >> ros_message->z_rot;
  }

  // Field name: w
  {
    cdr >> ros_message->w;
  }

  // Field name: x_rot_euler
  {
    cdr >> ros_message->x_rot_euler;
  }

  // Field name: y_rot_euler
  {
    cdr >> ros_message->y_rot_euler;
  }

  // Field name: z_rot_euler
  {
    cdr >> ros_message->z_rot_euler;
  }

  // Field name: segment_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->segment_name.data) {
      rosidl_runtime_c__String__init(&ros_message->segment_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->segment_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'segment_name'\n");
      return false;
    }
  }

  // Field name: subject_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->subject_name.data) {
      rosidl_runtime_c__String__init(&ros_message->subject_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->subject_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'subject_name'\n");
      return false;
    }
  }

  // Field name: frame_number
  {
    cdr >> ros_message->frame_number;
  }

  // Field name: translation_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->translation_type.data) {
      rosidl_runtime_c__String__init(&ros_message->translation_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->translation_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'translation_type'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t get_serialized_size_vicon_receiver__msg__Position(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Position__ros_msg_type * ros_message = static_cast<const _Position__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: x_trans
  {
    size_t item_size = sizeof(ros_message->x_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: y_trans
  {
    size_t item_size = sizeof(ros_message->y_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: z_trans
  {
    size_t item_size = sizeof(ros_message->z_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: x_rot
  {
    size_t item_size = sizeof(ros_message->x_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: y_rot
  {
    size_t item_size = sizeof(ros_message->y_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: z_rot
  {
    size_t item_size = sizeof(ros_message->z_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: w
  {
    size_t item_size = sizeof(ros_message->w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: x_rot_euler
  {
    size_t item_size = sizeof(ros_message->x_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: y_rot_euler
  {
    size_t item_size = sizeof(ros_message->y_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: z_rot_euler
  {
    size_t item_size = sizeof(ros_message->z_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: segment_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->segment_name.size + 1);

  // Field name: subject_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->subject_name.size + 1);

  // Field name: frame_number
  {
    size_t item_size = sizeof(ros_message->frame_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: translation_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->translation_type.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t max_serialized_size_vicon_receiver__msg__Position(
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

  // Field name: x_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: y_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: z_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: x_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: y_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: z_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: w
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: x_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: y_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: z_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: segment_name
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

  // Field name: subject_name
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

  // Field name: frame_number
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: translation_type
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
    using DataType = vicon_receiver__msg__Position;
    is_plain =
      (
      offsetof(DataType, translation_type) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
bool cdr_serialize_key_vicon_receiver__msg__Position(
  const vicon_receiver__msg__Position * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: x_trans
  {
    cdr << ros_message->x_trans;
  }

  // Field name: y_trans
  {
    cdr << ros_message->y_trans;
  }

  // Field name: z_trans
  {
    cdr << ros_message->z_trans;
  }

  // Field name: x_rot
  {
    cdr << ros_message->x_rot;
  }

  // Field name: y_rot
  {
    cdr << ros_message->y_rot;
  }

  // Field name: z_rot
  {
    cdr << ros_message->z_rot;
  }

  // Field name: w
  {
    cdr << ros_message->w;
  }

  // Field name: x_rot_euler
  {
    cdr << ros_message->x_rot_euler;
  }

  // Field name: y_rot_euler
  {
    cdr << ros_message->y_rot_euler;
  }

  // Field name: z_rot_euler
  {
    cdr << ros_message->z_rot_euler;
  }

  // Field name: segment_name
  {
    const rosidl_runtime_c__String * str = &ros_message->segment_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: subject_name
  {
    const rosidl_runtime_c__String * str = &ros_message->subject_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: frame_number
  {
    cdr << ros_message->frame_number;
  }

  // Field name: translation_type
  {
    const rosidl_runtime_c__String * str = &ros_message->translation_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t get_serialized_size_key_vicon_receiver__msg__Position(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Position__ros_msg_type * ros_message = static_cast<const _Position__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: x_trans
  {
    size_t item_size = sizeof(ros_message->x_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: y_trans
  {
    size_t item_size = sizeof(ros_message->y_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: z_trans
  {
    size_t item_size = sizeof(ros_message->z_trans);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: x_rot
  {
    size_t item_size = sizeof(ros_message->x_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: y_rot
  {
    size_t item_size = sizeof(ros_message->y_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: z_rot
  {
    size_t item_size = sizeof(ros_message->z_rot);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: w
  {
    size_t item_size = sizeof(ros_message->w);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: x_rot_euler
  {
    size_t item_size = sizeof(ros_message->x_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: y_rot_euler
  {
    size_t item_size = sizeof(ros_message->y_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: z_rot_euler
  {
    size_t item_size = sizeof(ros_message->z_rot_euler);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: segment_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->segment_name.size + 1);

  // Field name: subject_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->subject_name.size + 1);

  // Field name: frame_number
  {
    size_t item_size = sizeof(ros_message->frame_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: translation_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->translation_type.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t max_serialized_size_key_vicon_receiver__msg__Position(
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
  // Field name: x_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: y_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: z_trans
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: x_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: y_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: z_rot
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: w
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: x_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: y_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: z_rot_euler
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: segment_name
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

  // Field name: subject_name
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

  // Field name: frame_number
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: translation_type
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
    using DataType = vicon_receiver__msg__Position;
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
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const vicon_receiver__msg__Position * ros_message = static_cast<const vicon_receiver__msg__Position *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_vicon_receiver__msg__Position(ros_message, cdr);
}

static bool _Position__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  vicon_receiver__msg__Position * ros_message = static_cast<vicon_receiver__msg__Position *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_vicon_receiver__msg__Position(cdr, ros_message);
}

static uint32_t _Position__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vicon_receiver__msg__Position(
      untyped_ros_message, 0));
}

static size_t _Position__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vicon_receiver__msg__Position(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Position = {
  "vicon_receiver::msg",
  "Position",
  _Position__cdr_serialize,
  _Position__cdr_deserialize,
  _Position__get_serialized_size,
  _Position__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Position__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Position,
  get_message_typesupport_handle_function,
  &vicon_receiver__msg__Position__get_type_hash,
  &vicon_receiver__msg__Position__get_type_description,
  &vicon_receiver__msg__Position__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vicon_receiver, msg, Position)() {
  return &_Position__type_support;
}

#if defined(__cplusplus)
}
#endif
