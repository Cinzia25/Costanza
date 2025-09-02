// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice
#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "vicon_receiver/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vicon_receiver/msg/detail/position_list__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
bool cdr_serialize_vicon_receiver__msg__PositionList(
  const vicon_receiver__msg__PositionList * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
bool cdr_deserialize_vicon_receiver__msg__PositionList(
  eprosima::fastcdr::Cdr &,
  vicon_receiver__msg__PositionList * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t get_serialized_size_vicon_receiver__msg__PositionList(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t max_serialized_size_vicon_receiver__msg__PositionList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
bool cdr_serialize_key_vicon_receiver__msg__PositionList(
  const vicon_receiver__msg__PositionList * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t get_serialized_size_key_vicon_receiver__msg__PositionList(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
size_t max_serialized_size_key_vicon_receiver__msg__PositionList(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vicon_receiver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vicon_receiver, msg, PositionList)();

#ifdef __cplusplus
}
#endif

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
