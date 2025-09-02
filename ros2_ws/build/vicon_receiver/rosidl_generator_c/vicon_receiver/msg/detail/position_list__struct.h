// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vicon_receiver/msg/position_list.h"


#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__STRUCT_H_
#define VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'positions'
#include "vicon_receiver/msg/detail/position__struct.h"

/// Struct defined in msg/PositionList in the package vicon_receiver.
typedef struct vicon_receiver__msg__PositionList
{
  std_msgs__msg__Header header;
  int32_t n;
  vicon_receiver__msg__Position__Sequence positions;
} vicon_receiver__msg__PositionList;

// Struct for a sequence of vicon_receiver__msg__PositionList.
typedef struct vicon_receiver__msg__PositionList__Sequence
{
  vicon_receiver__msg__PositionList * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vicon_receiver__msg__PositionList__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION_LIST__STRUCT_H_
