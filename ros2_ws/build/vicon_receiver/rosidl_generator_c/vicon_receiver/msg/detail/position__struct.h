// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vicon_receiver:msg/Position.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "vicon_receiver/msg/position.h"


#ifndef VICON_RECEIVER__MSG__DETAIL__POSITION__STRUCT_H_
#define VICON_RECEIVER__MSG__DETAIL__POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'segment_name'
// Member 'subject_name'
// Member 'translation_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Position in the package vicon_receiver.
typedef struct vicon_receiver__msg__Position
{
  float x_trans;
  float y_trans;
  float z_trans;
  /// first element of the quaternion
  float x_rot;
  /// second element of the quaternion
  float y_rot;
  /// third element of the quaternion
  float z_rot;
  /// forth element of the quaternion
  float w;
  /// first element of the quaternion
  float x_rot_euler;
  /// second element of the quaternion
  float y_rot_euler;
  /// third element of the quaternion
  float z_rot_euler;
  /// name of a specific component of the object
  rosidl_runtime_c__String segment_name;
  /// name of the entire object
  rosidl_runtime_c__String subject_name;
  /// unit of time for each capture
  int32_t frame_number;
  /// Local or Global
  rosidl_runtime_c__String translation_type;
} vicon_receiver__msg__Position;

// Struct for a sequence of vicon_receiver__msg__Position.
typedef struct vicon_receiver__msg__Position__Sequence
{
  vicon_receiver__msg__Position * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vicon_receiver__msg__Position__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VICON_RECEIVER__MSG__DETAIL__POSITION__STRUCT_H_
