// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vicon_receiver:msg/Position.idl
// generated code does not contain a copyright notice

#include "vicon_receiver/msg/detail/position__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vicon_receiver
const rosidl_type_hash_t *
vicon_receiver__msg__Position__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x0b, 0x52, 0x94, 0x24, 0xd4, 0xae, 0xdf, 0x72,
      0x2e, 0x02, 0x32, 0x61, 0x44, 0xe3, 0x92, 0x19,
      0x36, 0x9b, 0x34, 0xbf, 0x71, 0x06, 0x32, 0x25,
      0x6c, 0x5a, 0x6e, 0x8f, 0x73, 0xa1, 0xb1, 0x03,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char vicon_receiver__msg__Position__TYPE_NAME[] = "vicon_receiver/msg/Position";

// Define type names, field names, and default values
static char vicon_receiver__msg__Position__FIELD_NAME__x_trans[] = "x_trans";
static char vicon_receiver__msg__Position__FIELD_NAME__y_trans[] = "y_trans";
static char vicon_receiver__msg__Position__FIELD_NAME__z_trans[] = "z_trans";
static char vicon_receiver__msg__Position__FIELD_NAME__x_rot[] = "x_rot";
static char vicon_receiver__msg__Position__FIELD_NAME__y_rot[] = "y_rot";
static char vicon_receiver__msg__Position__FIELD_NAME__z_rot[] = "z_rot";
static char vicon_receiver__msg__Position__FIELD_NAME__w[] = "w";
static char vicon_receiver__msg__Position__FIELD_NAME__x_rot_euler[] = "x_rot_euler";
static char vicon_receiver__msg__Position__FIELD_NAME__y_rot_euler[] = "y_rot_euler";
static char vicon_receiver__msg__Position__FIELD_NAME__z_rot_euler[] = "z_rot_euler";
static char vicon_receiver__msg__Position__FIELD_NAME__segment_name[] = "segment_name";
static char vicon_receiver__msg__Position__FIELD_NAME__subject_name[] = "subject_name";
static char vicon_receiver__msg__Position__FIELD_NAME__frame_number[] = "frame_number";
static char vicon_receiver__msg__Position__FIELD_NAME__translation_type[] = "translation_type";

static rosidl_runtime_c__type_description__Field vicon_receiver__msg__Position__FIELDS[] = {
  {
    {vicon_receiver__msg__Position__FIELD_NAME__x_trans, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__y_trans, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__z_trans, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__x_rot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__y_rot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__z_rot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__w, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__x_rot_euler, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__y_rot_euler, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__z_rot_euler, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__segment_name, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__subject_name, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__frame_number, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__FIELD_NAME__translation_type, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
vicon_receiver__msg__Position__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vicon_receiver__msg__Position__TYPE_NAME, 27, 27},
      {vicon_receiver__msg__Position__FIELDS, 14, 14},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 x_trans \n"
  "float32 y_trans\n"
  "float32 z_trans\n"
  "float32 x_rot #first element of the quaternion\n"
  "float32 y_rot #second element of the quaternion\n"
  "float32 z_rot #third element of the quaternion\n"
  "float32 w #forth element of the quaternion\n"
  "float32 x_rot_euler #first element of the quaternion\n"
  "float32 y_rot_euler #second element of the quaternion\n"
  "float32 z_rot_euler #third element of the quaternion\n"
  "string segment_name #name of a specific component of the object\n"
  "string subject_name #name of the entire object\n"
  "int32 frame_number #unit of time for each capture\n"
  "string translation_type #Local or Global";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vicon_receiver__msg__Position__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vicon_receiver__msg__Position__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 596, 596},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vicon_receiver__msg__Position__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vicon_receiver__msg__Position__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
