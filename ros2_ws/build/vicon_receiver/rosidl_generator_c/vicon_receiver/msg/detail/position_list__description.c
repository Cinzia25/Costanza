// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice

#include "vicon_receiver/msg/detail/position_list__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_vicon_receiver
const rosidl_type_hash_t *
vicon_receiver__msg__PositionList__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa5, 0xa7, 0x15, 0xbd, 0xa2, 0x0a, 0x99, 0x5d,
      0x51, 0x5c, 0x58, 0xd6, 0x49, 0xe8, 0x90, 0xa1,
      0xfd, 0x1e, 0x67, 0xf5, 0xed, 0x99, 0x0e, 0x5b,
      0x81, 0xbe, 0x81, 0x46, 0x48, 0x7b, 0xda, 0x39,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "vicon_receiver/msg/detail/position__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
static const rosidl_type_hash_t vicon_receiver__msg__Position__EXPECTED_HASH = {1, {
    0x0b, 0x52, 0x94, 0x24, 0xd4, 0xae, 0xdf, 0x72,
    0x2e, 0x02, 0x32, 0x61, 0x44, 0xe3, 0x92, 0x19,
    0x36, 0x9b, 0x34, 0xbf, 0x71, 0x06, 0x32, 0x25,
    0x6c, 0x5a, 0x6e, 0x8f, 0x73, 0xa1, 0xb1, 0x03,
  }};
#endif

static char vicon_receiver__msg__PositionList__TYPE_NAME[] = "vicon_receiver/msg/PositionList";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";
static char vicon_receiver__msg__Position__TYPE_NAME[] = "vicon_receiver/msg/Position";

// Define type names, field names, and default values
static char vicon_receiver__msg__PositionList__FIELD_NAME__header[] = "header";
static char vicon_receiver__msg__PositionList__FIELD_NAME__n[] = "n";
static char vicon_receiver__msg__PositionList__FIELD_NAME__positions[] = "positions";

static rosidl_runtime_c__type_description__Field vicon_receiver__msg__PositionList__FIELDS[] = {
  {
    {vicon_receiver__msg__PositionList__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__PositionList__FIELD_NAME__n, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__PositionList__FIELD_NAME__positions, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {vicon_receiver__msg__Position__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription vicon_receiver__msg__PositionList__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {vicon_receiver__msg__Position__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
vicon_receiver__msg__PositionList__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {vicon_receiver__msg__PositionList__TYPE_NAME, 31, 31},
      {vicon_receiver__msg__PositionList__FIELDS, 3, 3},
    },
    {vicon_receiver__msg__PositionList__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&vicon_receiver__msg__Position__EXPECTED_HASH, vicon_receiver__msg__Position__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = vicon_receiver__msg__Position__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "int32 n\n"
  "Position[] positions";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
vicon_receiver__msg__PositionList__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {vicon_receiver__msg__PositionList__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 51, 51},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
vicon_receiver__msg__PositionList__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *vicon_receiver__msg__PositionList__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[3] = *vicon_receiver__msg__Position__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
