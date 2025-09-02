// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vicon_receiver/msg/detail/position_list__rosidl_typesupport_introspection_c.h"
#include "vicon_receiver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vicon_receiver/msg/detail/position_list__functions.h"
#include "vicon_receiver/msg/detail/position_list__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `positions`
#include "vicon_receiver/msg/position.h"
// Member `positions`
#include "vicon_receiver/msg/detail/position__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vicon_receiver__msg__PositionList__init(message_memory);
}

void vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_fini_function(void * message_memory)
{
  vicon_receiver__msg__PositionList__fini(message_memory);
}

size_t vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__size_function__PositionList__positions(
  const void * untyped_member)
{
  const vicon_receiver__msg__Position__Sequence * member =
    (const vicon_receiver__msg__Position__Sequence *)(untyped_member);
  return member->size;
}

const void * vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__get_const_function__PositionList__positions(
  const void * untyped_member, size_t index)
{
  const vicon_receiver__msg__Position__Sequence * member =
    (const vicon_receiver__msg__Position__Sequence *)(untyped_member);
  return &member->data[index];
}

void * vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__get_function__PositionList__positions(
  void * untyped_member, size_t index)
{
  vicon_receiver__msg__Position__Sequence * member =
    (vicon_receiver__msg__Position__Sequence *)(untyped_member);
  return &member->data[index];
}

void vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__fetch_function__PositionList__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const vicon_receiver__msg__Position * item =
    ((const vicon_receiver__msg__Position *)
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__get_const_function__PositionList__positions(untyped_member, index));
  vicon_receiver__msg__Position * value =
    (vicon_receiver__msg__Position *)(untyped_value);
  *value = *item;
}

void vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__assign_function__PositionList__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  vicon_receiver__msg__Position * item =
    ((vicon_receiver__msg__Position *)
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__get_function__PositionList__positions(untyped_member, index));
  const vicon_receiver__msg__Position * value =
    (const vicon_receiver__msg__Position *)(untyped_value);
  *item = *value;
}

bool vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__resize_function__PositionList__positions(
  void * untyped_member, size_t size)
{
  vicon_receiver__msg__Position__Sequence * member =
    (vicon_receiver__msg__Position__Sequence *)(untyped_member);
  vicon_receiver__msg__Position__Sequence__fini(member);
  return vicon_receiver__msg__Position__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vicon_receiver__msg__PositionList, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "n",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vicon_receiver__msg__PositionList, n),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "positions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vicon_receiver__msg__PositionList, positions),  // bytes offset in struct
    NULL,  // default value
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__size_function__PositionList__positions,  // size() function pointer
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__get_const_function__PositionList__positions,  // get_const(index) function pointer
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__get_function__PositionList__positions,  // get(index) function pointer
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__fetch_function__PositionList__positions,  // fetch(index, &value) function pointer
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__assign_function__PositionList__positions,  // assign(index, value) function pointer
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__resize_function__PositionList__positions  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_members = {
  "vicon_receiver__msg",  // message namespace
  "PositionList",  // message name
  3,  // number of fields
  sizeof(vicon_receiver__msg__PositionList),
  false,  // has_any_key_member_
  vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_member_array,  // message members
  vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_init_function,  // function to initialize message memory (memory has to be allocated)
  vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_type_support_handle = {
  0,
  &vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_members,
  get_message_typesupport_handle_function,
  &vicon_receiver__msg__PositionList__get_type_hash,
  &vicon_receiver__msg__PositionList__get_type_description,
  &vicon_receiver__msg__PositionList__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vicon_receiver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vicon_receiver, msg, PositionList)() {
  vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vicon_receiver, msg, Position)();
  if (!vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_type_support_handle.typesupport_identifier) {
    vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vicon_receiver__msg__PositionList__rosidl_typesupport_introspection_c__PositionList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
