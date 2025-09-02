// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "vicon_receiver/msg/detail/position_list__functions.h"
#include "vicon_receiver/msg/detail/position_list__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace vicon_receiver
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PositionList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) vicon_receiver::msg::PositionList(_init);
}

void PositionList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<vicon_receiver::msg::PositionList *>(message_memory);
  typed_message->~PositionList();
}

size_t size_function__PositionList__positions(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<vicon_receiver::msg::Position> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PositionList__positions(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<vicon_receiver::msg::Position> *>(untyped_member);
  return &member[index];
}

void * get_function__PositionList__positions(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<vicon_receiver::msg::Position> *>(untyped_member);
  return &member[index];
}

void fetch_function__PositionList__positions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const vicon_receiver::msg::Position *>(
    get_const_function__PositionList__positions(untyped_member, index));
  auto & value = *reinterpret_cast<vicon_receiver::msg::Position *>(untyped_value);
  value = item;
}

void assign_function__PositionList__positions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<vicon_receiver::msg::Position *>(
    get_function__PositionList__positions(untyped_member, index));
  const auto & value = *reinterpret_cast<const vicon_receiver::msg::Position *>(untyped_value);
  item = value;
}

void resize_function__PositionList__positions(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<vicon_receiver::msg::Position> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PositionList_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vicon_receiver::msg::PositionList, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "n",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vicon_receiver::msg::PositionList, n),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "positions",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<vicon_receiver::msg::Position>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vicon_receiver::msg::PositionList, positions),  // bytes offset in struct
    nullptr,  // default value
    size_function__PositionList__positions,  // size() function pointer
    get_const_function__PositionList__positions,  // get_const(index) function pointer
    get_function__PositionList__positions,  // get(index) function pointer
    fetch_function__PositionList__positions,  // fetch(index, &value) function pointer
    assign_function__PositionList__positions,  // assign(index, value) function pointer
    resize_function__PositionList__positions  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PositionList_message_members = {
  "vicon_receiver::msg",  // message namespace
  "PositionList",  // message name
  3,  // number of fields
  sizeof(vicon_receiver::msg::PositionList),
  false,  // has_any_key_member_
  PositionList_message_member_array,  // message members
  PositionList_init_function,  // function to initialize message memory (memory has to be allocated)
  PositionList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PositionList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PositionList_message_members,
  get_message_typesupport_handle_function,
  &vicon_receiver__msg__PositionList__get_type_hash,
  &vicon_receiver__msg__PositionList__get_type_description,
  &vicon_receiver__msg__PositionList__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace vicon_receiver


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<vicon_receiver::msg::PositionList>()
{
  return &::vicon_receiver::msg::rosidl_typesupport_introspection_cpp::PositionList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, vicon_receiver, msg, PositionList)() {
  return &::vicon_receiver::msg::rosidl_typesupport_introspection_cpp::PositionList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
