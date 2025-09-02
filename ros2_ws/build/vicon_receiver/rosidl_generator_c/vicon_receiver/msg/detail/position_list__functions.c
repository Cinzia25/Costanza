// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vicon_receiver:msg/PositionList.idl
// generated code does not contain a copyright notice
#include "vicon_receiver/msg/detail/position_list__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `positions`
#include "vicon_receiver/msg/detail/position__functions.h"

bool
vicon_receiver__msg__PositionList__init(vicon_receiver__msg__PositionList * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    vicon_receiver__msg__PositionList__fini(msg);
    return false;
  }
  // n
  // positions
  if (!vicon_receiver__msg__Position__Sequence__init(&msg->positions, 0)) {
    vicon_receiver__msg__PositionList__fini(msg);
    return false;
  }
  return true;
}

void
vicon_receiver__msg__PositionList__fini(vicon_receiver__msg__PositionList * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // n
  // positions
  vicon_receiver__msg__Position__Sequence__fini(&msg->positions);
}

bool
vicon_receiver__msg__PositionList__are_equal(const vicon_receiver__msg__PositionList * lhs, const vicon_receiver__msg__PositionList * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // n
  if (lhs->n != rhs->n) {
    return false;
  }
  // positions
  if (!vicon_receiver__msg__Position__Sequence__are_equal(
      &(lhs->positions), &(rhs->positions)))
  {
    return false;
  }
  return true;
}

bool
vicon_receiver__msg__PositionList__copy(
  const vicon_receiver__msg__PositionList * input,
  vicon_receiver__msg__PositionList * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // n
  output->n = input->n;
  // positions
  if (!vicon_receiver__msg__Position__Sequence__copy(
      &(input->positions), &(output->positions)))
  {
    return false;
  }
  return true;
}

vicon_receiver__msg__PositionList *
vicon_receiver__msg__PositionList__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vicon_receiver__msg__PositionList * msg = (vicon_receiver__msg__PositionList *)allocator.allocate(sizeof(vicon_receiver__msg__PositionList), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vicon_receiver__msg__PositionList));
  bool success = vicon_receiver__msg__PositionList__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vicon_receiver__msg__PositionList__destroy(vicon_receiver__msg__PositionList * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vicon_receiver__msg__PositionList__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vicon_receiver__msg__PositionList__Sequence__init(vicon_receiver__msg__PositionList__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vicon_receiver__msg__PositionList * data = NULL;

  if (size) {
    data = (vicon_receiver__msg__PositionList *)allocator.zero_allocate(size, sizeof(vicon_receiver__msg__PositionList), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vicon_receiver__msg__PositionList__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vicon_receiver__msg__PositionList__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
vicon_receiver__msg__PositionList__Sequence__fini(vicon_receiver__msg__PositionList__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      vicon_receiver__msg__PositionList__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

vicon_receiver__msg__PositionList__Sequence *
vicon_receiver__msg__PositionList__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vicon_receiver__msg__PositionList__Sequence * array = (vicon_receiver__msg__PositionList__Sequence *)allocator.allocate(sizeof(vicon_receiver__msg__PositionList__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vicon_receiver__msg__PositionList__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vicon_receiver__msg__PositionList__Sequence__destroy(vicon_receiver__msg__PositionList__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vicon_receiver__msg__PositionList__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vicon_receiver__msg__PositionList__Sequence__are_equal(const vicon_receiver__msg__PositionList__Sequence * lhs, const vicon_receiver__msg__PositionList__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vicon_receiver__msg__PositionList__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vicon_receiver__msg__PositionList__Sequence__copy(
  const vicon_receiver__msg__PositionList__Sequence * input,
  vicon_receiver__msg__PositionList__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vicon_receiver__msg__PositionList);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vicon_receiver__msg__PositionList * data =
      (vicon_receiver__msg__PositionList *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vicon_receiver__msg__PositionList__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vicon_receiver__msg__PositionList__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vicon_receiver__msg__PositionList__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
