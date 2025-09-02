// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vicon_receiver:msg/Position.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "vicon_receiver/msg/detail/position__struct.h"
#include "vicon_receiver/msg/detail/position__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vicon_receiver__msg__position__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[38];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("vicon_receiver.msg._position.Position", full_classname_dest, 37) == 0);
  }
  vicon_receiver__msg__Position * ros_message = _ros_message;
  {  // x_trans
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_trans");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_trans = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_trans
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_trans");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_trans = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_trans
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_trans");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_trans = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_rot
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_rot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_rot = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_rot
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_rot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_rot = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_rot
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_rot");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_rot = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // w
    PyObject * field = PyObject_GetAttrString(_pymsg, "w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->w = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // x_rot_euler
    PyObject * field = PyObject_GetAttrString(_pymsg, "x_rot_euler");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->x_rot_euler = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // y_rot_euler
    PyObject * field = PyObject_GetAttrString(_pymsg, "y_rot_euler");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->y_rot_euler = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // z_rot_euler
    PyObject * field = PyObject_GetAttrString(_pymsg, "z_rot_euler");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->z_rot_euler = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // segment_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "segment_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->segment_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // subject_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "subject_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->subject_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // frame_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "frame_number");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->frame_number = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }
  {  // translation_type
    PyObject * field = PyObject_GetAttrString(_pymsg, "translation_type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->translation_type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vicon_receiver__msg__position__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Position */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vicon_receiver.msg._position");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Position");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vicon_receiver__msg__Position * ros_message = (vicon_receiver__msg__Position *)raw_ros_message;
  {  // x_trans
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_trans);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_trans", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_trans
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_trans);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_trans", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_trans
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_trans);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_trans", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_rot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_rot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_rot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_rot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_rot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_rot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_rot
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_rot);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_rot", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // w
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->w);
    {
      int rc = PyObject_SetAttrString(_pymessage, "w", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // x_rot_euler
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->x_rot_euler);
    {
      int rc = PyObject_SetAttrString(_pymessage, "x_rot_euler", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // y_rot_euler
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->y_rot_euler);
    {
      int rc = PyObject_SetAttrString(_pymessage, "y_rot_euler", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // z_rot_euler
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->z_rot_euler);
    {
      int rc = PyObject_SetAttrString(_pymessage, "z_rot_euler", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // segment_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->segment_name.data,
      strlen(ros_message->segment_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "segment_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // subject_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->subject_name.data,
      strlen(ros_message->subject_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "subject_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // frame_number
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->frame_number);
    {
      int rc = PyObject_SetAttrString(_pymessage, "frame_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // translation_type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->translation_type.data,
      strlen(ros_message->translation_type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "translation_type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
