// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robot_interfaces:msg/MotionHealth.idl
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
#include "robot_interfaces/msg/detail/motion_health__struct.h"
#include "robot_interfaces/msg/detail/motion_health__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robot_interfaces__msg__motion_health__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("robot_interfaces.msg._motion_health.MotionHealth", full_classname_dest, 48) == 0);
  }
  robot_interfaces__msg__MotionHealth * ros_message = _ros_message;
  {  // state
    PyObject * field = PyObject_GetAttrString(_pymsg, "state");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->state, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // reason
    PyObject * field = PyObject_GetAttrString(_pymsg, "reason");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->reason, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // v
    PyObject * field = PyObject_GetAttrString(_pymsg, "v");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // w
    PyObject * field = PyObject_GetAttrString(_pymsg, "w");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->w = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // sigma_xy
    PyObject * field = PyObject_GetAttrString(_pymsg, "sigma_xy");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sigma_xy = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // sigma_theta
    PyObject * field = PyObject_GetAttrString(_pymsg, "sigma_theta");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->sigma_theta = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // drift_deg_per_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "drift_deg_per_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->drift_deg_per_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // visual_known
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_known");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->visual_known = (Py_True == field);
    Py_DECREF(field);
  }
  {  // visual_moving
    PyObject * field = PyObject_GetAttrString(_pymsg, "visual_moving");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->visual_moving = (Py_True == field);
    Py_DECREF(field);
  }
  {  // since_s
    PyObject * field = PyObject_GetAttrString(_pymsg, "since_s");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->since_s = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // imu_known
    PyObject * field = PyObject_GetAttrString(_pymsg, "imu_known");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->imu_known = (Py_True == field);
    Py_DECREF(field);
  }
  {  // pitch_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "pitch_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pitch_deg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // roll_deg
    PyObject * field = PyObject_GetAttrString(_pymsg, "roll_deg");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->roll_deg = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robot_interfaces__msg__motion_health__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MotionHealth */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robot_interfaces.msg._motion_health");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MotionHealth");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robot_interfaces__msg__MotionHealth * ros_message = (robot_interfaces__msg__MotionHealth *)raw_ros_message;
  {  // state
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->state.data,
      strlen(ros_message->state.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reason
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->reason.data,
      strlen(ros_message->reason.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "reason", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v", field);
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
  {  // sigma_xy
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sigma_xy);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sigma_xy", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sigma_theta
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->sigma_theta);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sigma_theta", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // drift_deg_per_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->drift_deg_per_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "drift_deg_per_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_known
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->visual_known ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_known", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // visual_moving
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->visual_moving ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "visual_moving", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // since_s
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->since_s);
    {
      int rc = PyObject_SetAttrString(_pymessage, "since_s", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // imu_known
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->imu_known ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "imu_known", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pitch_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pitch_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pitch_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // roll_deg
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->roll_deg);
    {
      int rc = PyObject_SetAttrString(_pymessage, "roll_deg", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
