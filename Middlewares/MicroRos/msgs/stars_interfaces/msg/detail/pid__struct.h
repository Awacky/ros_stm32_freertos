// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Pid.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__PID__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__PID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pid in the package stars_interfaces.
typedef struct stars_interfaces__msg__Pid
{
  uint8_t name;
  float p;
  float d;
  float i;
} stars_interfaces__msg__Pid;

// Struct for a sequence of stars_interfaces__msg__Pid.
typedef struct stars_interfaces__msg__Pid__Sequence
{
  stars_interfaces__msg__Pid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Pid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__PID__STRUCT_H_
