// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Battery.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__BATTERY__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__BATTERY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Battery in the package stars_interfaces.
typedef struct stars_interfaces__msg__Battery
{
  float vol;
  float cur;
  float temp;
} stars_interfaces__msg__Battery;

// Struct for a sequence of stars_interfaces__msg__Battery.
typedef struct stars_interfaces__msg__Battery__Sequence
{
  stars_interfaces__msg__Battery * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Battery__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__BATTERY__STRUCT_H_
