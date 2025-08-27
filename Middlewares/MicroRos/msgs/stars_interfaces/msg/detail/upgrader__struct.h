// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Upgrader.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__UPGRADER__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__UPGRADER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Upgrader in the package stars_interfaces.
typedef struct stars_interfaces__msg__Upgrader
{
  uint16_t up_cmd;
  uint16_t up_frame;
  uint16_t up_crc;
  uint32_t up_paylen;
  rosidl_runtime_c__uint8__Sequence data;
} stars_interfaces__msg__Upgrader;

// Struct for a sequence of stars_interfaces__msg__Upgrader.
typedef struct stars_interfaces__msg__Upgrader__Sequence
{
  stars_interfaces__msg__Upgrader * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Upgrader__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__UPGRADER__STRUCT_H_
