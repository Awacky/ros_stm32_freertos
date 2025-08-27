// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Relaid.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__RELAID__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__RELAID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'redata'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Relaid in the package stars_interfaces.
typedef struct stars_interfaces__msg__Relaid
{
  uint8_t re_type;
  uint8_t re_dlc;
  uint32_t re_id;
  rosidl_runtime_c__uint8__Sequence redata;
} stars_interfaces__msg__Relaid;

// Struct for a sequence of stars_interfaces__msg__Relaid.
typedef struct stars_interfaces__msg__Relaid__Sequence
{
  stars_interfaces__msg__Relaid * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Relaid__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__RELAID__STRUCT_H_
