// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Sonar.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__SONAR__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__SONAR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Sonar in the package stars_interfaces.
typedef struct stars_interfaces__msg__Sonar
{
  float sonar1;
  float sonar2;
  float sonar3;
  float sonar4;
} stars_interfaces__msg__Sonar;

// Struct for a sequence of stars_interfaces__msg__Sonar.
typedef struct stars_interfaces__msg__Sonar__Sequence
{
  stars_interfaces__msg__Sonar * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Sonar__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__SONAR__STRUCT_H_
