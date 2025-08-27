// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Velocities.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__VELOCITIES__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__VELOCITIES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Velocities in the package stars_interfaces.
typedef struct stars_interfaces__msg__Velocities
{
  float linear_x;
  float linear_y;
  float angular_z;
  float rpm_m1;
  float rpm_m2;
  float rpm_m3;
  float rpm_m4;
  float encoder_m1;
  float encoder_m2;
  float encoder_m3;
  float encoder_m4;
} stars_interfaces__msg__Velocities;

// Struct for a sequence of stars_interfaces__msg__Velocities.
typedef struct stars_interfaces__msg__Velocities__Sequence
{
  stars_interfaces__msg__Velocities * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Velocities__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__VELOCITIES__STRUCT_H_
