// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Servo.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__SERVO__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__SERVO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Servo in the package stars_interfaces.
typedef struct stars_interfaces__msg__Servo
{
  float servo1;
  float servo2;
  float servo3;
  float servo4;
  float servo5;
  float servo6;
  float servo7;
  float servo8;
} stars_interfaces__msg__Servo;

// Struct for a sequence of stars_interfaces__msg__Servo.
typedef struct stars_interfaces__msg__Servo__Sequence
{
  stars_interfaces__msg__Servo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Servo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__SERVO__STRUCT_H_
