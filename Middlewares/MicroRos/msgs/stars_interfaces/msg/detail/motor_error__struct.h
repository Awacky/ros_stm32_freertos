// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/MotorError.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__MOTOR_ERROR__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__MOTOR_ERROR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorError in the package stars_interfaces.
typedef struct stars_interfaces__msg__MotorError
{
  uint8_t e1_core;
  uint8_t e2_core;
  uint8_t e3_core;
  uint8_t e4_core;
  uint16_t m1_temp;
  uint16_t m2_temp;
  uint16_t m3_temp;
  uint16_t m4_temp;
  float m1_vol;
  float m2_vol;
  float m3_vol;
  float m4_vol;
  float m1_cur;
  float m2_cur;
  float m3_cur;
  float m4_cur;
} stars_interfaces__msg__MotorError;

// Struct for a sequence of stars_interfaces__msg__MotorError.
typedef struct stars_interfaces__msg__MotorError__Sequence
{
  stars_interfaces__msg__MotorError * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__MotorError__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__MOTOR_ERROR__STRUCT_H_
