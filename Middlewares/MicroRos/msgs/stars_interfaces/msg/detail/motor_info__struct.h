// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/MotorInfo.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__MOTOR_INFO__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__MOTOR_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorInfo in the package stars_interfaces.
typedef struct stars_interfaces__msg__MotorInfo
{
  int32_t m1_duty;
  int32_t m2_duty;
  int32_t m3_duty;
  int32_t m4_duty;
  int32_t m1_expectations;
  int32_t m2_expectations;
  int32_t m3_expectations;
  int32_t m4_expectations;
  int32_t m1_feedback;
  int32_t m2_feedback;
  int32_t m3_feedback;
  int32_t m4_feedback;
} stars_interfaces__msg__MotorInfo;

// Struct for a sequence of stars_interfaces__msg__MotorInfo.
typedef struct stars_interfaces__msg__MotorInfo__Sequence
{
  stars_interfaces__msg__MotorInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__MotorInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__MOTOR_INFO__STRUCT_H_
