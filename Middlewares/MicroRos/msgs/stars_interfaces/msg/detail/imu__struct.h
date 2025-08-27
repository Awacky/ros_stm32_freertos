// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_
#define STARS_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'lin_acceleration'
// Member 'ang_velocity'
// Member 'mag_field'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Imu in the package stars_interfaces.
typedef struct stars_interfaces__msg__Imu
{
  geometry_msgs__msg__Vector3 lin_acceleration;
  geometry_msgs__msg__Vector3 ang_velocity;
  geometry_msgs__msg__Vector3 mag_field;
} stars_interfaces__msg__Imu;

// Struct for a sequence of stars_interfaces__msg__Imu.
typedef struct stars_interfaces__msg__Imu__Sequence
{
  stars_interfaces__msg__Imu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__msg__Imu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__IMU__STRUCT_H_
