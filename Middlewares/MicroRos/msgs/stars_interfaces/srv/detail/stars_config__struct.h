// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from stars_interfaces:srv/StarsConfig.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__SRV__DETAIL__STARS_CONFIG__STRUCT_H_
#define STARS_INTERFACES__SRV__DETAIL__STARS_CONFIG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'key'
// Member 'value'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StarsConfig in the package stars_interfaces.
typedef struct stars_interfaces__srv__StarsConfig_Request
{
  rosidl_runtime_c__String key;
  rosidl_runtime_c__String value;
} stars_interfaces__srv__StarsConfig_Request;

// Struct for a sequence of stars_interfaces__srv__StarsConfig_Request.
typedef struct stars_interfaces__srv__StarsConfig_Request__Sequence
{
  stars_interfaces__srv__StarsConfig_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__srv__StarsConfig_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'key'
// Member 'value'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/StarsConfig in the package stars_interfaces.
typedef struct stars_interfaces__srv__StarsConfig_Response
{
  rosidl_runtime_c__String key;
  rosidl_runtime_c__String value;
} stars_interfaces__srv__StarsConfig_Response;

// Struct for a sequence of stars_interfaces__srv__StarsConfig_Response.
typedef struct stars_interfaces__srv__StarsConfig_Response__Sequence
{
  stars_interfaces__srv__StarsConfig_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} stars_interfaces__srv__StarsConfig_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__SRV__DETAIL__STARS_CONFIG__STRUCT_H_
