// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from stars_interfaces:msg/Upgrader.idl
// generated code does not contain a copyright notice

#ifndef STARS_INTERFACES__MSG__DETAIL__UPGRADER__FUNCTIONS_H_
#define STARS_INTERFACES__MSG__DETAIL__UPGRADER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "stars_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "stars_interfaces/msg/detail/upgrader__struct.h"

/// Initialize msg/Upgrader message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * stars_interfaces__msg__Upgrader
 * )) before or use
 * stars_interfaces__msg__Upgrader__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
bool
stars_interfaces__msg__Upgrader__init(stars_interfaces__msg__Upgrader * msg);

/// Finalize msg/Upgrader message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
void
stars_interfaces__msg__Upgrader__fini(stars_interfaces__msg__Upgrader * msg);

/// Create msg/Upgrader message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * stars_interfaces__msg__Upgrader__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
stars_interfaces__msg__Upgrader *
stars_interfaces__msg__Upgrader__create();

/// Destroy msg/Upgrader message.
/**
 * It calls
 * stars_interfaces__msg__Upgrader__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
void
stars_interfaces__msg__Upgrader__destroy(stars_interfaces__msg__Upgrader * msg);

/// Check for msg/Upgrader message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
bool
stars_interfaces__msg__Upgrader__are_equal(const stars_interfaces__msg__Upgrader * lhs, const stars_interfaces__msg__Upgrader * rhs);

/// Copy a msg/Upgrader message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
bool
stars_interfaces__msg__Upgrader__copy(
  const stars_interfaces__msg__Upgrader * input,
  stars_interfaces__msg__Upgrader * output);

/// Initialize array of msg/Upgrader messages.
/**
 * It allocates the memory for the number of elements and calls
 * stars_interfaces__msg__Upgrader__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
bool
stars_interfaces__msg__Upgrader__Sequence__init(stars_interfaces__msg__Upgrader__Sequence * array, size_t size);

/// Finalize array of msg/Upgrader messages.
/**
 * It calls
 * stars_interfaces__msg__Upgrader__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
void
stars_interfaces__msg__Upgrader__Sequence__fini(stars_interfaces__msg__Upgrader__Sequence * array);

/// Create array of msg/Upgrader messages.
/**
 * It allocates the memory for the array and calls
 * stars_interfaces__msg__Upgrader__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
stars_interfaces__msg__Upgrader__Sequence *
stars_interfaces__msg__Upgrader__Sequence__create(size_t size);

/// Destroy array of msg/Upgrader messages.
/**
 * It calls
 * stars_interfaces__msg__Upgrader__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
void
stars_interfaces__msg__Upgrader__Sequence__destroy(stars_interfaces__msg__Upgrader__Sequence * array);

/// Check for msg/Upgrader message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
bool
stars_interfaces__msg__Upgrader__Sequence__are_equal(const stars_interfaces__msg__Upgrader__Sequence * lhs, const stars_interfaces__msg__Upgrader__Sequence * rhs);

/// Copy an array of msg/Upgrader messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_stars_interfaces
bool
stars_interfaces__msg__Upgrader__Sequence__copy(
  const stars_interfaces__msg__Upgrader__Sequence * input,
  stars_interfaces__msg__Upgrader__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // STARS_INTERFACES__MSG__DETAIL__UPGRADER__FUNCTIONS_H_
