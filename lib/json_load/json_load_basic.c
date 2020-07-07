/*
 * Copyright 2020 Makani Technologies LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "lib/json_load/json_load_basic.h"

#include <assert.h>
#include <float.h>
#include <jansson.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"

// Return a friendly string description of a json_type enum value.
static const char *JSONTypeToString(json_type enum_val) {
  switch (enum_val) {
    case JSON_OBJECT: return "object";
    case JSON_ARRAY: return "array";
    case JSON_STRING: return "string";
    case JSON_INTEGER: return "integer";
    case JSON_REAL: return "real";
    case JSON_TRUE:
    case JSON_FALSE: return "boolean";
    case JSON_NULL: return "null";
    default: return "unknown";
  }
}

// Print a friendly message that there was an error parsing an index.
//
// Args:
//   i: Index that had an error.
void JSONLoadPrintIndexError(uint32_t i) {
  fprintf(stderr, "Error parsing index %d.\n", i);
}

// Print a friendly message that there was an error parsing a key.
//
// Args:
//   key: Name of the field that had an error.
void JSONLoadPrintKeyError(const char *k) {
  fprintf(stderr, "Error parsing key \"%s\".\n", k);
}

// Print a friendly message when an unexpected type is encountered.
//
// This function does not itself check whether the object is not of
// the expected type.  Note that JSON_TRUE and JSON_FALSE are distinct
// types in the json_type enum, so one cannot simply compare
// json_types for equality for type checking. Instead use the helper
// macros like json_is_boolen(), as is done in the code that calls
// this function.
//
// Args:
//   found_obj:  The JSON object that was actually encountered (can be NULL).
//   expected_type: The JSON type that was expected.
static void JSONLoadPrintTypeError(const json_t *found_obj,
                                   json_type expected_type) {
  if (found_obj == NULL) {
    fprintf(stderr, "NULL pointer encountered but JSON %s expected.\n",
            JSONTypeToString(expected_type));
    return;
  }

  json_type found_type = json_typeof(found_obj);
  fprintf(stderr, "JSON %s found but %s expected.\n",
          JSONTypeToString(found_type),
          JSONTypeToString(expected_type));
}

// Fetch a given key from a JSON struct.
//
// Args:
//   obj: Pointer to a JSON object.
//   key: Name of the field to fetch.
//
// Returns:
//   Pointer to the value associated with the key or NULL if
//   no such key is found.
json_t *JSONLoadStructGet(const json_t *obj, const char *key) {
  json_t *value = json_object_get(obj, key);
  if (value == NULL) fprintf(stderr, "Key not found: %s.\n", key);
  return value;
}

// Check that a json_t pointer is an object with a given number of fields.
//
// Args:
//   obj: Pointer to test.
//   num_fields: Expected number of fields.
//
// Returns:
//   0 if obj points to an array of the given length, -1 otherwise.
int32_t JSONLoadStructCheck(const json_t *obj, uint32_t num_fields) {
  if (obj == NULL) {
    fprintf(stderr, "NULL pointer to JSON struct.\n");
    return -1;
  } else if (!json_is_object(obj)) {
    JSONLoadPrintTypeError(obj, JSON_OBJECT);
    return -1;
  } else if (num_fields != json_object_size(obj)) {
    fprintf(stderr, "JSON object has %zu fields, expected %d.\n",
            json_object_size(obj), num_fields);
    return -1;
  } else {
    return 0;
  }
}

// Check that a json_t pointer is an array of a specific length.
//
// Args:
//   obj: Pointer to test.
//   sz: Expected array length.
//
// Returns:
//   0 if obj points to an array of the given length, -1 otherwise.
int32_t JSONLoadArrayCheck(const json_t *obj, uint32_t sz) {
  if (obj == NULL) {
    fprintf(stderr, "NULL pointer to JSON array.\n");
    return -1;
  } else if (!json_is_array(obj)) {
    JSONLoadPrintTypeError(obj, JSON_ARRAY);
    return -1;
  } else if (sz != json_array_size(obj)) {
    fprintf(stderr, "JSON array has %zu elements, expected %d.\n",
            json_array_size(obj), sz);
    return -1;
  } else {
    return 0;
  }
}

// Retrieve a string of a given size from a JSON object.
//
// Args:
//   obj: Pointer to a JSON string.
//   str: Pointer to an array of at least sz+1 characters to be written to.
//   sz: Maximum length of the string to be written (not including
//       '\0' termination).
//
// Returns:
//   0 if there was no error, -1 otherwise.
int32_t JSONLoadString(const json_t *obj, uint32_t sz, char *str) {
  assert(str != NULL);
  if (str == NULL) {
    return -1;
  } else if (obj == NULL) {
    fprintf(stderr, "NULL pointer to JSON string.\n");
    return -1;
  } else if (!json_is_string(obj)) {
    JSONLoadPrintTypeError(obj, JSON_STRING);
    return -1;
  } else if (sz == UINT32_MAX) {
    fprintf(stderr, "Requested string length is too long.\n");
    return -1;
  } else {
    const char *json_str = json_string_value(obj);
    size_t len = strlen(json_str);
    if (len > sz) {
      fprintf(stderr, "JSON string value is too long.\n");
      return -1;
    }
    strncpy(str, json_str, len);
    str[len] = '\0';
    return 0;
  }
}

// Retrieve a bool from a JSON Boolean.
//
// Args:
//   obj: Pointer to a JSON Boolean.
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON Boolean value if there is no error, false otherwise.
bool JSONLoadBoolean(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return false;
  } else if ((obj == NULL) || !json_is_boolean(obj)) {
    // JSONLoadPrintTypeError understands that we mean "boolean" even though we
    // say JSON_TRUE.
    JSONLoadPrintTypeError(obj, JSON_TRUE);
    *status = -1;
    return false;
  }

  *status = 0;
  return json_is_true(obj);
}

// Retrieve a double from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON floating point number.
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON floating point value if there is no error, 0.0 otherwise.
double JSONLoadDouble(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0.0;
  } else if (obj == NULL || !json_is_real(obj)) {
    JSONLoadPrintTypeError(obj, JSON_REAL);
    *status = -1;
    return 0.0;
  }
  *status = 0;
  return json_real_value(obj);
}

// Retrieve a float from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON floating point number.
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON integer value if there is no error, otherwise 0.
float JSONLoadFloat(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0.0f;
  } else if (obj == NULL || !json_is_real(obj)) {
    JSONLoadPrintTypeError(obj, JSON_REAL);
    *status = -1;
    return 0.0f;
  }

  double value = json_real_value(obj);
  double mag = fabs(value);
  // TODO: Handle precision more carefully.
  if (mag < FLT_MIN || mag > FLT_MAX) {
    fprintf(stderr, "JSON float value is out of range.\n");
    *status = -1;
    return 0.0f;
  }

  *status = 0;
  return (float)value;
}

// Retrieve a uint32_t from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON integer in the range [0, UINT32_MAX].
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON integer value if there is no, otherwise 0.
uint32_t JSONLoadUInt32(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0U;
  } else if (obj == NULL || !json_is_integer(obj)) {
    JSONLoadPrintTypeError(obj, JSON_INTEGER);
    *status = -1;
    return 0U;
  }

  json_int_t value = json_integer_value(obj);
  if (value > UINT32_MAX || value < 0) {
    fprintf(stderr, "JSON uint32_t is out of range.\n");
    *status = -1;
    return 0U;
  }

  *status = 0;
  return (uint32_t)value;
}

// Retrieve an int32_t from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON integer in the range [INT32_MIN, INT32_MAX].
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON integer value if there is no error, otherwise 0.
int32_t JSONLoadInt32(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0;
  } else if (obj == NULL || !json_is_integer(obj)) {
    JSONLoadPrintTypeError(obj, JSON_INTEGER);
    *status = -1;
    return 0;
  }

  json_int_t value = json_integer_value(obj);
  if (value > INT32_MAX || value < INT32_MIN) {
    fprintf(stderr, "JSON int32_t is out of range.\n");
    *status = -1;
    return 0;
  }

  *status = 0;
  return (int32_t)value;
}

// Retrieve an uint16_t from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON integer in the range [0, UINT16_MAX].
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON integer value if there is no error, otherwise 0.
uint16_t JSONLoadUInt16(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0U;
  } else if (obj == NULL || !json_is_integer(obj)) {
    JSONLoadPrintTypeError(obj, JSON_INTEGER);
    *status = -1;
    return 0U;
  }

  json_int_t value = json_integer_value(obj);
  if (value < 0 || value > UINT16_MAX) {
    fprintf(stderr, "JSON uint16_t is out of range.\n");
    *status = -1;
    return 0U;
  }

  *status = 0;
  return (uint16_t)value;
}

// Retrieve an int16_t from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON integer in the range [INT16_MIN, INT16_MAX].
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON integer value if there is no error, otherwise 0.
int16_t JSONLoadInt16(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0;
  } else if (obj == NULL || !json_is_integer(obj)) {
    JSONLoadPrintTypeError(obj, JSON_INTEGER);
    *status = -1;
    return 0;
  }

  json_int_t value = json_integer_value(obj);
  if (value > INT16_MAX || value < INT16_MIN) {
    fprintf(stderr, "JSON int16_t is out of range.\n");
    *status = -1;
    return 0;
  }

  *status = 0;
  return (int16_t)value;
}

// Retrieve an uint8_t from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON integer in the range [0, UINT8_MAX].
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON integer value if there is no error, otherwise 0.
uint8_t JSONLoadUInt8(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0U;
  } else if (obj == NULL || !json_is_integer(obj)) {
    JSONLoadPrintTypeError(obj, JSON_INTEGER);
    *status = -1;
    return 0U;
  }

  json_int_t value = json_integer_value(obj);
  if (value < 0 || value > UINT8_MAX) {
    *status = -1;
    return 0U;
  }

  *status = 0;
  return (uint8_t)value;
}

// Retrieve an int8_t from a JSON integer.
//
// Args:
//   obj: Pointer to a JSON integer in the range [INT8_MIN, INT8_MAX].
//   status: 0 is written to this variable if there is no error (otherwise -1).
//
// Returns:
//   The JSON integer value if there is no error, otherwise 0.
int8_t JSONLoadInt8(const json_t *obj, int32_t *status) {
  if (status == NULL) {
    assert(false);
    return 0;
  } else if (obj == NULL || !json_is_integer(obj)) {
    JSONLoadPrintTypeError(obj, JSON_INTEGER);
    *status = -1;
    return 0;
  }

  json_int_t value = json_integer_value(obj);
  if (value > INT8_MAX || value < INT8_MIN) {
    *status = -1;
    return 0;
  }

  *status = 0;
  return (int8_t)value;
}

// Load a JSON array of a given number of doubles into an array.
//
// Args:
//   obj: A pointer to a JSON array of length len0 containing doubles.
//   s: Array to write the values to (should have length len0).
//   len0: Expected number of values to write.
//
// Returns:
//   0 if there is no error, -1 otherwise.
int32_t JSONLoadArray1D_Double(const json_t *obj, uint32_t len0, double *s) {
  if (JSONLoadArrayCheck(obj, len0) < 0) return -1;
  int32_t status;
  for (uint32_t i = 0U; i < len0; ++i) {
    s[i] = JSONLoadDouble(json_array_get(obj, i), &status);
    if (status == -1) {
      JSONLoadPrintIndexError(i);
      return -1;
    }
  }
  return 0;
}

// Load a JSON array of a given number of int32s into an array.
//
// Args:
//   obj: A pointer to a JSON array of length len0 containing int32s.
//   s: Array to write the values to (should have length len0).
//   len0: Expected number of values to write.
//
// Returns:
//   0 if there is no error, -1 otherwise.
int32_t JSONLoadArray1D_Int32(const json_t *obj, uint32_t len0, int32_t *s) {
  if (JSONLoadArrayCheck(obj, len0) < 0) return -1;
  int32_t status;
  for (uint32_t i = 0U; i < len0; ++i) {
    s[i] = JSONLoadInt32(json_array_get(obj, i), &status);
    if (status == -1) {
      JSONLoadPrintIndexError(i);
      return -1;
    }
  }
  return 0;
}

// Load a JSON array of three doubles into a Vec2 struct.
//
// Args:
//   obj: A pointer to a JSON array of two floating point numbers.
//   v: Vec2 to populate.
//
// Returns:
//   0 if there is no error, -1 otherwise.
int32_t JSONLoadStruct_Vec2(const json_t *obj, Vec2 *v) {
  assert(v != NULL);
  if (v == NULL) return -1;
  double values[2];
  int32_t status = JSONLoadArray1D_Double(obj, 2, &values[0]);
  if (status < 0) return -1;
  v->x = values[0];
  v->y = values[1];
  return 0;
}

// Load a JSON array of three elements into a Vec3 struct.
//
// Args:
//   obj: A pointer to a JSON array of three floating point numbers.
//   v: Vec3 to populate.
//
// Returns:
//   0 if there is no error, -1 otherwise.
int32_t JSONLoadStruct_Vec3(const json_t *obj, Vec3 *v) {
  assert(v != NULL);
  if (v == NULL) return -1;
  double values[3];
  int32_t status = JSONLoadArray1D_Double(obj, 3, &values[0]);
  if (status < 0) return -1;
  v->x = values[0];
  v->y = values[1];
  v->z = values[2];
  return 0;
}

// Load a JSON array of four elements into a Quat struct.
//
// Args:
//   obj: A pointer to a JSON array of four floating point numbers.
//   q: Quaternion to populate.
//
// Returns:
//   0 if there is no error, -1 otherwise.
int32_t JSONLoadStruct_Quat(const json_t *obj, Quat *q) {
  assert(q != NULL);
  if (q == NULL) return -1;
  double values[4];
  int32_t status = JSONLoadArray1D_Double(obj, 4, &values[0]);
  if (status < 0) return -1;
  q->q0 = values[0];
  q->q1 = values[1];
  q->q2 = values[2];
  q->q3 = values[3];
  return 0;
}
