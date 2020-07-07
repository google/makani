// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sim/json_formatter.h"

#include <assert.h>
#include <jansson.h>
#include <stdint.h>
#include <string.h>
#include <string>

#include "lib/json_load/json_load_or_die.h"
#include "sim/interfaces.h"

JsonFormatter::JsonFormatter(const JsonFormatter &fmt) : root_(fmt.root_) {
  json_incref(root_);
}

JsonFormatter::JsonFormatter(const std::string &s)
    : root_(json_load::LoadFileOrDie(s)) {}

JsonFormatter::~JsonFormatter() { json_decref(root_); }

int32_t JsonFormatter::WriteFile(const std::string &path) {
  return json_dump_file(root_, path.c_str(), JSON_INDENT(2));
}

DictFormatterInterface *JsonFormatter::CreateChild(const std::string &key,
                                                   int32_t *error) {
  json_t *child = json_object_get(root_, key.c_str());

  if (child == nullptr) {
    child = json_object();
    int32_t err = SetWithChecks(root_, key.c_str(), child);

    if (err) *error = err;
  }
  return new JsonFormatter(child);
}

int32_t JsonFormatter::Set(const std::string &key, double val) {
  json_t *value = json_real(val);
  return SetWithChecks(root_, key.c_str(), value);
}

int32_t JsonFormatter::Set(const std::string &key, bool val) {
  json_t *value = val ? json_true() : json_false();
  return SetWithChecks(root_, key.c_str(), value);
}

int32_t JsonFormatter::Set(const std::string &key, int32_t val) {
  json_t *value = json_integer(val);
  return SetWithChecks(root_, key.c_str(), value);
}

int32_t JsonFormatter::Set(const std::string &key, uint64_t val) {
  json_t *value = json_integer(val);
  return SetWithChecks(root_, key.c_str(), value);
}

int32_t JsonFormatter::Set(const std::string &key, const char *val) {
  json_t *value = json_string(val);
  return SetWithChecks(root_, key.c_str(), value);
}

int32_t JsonFormatter::Set(const std::string &key, const std::string &val) {
  return Set(key, val.c_str());
}

int32_t JsonFormatter::Set(const std::string &key, const double *values,
                           uint32_t length) {
  json_t *array = json_array();
  int32_t error = SetWithChecks(root_, key.c_str(), array);

  if (error) return error;

  for (uint32_t i = 0U; i < length; ++i) {
    json_t *value = json_real(values[i]);

    if (value == nullptr) return kDictFormatterErrorAllocation;

    error = json_array_append_new(array, value);
    if (error) return kDictFormatterErrorInternal;
  }

  return kDictFormatterErrorNone;
}

const DictFormatterInterface *JsonFormatter::GetChild(const std::string &key,
                                                      int32_t *error) const {
  json_t *value = json_object_get(root_, key.c_str());

  if (value == nullptr) {
    *error = kDictFormatterErrorKeyNotFound;
    return nullptr;
  } else if (json_is_object(value)) {
    *error = kDictFormatterErrorNone;
    return new JsonFormatter(value);
  } else {
    *error = kDictFormatterErrorIncorrectType;
    return nullptr;
  }
}

int32_t JsonFormatter::Get(const std::string &key, double *val) const {
  json_t *value = json_object_get(root_, key.c_str());

  if (value == nullptr) {
    return kDictFormatterErrorKeyNotFound;
  } else if (json_is_real(value)) {
    *val = json_real_value(value);
    return kDictFormatterErrorNone;
  } else if (json_is_integer(value)) {
    *val = static_cast<double>(json_integer_value(value));
    return kDictFormatterErrorNone;
  } else {
    return kDictFormatterErrorIncorrectType;
  }
}

int32_t JsonFormatter::Get(const std::string &key, bool *val) const {
  json_t *value = json_object_get(root_, key.c_str());

  if (value == nullptr) {
    return kDictFormatterErrorKeyNotFound;
  } else if (json_is_true(value)) {
    *val = true;
    return kDictFormatterErrorNone;
  } else if (json_is_false(value)) {
    *val = false;
    return kDictFormatterErrorNone;
  } else {
    return kDictFormatterErrorIncorrectType;
  }
}

int32_t JsonFormatter::Get(const std::string &key, int32_t *val) const {
  json_t *value = json_object_get(root_, key.c_str());

  if (value == nullptr) {
    return kDictFormatterErrorKeyNotFound;
  } else if (json_is_integer(value)) {
    *val = static_cast<int32_t>(json_integer_value(value));
    return kDictFormatterErrorNone;
  } else {
    return kDictFormatterErrorIncorrectType;
  }
}

int32_t JsonFormatter::Get(const std::string &key, uint64_t *val) const {
  json_t *value = json_object_get(root_, key.c_str());

  if (value == nullptr) {
    return kDictFormatterErrorKeyNotFound;
  } else if (json_is_integer(value)) {
    *val = static_cast<uint64_t>(json_integer_value(value));
    return kDictFormatterErrorNone;
  } else {
    return kDictFormatterErrorIncorrectType;
  }
}

int32_t JsonFormatter::Get(const std::string &key, const char **vals,
                           uint32_t *length) const {
  json_t *value = json_object_get(root_, key.c_str());

  if (value == nullptr) {
    return kDictFormatterErrorKeyNotFound;
  } else if (json_is_string(value)) {
    *vals = json_string_value(value);
    *length = static_cast<uint32_t>(strlen(*vals));
    return kDictFormatterErrorNone;
  } else {
    return kDictFormatterErrorIncorrectType;
  }
}

int32_t JsonFormatter::Get(const std::string &key, double *vals,
                           uint32_t expected_length) const {
  json_t *array = json_object_get(root_, key.c_str());

  if (array == nullptr) {
    return kDictFormatterErrorKeyNotFound;
  } else if (json_is_array(array)) {
    if (expected_length != json_array_size(array))
      return kDictFormatterErrorIncorrectSize;

    for (uint32_t i = 0U; i < expected_length; ++i) {
      json_t *val_i = json_array_get(array, i);
      if (!json_is_real(val_i)) return kDictFormatterErrorIncorrectType;
      vals[i] = json_real_value(val_i);
    }

    return kDictFormatterErrorNone;
  } else {
    return kDictFormatterErrorIncorrectType;
  }
}

int32_t JsonFormatter::SetWithChecks(json_t *object, const std::string &key,
                                     json_t *value) {
  if (value == nullptr) return kDictFormatterErrorAllocation;

  int32_t error = json_object_set_new(object, key.c_str(), value);

  return error ? kDictFormatterErrorInternal : kDictFormatterErrorNone;
}
