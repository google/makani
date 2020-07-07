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

#include "lib/json_load/json_load_or_die.h"

#include <glog/logging.h>
#include <jansson.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "lib/json_load/json_load_basic.h"

namespace json_load {

json_t *LoadFileOrDie(const std::string &filename) {
  json_error_t err;
  json_t *f = json_load_file(filename.c_str(), 0, &err);
  CHECK(f != nullptr) << "Error loading all parameters JSON file on "
                      << "line " << err.line << ", column " << err.column
                      << ": " << err.text;
  return f;
}

json_t *LoadFieldOrDie(const json_t *obj, const std::string &key) {
  CHECK_NOTNULL(obj);
  json_t *field = JSONLoadStructGet(obj, key.c_str());
  CHECK(field != nullptr) << "Failed to load field at key " << key << ".";
  return field;
}

int32_t LoadInt32OrDie(const json_t *obj, const std::string &key) {
  json_t *field = LoadFieldOrDie(obj, key);

  int32_t status;
  int32_t value = JSONLoadInt32(field, &status);
  CHECK_EQ(status, 0) << "Failed to load int32 for key " << key;
  return value;
}

double LoadDoubleOrDie(const json_t *obj, const std::string &key) {
  json_t *field = LoadFieldOrDie(obj, key);

  int32_t status;
  double value = JSONLoadDouble(field, &status);
  CHECK_EQ(status, 0) << "Failed to load double for key " << key;
  return value;
}

void LoadArray1D_DoubleOrDie(const json_t *obj, int32_t length,
                             std::vector<double> *out) {
  CHECK_NOTNULL(obj);
  CHECK_GT(length, 0);
  CHECK_NOTNULL(out);

  if (static_cast<int32_t>(out->size()) != length) {
    out->resize(length);
  }

  CHECK_EQ(0, JSONLoadArray1D_Double(obj, length, out->data()));
}

void LoadArray1D_Int32OrDie(const json_t *obj, int32_t length,
                            std::vector<int32_t> *out) {
  CHECK_NOTNULL(obj);
  CHECK_GT(length, 0);
  CHECK_NOTNULL(out);

  if (static_cast<int32_t>(out->size()) != length) {
    out->resize(length);
  }

  CHECK_EQ(0, JSONLoadArray1D_Int32(obj, length, out->data()));
}

}  // namespace json_load
