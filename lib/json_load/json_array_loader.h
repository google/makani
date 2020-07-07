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

#ifndef LIB_JSON_LOAD_JSON_ARRAY_LOADER_H_
#define LIB_JSON_LOAD_JSON_ARRAY_LOADER_H_

#include <glog/logging.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <jansson.h>
#include <stdint.h>

#include <array>
#include <string>
#include <vector>

#include "common/c_math/vec3.h"
#include "lib/json_load/json_load_or_die.h"

// TODO: Put this under the lib namespace.
namespace json_load {

// Utility class for loading JSON arrays into various vector and array
// structures such as Vec3, gsl_vector, and gsl_matrix.
class JsonArrayLoader {
 public:
  JsonArrayLoader(int32_t num_rows, int32_t num_cols, json_t *root)
      : root_(CHECK_NOTNULL(root)),
        num_rows_(num_rows),
        num_cols_(num_cols),
        matrix_buffer_(num_rows),
        buffer_(0) {
    CHECK_LT(0, num_rows);
    CHECK_LT(0, num_cols);
  }

  virtual ~JsonArrayLoader() {}

  template <int32_t rank>
  void LoadVector(const std::string &field_name,
                  const std::array<int32_t, rank> &dims, gsl_vector *dest) {
    CHECK_NOTNULL(dest);
    int32_t size = 1;
    for (int32_t i = 0; i < rank; ++i) size *= dims[i];
    CHECK_EQ(size, dest->size);

    LoadBuffer(field_name, size, &buffer_);
    CopyVector(buffer_, dest);
  }

  void LoadVector(const std::string &field_name, gsl_vector *dest) {
    LoadVector<1>(field_name, {{static_cast<int32_t>(dest->size)}}, dest);
  }

  void LoadVec3(const std::string &field_name, Vec3 *dest) {
    LoadBuffer(field_name, 3, &buffer_);
    dest->x = buffer_[0];
    dest->y = buffer_[1];
    dest->z = buffer_[2];
  }

  void LoadMatrix(const std::string &field_name, gsl_matrix *dest) {
    CHECK_NOTNULL(dest);

    json_t *field = json_load::LoadFieldOrDie(root_, field_name);
    CHECK(json_is_array(field));

    LoadMatrixFromField(field, dest);
  }

 protected:
  void LoadMatrixFromField(json_t *field, gsl_matrix *dest) {
    for (int32_t i = 0; i < num_rows_; ++i) {
      json_t *slice = json_array_get(field, i);
      CHECK(json_is_array(slice));
      CHECK_EQ(num_cols_, json_array_size(slice));
      json_load::LoadArray1D_DoubleOrDie(slice, num_cols_, &matrix_buffer_[i]);

      for (int32_t j = 0; j < num_cols_; ++j) {
        gsl_matrix_set(dest, i, j, matrix_buffer_[i][j]);
      }
    }
  }

  json_t *root_;

 private:
  void LoadBuffer(const std::string &field_name, int32_t size,
                  std::vector<double> *buffer) {
    CHECK_NOTNULL(buffer);
    json_t *array = json_load::LoadFieldOrDie(root_, field_name);
    CHECK(json_is_array(array));
    CHECK_EQ(size, json_array_size(array));
    json_load::LoadArray1D_DoubleOrDie(array, size, buffer);
  }

  void CopyVector(const std::vector<double> &buffer, gsl_vector *dest) {
    CHECK_NOTNULL(dest);
    CHECK_EQ(buffer.size(), dest->size);
    for (uint32_t i = 0U; i < buffer.size(); ++i) {
      gsl_vector_set(dest, i, buffer[i]);
    }
  }

  const int32_t num_rows_;
  const int32_t num_cols_;

  std::vector<std::vector<double>> matrix_buffer_;
  std::vector<double> buffer_;

  DISALLOW_COPY_AND_ASSIGN(JsonArrayLoader);
};

}  // namespace json_load

#endif  // LIB_JSON_LOAD_JSON_ARRAY_LOADER_H_
