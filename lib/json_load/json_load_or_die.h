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

// Simple wrappers for json_load_basic that die on error.

#ifndef LIB_JSON_LOAD_JSON_LOAD_OR_DIE_H_
#define LIB_JSON_LOAD_JSON_LOAD_OR_DIE_H_

#include <jansson.h>
#include <stdint.h>

#include <string>
#include <vector>

namespace json_load {

json_t *LoadFileOrDie(const std::string &filename);
json_t *LoadFieldOrDie(const json_t *obj, const std::string &key);
int32_t LoadInt32OrDie(const json_t *obj, const std::string &key);
double LoadDoubleOrDie(const json_t *obj, const std::string &key);

void LoadArray1D_DoubleOrDie(const json_t *obj, int32_t length,
                             std::vector<double> *out);
void LoadArray1D_Int32OrDie(const json_t *obj, int32_t length,
                            std::vector<int32_t> *out);

}  // namespace json_load

#endif  // LIB_JSON_LOAD_JSON_LOAD_OR_DIE_H_
