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

#ifndef LIB_JSON_LOAD_JSON_LOAD_BASIC_H_
#define LIB_JSON_LOAD_JSON_LOAD_BASIC_H_

#include <jansson.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

void JSONLoadPrintIndexError(uint32_t i);
void JSONLoadPrintKeyError(const char *k);

json_t *JSONLoadStructGet(const json_t *obj, const char *key);
int32_t JSONLoadStructCheck(const json_t *obj, uint32_t sz);
int32_t JSONLoadArrayCheck(const json_t *obj, uint32_t sz);

bool JSONLoadBoolean(const json_t *obj, int32_t *status);
int32_t JSONLoadString(const json_t *obj, uint32_t sz, char *v);
double JSONLoadDouble(const json_t *obj, int32_t *status);
float JSONLoadFloat(const json_t *obj, int32_t *status);
uint32_t JSONLoadUInt32(const json_t *obj, int32_t *status);
int32_t JSONLoadInt32(const json_t *obj, int32_t *status);
uint16_t JSONLoadUInt16(const json_t *obj, int32_t *status);
int16_t JSONLoadInt16(const json_t *obj, int32_t *status);
uint8_t JSONLoadUInt8(const json_t *obj, int32_t *status);
int8_t JSONLoadInt8(const json_t *obj, int32_t *status);

int32_t JSONLoadArray1D_Double(const json_t *obj, uint32_t len0, double *s);
int32_t JSONLoadArray1D_Int32(const json_t *obj, uint32_t len0, int32_t *s);
int32_t JSONLoadStruct_Vec2(const json_t *obj, Vec2 *v);
int32_t JSONLoadStruct_Vec3(const json_t *obj, Vec3 *v);
int32_t JSONLoadStruct_Quat(const json_t *obj, Quat *q);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIB_JSON_LOAD_JSON_LOAD_BASIC_H_
