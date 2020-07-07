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

#ifndef AVIONICS_COMMON_STRINGS_H_
#define AVIONICS_COMMON_STRINGS_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>  // For size_t.

#ifdef __cplusplus
extern "C" {
#endif

int32_t ReadDecInt32(const char *buf, int32_t max_length, int32_t *value);
int32_t ReadDecUint32(const char *buf, int32_t max_length, uint32_t *value);
// ReadDecFloat only supports -nnn.nnn, +nnn.nnn, and nnn.nnn formats.
int32_t ReadDecFloat(const char *buf, int32_t max_length, float *value);
int32_t ReadHexUint8(const char *buf, int32_t max_length, uint8_t *value);
int32_t ReadHexUint32(const char *buf, int32_t max_length, uint32_t *value);

int32_t WriteDecInt32(int32_t value, int32_t max_length, char *buf);
int32_t WriteDecUint32(uint32_t value, int32_t max_length, char *buf);
int32_t WriteHexUint32(uint32_t value, int32_t min_length, int32_t max_length,
                       char *buf);
int32_t WriteString(const char *str, int32_t max_length, char *buf);

const char *FindString(int32_t haystack_length, const char *haystack,
                       int32_t needle_length, const char *needle);
bool WildCompare(int32_t ref_length, const char *ref, int32_t wild_length,
                 const char *wild);
bool WildCompareString(const char *ref, const char *wild);
bool IsNumeric(const char *str);

// Standard library equivalent memset/memcpy functions for volatile types.
void vmemset(volatile void *s, int c, size_t n);
void vmemcpy(volatile void *dst, const volatile void *src, size_t n);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_STRINGS_H_
