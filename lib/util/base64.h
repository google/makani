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

#ifndef LIB_UTIL_BASE64_H_
#define LIB_UTIL_BASE64_H_

#include <stdint.h>

// TODO: Move this library to be C++.
#ifdef __cplusplus
extern "C" {
#endif

void Base64Free(void *str);
char *Base64Encode(const void *buf, uint32_t len);
void *Base64Decode(const char *buf, uint32_t *len);

#ifdef __cplusplus
}  // extern "C"
#endif


#endif  // LIB_UTIL_BASE64_H_
