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

#ifndef AVIONICS_FIRMWARE_CPU_MEMCPY_H_
#define AVIONICS_FIRMWARE_CPU_MEMCPY_H_

#include <stdint.h>

// FastCopy requires 32-bit aligned source and destination addresses. For
// unaligned addressses, FastCopy falls back to WordCopy.
void FastCopy(int32_t n, const void *src, void *dst);

// WordCopy handles unaligned source and destination addresses. Call this
// function instead of FastCopy if you know that the source or destination
// address is not aligned.
void WordCopy(int32_t n, const void *src, void *dst);

#endif  // AVIONICS_FIRMWARE_CPU_MEMCPY_H_
