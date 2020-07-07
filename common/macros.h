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

#ifndef COMMON_MACROS_H_
#define COMMON_MACROS_H_

#include <stddef.h>  // For size_t.
#include <stdint.h>  // For int32_t.

#define ARRAYSIZE(a) \
  ((int32_t)((sizeof(a) / sizeof(*(a))) / (size_t) !(sizeof(a) % sizeof(*(a)))))

#define COMPILE_ASSERT(c, m) typedef char m[(c) ? 1 : -1]

#define UNSAFE_STRUCT_FIELD(type, field) ((const type *)0)->field
#define OFFSETOF(type, field) ((size_t)&UNSAFE_STRUCT_FIELD(type, field))
#define SIZEOF(type, field) sizeof(UNSAFE_STRUCT_FIELD(type, field))
#define UNUSED(x) (void)(x)

#ifdef __cplusplus
#ifndef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName &);              \
  void operator=(const TypeName &)
#endif
#endif

#define STR_NAME(s) #s      // Convert argument name to string.
#define STR(s) STR_NAME(s)  // Convert argument value to string.
#define UNUSED(x) (void)(x)

#endif  // COMMON_MACROS_H_
