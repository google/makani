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

#ifndef LIB_BAZEL_SWIG_TEST_ARRAY_H_
#define LIB_BAZEL_SWIG_TEST_ARRAY_H_

#include <stdint.h>

// For some reason, SWIG does not handle `const Enum *`. However,
// it handles `const int *` and `Enum *`.
// Although the function is defined in *.c as
// "double MixedPrice(const Fruit * fruits, int32_t count)",
// we use "const int *" here to make the header SWIG friendly.
double MixedPrice(const int *fruits, int32_t count);

int32_t PrintBuffer(int32_t default_len, const void *buffer, int32_t len);

#endif  // LIB_BAZEL_SWIG_TEST_ARRAY_H_
