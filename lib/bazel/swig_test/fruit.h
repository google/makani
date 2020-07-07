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

#ifndef LIB_BAZEL_SWIG_TEST_FRUIT_H_
#define LIB_BAZEL_SWIG_TEST_FRUIT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kFruitForceSigned = -1,
  kFruitApple,
  kFruitBanana,
  kFruitCherry,
} Fruit;

typedef struct {
  Fruit fruit;
  double price;
  uint16_t count;
  double total;
} Produce;

Produce WholesalePrice(double price, uint16_t count);

void TotalPrice(int32_t count, const double *price, double *total);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // LIB_BAZEL_SWIG_TEST_FRUIT_H_
