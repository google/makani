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

#include <stdint.h>
#include "lib/bazel/swig_test/fruit.h"
#include "lib/bazel/swig_test/utils.h"

double Scale(uint16_t raw, double factor) {
  return (double) raw * factor;
}

Produce WholesalePrice(double price, uint16_t count) {
  Produce produce;
  produce.fruit = kFruitApple;
  produce.price = price;
  produce.count = count;
  produce.total = price * count;
  return produce;
}

void TotalPrice(int32_t count, const double *price, double *total) {
  *total = (double) count * (*price);
}
