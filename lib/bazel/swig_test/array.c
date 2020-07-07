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

#include <stdio.h>

#include "lib/bazel/swig_test/fruit.h"

double MixedPrice(const Fruit *fruits, int32_t count) {
  int32_t i;
  double total_price = 0.0;
  for (i = 0; i < count; ++i) {
    switch (fruits[i]) {
      case kFruitApple:
        total_price += 0.3;
        break;
      case kFruitBanana:
        total_price += 0.2;
        break;
      case kFruitCherry:
        total_price += 0.1;
        break;
      case kFruitForceSigned:
        total_price += 0.0;
        break;
      default:
        total_price += 0.0;
        break;
    }
  }
  return total_price;
}

int32_t PrintBuffer(int32_t default_len, const void *buffer, int32_t len) {
  printf("Buffer length: %d\n%s", len, (const char *)buffer);
  return (len > 0) ? len : default_len;
}
