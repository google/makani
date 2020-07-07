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

#include "avionics/firmware/drivers/dac104s085.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/cpu/mibspi.h"

typedef struct {
  unsigned int address:2;
  unsigned int mode:2;
  unsigned int data:12;
} __attribute__((packed)) Input;

static int32_t g_device;

static inline int32_t ScaleAndBound(float x) {
  // DAC requires output on [0, 4095].
  return (int32_t)Saturatef(x * (4096.0f / 3.0f), 0.0f, 4095.0f);
}

void Dac104S085Init(void) {
  g_device
      = MibSPIAddDevice(1, 1 << 4, 20e6, 0, false, true, 8, 4 * sizeof(Input));
}

bool Dac104S085SetRaw(int32_t a, int32_t b, int32_t c, int32_t d) {
  assert(0 <= a && a <= 4095);
  assert(0 <= b && b <= 4095);
  assert(0 <= c && c <= 4095);
  assert(0 <= d && d <= 4095);

  Input input[4] = {{0, 0, a}, {1, 0, b}, {2, 0, c}, {3, 1, d}};
  if (!MibSPIWriteUint8(1, g_device, 4, sizeof(Input),
                        (const uint8_t *)input)) {
    return false;
  }
  MibSPITriggerBySoftware(1, g_device);

  return true;
}

bool Dac104S085SetVoltage(float a, float b, float c, float d) {
  return Dac104S085SetRaw(ScaleAndBound(a),
                          ScaleAndBound(b),
                          ScaleAndBound(c),
                          ScaleAndBound(d));
}
