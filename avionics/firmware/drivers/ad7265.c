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

// NOTE: The Ad7265 datasheet specifies a SPI clock frequency of 16MHz
// and mode 3. Reality specifies a SPI clock frequency of 12MHz and mode 0.

#include "avionics/firmware/drivers/ad7265.h"

#include <stdint.h>

#include "avionics/firmware/cpu/mibspi.h"
#include "common/macros.h"

static int32_t g_device;

void Ad7265Init(void) {
  // Setup 9 8-bit word transfers.
  g_device = MibSPIAddDevice(3, 1 << 1 | 1 << 5, 12e6, 0, false, false, 8, 9);

  // Write zeros.
  MibSPIWriteConst(3, g_device, 1, 9, 0);

  // Channel 2: hold chip select 1 low for words 0 to 3.
  MibSPISetChipSelect(3, g_device, 1 << 1, true, 0, 4);

  // Acq delay: hold chip select 5 low for words 4 to 8.
  MibSPISetChipSelect(3, g_device, 1 << 5, true, 4, 5);

  // Channel 1: hold chip select 1 and 5 low for words 5 to 8.
  MibSPISetChipSelect(3, g_device, 1 << 1 | 1 << 5, true, 5, 4);
}

bool Ad7265ReadAsyncStart(void) {
  return MibSPITriggerBySoftware(3, g_device);
}

bool Ad7265ReadAsyncGet(int32_t *a1, int32_t *b1, int32_t *a2, int32_t *b2) {
  uint8_t miso[9];

  if (!MibSPIReadUint8(3, g_device, 1, ARRAYSIZE(miso), miso)) {
    return false;
  }
  *a2 = (miso[0] << 6 | miso[1] >> 2) & 0x0FFF;  // Signed 12-bit result.
  if ((*a2 & 0x0800) != 0) {
    *a2 |= 0xFFFFF000;
  }

  *b2 = (miso[2] << 6 | miso[3] >> 2) & 0x0FFF;  // Signed 12-bit result.
  if ((*b2 & 0x0800) != 0) {
    *b2 |= 0xFFFFF000;
  }

  // Ignore miso[4].

  *a1 = (miso[5] << 6 | miso[6] >> 2) & 0x0FFF;  // Signed 12-bit result.
  if ((*a1 & 0x0800) != 0) {
    *a1 |= 0xFFFFF000;
  }

  *b1 = (miso[7] << 6 | miso[8] >> 2) & 0x0FFF;  // Signed 12-bit result.
  if ((*b1 & 0x0800) != 0) {
    *b1 |= 0xFFFFF000;
  }

  return true;
}

void Ad7265Read(int32_t *a1, int32_t *b1, int32_t *a2, int32_t *b2) {
  while (!Ad7265ReadAsyncStart()) {}
  while (!Ad7265ReadAsyncGet(a1, b1, a2, b2)) {}
}
