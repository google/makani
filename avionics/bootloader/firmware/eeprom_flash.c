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

#include "avionics/bootloader/firmware/eeprom_flash.h"

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "avionics/firmware/cpu/adc.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/eeprom24.h"
#include "avionics/firmware/drivers/ext_watchdog.h"
#include "common/macros.h"

#define ADC_MODULE 1
#define EEPROM_WP_CHANNEL 23

static bool EepromVerify(const FlashSegment *segment, int32_t offset,
                         int32_t data_len, const uint8_t *data) {
  uint8_t buf[16];
  const int32_t buf_len = (int32_t)sizeof(buf);
  int32_t verified_len = 0;

  while (verified_len < data_len) {
    int32_t remaining_len = data_len - verified_len;
    int32_t op_len = remaining_len > buf_len ? buf_len : remaining_len;

    ExtWatchdogPoll();

    if (!Eeprom24ReadSync(segment->begin_address + offset + verified_len,
                          op_len, buf)) {
      return false;
    }
    if (memcmp(data + verified_len, buf, op_len)) {
      return false;
    }
    verified_len += op_len;
  }
  return true;
}

void EepromFlashInit(const FlashSegment *segment) {
  (void)segment;

  AdcInit(ADC_MODULE);
  I2cInit(400e3);
  Eeprom24Init();
}

bool EepromFlashEraseSectors(const FlashSegment *segment) {
  // The eeprom does not need to be erased before writing.
  (void)segment;
  return true;
}

bool EepromFlashWrite(const FlashSegment *segment, int32_t offset,
                      int32_t data_len, const uint8_t *data) {
  // A single iteration completes in plenty of time to keep the watchdog
  // happy.  However, with a large enough multi-part write, we see the
  // watchdog fire.
  ExtWatchdogPoll();

  int16_t adc_result[ADC_CHANNELS];
  AdcTriggerGroup1(ADC_MODULE, 1 << EEPROM_WP_CHANNEL);
  while (!AdcReadGroup1(ADC_MODULE, ARRAYSIZE(adc_result), adc_result)) {}

  // Write protect is active high.
  if (adc_result[EEPROM_WP_CHANNEL] > ADC_LOGIC_LOW) {
    printf("Error: Carrier eeprom write protected.\n");
    return false;
  }
  if (!Eeprom24WriteSync(segment->begin_address + offset, data_len, data)) {
    return false;
  }
  return EepromVerify(segment, offset, data_len, data);
}
