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

#include "avionics/firmware/drivers/ads7828.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/ads7828_types.h"

// This module implements a driver for TI's ADS7828 I2C device.
// See the datasheet at: http://www.ti.com/lit/ds/symlink/ads7828-q1.pdf

// Datasheet indicates that a 1-uF capacitor requires 1240 uS settling time.
#define REFERENCE_SETTLING_CYCLES CLOCK32_USEC_TO_CYCLES(1240)

static bool IsReferenceReady(Ads7828 *device) {
  // Wait for reference settling time.
  if (device->waiting_for_reference
      && CLOCK32_GE(Clock32GetCycles(), device->timeout)) {
    device->waiting_for_reference = false;
    device->waiting_for_conversion = false;  // Start new conversion.
  }
  return !device->waiting_for_reference;  // Ready for conversion.
}

static bool ConvertAdc(const Ads7828Config *config, Ads7828 *device) {
  static uint8_t i2c_write = 0x0;
  static uint8_t i2c_read[2];

  if (!device->waiting_for_conversion) {
    // Trigger start of conversion.
    i2c_write = config->command;
    device->waiting_for_conversion =
        I2cWriteReadAsync(config->addr, 1, &i2c_write, 2, i2c_read);
  } else if ((device->last_command & kAds7828PowerReferenceOn) == 0x0
             && (i2c_write & kAds7828PowerReferenceOn) != 0x0) {
    // Wait for reference settling time if the last command disabled the
    // reference and this command enables it.
    device->last_command = i2c_write;
    device->timeout = Clock32GetCycles() + REFERENCE_SETTLING_CYCLES;
    device->waiting_for_reference = true;
    device->waiting_for_conversion = false;  // Reconvert after settling time.
  } else {
    device->last_command = i2c_write;
    device->waiting_for_conversion = false;  // Ready for next conversion.
    ReadUint16Be(i2c_read, &device->output);
    return true;  // Output valid.
  }
  return false;  // Output not valid.
}

static bool HandleI2cError(bool i2c_error, Ads7828 *device) {
  if (i2c_error) {
    device->waiting_for_conversion = false;
  }
  return !i2c_error;
}

void Ads7828Init(Ads7828 *device) {
  assert(device != NULL);
  memset(device, 0x0, sizeof(*device));
}

bool Ads7828Poll(const Ads7828Config *config, Ads7828 *device) {
  assert(config != NULL);
  assert(device != NULL);

  bool i2c_error;
  return I2cPoll(&i2c_error)                // Wait for I2C operation.
      && HandleI2cError(i2c_error, device)  // Allow one sleep cycle on error.
      && IsReferenceReady(device)           // Wait for reference settling time.
      && ConvertAdc(config, device);        // Process ADC conversion.
}
