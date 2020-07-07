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

#include "avionics/firmware/drivers/ltc2309.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/ltc2309_types.h"

// This module implements a driver for LTC's Ltc2309 I2C device.
// See the datasheet at: https://goo.gl/eGAQLw.

// Datasheet indicates a reference wake up time of 200ms.
#define REFERENCE_SETTLING_CYCLES CLOCK32_MSEC_TO_CYCLES(200)

static bool IsReferenceReady(Ltc2309 *device) {
  // Wait for reference settling time.
  if (device->waiting_for_reference
      && CLOCK32_GE(Clock32GetCycles(), device->timeout)) {
    device->waiting_for_reference = false;
    device->waiting_for_conversion = false;  // Start new conversion.
  }
  return !device->waiting_for_reference;  // Ready for conversion.
}

static bool ConvertAdc(const Ltc2309Config *config, Ltc2309 *device) {
  static uint8_t i2c_write = 0x0;
  static uint8_t i2c_read[2];

  if (!device->waiting_for_conversion) {
    // Trigger start of conversion.
    i2c_write = config->command;
    device->waiting_for_conversion =
        I2cWriteAsync(config->addr, 1, &i2c_write);
    // Stop bit from I2cWrite triggers conversion.
    device->waiting_for_data = true;
  } else if ((device->last_command & kLtc2309SleepMode) != 0x0) {
    // TODO: Test sleep mode operation.
    // Wait for reference settling time if the last command sent Lt2309 to sleep
    // mode.
    device->last_command = i2c_write;
    device->timeout = Clock32GetCycles() + REFERENCE_SETTLING_CYCLES;
    device->waiting_for_reference = true;
    device->waiting_for_conversion = false;  // Reconvert after settling time.
  } else if (device->waiting_for_data) {
    // Start reading back data.
    device->waiting_for_data = !(I2cReadAsync(config->addr, 2, i2c_read));
  } else {
    device->last_command = i2c_write;
    device->waiting_for_conversion = false;  // Ready for next conversion.
    ReadUint16Be(i2c_read, (uint16_t *)&device->output);
    // Remove 4 trailing zeros.
    if (i2c_write & kLtc2309Unipolar)
      device->output = ((uint16_t) device->output) / 16;
    else
      device->output = device->output / 16;
    return true;  // Output valid.
  }
  return false;  // Output not valid.
}

static bool HandleI2cError(bool i2c_error, Ltc2309 *device) {
  if (i2c_error) {
    device->waiting_for_conversion = false;
  }
  return !i2c_error;
}

void Ltc2309Init(Ltc2309 *device) {
  assert(device != NULL);
  memset(device, 0x0, sizeof(*device));
}

bool Ltc2309Poll(const Ltc2309Config *config, Ltc2309 *device) {
  assert(config != NULL);
  assert(device != NULL);

  bool i2c_error;
  return I2cPoll(&i2c_error)                // Wait for I2C operation.
      && HandleI2cError(i2c_error, device)  // Allow one sleep cycle on error.
      && IsReferenceReady(device)           // Wait for reference settling time.
      && ConvertAdc(config, device);        // Process ADC conversion.
}
