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

#include "avionics/firmware/drivers/bq34z100.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/i2c.h"
#include "common/macros.h"


// Write and read buffers for I2C communications with BQ34Z100 devices.
static uint8_t g_read[2];

static const uint8_t kCmdReadVoltage[]      = {0x08};
static const uint8_t kCmdReadCurrent[]      = {0x0A};
static const uint8_t kCmdReadCapacity[]     = {0x04};
static const uint8_t kCmdReadFullCapacity[] = {0x06};
static const uint8_t kCmdReadSoc[]          = {0x02};
static const uint8_t kCmdReadTemp[]         = {0x0C};

// StateNext merely advances device to chosen subsequent state.
static void StateNext(Bq34z100State next, Bq34z100 *device) {
  device->state = next;
}

// Write a given byte sequence over I2C, then read a given number of bytes.
static void StateWriteRead(Bq34z100State next, const Bq34z100Config *config,
                           const uint8_t *cmd, int32_t write_len,
                           int32_t read_len, Bq34z100 *device) {
  if (device->first_entry) {
    device->error = !I2cWriteReadAsync(config->addr, write_len, cmd,
                                       read_len, g_read);
  } else {
    device->state = next;
  }
}

// Advance state machine, where the device->state resulting from the
// state-specific function specifies the state for the next iteration.
// This state machine only runs after the I2C bus returns to idle.
static bool StateProcess(const Bq34z100Config *config, Bq34z100 *device) {
  Bq34z100State current = device->state;
  switch (current) {
    case kBq34z100StateInit:
      StateNext(kBq34z100StateReadVoltage, device);
      break;
    case kBq34z100StateReadVoltage:
      StateWriteRead(kBq34z100StateReadCurrent, config, kCmdReadVoltage,
                     ARRAYSIZE(kCmdReadVoltage), 2, device);
      if (!device->first_entry) {
        ReadUint16Le(g_read, &device->output.bus_raw);
      }
      break;
    case kBq34z100StateReadCurrent:
      StateWriteRead(kBq34z100StateReadCapacity, config, kCmdReadCurrent,
                     ARRAYSIZE(kCmdReadCurrent), 2, device);
      if (!device->first_entry) {
        ReadInt16Le(g_read, &device->output.avg_current_raw);
      }
      break;
    case kBq34z100StateReadCapacity:
      StateWriteRead(kBq34z100StateReadFullCapacity, config,
                     kCmdReadCapacity, ARRAYSIZE(kCmdReadCapacity), 2,
                     device);
      if (!device->first_entry) {
        ReadUint16Le(g_read, &device->output.cur_capacity_raw);
      }
      break;
    case kBq34z100StateReadFullCapacity:
      StateWriteRead(kBq34z100StateReadSoc, config, kCmdReadFullCapacity,
                     ARRAYSIZE(kCmdReadFullCapacity), 2, device);
      if (!device->first_entry) {
        ReadUint16Le(g_read, &device->output.full_capacity_raw);
      }
      break;
    case kBq34z100StateReadSoc:
      StateWriteRead(kBq34z100StateReadTemp, config, kCmdReadSoc,
                     ARRAYSIZE(kCmdReadSoc), 1, device);
      if (!device->first_entry) {
        device->output.soc_raw = g_read[0];
      }
      break;
    case kBq34z100StateReadTemp:
      StateWriteRead(kBq34z100StateIdle, config, kCmdReadTemp,
                     ARRAYSIZE(kCmdReadTemp), 2, device);
      if (!device->first_entry) {
        ReadUint16Le(g_read, &device->output.temp_raw);
      }
      break;
    case kBq34z100StateIdle:
      StateNext(kBq34z100StateInit, device);
      break;
    default:
      device->state = kBq34z100StateInit;
      assert(false);
      break;
  }
  device->first_entry = (current != device->state);
  return device->state == kBq34z100StateIdle;
}

static bool HandleI2cError(Bq34z100 *device) {
  // Reset on I2C bus error.
  if (device->error) {
    device->state = kBq34z100StateInit;
    device->first_entry = true;
    device->error = false;
    return false;
  } else {
    return true;
  }
}

static bool IsValid(const Bq34z100 *device) {
  return !device->error;
}

void Bq34z100Init(Bq34z100 *device) {
  assert(device != NULL);

  memset(device, 0, sizeof(*device));
  device->state = kBq34z100StateInit;
  device->first_entry = true;
}

bool Bq34z100Poll(const Bq34z100Config *config, Bq34z100 *device) {
  assert(config != NULL);
  assert(device != NULL);
  assert(config->addr <= 127U);

  // Return true for valid data.
  return I2cPoll(&device->error)
      && HandleI2cError(device)
      && StateProcess(config, device)
      && IsValid(device);
}
