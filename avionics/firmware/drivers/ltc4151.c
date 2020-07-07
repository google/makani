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

#include "avionics/firmware/drivers/ltc4151.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/i2c.h"
#include "common/macros.h"


// Write and read buffers for I2C communications with LTC4151 devices.
static uint8_t g_read[2];
static uint8_t g_write[2];

static const uint8_t kCmdReadConfig[] = {0x06};
static const uint8_t kCmdReadBus[] = {0x02};
static const uint8_t kCmdReadShunt[] = {0x00};

// Initial state merely advances device to chosen 2nd state.
static void StateInit(Ltc4151State next, Ltc4151 *device) {
  if (device->flag_set_config) {
    device->flag_set_config = false;
    device->state = kLtc4151StateSetConfig;
  } else {
    device->state = next;
  }
}

// Set LTC4151 device configuration to the factory default.
static void StateSetConfig(Ltc4151State next, const Ltc4151Config *config,
                           Ltc4151 *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 2);
    g_write[0] = 0x06;
    g_write[1] = config->binary_config;
    device->error = !I2cWriteAsync(config->addr, 2, g_write);
  } else {
    device->state = next;
  }
}

// Not used yet; read LTC4151 device config to make sure it's correct.
static void StateReadConfig(Ltc4151State next, const Ltc4151Config *config,
                            Ltc4151 *device) {
  if (device->first_entry) {
    assert(sizeof(g_read) >= 1);
    device->error = !I2cWriteReadAsync(config->addr, ARRAYSIZE(kCmdReadConfig),
                                       kCmdReadConfig, 1, g_read);
  } else {
    if (g_read[0] != config->binary_config) {
      device->state = kLtc4151StateSetConfig;
    } else {
      device->state = next;
    }
  }
}

// Get raw ADC reading for Vin bus and store temporarily.
static void StateReadBus(Ltc4151State next, const Ltc4151Config *config,
                         Ltc4151 *device) {
  if (device->first_entry) {
    assert(sizeof(g_read) >= 2);
    device->error = !I2cWriteReadAsync(config->addr, ARRAYSIZE(kCmdReadBus),
                                       kCmdReadBus, 2, g_read);
  } else {
    ReadUint16Be(g_read, &device->output.bus_raw);
    device->state = next;
  }
}

// Get raw ADC reading for current through shunt resistor and store temporarily.
static void StateReadShunt(Ltc4151State next, const Ltc4151Config *config,
                           Ltc4151 *device) {
  if (device->first_entry) {
    assert(sizeof(g_read) >= 2);
    device->error = !I2cWriteReadAsync(config->addr, ARRAYSIZE(kCmdReadShunt),
                                       kCmdReadShunt, 2, g_read);
  } else {
    ReadInt16Be(g_read, &device->output.shunt_raw);
    device->state = next;
  }
}

// LTC4151 device is ready to cycle through states again.
static void StateIdle(Ltc4151State next, Ltc4151 *device) {
  device->state = next;
}

// Advance state machine, where the device->state resulting from the
// state-specific function specifies the state for the next iteration.
// This state machine only runs after the I2C bus returns to idle.
static bool StateProcess(const Ltc4151Config *config, Ltc4151 *device) {
  Ltc4151State current = device->state;
  switch (current) {
    case kLtc4151StateInit:
      StateInit(kLtc4151StateReadBus, device);
      break;
    case kLtc4151StateSetConfig:
      StateSetConfig(kLtc4151StateReadConfig, config, device);
      break;
    case kLtc4151StateReadConfig:
      StateReadConfig(kLtc4151StateInit, config, device);
      break;
    case kLtc4151StateReadBus:
      StateReadBus(kLtc4151StateReadShunt, config, device);
      break;
    case kLtc4151StateReadShunt:
      StateReadShunt(kLtc4151StateIdle, config, device);
      break;
    case kLtc4151StateIdle:
      StateIdle(kLtc4151StateInit, device);
      break;
    default:
      device->state = kLtc4151StateInit;
      assert(false);
      break;
  }
  device->first_entry = (current != device->state);
  return device->state == kLtc4151StateIdle;
}

static bool HandleI2cError(Ltc4151 *device) {
  // Reset on I2C bus error.
  if (device->error) {
    device->state = kLtc4151StateInit;
    device->first_entry = true;
    device->error = false;
    return false;
  } else {
    return true;
  }
}

static bool IsValid(const Ltc4151 *device) {
  return !device->error;
}

void Ltc4151Init(Ltc4151 *device) {
  assert(device != NULL);

  memset(device, 0, sizeof(*device));
  device->state = kLtc4151StateInit;
  device->first_entry = true;
}

void Ltc4151SetConfig(Ltc4151 *device) {
  assert(device != NULL);
  device->flag_set_config = true;
}

bool Ltc4151Poll(const Ltc4151Config *config, Ltc4151 *device) {
  assert(config != NULL);
  assert(device != NULL);
  assert(config->addr <= 127U);

  // Return true for valid data.
  return I2cPoll(&device->error)
      && HandleI2cError(device)
      && StateProcess(config, device)
      && IsValid(device);
}
