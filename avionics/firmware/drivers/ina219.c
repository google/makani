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

#include "avionics/firmware/drivers/ina219.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/i2c.h"

#define INA219_REG_CONFIG  0x00
#define INA219_REG_SHUNT   0x01
#define INA219_REG_BUS     0x02
#define INA219_REG_POWER   0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIB   0x05

#define INA219_BUS_CNVR 0x02  // Conversion ready flag.

static uint8_t g_write[3];
static uint8_t g_read[2];

static void StateReset(Ina219State next, const Ina219Config *config,
                       Ina219 *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 3);
    g_write[0] = INA219_REG_CONFIG;
    g_write[1] = 0x80;  // Set reset bit.
    g_write[2] = 0x00;
    device->error = !I2cWriteAsync(config->addr, 3, g_write);
  } else {
    device->state = next;
  }
}

static void StateConfigure(Ina219State next, const Ina219Config *config,
                           Ina219 *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 3);
    g_write[0] = INA219_REG_CONFIG;
    WriteUint16Be(config->config, &g_write[1]);
    device->error = !I2cWriteAsync(config->addr, 3, g_write);
  } else {
    device->state = next;
  }
}

static void StateReadShunt(Ina219State next, const Ina219Config *config,
                           Ina219 *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 1);
    assert(sizeof(g_read) >= 2);
    g_write[0] = INA219_REG_SHUNT;
    device->error = !I2cWriteReadAsync(config->addr, 1, g_write, 2, g_read);
  } else {
    ReadInt16Be(g_read, &device->output.shunt_raw);
    device->state = next;
  }
}

static void StateReadBus(Ina219State next, const Ina219Config *config,
                         Ina219 *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 1);
    assert(sizeof(g_read) >= 2);
    g_write[0] = INA219_REG_BUS;
    device->error = !I2cWriteReadAsync(config->addr, 1, g_write, 2, g_read);
  } else {
    ReadUint16Be(g_read, &device->output.bus_raw);
    device->state = next;
  }
}

// This state machine only runs after the I2C bus returns to idle.
static bool StateProcess(const Ina219Config *config, Ina219 *device) {
  // Advance state machine, where the resulting device->state specifies the
  // state for the next iteration.
  Ina219State current = device->state;
  switch (current) {
    case kIna219StateReset:
      StateReset(kIna219StateConfigure, config, device);
      break;
    case kIna219StateConfigure:
      StateConfigure(kIna219StateReadShunt, config, device);
      break;
    case kIna219StateReadShunt:
      StateReadShunt(kIna219StateReadBus, config, device);
      break;
    case kIna219StateReadBus:
      StateReadBus(kIna219StateIdle, config, device);
      break;
    case kIna219StateIdle:
      device->state = kIna219StateConfigure;
      break;
    default:
      device->state = kIna219StateReset;
      assert(false);
      break;
  }
  device->first_entry = (current != device->state);
  return device->state == kIna219StateIdle;
}

static bool HandleI2cError(Ina219 *device) {
  // Reset on I2C bus error.
  if (device->error) {
    device->state = kIna219StateReset;
    device->first_entry = true;
    device->error = false;
    return false;
  }
  return true;
}

static bool IsValid(const Ina219 *device) {
  // Wait for conversion ready flag.
  return !device->error && (device->output.bus_raw & INA219_BUS_CNVR) != 0;
}

void Ina219Init(Ina219 *device) {
  assert(device != NULL);

  memset(device, 0, sizeof(*device));
  device->state = kIna219StateReset;
  device->first_entry = true;
}

bool Ina219Poll(const Ina219Config *config, Ina219 *device) {
  assert(config != NULL);
  assert(device != NULL);

  // Return true for valid data.
  return I2cPoll(&device->error)       // Wait for I2C operation.
      && HandleI2cError(device)        // Allow one sleep cycle on error.
      && StateProcess(config, device)  // Wait for state machine.
      && IsValid(device);              // Check validity of data.
}
