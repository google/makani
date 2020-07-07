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

#include "avionics/firmware/drivers/mcp9800.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/mcp9800_types.h"

#define MCP9800_REG_TA     0x00
#define MCP9800_REG_CONFIG 0x01
#define MCP9800_REG_THYST  0x02
#define MCP9800_REG_TSET   0x03

static uint8_t g_write[3];
static uint8_t g_read[2];

static void StateSetConfig(Mcp9800State next, const Mcp9800Config *config,
                           Mcp9800 *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 2);
    g_write[0] = MCP9800_REG_CONFIG;
    g_write[1] = config->binary_config;
    device->error = !I2cWriteAsync(config->addr, 2, g_write);
  } else {
    device->state = next;
  }
}

static void StateGetTa(Mcp9800State next, const Mcp9800Config *config,
                       Mcp9800 *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 1);
    assert(sizeof(g_read) >= 2);
    g_write[0] = MCP9800_REG_TA;
    device->error = !I2cWriteReadAsync(config->addr, 1, g_write, 2, g_read);
  } else {
    ReadUint16Be(g_read, &device->ta_raw);
    device->state = next;
  }
}

// This state machine only runs after the I2C bus returns to idle.
static bool StateProcess(const Mcp9800Config *config, Mcp9800 *device) {
  // Advance state machine, where the resulting device->state specifies the
  // state for the next iteration.
  Mcp9800State current = device->state;
  switch (current) {
    case kMcp9800StateSetConfig:
      StateSetConfig(kMcp9800StateGetTa, config, device);
      break;
    case kMcp9800StateGetTa:
      StateGetTa(kMcp9800StateIdle, config, device);
      break;
    default:
      assert(false);
      // Fall-through intentional.
    case kMcp9800StateIdle:
      device->state = kMcp9800StateSetConfig;
      break;
  }
  device->first_entry = (current != device->state);
  return device->state == kMcp9800StateIdle;
}

static bool HandleI2cError(Mcp9800 *device) {
  // Reset on I2C bus error.
  if (device->error) {
    device->state = kMcp9800StateSetConfig;
    device->first_entry = true;
    device->error = false;
    return false;
  }
  return true;
}

static bool IsValid(const Mcp9800 *device) {
  return !device->error;
}

void Mcp9800Init(Mcp9800 *device) {
  assert(device != NULL);

  memset(device, 0, sizeof(*device));
  device->state = kMcp9800StateSetConfig;
  device->first_entry = true;
}

bool Mcp9800Poll(const Mcp9800Config *config, Mcp9800 *device) {
  assert(device != NULL);

  // Return true for valid data.
  return I2cPoll(&device->error)       // Wait for I2C operation.
      && HandleI2cError(device)        // Allow one sleep cycle on error.
      && StateProcess(config, device)  // Wait for state machine.
      && IsValid(device);              // Check validity of data.
}
