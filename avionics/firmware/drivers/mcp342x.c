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

#include "avionics/firmware/drivers/mcp342x.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"

#ifndef MSEC_PER_SEC
#define MSEC_PER_SEC 1000
#endif

#define MCP342X_STATUS_NRDY   0x80
#define MCP342X_TIMEOUT_RATIO 10
#define MCP342X_CONFIG_TRIES  3

// Limit the maximum rate we can issue an I2C General Call. Issuing a General
// Call too frequently may cause issues with other I2C sensors (e.g., SI7021).
#define MCP342X_POWER_ON_CYCLES CLOCK32_MSEC_TO_CYCLES(100)


static uint8_t g_write[1];
static uint8_t g_read[3];

static int32_t GetTimeoutCycles(Mcp342xSps sps) {
  switch (sps) {
    case kMcp342xSps240:
      return CLOCK32_MSEC_TO_CYCLES(MCP342X_TIMEOUT_RATIO * MSEC_PER_SEC / 240);
    case kMcp342xSps60:
      return CLOCK32_MSEC_TO_CYCLES(MCP342X_TIMEOUT_RATIO * MSEC_PER_SEC / 60);
    case kMcp342xSps15:
      return CLOCK32_MSEC_TO_CYCLES(MCP342X_TIMEOUT_RATIO * MSEC_PER_SEC / 15);
    default:
      return 0;
  }
}

static void StateInit(Mcp342xState next, Mcp342x *device) {
  if (device->first_entry) {
    device->timeout = Clock32GetCycles() + MCP342X_POWER_ON_CYCLES;
  } else if (CLOCK32_GE(Clock32GetCycles(), device->timeout)) {
    device->error = false;
    device->state = next;
  }
}

static void StateGeneralCallLatch(Mcp342xState next, Mcp342x *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 1);
    g_write[0] = 0x04;
    device->error = !I2cWriteAsync(0x00, 1, g_write);
  } else {
    device->state = next;
    device->config_tries = 0;
  }
  device->status |= MCP342X_STATUS_NRDY;
}

static void StateSetConfig(Mcp342xState next, const Mcp342xConfig *config,
                           Mcp342x *device) {
  if (device->first_entry) {
    assert(sizeof(g_write) >= 1);
    device->config = Mcp342xBuildConfig(config);
    g_write[0] = device->config;
    device->error = !I2cWriteAsync(device->addr, 1, g_write);
  } else {
    device->state = next;
    device->timeout = Clock32GetCycles() + GetTimeoutCycles(config->sps);
    ++device->config_tries;
  }
  device->status |= MCP342X_STATUS_NRDY;
}

static void StateGetConfig(Mcp342xState next, Mcp342x *device) {
  if (device->first_entry) {
    assert(sizeof(g_read) >= 3);
    device->error = !I2cReadAsync(device->addr, 3, g_read);
  } else {
    device->status = g_read[2];
    if (((device->status ^ device->config) & ~MCP342X_STATUS_NRDY) != 0) {
      // Configuration does not match desired configuration.
      if (device->config_tries > MCP342X_CONFIG_TRIES) {
        device->error = true;
        device->state = kMcp342xStateIdle;
      } else {
        device->state = kMcp342xStateSetConfig;
      }
    } else if ((device->status & MCP342X_STATUS_NRDY) != 0) {
      // Retry.
      device->state = kMcp342xStateNotReady;
    } else {
      // Measurement ready, configuration matches desired configuration.
      device->state = next;
    }
  }
  device->status |= MCP342X_STATUS_NRDY;
}

static void StateNotReady(Mcp342xState next, Mcp342x *device) {
  // Measurement not ready, retry.
  if (CLOCK32_GE(Clock32GetCycles(), device->timeout)) {
    device->error = true;
  } else {
    device->state = next;
  }
}

static void StateGetResult(Mcp342xState next, const Mcp342xConfig *config,
                           Mcp342x *device) {
  if (device->first_entry) {
    assert(sizeof(g_read) >= 2);
    device->error = !I2cReadAsync(device->addr, 2, g_read);
    device->status |= MCP342X_STATUS_NRDY;
  } else {
    int16_t result;
    ReadInt16Be(g_read, &result);
    if (config->polarity == kMcp342xPolarityNegative) {
      device->result = -result;
    } else {
      device->result = result;
    }
    device->timeout = Clock32GetCycles() + GetTimeoutCycles(config->sps);
    device->status &= ~MCP342X_STATUS_NRDY;
    device->config_tries = 0;
    device->state = next;
  }
}

static void StateIdle(const Mcp342xConfig *config, Mcp342x *device) {
  device->config = Mcp342xBuildConfig(config);
  if (((device->status ^ device->config) & ~MCP342X_STATUS_NRDY)
      || config->mode != kMcp342xModeContinuous) {
    device->state = kMcp342xStateSetConfig;
  } else {
    device->state = kMcp342xStateGetResult;
  }
}

// This state machine only runs after the I2C bus returns to idle.
static bool StateProcess(const Mcp342xConfig *config, Mcp342x *device) {
  // Advance state machine, where the resulting device->state specifies the
  // state for the next iteration.
  Mcp342xState current = device->state;
  switch (current) {
    case kMcp342xStateInit:
      StateInit(kMcp342xStateGeneralCallLatch, device);
      break;
    case kMcp342xStateGeneralCallLatch:
      StateGeneralCallLatch(kMcp342xStateSetConfig, device);
      break;
    case kMcp342xStateSetConfig:
      StateSetConfig(kMcp342xStateGetConfig, config, device);
      break;
    case kMcp342xStateGetConfig:
      StateGetConfig(kMcp342xStateGetResult, device);
      break;
    case kMcp342xStateNotReady:
      StateNotReady(kMcp342xStateGetConfig, device);
      break;
    case kMcp342xStateGetResult:
      StateGetResult(kMcp342xStateIdle, config, device);
      break;
    case kMcp342xStateIdle:
      StateIdle(config, device);
      break;
    default:
      device->state = kMcp342xStateInit;
      assert(false);
      break;
  }
  device->first_entry = (current != device->state);
  return device->state == kMcp342xStateIdle
      || device->state == kMcp342xStateInit;
}

static bool HandleI2cError(Mcp342x *device) {
  // Reset on I2C bus error.
  if (device->error) {
    device->state = kMcp342xStateInit;
    device->first_entry = true;
    device->config_tries = 0;
  }
  return true;
}

void Mcp342xInit(Mcp342x *device) {
  assert(device != NULL);

  memset(device, 0, sizeof(*device));
  device->state = kMcp342xStateInit;
  device->first_entry = true;
  device->status = MCP342X_STATUS_NRDY;
}

void Mcp342xSetConfig(const Mcp342xConfig *config, Mcp342x *device) {
  assert(config != NULL);
  assert(device != NULL);

  device->status = MCP342X_STATUS_NRDY;
  device->config = Mcp342xBuildConfig(config);
  if (device->state > kMcp342xStateSetConfig) {
    device->state = kMcp342xStateSetConfig;
  }
}

bool Mcp342xGetResult(const Mcp342x *device, int32_t *result) {
  *result = device->result;

  // Wait for nREADY to clear and configuration to match status.
  return !device->error
      && device->state == kMcp342xStateIdle
      && device->status == (device->config & ~MCP342X_STATUS_NRDY);
}

bool Mcp342xPoll(uint8_t addr, const Mcp342xConfig *config, Mcp342x *device) {
  assert(addr <= 127U);
  assert(device != NULL);
  assert(device != NULL);

  // Update address from constant space.
  device->addr = addr;

  // Return true for valid data.
  return I2cPoll(&device->error)        // Wait for I2C operation.
      && HandleI2cError(device)         // Allow one sleep cycle on error.
      && StateProcess(config, device);  // Wait for state machine.
}
