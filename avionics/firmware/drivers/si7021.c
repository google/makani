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

#include "avionics/firmware/drivers/si7021.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/si7021_types.h"

// Limit rate to reinitialize device.
#define SI7021_POWER_ON_CYCLES CLOCK32_MSEC_TO_CYCLES(250)
// Datasheet specifies 15ms max.
#define SI7021_RESET_RECOVERY_CYCLES CLOCK32_MSEC_TO_CYCLES(250)
// 12-bit RH + 14-bit temp max.
#define SI7021_READ_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(250)

#define SI7021_USER_REG1_RSVD_MASK      0x3A
#define SI7021_HEATER_CONTROL_RSVD_MASK 0xF0


static uint8_t g_write[3];
static uint8_t g_read[2];

static void StateInit(Si7021State next, Si7021 *device) {
  if (device->first_entry) {
    device->timeout = Clock32GetCycles() + SI7021_POWER_ON_CYCLES;
  } else if (CLOCK32_GE(Clock32GetCycles(), device->timeout)) {
    device->state = next;
  }
}

static void StateFlush(Si7021State next, const Si7021Config *config,
                       Si7021 *device) {
  // Get chip out of No Hold Master Mode, if it's stuck there.
  if (device->first_entry) {
    device->error = !I2cReadAsync(config->addr, 2, g_read);
  } else {
    device->state = next;
  }
}

static void StateReset(Si7021State next, const Si7021Config *config,
                       Si7021 *device) {
  if (device->first_entry) {
    g_write[0] = kSi7021CommandReset;
    device->error = !I2cWriteAsync(config->addr, 1, g_write);
    device->timeout = Clock32GetCycles() + SI7021_RESET_RECOVERY_CYCLES;
  } else if (CLOCK32_GE(Clock32GetCycles(), device->timeout)) {
    device->state = next;
  }
}

static void StateReadUserReg1(Si7021State next, const Si7021Config *config,
                              Si7021 *device) {
  if (device->first_entry) {
    g_write[0] = kSi7021CommandReadUserReg1;
    device->error = !I2cWriteReadAsync(config->addr, 1, g_write, 1, g_read);
  } else {
    device->status.user_reg1 = g_read[0];
    device->config.user_reg1 = config->user_reg1;
    // Reserved bits should be set as read to ensure compatibility.
    device->config.user_reg1 &= ~SI7021_USER_REG1_RSVD_MASK;
    device->config.user_reg1 |=
        device->status.user_reg1 & SI7021_USER_REG1_RSVD_MASK;
    if (device->config.user_reg1 != device->status.user_reg1) {
      device->state = kSi7021StateWriteUserReg1;
    } else {
      device->state = next;
    }
  }
}

static void StateWriteUserReg1(Si7021State next, const Si7021Config *config,
                               Si7021 *device) {
  if (device->first_entry) {
    g_write[0] = kSi7021CommandWriteUserReg1;
    g_write[1] = device->config.user_reg1;
    device->error = !I2cWriteAsync(config->addr, 2, g_write);
  } else {
    device->state = next;
  }
}

static void StateReadHeaterControl(Si7021State next, const Si7021Config *config,
                                   Si7021 *device) {
  if (device->first_entry) {
    g_write[0] = kSi7021CommandReadHeaterControlReg;
    device->error = !I2cWriteReadAsync(config->addr, 1, g_write, 1, g_read);
  } else {
    device->status.heater_control = g_read[0];
    device->config.heater_control &= ~SI7021_HEATER_CONTROL_RSVD_MASK;
    // Reserved bits should be set as read to ensure compatibility.
    device->config.heater_control |=
        device->status.heater_control & SI7021_HEATER_CONTROL_RSVD_MASK;
    if (device->config.heater_control != device->status.heater_control) {
      device->state = kSi7021StateWriteHeaterControl;
    } else {
      device->state = next;
    }
  }
}

static void StateWriteHeaterControl(Si7021State next,
                                    const Si7021Config *config,
                                    Si7021 *device) {
  if (device->first_entry) {
    g_write[0] = kSi7021CommandWriteHeaterControlReg;
    g_write[1] = device->config.heater_control;
    device->error = !I2cWriteAsync(config->addr, 2, g_write);
  } else {
    device->state = next;
  }
}

static void StateMeasureRelHumidity(Si7021State next,
                                    const Si7021Config *config,
                                    Si7021 *device) {
  if (device->first_entry) {
    g_write[0] = kSi7021CommandMeasureRelHumidityNoHold;
    device->error = !I2cWriteAsync(config->addr, 1, g_write);
  } else {
    device->timeout = Clock32GetCycles() + SI7021_READ_TIMEOUT_CYCLES;
    device->state = next;
  }
}

static void StateReadRelHumidity(Si7021State next, const Si7021Config *config,
                                 Si7021 *device) {
  if (device->first_entry) {
    // Timeout set in StateMeasureRelHumidity().
    if (CLOCK32_LT(Clock32GetCycles(), device->timeout)) {
      // This command will fail while the device handles the conversion.
      device->error = !I2cReadAsync(config->addr, 2, g_read);
    } else {
      device->state = next;
    }
  } else {
    ReadUint16Be(g_read, &device->output.rel_humidity);
    device->state = next;
  }
}

static void StateReadTemperature(Si7021State next, const Si7021Config *config,
                                 Si7021 *device) {
  if (device->first_entry) {
    g_write[0] = kSi7021CommandReadTemperature;
    device->error = !I2cWriteReadAsync(config->addr, 1, g_write, 2, g_read);
  } else {
    ReadUint16Be(g_read, &device->output.temperature);
    device->state = next;
  }
}

static bool StateProcess(const Si7021Config *config, Si7021 *device) {
  // Advance state machine, where the resulting device->state specifies the
  // state for the next iteration.
  Si7021State current = device->state;
  switch (current) {
    case kSi7021StateInit:
      StateInit(kSi7021StateFlush, device);
      break;
    case kSi7021StateFlush:
      StateFlush(kSi7021StateReset, config, device);
      break;
    case kSi7021StateReset:
      StateReset(kSi7021StateReadUserReg1, config, device);
      break;
    case kSi7021StateReadUserReg1:
      StateReadUserReg1(kSi7021StateReadHeaterControl, config, device);
      break;
    case kSi7021StateWriteUserReg1:
      StateWriteUserReg1(kSi7021StateReadUserReg1, config, device);
      break;
    case kSi7021StateReadHeaterControl:
      StateReadHeaterControl(kSi7021StateMeasureRelHumidity, config, device);
      break;
    case kSi7021StateWriteHeaterControl:
      StateWriteHeaterControl(kSi7021StateReadHeaterControl, config, device);
      break;
    case kSi7021StateMeasureRelHumidity:
      StateMeasureRelHumidity(kSi7021StateReadRelHumidity, config, device);
      break;
    case kSi7021StateReadRelHumidity:
      StateReadRelHumidity(kSi7021StateReadTemperature, config, device);
      break;
    case kSi7021StateReadTemperature:
      StateReadTemperature(kSi7021StateIdle, config, device);
      break;
    case kSi7021StateIdle:
      device->state = kSi7021StateReadUserReg1;
      break;
    default:
      assert(false);
      device->state = kSi7021StateInit;
      break;
  }
  device->first_entry = (current != device->state);
  return device->state == kSi7021StateIdle;
}

static bool HandleI2cError(Si7021 *device) {
  // Reset on I2C bus error.
  if (device->error) {
    if (device->state != kSi7021StateReadRelHumidity) {
      device->state = kSi7021StateInit;
    }
    device->first_entry = true;
    device->error = false;
    return false;
  }
  return true;
}

static bool IsValid(const Si7021 *device) {
  return !device->error
      && (device->output.rel_humidity & 0x03) == 0x02
      && (device->output.temperature & 0x03) == 0x00;
}

void Si7021Init(Si7021 *device) {
  assert(device != NULL);

  memset(device, 0, sizeof(*device));
  device->state = kSi7021StateInit;
  device->first_entry = true;
}

bool Si7021Poll(const Si7021Config *config, Si7021 *device) {
  assert(config != NULL);
  assert(device != NULL);

  // Return true for valid data.
  return I2cPoll(&device->error)       // Wait for I2C operation.
      && HandleI2cError(device)        // Allow one sleep cycle on error.
      && StateProcess(config, device)  // Wait for state machine.
      && IsValid(device);              // Check validity of data.
}
