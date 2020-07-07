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

#include "avionics/firmware/drivers/pca9546a.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/i2c.h"

// PCA9546A: this device creates a 4x I2C bus switch, thereby allowing the
// host controller to communicate on four independent I2C buses via one
// physical interface. The host sends a one byte bus mask to the PCA9546A
// controller over I2C to select the desired bus(es). Here, we write the
// desired bus mask, then read it back.

// nRESET SDA clear maximum time is 500 ns.
#define RESET_RECOVERY_CYCLES CLOCK32_USEC_TO_CYCLES(10)

static void StateReset(Pca9546aState next, const Pca9546aConfig *config,
                       Pca9546a *device) {
  // nRESET requires a pulse duration (t_WL) of 6 ns.
  if (config->reset_func) {
    config->reset_func();
  }
  device->recovery_timeout = Clock32GetCycles() + RESET_RECOVERY_CYCLES;
  device->error = false;
  device->state = next;
}

static void StateResetRecovery(Pca9546aState next, Pca9546a *device) {
  if (CLOCK32_GE(Clock32GetCycles(), device->recovery_timeout)) {
    device->state = next;
  }
}

static void StateWriteControl(Pca9546aState next, const Pca9546aConfig *config,
                              Pca9546a *device) {
  // Write selected bit mask.
  device->error = !I2cWriteAsync(config->addr, 1, &device->config);
  device->state = next;
}

static void StateReadControl(Pca9546aState next, const Pca9546aConfig *config,
                             Pca9546a *device) {
  // Read selected bit mask.
  device->error = !I2cReadAsync(config->addr, 1, &device->status);
  device->state = next;
}

// This state machine only runs after the I2C bus returns to idle.
static bool StateProcess(const Pca9546aConfig *config, Pca9546a *device) {
  // Reset on I2C bus error.
  if (device->error) {
    device->state = kPca9546aStateReset;
  }

  // Advance state machine, where the resulting device->state specifies the
  // state for the next iteration.
  Pca9546aState current = device->state;
  switch (current) {
    case kPca9546aStateReset:
      StateReset(kPca9546aStateResetRecovery, config, device);
      break;
    case kPca9546aStateResetRecovery:
      StateResetRecovery(kPca9546aStateWriteControl, device);
      break;
    case kPca9546aStateWriteControl:
      StateWriteControl(kPca9546aStateReadControl, config, device);
      break;
    case kPca9546aStateReadControl:
      StateReadControl(kPca9546aStateIdle, config, device);
      break;
    case kPca9546aStateIdle:
      device->state = kPca9546aStateWriteControl;
      break;
    default:
      device->state = kPca9546aStateReset;
      assert(false);
      break;
  }
  return current == kPca9546aStateIdle;
}

static bool IsValid(const Pca9546a *device) {
  // Status must match desired configuration.
  return !device->error && device->config == device->status;
}

void Pca9546aInit(Pca9546a *device) {
  assert(device != NULL);

  memset(device, 0, sizeof(*device));
  device->state = kPca9546aStateReset;
}

bool Pca9546aPoll(uint8_t bus_mask, const Pca9546aConfig *config,
                  Pca9546a *device) {
  assert(config != NULL);
  assert(device != NULL);

  // Update desired configuration.
  device->config = bus_mask;

  // Return true upon successful bus selection update.
  return I2cPoll(&device->error)       // Wait for I2C operation.
      && StateProcess(config, device)  // Wait for state machine.
      && IsValid(device);              // Check validity of data.
}
