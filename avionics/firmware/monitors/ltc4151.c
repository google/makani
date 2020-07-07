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

#include "avionics/firmware/monitors/ltc4151.h"

#include <stdbool.h>
#include <stddef.h>  // For NULL.
#include <stdint.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/ltc4151.h"
#include "avionics/firmware/drivers/ltc4151_types.h"
#include "avionics/firmware/monitors/ltc4151_types.h"

static void Poll(const Ltc4151Monitor *config,
                 Ltc4151OutputFunction output_function, Ltc4151 *device) {
  if (Ltc4151Poll(&config->config, device)) {
    Ltc4151OutputData data;
    Ltc4151Convert(&config->config, &device->output, &data);

    uint32_t flags = 0x0;
    if (data.current > config->current_max && config->current_max > 0.0f) {
      flags |= kLtc4151MonitorFlagOverCurrent;
    }
    if (data.voltage > config->voltage_max && config->voltage_max > 0.0f) {
      flags |= kLtc4151MonitorFlagOverVoltage;
    }
    if (data.voltage < config->voltage_min) {
      flags |= kLtc4151MonitorFlagUnderVoltage;
    }
    output_function(config->monitor, &data, flags);
  }
}

void Ltc4151MonitorInit(int32_t num_devices, Ltc4151 *devices) {
  for (int32_t i = 0; i < num_devices; ++i) {
    Ltc4151Init(&devices[i]);
  }
}

bool Ltc4151MonitorPoll(const Ltc4151Monitors *config,
                        Ltc4151OutputFunction output_function,
                        uint32_t *device_index, Ltc4151 *devices) {
  // If no valid configuration exists, return true to advance to next monitor.
  if (config == NULL || config->num_devices <= 0) {
    return true;
  }

  // Make certain that device_index is always valid.
  *device_index %= config->num_devices;
  Poll(&config->device[*device_index], output_function,
       &devices[*device_index]);

  // Limit each device to one I2C transaction per iteration.
  if (I2cIsBusIdle()) {
    ++(*device_index);  // Process all devices concurrently.
    return true;        // Set true when done polling. Data may not update.
  }
  return false;
}
