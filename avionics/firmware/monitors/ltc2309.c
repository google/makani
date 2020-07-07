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

#include "avionics/firmware/monitors/ltc2309.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/ltc2309.h"
#include "avionics/firmware/drivers/ltc2309_types.h"
#include "avionics/firmware/monitors/ltc2309_types.h"

static float RawToVoltage(const Ltc2309MonitorConfig *config, int16_t raw) {
  return (float)raw * config->volts_per_count - config->offset;
}

static bool Poll(const Ltc2309MonitorConfig *config,
                 Ltc2309OutputFunction output_function, Ltc2309 *device) {
  if (Ltc2309Poll(&config->config, device)) {
    float data = RawToVoltage(config, device->output);

    uint32_t flags = 0x0;
    if (data > config->max && config->max > 0.0f) {
      flags |= kLtc2309MonitorFlagOverVoltage;
    }
    if (data < config->min) {
      flags |= kLtc2309MonitorFlagUnderVoltage;
    }
    output_function(config->monitor, data, flags);
    return true;
  }
  return false;
}

void Ltc2309MonitorInit(int32_t num_devices, Ltc2309 *devices) {
  for (int32_t i = 0; i < num_devices; ++i) {
    Ltc2309Init(&devices[i]);
  }
}

bool Ltc2309MonitorPoll(const Ltc2309Monitors *monitors,
                        Ltc2309OutputFunction output_function,
                        uint32_t *device_index, uint32_t *config_indices,
                        Ltc2309 *devices) {
  // If no valid monitors exists, return true to advance to next monitor.
  if (monitors == NULL || monitors->num_devices <= 0) {
    return true;
  }

  // Make certain that device_index is always valid.
  *device_index %= monitors->num_devices;
  const Ltc2309MonitorDevice *device = &monitors->device[*device_index];

  // Make certain that config_indices is always valid.
  config_indices[*device_index] %= device->num_configs;
  uint32_t config_index = config_indices[*device_index];
  const Ltc2309MonitorConfig *config = &device->config[config_index];

  // Wait for completion before incrementing the configuration.
  if (Poll(config, output_function, &devices[*device_index])) {
    ++config_indices[*device_index];
  }

  // Limit each device to one I2C transaction per iteration.
  if (I2cIsBusIdle()) {
    ++(*device_index);  // Process all devices concurrently.
    return true;        // Set true when done polling. Data may not update.
  }
  return false;
}
