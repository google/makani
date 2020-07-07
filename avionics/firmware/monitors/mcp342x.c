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

#include "avionics/firmware/monitors/mcp342x.h"

#include <stdbool.h>
#include <stddef.h>  // For NULL.
#include <stdint.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/mcp342x.h"
#include "avionics/firmware/drivers/mcp342x_types.h"
#include "avionics/firmware/monitors/mcp342x_types.h"

static bool Poll(const Mcp342xMonitorConfig *config,
                 Mcp342xOutputFunction output_function, Mcp342x *device) {
  if (Mcp342xPoll(config->addr, &config->config, device)) {
    int32_t result;
    bool valid = Mcp342xGetResult(device, &result);
    output_function(config->monitor, result, valid);
    return true;
  }
  return false;
}

void Mcp342xMonitorInit(int32_t num_devices, Mcp342x *devices) {
  for (int32_t i = 0; i < num_devices; ++i) {
    Mcp342xInit(&devices[i]);
  }
}

bool Mcp342xMonitorPoll(const Mcp342xMonitors *monitors,
                        Mcp342xOutputFunction output_function,
                        uint32_t *device_index, uint32_t *config_indices,
                        Mcp342x *devices) {
  // If no valid monitors exists, return true to advance to next monitor.
  if (monitors == NULL || monitors->num_devices <= 0) {
    return true;
  }

  // Make certain that device_index is always valid.
  *device_index %= monitors->num_devices;
  const Mcp342xMonitorDevice *device = &monitors->device[*device_index];

  // Make certain that config_indices is always valid.
  config_indices[*device_index] %= device->num_configs;
  uint32_t config_index = config_indices[*device_index];
  const Mcp342xMonitorConfig *config = &device->config[config_index];

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
