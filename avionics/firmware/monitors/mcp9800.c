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

#include "avionics/firmware/monitors/mcp9800.h"

#include <stdbool.h>
#include <stddef.h>  // For NULL.
#include <stdint.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/mcp9800.h"

static void Poll(const Mcp9800Monitor *config,
                 Mcp9800OutputFunction output_function, Mcp9800 *device) {
  if (Mcp9800Poll(&config->config, device)) {
    output_function(config->monitor, Mcp9800TempRawToC(device->ta_raw));
  }
}

void Mcp9800MonitorInit(int32_t num_devices, Mcp9800 *devices) {
  for (int32_t i = 0; i < num_devices; ++i) {
    Mcp9800Init(&devices[i]);
  }
}

bool Mcp9800MonitorPoll(const Mcp9800Monitors *config,
                        Mcp9800OutputFunction output_function,
                        uint32_t *device_index, Mcp9800 *devices) {
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
