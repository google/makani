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

#include "avionics/firmware/monitors/si7021.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>  // For NULL.
#include <stdint.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/si7021.h"

static void Poll(const Si7021Monitor *config,
                 Si7021OutputFunction output_function, Si7021 *device) {
  if (Si7021Poll(&config->config, device)) {
    Si7021OutputData data;
    Si7021Convert(&device->output, &data);
    output_function(config->monitor, &data);
  }
}

void Si7021MonitorInit(int32_t num_devices, Si7021 *devices) {
  for (int32_t i = 0; i < num_devices; ++i) {
    Si7021Init(&devices[i]);
  }
}

bool Si7021MonitorPoll(const Si7021Monitors *config,
                       Si7021OutputFunction output_function,
                       uint32_t *device_index, Si7021 *devices) {
  assert(output_function != NULL && device_index != NULL && devices != NULL);
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
