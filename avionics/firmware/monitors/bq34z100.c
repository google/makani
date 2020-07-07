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

#include "avionics/firmware/monitors/bq34z100.h"

#include <stdbool.h>
#include <stddef.h>  // For NULL.
#include <stdint.h>

#include "avionics/firmware/cpu/i2c.h"
#include "avionics/firmware/drivers/bq34z100.h"
#include "avionics/firmware/drivers/bq34z100_types.h"
#include "avionics/firmware/monitors/bq34z100_types.h"

static void Poll(const Bq34z100Monitor *config,
                 Bq34z100OutputFunction output_function, Bq34z100 *device) {
  if (Bq34z100Poll(&config->config, device)) {
    Bq34z100OutputData data;
    Bq34z100Convert(&config->config, &device->output, &data);

    uint32_t flags = 0x0;
    if (data.avg_current > config->current_max && config->current_max > 0.0f) {
      flags |= kBq34z100MonitorFlagOverCurrent;
    }
    if (data.bus_voltage > config->voltage_max && config->voltage_max > 0.0f) {
      flags |= kBq34z100MonitorFlagOverVoltage;
    }
    if (data.bus_voltage < config->voltage_min) {
      flags |= kBq34z100MonitorFlagUnderVoltage;
    }
    if (data.soc_per_cent < config->soc_min) {
      flags |= kBq34z100MonitorFlagLowCharge;
    }
    output_function(config->monitor, &data, flags);
  }
}

void Bq34z100MonitorInit(int32_t num_devices, Bq34z100 *devices) {
  for (int32_t i = 0; i < num_devices; ++i) {
    Bq34z100Init(&devices[i]);
  }
}

bool Bq34z100MonitorPoll(const Bq34z100Monitors *config,
                         Bq34z100OutputFunction output_function,
                         uint32_t *device_index, Bq34z100 *devices) {
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
