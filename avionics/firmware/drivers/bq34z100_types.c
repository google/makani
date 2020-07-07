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

#include "avionics/firmware/drivers/bq34z100_types.h"

#include <stdint.h>

float Bq34z100BusRawToVolts(uint16_t bus_raw, float cell_mult) {
  return 0.001f * (bus_raw * cell_mult);
}

void Bq34z100Convert(const Bq34z100Config *config,
                     const Bq34z100OutputRaw *raw,
                     Bq34z100OutputData *data) {
  data->bus_voltage = Bq34z100BusRawToVolts(raw->bus_raw, config->cell_mult);
  data->avg_current = 0.001f * (float)(raw->avg_current_raw);
  data->remaining_mah = raw->cur_capacity_raw;
  data->soc_per_cent = raw->soc_raw;
}
