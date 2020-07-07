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

#ifndef AVIONICS_FIRMWARE_DRIVERS_BQ34Z100_TYPES_H_
#define AVIONICS_FIRMWARE_DRIVERS_BQ34Z100_TYPES_H_

#include <stdint.h>

typedef struct {
  uint8_t addr;       // Different from chip default bc of address translator.
  float cell_mult;  // Multiplier for voltage reading and stored num_cells.
  // TODO: 0xD948 calibration config, check software for post-seal.
  // TODO: Add float shunt_resistor and other data flash params.
} Bq34z100Config;

typedef struct {
  int16_t avg_current_raw;     // Average current reading, 1 mA per bit.
  uint16_t bus_raw;            // Bus voltage / cell_mult, 1 mV per bit.
  uint16_t cur_capacity_raw;   // Currently remaining capacity, 1 mAh per bit.
  uint16_t full_capacity_raw;  // Full charge capacity, 1 mAh per bit.
  uint8_t soc_raw;             // Estimated state of charge, 1% per bit.
  uint16_t temp_raw;           // Internal temperature sensor, 0.1 K per bit.
} Bq34z100OutputRaw;

typedef struct {
  float avg_current;       // Bus current, [A].
  float bus_voltage;       // Bus voltage, [V].
  uint16_t remaining_mah;  // Remaining capacity, [mAh].
  uint8_t soc_per_cent;    // Estimated state of charge, 1% per bit.
} Bq34z100OutputData;

float Bq34z100BusRawToVolts(uint16_t bus_raw, float cell_mult);
void Bq34z100Convert(const Bq34z100Config *config,
                     const Bq34z100OutputRaw *raw,
                     Bq34z100OutputData *data);

#endif  // AVIONICS_FIRMWARE_DRIVERS_BQ34Z100_TYPES_H_
