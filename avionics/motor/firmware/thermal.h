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

#ifndef AVIONICS_MOTOR_FIRMWARE_THERMAL_H_
#define AVIONICS_MOTOR_FIRMWARE_THERMAL_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/motor_thermal_types.h"
#include "avionics/firmware/drivers/si7021_types.h"
#include "avionics/firmware/serial/motor_serial_params.h"
#include "avionics/motor/firmware/config_params.h"

typedef struct {
  float channel_temps[kNumMotorThermalChannels];
  int32_t over_temp_count[kNumMotorThermalChannels];
  int32_t temp_read_errors;
  uint32_t warning_bitmask;
} MotorThermalData;

typedef struct {
  float thermal_limits[kNumMotorThermalChannels];
  int32_t over_temp_threshold;
  int32_t temp_read_error_threshold;
} MotorThermalParams;

// Initialize motor thermal limits.
void MotorThermalInit(MotorType motor_type, MotorHardware controller_type);

// Returns a bitmask of motor thermal warnings.
uint32_t MotorThermalTempWarnings(void);

// HT3000 thermal interface.
void MotorThermalHt3000Init(float sample_freq);

// Called from hardware monitoring infrastructure.
void MotorThermalUpdateMcp342x(int32_t monitor, int32_t t_raw, bool valid);
void MotorThermalUpdateMcp9800(int32_t monitor, float data);
void MotorThermalUpdateSi7021(int32_t monitor, const Si7021OutputData *data);
void MotorThermalUpdateHt3000(void);

const MotorThermalData *MotorThermalGetData(void);

extern MotorThermalParams g_motor_thermal_params;

#endif  // AVIONICS_MOTOR_FIRMWARE_THERMAL_H_
