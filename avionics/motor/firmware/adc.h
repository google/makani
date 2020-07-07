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

#ifndef AVIONICS_MOTOR_FIRMWARE_ADC_H_
#define AVIONICS_MOTOR_FIRMWARE_ADC_H_

#include <stdbool.h>

#include "avionics/common/motor_adc_defines.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/serial/motor_serial_params.h"

// ADC data order.
typedef union {
  struct {
    union ADCRAM_RAM gin_v_in_monitor[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_i_phase_a[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_i_phase_b[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_i_bus[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_unused[NUM_ADC_SAMPLES];
  };
  struct {
    union ADCRAM_RAM ozone_v_in_monitor[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_v_chassis[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_i_phase_a[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_i_phase_b[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_i_bus[NUM_ADC_SAMPLES];
  };
} Adc1PackedData;

typedef union {
  struct {
    union ADCRAM_RAM gin_v_bus[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_i_phase_c[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_i_phase_b[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_v_aux_monitor[NUM_ADC_SAMPLES];
    union ADCRAM_RAM gin_unused[NUM_ADC_SAMPLES];
  };
  struct {
    union ADCRAM_RAM ozone_v_cm[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_i_phase_c[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_i_phase_b[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_v_aux_monitor[NUM_ADC_SAMPLES];
    union ADCRAM_RAM ozone_v_bus[NUM_ADC_SAMPLES];
  };
} Adc2PackedData;

extern volatile Adc1PackedData g_adc_1_buffer;
extern volatile Adc2PackedData g_adc_2_buffer;

void MotorAdcInit(MotorHardware motor_controller_type);
void MotorAdcCalibrateOffsets(void);
uint32_t MotorAdcMagnitudeInterruptReset(float phase_limit);
uint32_t MotorAdcMagnitudeInterruptInit(float phase_limit);
void MotorAdcMagnitudeInterruptEnable(void);
uint32_t MotorAdcGetMagnitudeErrors(void);
void MotorAdcGetValues(float *ia, float *ib, float *ic,
                       float *i_bus, float *v_bus, float *v_chassis,
                       float *v_cm, float *v_in_mon, float *v_aux_mon)
    __attribute__((optimize("unroll-loops")));

#endif  // AVIONICS_MOTOR_FIRMWARE_ADC_H_
