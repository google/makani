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

#ifndef AVIONICS_FIRMWARE_CPU_PERIPHERALS_H_
#define AVIONICS_FIRMWARE_CPU_PERIPHERALS_H_

#include <stdint.h>

typedef enum {
  kPeripheralInvalid = -1,
  kPeripheralDcan1,
  kPeripheralDcan2,
  kPeripheralDcan3,
  kPeripheralEcap,
  kPeripheralEpwm,
  kPeripheralGio,
  kPeripheralI2c,
  kPeripheralMibAdc1,
  kPeripheralMibAdc2,
  kPeripheralMibSpi1,
  kPeripheralMibSpi3,
  kPeripheralMibSpi5,
  kPeripheralN2Het1,
  kPeripheralN2Het1Tu1,
  kPeripheralN2Het2,
  kPeripheralN2Het2Tu2,
  kPeripheralSci1,
  kPeripheralSci2,
  kPeripheralSpi2,
  kPeripheralSpi4,
  kNumPeripherals
} Peripheral;

int32_t PeripheralGetClockFreq(Peripheral per);
int32_t PeripheralGetClockPrescale(Peripheral per, int32_t output_freq);
void PeripheralEnable(Peripheral per);

#endif  // AVIONICS_FIRMWARE_CPU_PERIPHERALS_H_
