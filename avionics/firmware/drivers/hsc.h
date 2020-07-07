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

#ifndef AVIONICS_FIRMWARE_DRIVERS_HSC_H_
#define AVIONICS_FIRMWARE_DRIVERS_HSC_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/hsc_model.h"

typedef enum {
  kHscDeviceInvalid = -1,
  kHscDeviceAltitude,
  kHscDevicePitch,
  kHscDeviceSpeed,
  kHscDeviceYaw,
  kNumHscDevices
} HscDevice;

typedef struct {
  uint8_t status;
  uint16_t pressure;
  uint16_t temperature;
  bool invalid;
} HscData;

void HscInit(void);
bool HscTrigger(HscDevice dev);
bool HscRead(HscData *data);
float HscPressureRawToPa(HscModel model, const HscData *raw);
float HscTemperatureRawToC(const HscData *raw);
void HscSample(HscDevice dev, HscData *data);

#endif  // AVIONICS_FIRMWARE_DRIVERS_HSC_H_
