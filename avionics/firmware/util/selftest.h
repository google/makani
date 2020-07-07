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

#ifndef AVIONICS_FIRMWARE_UTIL_SELFTEST_H_
#define AVIONICS_FIRMWARE_UTIL_SELFTEST_H_

#include <stdint.h>

#include "avionics/firmware/identity/identity.h"

typedef struct {
  HardwareType hardware_type;
  int32_t hardware_revision;
} HardwareSpec;

typedef struct {
  CarrierHardwareType hardware_type;
  int32_t hardware_revision;
} CarrierHardwareSpec;

void SelfTestCompatibleHardware(int32_t num, const HardwareSpec *possible);
void SelfTestNetworkIdentity(void);
void SelfTestBootloader(void);
void SelfTestCommon(int32_t num, const HardwareSpec *possible);
void SelfTestCarrierCommon(int32_t num, const CarrierHardwareSpec *possible);
void SelfTestCalibParameters(uint32_t possible);
void SelfTestCalibParametersMany(int32_t num, const uint32_t *possible);
void SelfTestConfigParameters(uint32_t possible);
void SelfTestConfigParametersMany(int32_t num, const uint32_t *possible);
void SelfTestSerialParameters(void);
void SelfTestFailedLoop(uint32_t code, const char *reason, ...)
    __attribute__((format(printf, 2, 3)));

#endif  // AVIONICS_FIRMWARE_UTIL_SELFTEST_H_
