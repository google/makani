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

#include "avionics/cs/firmware/selftest.h"

#include "avionics/cs/firmware/config_params.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/serial/cs_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "common/macros.h"

void SelfTest(void) {
  const HardwareSpec valid_hardware[] = {
    {kHardwareTypeCs, kCsHardwareRevAa},
    {kHardwareTypeCs, kCsHardwareRevAb},
    {kHardwareTypeCs, kCsHardwareRevAc},
    {kHardwareTypeCs, kCsHardwareRevAdClk8},
    {kHardwareTypeCs, kCsHardwareRevAdClk16},
  };
  SelfTestCommon(ARRAYSIZE(valid_hardware), valid_hardware);
  SelfTestConfigParameters(CsConfigParamsGetTypeVersion());
}
