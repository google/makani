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

#include "avionics/batt/firmware/selftest.h"

#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/batt_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "common/macros.h"


void SelfTest(void) {
  const HardwareSpec valid_hardware[] = {
    {kHardwareTypeAio, kAioHardwareRevAb},
    {kHardwareTypeAio, kAioHardwareRevAc},
    {kHardwareTypeAio, kAioHardwareRevAd},
    {kHardwareTypeAio, kAioHardwareRevBa},
  };
  const CarrierHardwareSpec valid_carrier_hardware[] = {
    {kCarrierHardwareTypeBattery, kBattHardwareSmallCell15V1},
    {kCarrierHardwareTypeBattery, kBattHardwareBigCell18V1},
    {kCarrierHardwareTypeBattery, kBattHardwareSmallCell15Aa},
    {kCarrierHardwareTypeBattery, kBattHardwareBigCell18Aa},
    {kCarrierHardwareTypeBattery, kBattHardwareSmallCell15Ab},
    {kCarrierHardwareTypeBattery, kBattHardwareBigCell18Ab},
    {kCarrierHardwareTypeBattery, kBattHardwareSmallCell17Ab},
    {kCarrierHardwareTypeBattery, kBattHardwareSmallCell15Ac},
    {kCarrierHardwareTypeBattery, kBattHardwareBigCell18Ac},
    {kCarrierHardwareTypeBattery, kBattHardwareSmallCell17Ac},
    {kCarrierHardwareTypeBattery, kBattHardwareSmallCell17Ad},
  };

  SelfTestCommon(ARRAYSIZE(valid_hardware), valid_hardware);
  SelfTestCarrierCommon(ARRAYSIZE(valid_carrier_hardware),
                        valid_carrier_hardware);
}
