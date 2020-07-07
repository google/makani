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

#include "avionics/platform/firmware/selftest.h"

#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/ground_io_serial_params.h"
#include "avionics/firmware/serial/servo_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "avionics/platform/firmware/config_params.h"
#include "common/macros.h"

void SelfTest(void) {
  const HardwareSpec valid_hardware[] = {
    {kHardwareTypeAio, kAioHardwareRevAb},
    {kHardwareTypeAio, kAioHardwareRevAc},
    {kHardwareTypeAio, kAioHardwareRevAd},
    {kHardwareTypeAio, kAioHardwareRevBa},
    {kHardwareTypeServo, kServoHardwareRevAa},  // TODO: Deprecate.
  };
  SelfTestCommon(ARRAYSIZE(valid_hardware), valid_hardware);

  if (BootConfigGetHardwareType() == kHardwareTypeAio) {
    const CarrierHardwareSpec valid_carrier_hardware[] = {
      {kCarrierHardwareTypeGroundIo, kGroundIoHardwareRevAa},
    };
    SelfTestCarrierCommon(ARRAYSIZE(valid_carrier_hardware),
                          valid_carrier_hardware);
  }
  SelfTestConfigParameters(kPlatformConfigParamsCrc);
}
