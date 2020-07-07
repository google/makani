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

#include "avionics/joystick/firmware/selftest.h"

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/joystick/firmware/calib_params.h"
#include "avionics/joystick/firmware/config_params.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/ground_io_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "common/macros.h"

static void ValidateAxisCalibration(const char *name,
                                    const AxisCalibration *c) {
  if (c->index < 0 || c->index >= JOYSTICK_NUM_RAW_CHANNELS) {
    SelfTestFailedLoop(kSelfTestFailureInvalidCalibParams,
                       "Axis %s index (%ld) is not between 0 and %d",
                       name, c->index, JOYSTICK_NUM_RAW_CHANNELS);
  }
}

void SelfTest(void) {
  const HardwareSpec valid_hardware[] = {
    {kHardwareTypeAio, kAioHardwareRevAb},
    {kHardwareTypeAio, kAioHardwareRevAc},
    {kHardwareTypeAio, kAioHardwareRevAd},
    {kHardwareTypeAio, kAioHardwareRevBa},
  };
  const CarrierHardwareSpec valid_carrier_hardware[] = {
    {kCarrierHardwareTypeJoystick, kJoystickHardwareRevAa},
    {kCarrierHardwareTypeGroundIo, kGroundIoHardwareRevAa},
  };

  SelfTestCommon(ARRAYSIZE(valid_hardware), valid_hardware);
  SelfTestCarrierCommon(ARRAYSIZE(valid_carrier_hardware),
                        valid_carrier_hardware);

  SelfTestConfigParameters(JoystickConfigParamsGetTypeVersion());

  SelfTestCalibParameters(JoystickCalibParamsGetTypeVersion());

  ValidateAxisCalibration("roll", &kJoystickCalibParams->roll);
  ValidateAxisCalibration("pitch", &kJoystickCalibParams->pitch);
  ValidateAxisCalibration("yaw", &kJoystickCalibParams->yaw);
  ValidateAxisCalibration("throttle", &kJoystickCalibParams->throttle);
}
