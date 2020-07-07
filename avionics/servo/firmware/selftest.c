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

#include "avionics/servo/firmware/selftest.h"

#include <stdbool.h>
#include <stdio.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/network/aio_node.h"
#include "avionics/servo/firmware/input.h"
#include "avionics/servo/firmware/config_params.h"
#include "avionics/servo/firmware/calib_params.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/servo_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "common/macros.h"


static const char *CheckServoConfigParams(void) {
  if (kServoConfigParams->servo_max_limit > PI_F
      || kServoConfigParams->servo_max_limit < -PI_F) {
    return "Flash param servo_max_limit outside expected range [-PI, PI].";
  }
  if (kServoConfigParams->servo_min_limit > PI_F
      || kServoConfigParams->servo_min_limit < -PI_F) {
    return "Flash param servo_min_limit outside expected range [-PI, PI].";
  }
  if (kServoConfigParams->servo_max_limit
      <= kServoConfigParams->servo_min_limit) {
    return "Flash param servo_max_limit is less than servo_min_limit.";
  }
  if (ServoInvertAngleCal(kServoConfigParams->servo_max_limit)
      <= ServoInvertAngleCal(kServoConfigParams->servo_min_limit)) {
    return "Invalid servo limit parameters wrap around R22 encoder.";
  }
  if (kServoConfigParams->current_limit < 0.0
      || kServoConfigParams->current_limit > 15.0) {
    return "Invalid current limit outside range [0.0, 15.0].";
  }
  if (kServoConfigParams->current_velocity_limit < 0.0
      || kServoConfigParams->current_velocity_limit > 15.0) {
    return "Invalid current velocity limit outside range [0.0, 15.0].";
  }
  return NULL;
}

static const char *CheckServoCalibParams(void) {
  if (kServoCalibParams->resolver_zero > PI_F / 2.0
      || kServoCalibParams->resolver_zero < -PI_F / 2.0) {
    return "Calib param resolver_zero outside expected range [-PI/2, PI/2].";
  }
  if (kServoCalibParams->direction != 1
      && kServoCalibParams->direction != -1) {
    return "Invalid servo direction flag not 1 or -1.";
  }
  return NULL;
}

void SelfTest(void) {
  const HardwareSpec valid_hardware[] = {
    {kHardwareTypeAio, kAioHardwareRevAb},
    {kHardwareTypeAio, kAioHardwareRevAc},
    {kHardwareTypeAio, kAioHardwareRevAd},
    {kHardwareTypeAio, kAioHardwareRevBa},
  };
  const CarrierHardwareSpec valid_carrier_hardware[] = {
    {kCarrierHardwareTypeServo, kServoHardwareRevBa},
    {kCarrierHardwareTypeServo, kServoHardwareRevBb},
    {kCarrierHardwareTypeServo, kServoHardwareRevBc},
  };

  SelfTestCommon(ARRAYSIZE(valid_hardware), valid_hardware);
  SelfTestCarrierCommon(ARRAYSIZE(valid_carrier_hardware),
                        valid_carrier_hardware);

  SelfTestConfigParameters(ServoConfigParamsGetTypeVersion());
  const char *failure = CheckServoConfigParams();
  if (failure != NULL) {
    SelfTestFailedLoop(kSelfTestFailureInvalidConfigParams, failure);
  }

  SelfTestCalibParameters(ServoCalibParamsGetTypeVersion());
  failure = CheckServoCalibParams();
  if (failure != NULL) {
    SelfTestFailedLoop(kSelfTestFailureInvalidCalibParams, failure);
  }
}
