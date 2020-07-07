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

#include "avionics/plc/firmware/selftest.h"

#include "avionics/common/avionics_messages.h"
#include "avionics/common/tether_convert.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/params/param_util.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/ground_io_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "avionics/plc/firmware/config_params.h"
#include "avionics/plc/firmware/plc_interface.h"
#include "common/macros.h"

static const char *CheckPlcConfigParams(void) {
  if (!ParamStringToEthernetAddress(kPlcConfigParams->plc_mac, NULL)) {
    return "Flash param string plc_mac invalid. Format is XXXXXXXXXXXX.";
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
    {kCarrierHardwareTypeGroundIo, kGroundIoHardwareRevAa},
  };

  SelfTestCommon(ARRAYSIZE(valid_hardware), valid_hardware);
  SelfTestCarrierCommon(ARRAYSIZE(valid_carrier_hardware),
                        valid_carrier_hardware);
  SelfTestConfigParameters(PlcConfigParamsGetTypeVersion());
  const char *failure = CheckPlcConfigParams();
  if (failure != NULL) {
    SelfTestFailedLoop(kSelfTestFailureInvalidConfigParams, failure);
  }
}
