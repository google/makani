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

#include "avionics/loadcell/firmware/selftest.h"

#include <math.h>
#include <stddef.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/identity/identity_types.h"
#include "avionics/firmware/serial/aio_serial_params.h"
#include "avionics/firmware/serial/loadcell_serial_params.h"
#include "avionics/firmware/util/selftest.h"
#include "avionics/loadcell/firmware/calib_params.h"
#include "common/macros.h"


static const char *CheckLoadcellCalibParams(void) {
  if ((kLoadcellCalibParams->pin_calib.strain_0_scale < 1.0f) ||
      (kLoadcellCalibParams->pin_calib.strain_0_scale > 100.0f) ||
      (kLoadcellCalibParams->pin_calib.strain_1_scale < 1.0f) ||
      (kLoadcellCalibParams->pin_calib.strain_1_scale > 100.0f)) {
    return "Strain scale config param outside expected range [1.0, 100.0].";
  }
  if (isnan(kLoadcellCalibParams->pin_calib.strain_0_scale) ||
      isnan(kLoadcellCalibParams->pin_calib.strain_1_scale)) {
    return "Strain scale config param is NaN.";
  }
  if ((kLoadcellCalibParams->pin_calib.strain_0_zero < 0) ||
      (kLoadcellCalibParams->pin_calib.strain_0_zero >= (1 << 24)) ||
      (kLoadcellCalibParams->pin_calib.strain_1_zero < 0) ||
      (kLoadcellCalibParams->pin_calib.strain_1_zero >= (1 << 24))) {
    return "Strain zero config param outside expected range [0, 2^24)";
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
    {kCarrierHardwareTypeLoadcell, kLoadcellHardwareRevAa},
    {kCarrierHardwareTypeLoadcell, kLoadcellHardwareRevAb},
  };

  SelfTestCommon(ARRAYSIZE(valid_hardware), valid_hardware);
  SelfTestCarrierCommon(ARRAYSIZE(valid_carrier_hardware),
                        valid_carrier_hardware);
  SelfTestCalibParameters(LoadcellCalibParamsGetTypeVersion());
  const char *failure = CheckLoadcellCalibParams();
  if (failure != NULL) {
    SelfTestFailedLoop(kSelfTestFailureInvalidCalibParams, failure);
  }
}
