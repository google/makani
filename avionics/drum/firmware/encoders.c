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

#include "avionics/drum/firmware/encoders.h"

#include <stddef.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/faults.h"
#include "avionics/drum/firmware/config_params.h"
#include "avionics/firmware/drivers/encoder.h"
#include "avionics/firmware/drivers/encoder_params.h"
#include "avionics/firmware/drivers/ssi_encoder.h"
#include "avionics/firmware/identity/identity.h"

static const SsiEncoderConfig *GetConfig(void) {
  if (BootConfigGetHardwareType() == kHardwareTypeServo) {
    return &kSsiEncoderConfigEncoderInterface;
  }
  return &kSsiEncoderConfigGroundIo;
}

void EncodersInit(void) {
  SsiEncoderInit(GetConfig());
}

void EncodersRead(GsDrumEncoders *out) {
  SsiEncoderOutput encoder;
  SsiEncoderRead(GetConfig(), &encoder);
  // TODO(b/118317417): Temporarily clear encoder errors so that they do not
  // latch, to debug observed encoder errors.
  ClearErrors(&out->status);

  // Tether detwist.
  // The detwist encoder is no longer read by the drum nodes.

  // GSG axis 1.
  EncoderStatus gsg_azi_status = EncoderGet(&kDrumConfigParams->gsg_azi,
                                            &encoder, &out->gsg_azi, NULL);
  SignalWarning(kGsDrumEncodersWarningGsgAzimuth,
                gsg_azi_status & kEncoderStatusWarning,
                &out->status);
  SignalError(kGsDrumEncodersErrorGsgAzimuth,
              gsg_azi_status & kEncoderStatusError,
              &out->status);

  // GSG axis 2.
  EncoderStatus gsg_ele_status = EncoderGet(&kDrumConfigParams->gsg_ele,
                                            &encoder, &out->gsg_ele, NULL);
  SignalWarning(kGsDrumEncodersWarningGsgElevation,
                gsg_ele_status & kEncoderStatusWarning,
                &out->status);
  SignalError(kGsDrumEncodersErrorGsgElevation,
              gsg_ele_status & kEncoderStatusError,
              &out->status);

  // Detwist.
  EncoderStatus detwist_status = EncoderGet(&kDrumConfigParams->tether_detwist,
                                            &encoder, &out->detwist, NULL);
  SignalWarning(kGsDrumEncodersWarningDetwist,
                detwist_status & kEncoderStatusWarning,
                &out->status);
  SignalError(kGsDrumEncodersErrorDetwist,
              detwist_status & kEncoderStatusError,
              &out->status);
}
