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

#include "avionics/platform/firmware/encoders.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/faults.h"
#include "avionics/firmware/drivers/encoder.h"
#include "avionics/firmware/drivers/ssi_encoder.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/platform/firmware/config_params.h"
#include "common/macros.h"

static const SsiEncoderConfig *GetConfig(void) {
  if (BootConfigGetHardwareType() == kHardwareTypeServo) {
    return &kSsiEncoderConfigEncoderInterface;
  }
  return &kSsiEncoderConfigGroundIo;
}

static void HandleEncoder(const EncoderCalib *calib,
                          const SsiEncoderOutput *encoder,
                          uint16_t warning_flag, uint16_t error_flag,
                          float *out, uint8_t *flags,
                          StatusFlags *status) {
  EncoderStatus encoder_status = EncoderGet(calib, encoder, out, flags);
  SignalWarning(warning_flag, encoder_status & kEncoderStatusWarning, status);
  SignalError(error_flag, encoder_status & kEncoderStatusError, status);
}
void EncodersInit(void) {
  SsiEncoderInit(GetConfig());
}

void EncodersRead(GsPerchEncoders *out) {
  SsiEncoderOutput encoder;
  SsiEncoderRead(GetConfig(), &encoder);
  // TODO(b/118317417): Temporarily clear encoder errors so that they do not
  // latch, to debug observed encoder errors.
  ClearErrors(&out->status);

  HandleEncoder(&kPlatformConfigParams->perch_azi, &encoder,
                kGsPerchEncodersWarningPerchAzimuth,
                kGsPerchEncodersErrorPerchAzimuth,
                &out->perch_azi, &out->perch_azi_flags, &out->status);

  HandleEncoder(&kPlatformConfigParams->levelwind_shoulder, &encoder,
                kGsPerchEncodersWarningLevelwindShoulder,
                kGsPerchEncodersErrorLevelwindShoulder,
                &out->levelwind_shoulder, NULL, &out->status);

  HandleEncoder(&kPlatformConfigParams->levelwind_wrist, &encoder,
                kGsPerchEncodersWarningLevelwindWrist,
                kGsPerchEncodersErrorLevelwindWrist,
                &out->levelwind_wrist, NULL, &out->status);

  out->levelwind_ele = out->levelwind_shoulder + out->levelwind_wrist;
  SignalWarning(
      kGsPerchEncodersWarningLevelwindElevation,
      CheckWarning(&out->status, kGsPerchEncodersWarningLevelwindShoulder) ||
      CheckWarning(&out->status, kGsPerchEncodersWarningLevelwindWrist),
      &out->status);
  SignalError(
      kGsPerchEncodersErrorLevelwindElevation,
      CheckWarning(&out->status, kGsPerchEncodersErrorLevelwindShoulder) ||
      CheckWarning(&out->status, kGsPerchEncodersErrorLevelwindWrist),
      &out->status);

  HandleEncoder(&kPlatformConfigParams->drum_pos, &encoder,
                kGsPerchEncodersWarningDrumPosition,
                kGsPerchEncodersErrorDrumPosition,
                &out->drum_pos, NULL, &out->status);
}
