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

#ifndef AVIONICS_FIRMWARE_DRIVERS_ENCODER_H_
#define AVIONICS_FIRMWARE_DRIVERS_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/drivers/encoder_params.h"
#include "avionics/firmware/drivers/ssi_encoder.h"

typedef enum {
  kEncoderStatusSuccess = 0x0,
  kEncoderStatusWarning = 1 << 1,
  kEncoderStatusError   = 1 << 2,
} EncoderStatus;

bool CalibrateAngleEncoder(const EncoderCalib *cal, int32_t in, float *out);

EncoderStatus EncoderGet(const EncoderCalib *cal, const SsiEncoderOutput *in,
                         float *value, uint8_t *flags);

#endif  // AVIONICS_FIRMWARE_DRIVERS_ENCODER_H_
