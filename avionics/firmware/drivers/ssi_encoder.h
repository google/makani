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

#ifndef AVIONICS_FIRMWARE_DRIVERS_SSI_ENCODER_H_
#define AVIONICS_FIRMWARE_DRIVERS_SSI_ENCODER_H_

// This module implements a Synchronous Serial Interface (SSI) for various
// encoders. See the following documents for the protocol specification.
// For Leinelinde, see the "Programmable SSI Manual" from
// http://www.leinelinde.com/Products/Downloads/
// For AMO GmbH, see "Specification, Interface Description - SSI" from
// http://www.amo-gmbh.com/fileadmin/amo/produkte/Schnittstellen/SP-EW_InterfaceDescription_SSI_rev05.1.pdf

#include <stdint.h>

#include "avionics/firmware/cpu/io.h"

#define NUM_SSI_ENCODERS 6

typedef struct {
  IoPin clock;
  IoPin data[NUM_SSI_ENCODERS];
} SsiEncoderConfig;

extern const SsiEncoderConfig kSsiEncoderConfigEncoderInterface;
extern const SsiEncoderConfig kSsiEncoderConfigGroundIo;

typedef struct {
  uint64_t raw[NUM_SSI_ENCODERS];
} SsiEncoderOutput;

void SsiEncoderInit(const SsiEncoderConfig *config);

// Caller should observe >30us recovery time before calling this function again.
void SsiEncoderRead(const SsiEncoderConfig *config, SsiEncoderOutput *enc);

// Leinelinde rotary encoders.
bool SsiGetIha608Multi(const SsiEncoderOutput *enc, int32_t ch,
                       int32_t *counts);
bool SsiGetRha507Single(const SsiEncoderOutput *enc, int32_t ch,
                        int16_t *counts);

// Automatisierung Messtechnik Optik GmbH encoders.
bool SsiGetAmo4306Single(const SsiEncoderOutput *enc, int32_t ch,
                         int32_t *counts, uint8_t *flags);

// Hubner MAGA encoders.
bool SsiGetMagA300(const SsiEncoderOutput *enc, int32_t ch, int16_t *counts);
bool SsiGetMagA550(const SsiEncoderOutput *enc, int32_t ch, int32_t *counts);

#endif  // AVIONICS_FIRMWARE_DRIVERS_SSI_ENCODER_H_
