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

#include "avionics/firmware/drivers/ssi_encoder.h"

#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "avionics/common/encoder_types.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

// Timing.
#define SSI_HALF_PERIOD_CYCLES CLOCK32_USEC_TO_CYCLES(2)
#define SSI_RECOVERY_CYCLES CLOCK32_USEC_TO_CYCLES(30)
// Set maximum number of bits to clock (at least 1 more than encoder bits).
#define SSI_WORD_LENGTH 33  // Bits.

COMPILE_ASSERT(SSI_WORD_LENGTH > 0, SSI_WORD_LENGTH_must_be_positive);
COMPILE_ASSERT(SSI_WORD_LENGTH <= 64, SSI_WORD_LENGTH_must_fit_within_64_bits);

static uint32_t g_last_read = 0;

const SsiEncoderConfig kSsiEncoderConfigEncoderInterface = {
  .clock = kIoSpi3PinClk,
  .data = {
    [0] = kIoSpi3PinMosi0,
    [1] = kIoSpi3PinMiso0,
    [2] = kIoSpi3PinScs0,
    [3] = kIoSpi3PinScs1,
    [4] = kIoSpi3PinScs4,
    [5] = kIoSpi3PinScs5,
  },
};

const SsiEncoderConfig kSsiEncoderConfigGroundIo = {
  .clock = kIoGioPinA5,
  .data = {
    [0] = kIoN2het1Pin20,
    [1] = kIoN2het1Pin16,
    [2] = kIoN2het1Pin22,
    [3] = kIoN2het1Pin14,
    [4] = kIoN2het1Pin18,
    [5] = kIoN2het1Pin4,
  },
};

static void SetClockHigh(const SsiEncoderConfig *config) {
  assert(config != NULL);
  IoSetValue(config->clock, true);
}

static void SetClockLow(const SsiEncoderConfig *config) {
  assert(config != NULL);
  IoSetValue(config->clock, false);
}

static void ZeroEncoders(SsiEncoderOutput *enc) {
  for (int32_t i = 0; i < ARRAYSIZE(enc->raw); ++i) {
    enc->raw[i] = 0x0;
  }
}

static void SampleBit(const SsiEncoderConfig *config, SsiEncoderOutput *enc) {
  assert(config != NULL);
  // Bits sampled MSB first.
  for (int32_t i = 0; i < ARRAYSIZE(enc->raw); ++i) {
    enc->raw[i] <<= 1;
    enc->raw[i] |= IoGetValue(config->data[i]);
  }
}

static uint64_t ConvertFromGray64(uint64_t value) {
  value = value ^ (value >> 32);
  value = value ^ (value >> 16);
  value = value ^ (value >> 8);
  value = value ^ (value >> 4);
  value = value ^ (value >> 2);
  value = value ^ (value >> 1);
  return value;
}

static bool ReadEncoderInt32(int32_t word_length, uint64_t raw,
                             int32_t *counts) {
  int32_t shift = SSI_WORD_LENGTH - word_length;
  uint32_t mask = 0xFFFFFFFF >> (32 - word_length);

  assert(shift > 1);
  *counts = (int32_t)(raw >> shift);
  *counts &= mask;

  // SSI specification indicates that encoders repeat their value when the
  // host clocks out additional cycles. Between repeated values, encoders
  // clock out an extra zero bit. We use this zero bit to differentiate
  // between a disconnected encoder (all F's) and a connected encoder at the
  // maximum value.
  return (raw & (1ULL << (shift - 1))) == 0ULL;
}

static bool ReadEncoderInt16(int32_t word_length, uint64_t raw,
                             int16_t *counts) {
  int32_t i32;
  bool valid = ReadEncoderInt32(word_length, raw, &i32);
  *counts = (int16_t)i32;
  return valid;
}

void SsiEncoderInit(const SsiEncoderConfig *config) {
  assert(config != NULL);
  g_last_read = Clock32GetCycles();
  // SSI hardware configuration.
  // Part no. 885201-01 with 17-pin M23 (EML) connector and SSI interface.
  // All encoders receive CLOCK.
  // Each individual encoder transmits DATAx.
  IoConfigureAsOutputPushPull(config->clock, true);
  for (int32_t i = 0; i < ARRAYSIZE(config->data); ++i) {
    IoConfigureAsInput(config->data[i]);
  }

  // Set default clock state (high).
  SetClockHigh(config);
}

// SSI data interface:
//   Clock remains high during quiescent state.
//   Bit transitions occur on rising edge of clock.
//   Bit sampling occurs on falling edge of clock.
//
// Clock: ____    __    __    __    __        __    ________
//            |__|  |__|  |__|  |__|   ... __|  |__|
//             cal
// Data:  _______ _____ _____ _____ __ ... __ _____      ___
//               X_____X_____X_____X__ ... __X_____\____/
//                 n     n-1   n-2             1    recovery
//
// Timing:
//   Calibration pulse width: t_cal < 5 us.
//   Clock period: 1 < t_clock < 10 us.
//   Recovery time: 12 < t_recovery < 30 us.
void SsiEncoderRead(const SsiEncoderConfig *config, SsiEncoderOutput *enc) {
  assert(config != NULL);
  assert(enc != NULL);

  // Enforce recovery time.
  int32_t recovery_cycles = CLOCK32_SUBTRACT(Clock32GetCycles(), g_last_read);
  if (0 < recovery_cycles && recovery_cycles < SSI_RECOVERY_CYCLES) {
    Clock32WaitCycles(SSI_RECOVERY_CYCLES - recovery_cycles);
  }

  // Calibration start (latch data).
  SetClockLow(config);
  ZeroEncoders(enc);
  Clock32WaitCycles(SSI_HALF_PERIOD_CYCLES);
  SetClockHigh(config);
  for (int32_t i = 0; i < SSI_WORD_LENGTH; ++i) {
    // Sample on falling edge.
    Clock32WaitCycles(SSI_HALF_PERIOD_CYCLES);
    SetClockLow(config);
    SampleBit(config, enc);
    Clock32WaitCycles(SSI_HALF_PERIOD_CYCLES);
    SetClockHigh(config);
  }
  g_last_read = Clock32GetCycles();
}

bool SsiGetIha608Multi(const SsiEncoderOutput *enc, int32_t ch,
                       int32_t *counts) {
  assert(enc != NULL);
  assert(0 <= ch && ch < ARRAYSIZE(enc->raw));
  assert(counts != NULL);

  return ReadEncoderInt32(25, enc->raw[ch], counts);
}

bool SsiGetRha507Single(const SsiEncoderOutput *enc, int32_t ch,
                        int16_t *counts) {
  assert(enc != NULL);
  assert(0 <= ch && ch < ARRAYSIZE(enc->raw));
  assert(counts != NULL);

  return ReadEncoderInt16(13, enc->raw[ch], counts);
}

bool SsiGetAmo4306Single(const SsiEncoderOutput *enc, int32_t ch,
                         int32_t *counts, uint8_t *flags) {
  assert(enc != NULL);
  assert(0 <= ch && ch < ARRAYSIZE(enc->raw));
  assert(counts != NULL);
  assert(flags != NULL);

  // Read raw value.
  int32_t read_data;
  bool read_valid = ReadEncoderInt32(28, enc->raw[ch], &read_data);

  // Lower 10 bits of encoder value are in microns and range from 0 to 1000.
  // Higher bits are in mm.
  *counts = ((read_data >> 3) & 0x3FF) + (read_data >> 13) * 1000;

  // Compute parity (even).
  bool parity_valid = true;
  for (int32_t tmp = read_data; tmp != 0x0; tmp &= tmp - 1) {
    parity_valid = !parity_valid;
  }

  // Rewrite parity bit as parity error.
  *flags = (uint8_t)(read_data & (kAmo4306Warning | kAmo4306Error));
  if (!parity_valid) {
    *flags |= kAmo4306InvalidParity;
  }
  return read_valid;
}

bool SsiGetMagA300(const SsiEncoderOutput *enc, int32_t ch,
                   int16_t *counts) {
  assert(enc != NULL);
  assert(0 <= ch && ch < ARRAYSIZE(enc->raw));
  assert(counts != NULL);

  bool valid = ReadEncoderInt16(13, enc->raw[ch], counts);
  *counts = (uint32_t)ConvertFromGray64(*counts);
  return valid;
}

bool SsiGetMagA550(const SsiEncoderOutput *enc, int32_t ch,
                   int32_t *counts) {
  assert(enc != NULL);
  assert(0 <= ch && ch < ARRAYSIZE(enc->raw));
  assert(counts != NULL);

  bool valid = ReadEncoderInt32(16, enc->raw[ch], counts);
  *counts = (uint32_t)ConvertFromGray64(*counts);
  return valid;
}
