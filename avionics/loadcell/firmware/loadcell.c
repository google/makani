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

#include "avionics/loadcell/firmware/loadcell.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/drivers/ad7192.h"
#include "avionics/firmware/serial/carrier_serial.h"
#include "avionics/firmware/serial/loadcell_serial_params.h"
#include "avionics/firmware/util/timer.h"
#include "avionics/loadcell/firmware/calib_params.h"
#include "avionics/loadcell/firmware/output.h"

#define SAMPLE_EXPIRE_MSEC 100
#define FILTER_RATE 0x31  // 100.4 Hz at 4.92 MHz Fclk.
// AD7192 PGA gain is set to 128 for both devices.
#define PGA_GAIN kAd7192Gain128
static const float kPgaScale = 1.0f / 128;
static const int32_t kZeroOffset = 1 << 23;

static LoadcellData g_data;
static Ad7192 g_device[2];
static Timer g_sample_timer[2];
static bool g_sync_performed;

// Rev AA boards have one AD7192 and read diff input 1-2 on A boards and 3-4
// on B boards.
static const Ad7192Config kAd7192ConfigRevAaBoardA = {
  .clock = kAd7192ClockInternal,
  .sinc3 = true,
  .clock_divide = false,
  .single_cycle = false,
  .reject_60hz = false,
  .filter_rate = FILTER_RATE,
  .ref = kAd7192RefIn1,
  .input_select = (1 << kAd7192InputDiff12),
  .burn = false,
  .ref_detect = false,
  .buffer = true,
  .unipolar = false,
  .gain = PGA_GAIN,
  .configure_sync = true,
  .cs_pin = 0,
};

static const Ad7192Config kAd7192ConfigRevAaBoardB = {
  .clock = kAd7192ClockInternal,
  .sinc3 = true,
  .clock_divide = false,
  .single_cycle = false,
  .reject_60hz = false,
  .filter_rate = FILTER_RATE,
  .ref = kAd7192RefIn1,
  .input_select = (1 << kAd7192InputDiff34),
  .burn = false,
  .ref_detect = false,
  .buffer = true,
  .unipolar = false,
  .gain = PGA_GAIN,
  .configure_sync = true,
  .cs_pin = 0,
};

// Rev AB boards have 2 AD7192s and each chip reads diff input 1-2.  Chip 0
// outputs a clock from its internal oscillator and chip 1 uses that clock
// as a reference input to maintain synchronization.
static const Ad7192Config kAd7192ConfigRevAbChip0 = {
  .clock = kAd7192ClockInternalOutput,
  .sinc3 = true,
  .clock_divide = false,
  .single_cycle = false,
  .reject_60hz = false,
  .filter_rate = FILTER_RATE,
  .ref = kAd7192RefIn1,
  .input_select = (1 << kAd7192InputDiff12),
  .burn = false,
  .ref_detect = false,
  .buffer = true,
  .unipolar = false,
  .gain = PGA_GAIN,
  .configure_sync = true,
  .cs_pin = 0,
};

static const Ad7192Config kAd7192ConfigRevAbChip1 = {
  .clock = kAd7192ClockExternalClock,
  .sinc3 = true,
  .clock_divide = false,
  .single_cycle = false,
  .reject_60hz = false,
  .filter_rate = FILTER_RATE,
  .ref = kAd7192RefIn1,
  .input_select = (1 << kAd7192InputDiff12),
  .burn = false,
  .ref_detect = false,
  .buffer = true,
  .unipolar = false,
  .gain = PGA_GAIN,
  .configure_sync = false,
  .cs_pin = 1,
};

static const Ad7192Config *GetAd7192Config(int32_t index) {
  switch (GetCarrierHardwareRevision()) {
    case kLoadcellHardwareRevAa:
      assert(index == 0);
      switch (AppConfigGetAioNode()) {
        case kAioNodeLoadcellPortA:
        case kAioNodeLoadcellStarboardA:
          return &kAd7192ConfigRevAaBoardA;
        case kAioNodeLoadcellPortB:
        case kAioNodeLoadcellStarboardB:
          return &kAd7192ConfigRevAaBoardB;
        default:
          assert(false);
          return &kAd7192ConfigRevAaBoardA;
      }
    case kLoadcellHardwareRevAb:
      switch (index) {
        case 0:
          return &kAd7192ConfigRevAbChip0;
        case 1:
          return &kAd7192ConfigRevAbChip1;
        default:
          assert(false);
          return &kAd7192ConfigRevAbChip0;
      }
    default:
      assert(false);
      return &kAd7192ConfigRevAbChip0;
  }
}

void LoadcellInit(void) {
  memset(&g_data, 0, sizeof(g_data));
  Ad7192Init(&g_device[0]);
  Ad7192Init(&g_device[1]);
  TimerStart(0, &g_sample_timer[0]);
  TimerStart(0, &g_sample_timer[1]);
  g_sync_performed = false;
}

static void LoadcellPollDevice(int32_t index) {
  Ad7192Poll(GetAd7192Config(index), &g_device[index]);
  const Ad7192Data *sample;
  if (Ad7192Read(&g_device[index], &sample)) {
    int32_t strain_index = index;
    if (GetCarrierHardwareRevision() == kLoadcellHardwareRevAa) {
      switch (sample->input) {
        case kAd7192InputDiff12:
          strain_index = 0;
          break;
        case kAd7192InputDiff34:
          strain_index = 1;
          break;
        default:
          assert(false);
          return;
      }
    }
    LoadcellStrain *strain = &g_data.strain[strain_index];
    strain->value_raw = sample->value_raw;

    int32_t raw_signed = (int32_t)sample->value_raw - kZeroOffset;
    strain->value_raw_mv_per_v = 1000.0f * raw_signed * kPgaScale / kZeroOffset;

    if (!sample->valid_parity) {
      strain->status |= kLoadcellStatusParityError;
    }
    if (sample->error) {
      strain->status |= kLoadcellStatusAdcError;
    }
    TimerStartMsec(SAMPLE_EXPIRE_MSEC, &g_sample_timer[index]);
    strain->seq_num++;
    const LoadcellPinCalib *calib = &kLoadcellCalibParams->pin_calib;
    if (strain_index == 0) {
      strain->value = ((int32_t)strain->value_raw - calib->strain_0_zero)
          * calib->strain_0_scale * kPgaScale;
    } else if (strain_index == 1) {
      strain->value = ((int32_t)strain->value_raw - calib->strain_1_zero)
          * calib->strain_1_scale * kPgaScale;
    } else {
      assert(false);
    }
  }
}

void LoadcellPoll(void) {
  if (!g_sync_performed &&
      GetCarrierHardwareRevision() != kLoadcellHardwareRevAa) {
    if (Ad7192Ready(&g_device[0]) && Ad7192Ready(&g_device[1])) {
      Ad7192Sync(&g_device[0]);
      Ad7192Sync(&g_device[1]);
      g_sync_performed = true;
    }
  }
  LoadcellPollDevice(0);
  if (GetCarrierHardwareRevision() != kLoadcellHardwareRevAa) {
    LoadcellPollDevice(1);
  }
  bool warn_ch0 = (g_data.strain[0].status &
                   (kLoadcellStatusParityError | kLoadcellStatusAdcError)) != 0;
  bool warn_ch1 = (g_data.strain[1].status &
                   (kLoadcellStatusParityError | kLoadcellStatusAdcError)) != 0;
  LoadcellOutputLoadcellInvalidWarning(
      warn_ch0 || TimerExpired(&g_sample_timer[0]),
      warn_ch1 || TimerExpired(&g_sample_timer[1]));
}

const LoadcellData *GetLoadcellData() {
  return &g_data;
}
