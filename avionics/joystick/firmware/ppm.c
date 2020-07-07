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

#include "avionics/joystick/firmware/ppm.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/joystick/firmware/calib_params.h"
#include "avionics/joystick/firmware/ecap_bits.h"
#include "avionics/joystick/firmware/output.h"
#include "common/macros.h"

// 40ms is twice as long as a full PPM cycle (see Ecap1Poll() below.)
#define SAMPLE_TIMEOUT_PERIOD_CYCLES CLOCK32_MSEC_TO_CYCLES(40)

static uint32_t g_last_sample;
static bool g_send_raw_status;

// We only support 7 channel joysticks currently.
COMPILE_ASSERT(JOYSTICK_NUM_RAW_CHANNELS == 7,
               JOYSTICK_NUM_RAW_CHANNELS_must_be_seven);

static uint32_t MillisecondsToCounterTicks(uint32_t ms) {
  int32_t freq = PeripheralGetClockFreq(kPeripheralEcap);
  assert(freq > 0);
  return freq / 1000 * ms;
}

static float Clamp(float value, float min, float max) {
  if (value < min) {
    return min;
  } else if (value > max) {
    return max;
  } else {
    return value;
  }
}

static float ApplyCalibration(
    const uint32_t raw_data[JOYSTICK_NUM_RAW_CHANNELS],
    const AxisCalibration *c) {

  float value = raw_data[c->index];
  value += c->offset;
  value *= c->gain;

  return Clamp(value, c->min, c->max);
}

static JoystickSwitchPositionLabel DecodeSwitch(
    const uint32_t raw_data[JOYSTICK_NUM_RAW_CHANNELS],
    const SwitchCalibration *c) {
  uint32_t half = (c->max - c->min) / 2;
  uint32_t value = raw_data[c->index];

  if (value < c->min + half) {
    return kJoystickSwitchPositionDown;
  } else {
    return kJoystickSwitchPositionUp;
  }
}

static JoystickSwitchPositionLabel DecodeTriSwitch(
    const uint32_t raw_data[JOYSTICK_NUM_RAW_CHANNELS],
    const SwitchCalibration *c) {

  uint32_t third = (c->max - c->min) / 3;
  uint32_t value = raw_data[c->index];

  if (value < c->min + third) {
    return kJoystickSwitchPositionDown;
  } else if (value < c->min + 2 * third) {
    return kJoystickSwitchPositionMiddle;
  } else {
    return kJoystickSwitchPositionUp;
  }
}

static void UpdateJoystickStatus(
    uint32_t channel_data[JOYSTICK_NUM_RAW_CHANNELS]) {
  JoystickRawStatusMessage raw_msg;
  int32_t i;

  JoystickOutputRoll(
      ApplyCalibration(channel_data, &kJoystickCalibParams->roll));
  JoystickOutputPitch(
      ApplyCalibration(channel_data, &kJoystickCalibParams->pitch));
  JoystickOutputYaw(
      ApplyCalibration(channel_data, &kJoystickCalibParams->yaw));
  JoystickOutputThrottle(
      ApplyCalibration(channel_data, &kJoystickCalibParams->throttle));

  // These switches appear to be debounced by the joystick so there is no need
  // to handle that here.
  JoystickOutputTriSwitch(
      DecodeTriSwitch(channel_data, &kJoystickCalibParams->tri_switch));
  JoystickOutputMomentarySwitch(
      DecodeSwitch(channel_data, &kJoystickCalibParams->momentary_switch));

  if (g_send_raw_status) {
    for (i = 0; i < JOYSTICK_NUM_RAW_CHANNELS; ++i) {
      raw_msg.channel[i] = channel_data[i];
    }
    NetSendAioJoystickRawStatusMessage(&raw_msg);
  }
}

void PpmInit(void) {
  // Setup eCAP1 for Double-VCLK4-Synchronized input.
  IommClearPinmux(43, 0);

  // Enable clock for eCAP1.
  IommSetPinmux(39, 0);

  // Select eCAP1 for pin N2HET1[15]/MIBSPI1NCS[4]/ECAP1.
  IommSetPinmux(8, 18);

  // Set eCAP1 up for:
  //   * Two capture events cap1=rising cap2=falling.
  //   * Delta mode (measure time between edges.)
  //   * Continuous operation.
  ECAP(1).ECCTL1.CAP1POL = EC_RISING;
  ECAP(1).ECCTL1.CAP2POL = EC_FALLING;

  ECAP(1).ECCTL1.CTRRST1 = EC_DELTA_MODE;
  ECAP(1).ECCTL1.CTRRST2 = EC_DELTA_MODE;

  ECAP(1).ECCTL1.CAPLDEN = 1;

  // Don't divide VCLK4 resulting in an 80MHz clock.
  // This gives ~53 seconds before counter overflow.
  ECAP(1).ECCTL1.PRESCALE = EC_DIV1;

  ECAP(1).ECCTL2.CAP_APWM = EC_CAP_MODE;
  ECAP(1).ECCTL2.CONT_ONESHT = EC_CONTINUOUS;
  ECAP(1).ECCTL2.SYNCO_SEL = EC_SYNCO_DIS;
  ECAP(1).ECCTL2.SYNCI_EN = 0;
  ECAP(1).ECCTL2.STOP_WRAP = EC_EVENT2;

  // Clear spurious event status.
  ECAP(1).ECCLR.raw = ECAP(1).ECFLG.raw;
}

void PpmRun(void) { ECAP(1).ECCTL2.TSCTRSTOP = EC_RUN; }

// The Futaba joystick uses Pulse Position Modulation.  The channel
// data is encoded as a series of 0.4ms low pulses (A) separated by a
// high pulse (B) whose length (between 7.2ms and 1.7ms) is proportional
// to the channel value.  Once all 7 channels have been encoded the signal
// remains high (C, sync pulse) until the start of the next cycle.  This
// repeats every 20ms.
// ___   __   ___   __   __   __   __   __   ______________________
//    | | B|A| B |A| B|A| B|A| B|A| B|A| B|A|          C           |
//    |_|  |_|   |_|  |_|  |_|  |_|  |_|  |_|                      |_
//
void PpmPoll(void) {
  static uint32_t channel_data[JOYSTICK_NUM_RAW_CHANNELS];
  static int32_t channel = -1;  // Negative values denote unarmed state.

  uint32_t now = Clock32GetCycles();

  union ECAP_ECFLG status = ECAP(1).ECFLG;

  if (status.CTROVF) {
    // An overflow should only happen when we are not getting
    // a pulse train from the joystick.  Disarm sampling engine.
    channel = -1;
  }

  if (status.CEVT2) {
    // Falling edge.
    uint32_t cap2 = ECAP(1).CAP2.raw;

    // If all channels are at their max values the sync pulse will be
    // at least 6ms.
    if (cap2 > MillisecondsToCounterTicks(6)) {
      // Sync pulse.  Arm sampling engine.
      channel = 0;
    } else if (channel >= 0) {
      // Channel pulse.
      channel_data[channel] = cap2;

      ++channel;
      if (channel >= JOYSTICK_NUM_RAW_CHANNELS) {
        UpdateJoystickStatus(channel_data);
        channel = -1;

        JoystickOutputPresence(true);
        g_last_sample = now;
      }
    }
  }

  if (CLOCK32_GE(now, g_last_sample + SAMPLE_TIMEOUT_PERIOD_CYCLES)) {
    // Default momentary switch (tether release) to down (inactive) when
    // we lose signal from joystick.
    JoystickOutputMomentarySwitch(kJoystickSwitchPositionDown);
    JoystickOutputPresence(false);
  }

  // Clear status.
  ECAP(1).ECCLR.raw = status.raw;
}

void PpmEnableRawStatus(bool enable) {
  g_send_raw_status = enable;
}
