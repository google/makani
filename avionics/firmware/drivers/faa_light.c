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

#include "avionics/firmware/drivers/faa_light.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/faa_light_types.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/io.h"
#include "avionics/firmware/gps/gps_interface.h"
#include "avionics/firmware/identity/identity.h"
#include "avionics/firmware/network/net_send.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"

#define MIN_FLASH_RATE        1.0f  // Flashes per minute.
#define MAX_PULSE_WIDTH_US    1e6f
#define MIN_BIT_PERIOD_US     1e3f

#define DEFAULT_FLASH_RATE    40.0f  // Flashes per minute.
#define DEFAULT_PULSE_WIDTH_US   1.6e5f
#define DEFAULT_DUTY_CYCLE    0.5f

#define OFF_PATTERN           0x0000
#define PORT_PATTERN          0xAAAA
#define STARBOARD_PATTERN     0xAA55
#define TAIL_BOTTOM_PATTERN   0xB44B
#define TAIL_TOP_PATTERN      0xB4B4
// The ground station GPS node does not transmit a code but rather its output is
// connected to the sync line of the tower light.  We set the code to a single
// bit so that a single pulse is generated per flash regardless of PWM setting.
#define GROUND_PATTERN        0x8000
#define NUM_BITS      16

// These are listed in order of preference.  We will select the earliest valid
// one.
const AioNode g_time_sources[] = {
  kAioNodeFcA,
  kAioNodeFcB,
  kAioNodeLightPort,
  kAioNodeLightStbd,
  kAioNodeGpsBaseStation,
};

static struct {
  LightInputParams input_params[kNumLightTypes];
  LightState light_state[kNumLightTypes];
  LightTiming timing;
  int32_t time_source_index;
  int32_t best_time_source_index;
  AioNode current_node;
} g_state;

// Compute latest pulse width modulation and flashes-per-minute variables.
static void LightInputParamsUpdated(LightType light_type) {
  // Create helper pointers.
  float *flashes_per_minute =
      &g_state.input_params[light_type].flashes_per_minute;
  float *pulse_width_us =
      &g_state.input_params[light_type].flash_pulse_width_us;
  float *duty_cycle = &g_state.input_params[light_type].pwm_duty_cycle;

  // Enforce valid values for input params.
  *duty_cycle = Saturatef(*duty_cycle, 0.0f, 1.0f);
  *pulse_width_us =
      Saturatef(fabsf(*pulse_width_us), NUM_BITS * MIN_BIT_PERIOD_US,
                MAX_PULSE_WIDTH_US);
  *flashes_per_minute =
      Saturatef(fabsf(*flashes_per_minute), MIN_FLASH_RATE,
                1e6f * 60.0 / *pulse_width_us);

  // Calculate output params.
  g_state.light_state[light_type].flashes_per_minute_period_us =
      1e6 * 60 / *flashes_per_minute;

  g_state.light_state[light_type].pwm_period_us =
      *pulse_width_us / NUM_BITS;

  g_state.light_state[light_type].pwm_on_time_us =
      *duty_cycle * g_state.light_state[light_type].pwm_period_us;
}

void FaaLightInit(void) {
  // Get current node.
  g_state.current_node = AppConfigGetAioNode();

  // Visible light pins.
  IoConfigureAsInput(kIoDcan2PinRx);
  IoConfigureAsOutputPushPull(kIoDcan2PinTx, true);
  IoSetValue(kIoDcan2PinTx, 0);

  // Infrared light pins.
  IoConfigureAsInput(kIoDcan3PinRx);
  IoConfigureAsOutputPushPull(kIoDcan3PinTx, true);
  IoSetValue(kIoDcan3PinTx, 0);

  // Configure pattern to send based on node name.
  switch (g_state.current_node) {
    case kAioNodeLightPort:
      g_state.light_state[kLightTypeVisible].bit_pattern = PORT_PATTERN;
      break;
    case kAioNodeLightStbd:
      g_state.light_state[kLightTypeVisible].bit_pattern = STARBOARD_PATTERN;
      break;
    case kAioNodeLightTailBottom:
      g_state.light_state[kLightTypeVisible].bit_pattern = TAIL_BOTTOM_PATTERN;
      break;
    case kAioNodeLightTailTop:
      g_state.light_state[kLightTypeVisible].bit_pattern = TAIL_TOP_PATTERN;
      break;
    case kAioNodeGpsBaseStation:
      g_state.light_state[kLightTypeVisible].bit_pattern = GROUND_PATTERN;
      break;
    default:
      assert(false);
      break;
  }
  // TODO: Support kLightTypeInfrared when pulse width is defined.
  // Remember that it would be beneficial to flash the infrared a half cycle
  // out of phase with the visible due to MVLV power considerations.
  g_state.light_state[kLightTypeInfrared].bit_pattern = OFF_PATTERN;

  for (LightType light_type = 0; light_type < kNumLightTypes; ++light_type) {
    g_state.input_params[light_type].flashes_per_minute = DEFAULT_FLASH_RATE;
    g_state.input_params[light_type].flash_pulse_width_us =
        DEFAULT_PULSE_WIDTH_US;
    g_state.input_params[light_type].pwm_duty_cycle = DEFAULT_DUTY_CYCLE;
    LightInputParamsUpdated(light_type);
  }

  g_state.timing.time_us = 0;
  g_state.timing.gps_time_of_week_us = 0;
  g_state.timing.gps_update_timestamp_us = 0;
  g_state.timing.source = -1;
  g_state.timing.source_valid = false;

  g_state.time_source_index = 0;
  g_state.best_time_source_index = ARRAYSIZE(g_time_sources) - 1;
}

// Pointers to parameters that we can modify through Get/SetParams
// commands.
static float *g_mutable_param_addrs[] = {
  // Visible light parameters.
  &g_state.input_params[kLightTypeVisible].flashes_per_minute,
  &g_state.input_params[kLightTypeVisible].flash_pulse_width_us,
  &g_state.input_params[kLightTypeVisible].pwm_duty_cycle,
  // Infrared light parameters.
  &g_state.input_params[kLightTypeInfrared].flashes_per_minute,
  &g_state.input_params[kLightTypeInfrared].flash_pulse_width_us,
  &g_state.input_params[kLightTypeInfrared].pwm_duty_cycle,
};

static void LightPinCommand(LightType light_type, bool light_command) {
  // Set light command.
  switch (light_type) {
    case kLightTypeVisible:
      IoSetValue(kIoDcan2PinTx, light_command);
      break;
    case kLightTypeInfrared:
      IoSetValue(kIoDcan3PinTx, light_command);
      break;
    default:
      assert(false);
      break;
  }
}

// A simple Pulse Width Modulation (PWM) scheme. This function creates a pwm
// signal given a waveform period and an on time in clock  cycle counts. A
// clock cycle counter is also passed in to keep track of when to toggle the
// signal based on the on time cycles and when a waveform period has expired.
static void LightPwm(LightType light_type,
                     int64_t time_wrapped_flash_period_us) {
  // Wrap time to PWM periods.
  int32_t time_wrapped_pwm_period_us =
      time_wrapped_flash_period_us %
      g_state.light_state[light_type].pwm_period_us;

  int8_t bit_index =
      time_wrapped_flash_period_us /
      g_state.light_state[light_type].pwm_period_us;

  bool bit_value =
      (bool)(g_state.light_state[light_type].bit_pattern >>
             (NUM_BITS - 1 - bit_index) & 1);

  // Update PWM.
  if (time_wrapped_pwm_period_us
      < g_state.light_state[light_type].pwm_on_time_us) {
    LightPinCommand(light_type, bit_value);
  } else {
    LightPinCommand(light_type, 0);
  }
}

static void LightFlashPattern(LightType light_type) {
  int32_t time_wrapped_flash_period_us
      = (g_state.timing.time_us
         % g_state.light_state[light_type].flashes_per_minute_period_us);

  if (0 <= time_wrapped_flash_period_us
      && time_wrapped_flash_period_us
      < g_state.input_params[light_type].flash_pulse_width_us) {
    LightPwm(light_type, time_wrapped_flash_period_us);
  } else {
    LightPinCommand(light_type, 0);
  }
}

// Checks if we have received a valid FaaLightGetParamMessage.  If we
// have, this sends a FaaLightAckParamMessage with the parameter data.
static void HandleGetParam(AioNode source) {
  FaaLightGetParamMessage message;
  if (CvtGetFaaLightGetParamMessage(source, &message, NULL, NULL)
      && g_state.current_node == message.target) {
    FaaLightAckParamMessage ack_param = {
      message.id, (float)*g_mutable_param_addrs[message.id]};
    NetSendAioFaaLightAckParamMessage(&ack_param);
  }
}

// Checks if we have received a valid FaaLightSetParamMessage.  If we
// have, this sets the parameter and sends a FaaLightAckParamMessage with
// the parameter data.
static void HandleSetParam(AioNode source) {
  FaaLightSetParamMessage message;
  if (CvtGetFaaLightSetParamMessage(source, &message, NULL, NULL)
      && g_state.current_node == message.target) {
    *g_mutable_param_addrs[message.id] = message.value;
    FaaLightAckParamMessage ack_param = {message.id, message.value};
    for (LightType i = 0; i < kNumLightTypes; ++i) {
      LightInputParamsUpdated(i);
    }
    NetSendAioFaaLightAckParamMessage(&ack_param);
  }
}

// We expect that the max latency of a GPS update is 2 seconds (1 second GpsTime
// update interval plus < 1 second PPS latency in that message).  2.5 seconds
// gives us some margin to account for message delivery.
#define GPS_TIME_MAX_LATENCY_US (2.5 * 1000 * 1000)

// TODO: All of this logic could be abstracted into a generic real time
// clock / network time library.
static bool LightUpdateTime(AioNode source) {
  GpsTimeMessage in;

  bool has_update = false;
  bool gps_time_valid = false;

  int32_t latency_us;
  int32_t time_of_week_ms;
  int64_t timestamp_us;
  int64_t now = ClockGetUs();
  // Try to acquire a GPS time of week value.
  if (source == g_state.current_node) {
    GpsGetTimeOfWeek(now, &latency_us, &time_of_week_ms);
    timestamp_us = now;
    has_update = true;
  } else if (CvtGetGpsTimeMessage(source, &in, NULL, &timestamp_us)) {
    latency_us = in.latency;
    time_of_week_ms = in.time_of_week;
    has_update = true;
  }

  // Was there an updated time available from local or remote GPS?
  if (has_update) {
    // Check that the total GPS latency is within an acceptable range.
    if (0 <= latency_us && latency_us < GPS_TIME_MAX_LATENCY_US) {
      g_state.timing.gps_time_of_week_us =
          (int64_t)time_of_week_ms * 1000;
      g_state.timing.gps_update_timestamp_us = timestamp_us - latency_us;
      g_state.timing.source = source;
      gps_time_valid = true;
    }
  } else {
    // We are free-running but within our latency bound, and polling our
    // current GPS source.
    if (SaturateLatency(now - g_state.timing.gps_update_timestamp_us) <
        GPS_TIME_MAX_LATENCY_US && source == g_state.timing.source) {
      gps_time_valid = true;
    }
  }

  // Add latency to compute current time of week.  This will free-run by default
  // if no source is available.
  g_state.timing.time_us
      = (now - g_state.timing.gps_update_timestamp_us
         + g_state.timing.gps_time_of_week_us);

  return gps_time_valid;
}

void FaaLightSendStatusMessage(void) {
  static FaaLightStatusMessage light_status;

  light_status.light_timing = g_state.timing;

  light_status.input_params[kLightTypeVisible]
      = g_state.input_params[kLightTypeVisible];
  light_status.input_params[kLightTypeInfrared]
      = g_state.input_params[kLightTypeInfrared];

  NetSendAioFaaLightStatusMessage(&light_status);
}

void FaaLightPoll(void) {
  HandleSetParam(kAioNodeOperator);
  HandleGetParam(kAioNodeOperator);

  g_state.time_source_index++;
  if (g_state.time_source_index > g_state.best_time_source_index) {
    g_state.time_source_index = 0;
  }
  bool gps_update_valid =
      LightUpdateTime(g_time_sources[g_state.time_source_index]);
  if (gps_update_valid) {
    // This source is valid, and it must be same or better priority.
    g_state.timing.source_valid = true;
    g_state.best_time_source_index = g_state.time_source_index;
  } else {
    if (g_state.best_time_source_index == g_state.time_source_index) {
      // Our best time source is not valid, fall back if we're not on the last
      // one.
      g_state.timing.source_valid = false;
      g_state.best_time_source_index = ARRAYSIZE(g_time_sources) - 1;
    }
  }

  LightFlashPattern(kLightTypeVisible);
  LightFlashPattern(kLightTypeInfrared);
}
