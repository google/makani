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

#include "gs/monitor2/high_frequency_filters/filter_handlers.h"
#include <math.h>

#include "avionics/common/aio_header.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message.h"
#include "avionics/linux/clock.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/crosswind/crosswind_types.h"
#include "control/cvt_control_telemetry.h"
#include "control/pack_control_telemetry.h"
#include "control/system_params.h"

void InitFilterFrequencyState(FilterState *filter_state,
                              FilteredData *filtered_data) {
  (void)filtered_data;
  assert(FILTER_FREQUENCY_WINDOW_SIZE > 0);
  FilterFrequencyState *state = &filter_state->filter_frequency;
  state->prev_time_us = 0LL;
  InitCircularAveragingBuffer(state->intervals, FILTER_FREQUENCY_WINDOW_SIZE,
                              &state->buffer);
}

// A moving average filter to keep track of the frequency we are filtering at.
void FilterFrequencyHandler(FilterState *filter_state,
                            FilteredData *filtered_data) {
  FilterFrequencyState *state = &filter_state->filter_frequency;

  if (!state->prev_time_us) {
    state->prev_time_us = ClockGetUs();
    return;
  }

  // Get latest frequency measurement.
  int64_t current_time_us = ClockGetUs();
  double current_window =
      (double)(current_time_us - state->prev_time_us) / (double)1e6;
  double avg_interval =
      UpdateCircularAveragingBuffer(current_window, &state->buffer);
  if (state->buffer.next_idx == 0) {
    filtered_data->filter_frequency.valid = true;
  }
  state->prev_time_us = current_time_us;
  filtered_data->filter_frequency.frequency = 1.0 / avg_interval;
}

void InitLoopCountState(FilterState *filter_state,
                        FilteredData *filtered_data) {
  LoopCountFilteredData *data = &filtered_data->loop_count;
  data->current_rev_count = data->total_rev_count = 0;

  LoopCountState *state = &filter_state->loop_count;
  state->last_angle = 0.0;
  state->initialized_in_crosswind = false;
  state->first_loop_section_complete = false;

  // Only consider the clockwise case.
  assert(GetControlParams()->crosswind.loop_dir == kLoopDirectionCw);
}

static void HandleLoopCountInCrosswind(double loop_angle, LoopCountState *state,
                                       LoopCountFilteredData *data) {
  double wrapped_angle = Wrap(loop_angle, -2.0 * PI, 0.0);
  if (!state->initialized_in_crosswind) {
    state->last_angle = wrapped_angle;
    state->initialized_in_crosswind = true;
    return;
  }

  // If wrapped angle is within 30 deg of -PI, regard the loop as started.
  if (fabs(wrapped_angle + PI) < PI / 6.0) {
    state->first_loop_section_complete = true;
  }

  double difference = wrapped_angle - state->last_angle;
  // If we assume the kite is moving smoothly (no jittering), then
  // wrapped_angle will be monotonically decreasing from 0 to -2 * pi
  // and then wrap around.
  //
  // If the angle hasn't wrapped around, then the difference falls in
  // (-delta, 0]. If it has, then the difference falls in
  // (2 * pi - delta, 2 * pi).
  //
  // When there is jittering, it is possible that the difference goes
  // backwards to (0, delta), or, if wrapped around, to
  // (-2 * pi, -2 * pi + delta). We should ignore such jittering by not
  // updating the last loop angle.
  static const double kDelta = PI;

  if (difference > kDelta) {
    // Do not increment loop count if this is near the starting point.
    if (state->first_loop_section_complete) {
      data->current_rev_count += 1;
      data->total_rev_count += 1;
    }
    state->last_angle = wrapped_angle;
  } else if (-kDelta < difference && difference < 0.0) {
    state->last_angle = wrapped_angle;
  }
}

void LoopCountHandler(FilterState *filter_state, FilteredData *filtered_data) {
  LoopCountState *state = &filter_state->loop_count;
  LoopCountFilteredData *data = &filtered_data->loop_count;
  const TetherDownMessage *tether_down =
      &filtered_data->merge_tether_down.tether_down;
  double loop_angle = (double)tether_down->control_telemetry.loop_angle;
  uint8_t flight_mode = tether_down->control_telemetry.flight_mode;
  if (!AnyCrosswindFlightMode(flight_mode)) {
    if (flight_mode == kFlightModeTransIn) {
      state->initialized_in_crosswind = false;
      state->first_loop_section_complete = false;
      data->current_rev_count = 0;
    }
  } else {
    HandleLoopCountInCrosswind(loop_angle, state, data);
  }
}

void InitMergeTetherDownState(FilterState *filter_state,
                              FilteredData *filtered_data) {
  MergeTetherDownFilteredData *data = &filtered_data->merge_tether_down;
  MergeTetherDownState *state = &filter_state->merge_tether_down;
  TetherDownMergeStateInit(&state->merge_state);

  for (uint8_t i = 0; i < kNumAioNodes; ++i) {
    state->latest_node_status_timestamp_us[i] = 0L;
    data->node_status_valid[i] = false;
  }
  data->valid = false;
  data->max_loop_time = 0.0;
}

void MergeTetherDownHandler(FilterState *filter_state,
                            FilteredData *filtered_data) {
  // TODO: Do not access TetherDownMergeState directly!
  TetherDownMergeState *state = &filter_state->merge_tether_down.merge_state;
  int64_t *latest_node_status_timestamp_us =
      filter_state->merge_tether_down.latest_node_status_timestamp_us;
  const TetherDownMessage *merged = TetherDownMergeCvtPeek(state);

  int64_t now_timestamp_us = ClockGetUs();
  int64_t latest_timestamp_us = state->output_timestamp;

  filtered_data->merge_tether_down.valid =
      (now_timestamp_us - latest_timestamp_us) < AIO_EXPIRATION_TIME_US;

  if (filtered_data->merge_tether_down.valid) {
    filtered_data->merge_tether_down.timestamp_sec =
        (double)latest_timestamp_us * 1.0e-6;
  }

  for (int32_t i = 0; i < kNumTetherDownSources; ++i) {
    if (state->input_updated[i]) {
      TetherDownMessage *message = &state->input_messages[i];
      filtered_data->merge_tether_down.comms_status[i] = message->comms_status;

      AioNode aio_node = message->node_status.node;

      if (message->node_status.no_update_count <
          MAX_NODE_STATUS_NO_UPDATE_COUNT) {
        filtered_data->merge_tether_down.node_status_valid[aio_node] = true;
        filtered_data->merge_tether_down.node_status[aio_node] =
            message->node_status;
        latest_node_status_timestamp_us[aio_node] = now_timestamp_us;
      }
    }

    filtered_data->merge_tether_down.comms_status_valid[i] =
        now_timestamp_us - state->input_timestamps[i] < AIO_EXPIRATION_TIME_US;
  }

  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    filtered_data->merge_tether_down.node_status_valid[i] &=
        (now_timestamp_us - latest_node_status_timestamp_us[i] <
         NODE_STATUS_EXPIRATION_TIME_US);
  }

  filtered_data->merge_tether_down.tether_down = *merged;

  filtered_data->merge_tether_down.max_loop_time =
      fmax(filtered_data->merge_tether_down.max_loop_time,
           merged->control_telemetry.loop_time);
}

void InitWindGustState(FilterState *filter_state, FilteredData *filtered_data) {
  assert(WIND_GUST_WINDOW_SIZE > 0);
  assert(WIND_GUST_INTERVAL_WINDOW_SIZE > 0);
  WindGustState *state = &filter_state->wind_gust;
  filtered_data->wind_gust.valid = false;
  state->desired_period = 1.0 / 20.0;
  state->timeout = 0.5;
  state->prev_sample_time_us = 0LL;
  state->prev_data_time_us = 0LL;
  InitCircularAveragingBuffer(state->intervals, WIND_GUST_INTERVAL_WINDOW_SIZE,
                              &state->interval_buffer);
  InitCircularAveragingBuffer(state->u_samples, WIND_GUST_WINDOW_SIZE,
                              &state->u_buffer);
  InitCircularAveragingBuffer(state->v_samples, WIND_GUST_WINDOW_SIZE,
                              &state->v_buffer);
  InitCircularAveragingBuffer(state->speed_samples, WIND_GUST_WINDOW_SIZE,
                              &state->speed_buffer);
  InitCircularAveragingBuffer(state->u_squared, WIND_GUST_WINDOW_SIZE,
                              &state->u_squared_buffer);
  InitCircularAveragingBuffer(state->v_squared, WIND_GUST_WINDOW_SIZE,
                              &state->v_squared_buffer);
  InitCircularAveragingBuffer(state->speed_squared, WIND_GUST_WINDOW_SIZE,
                              &state->speed_squared_buffer);
  state->sequence = 0U;
}

// A fixed frequency filter that samples values at 20Hz, buffers the last
// 5 minutes of samples, then computes and returns the tke.
void WindGustHandler(FilterState *filter_state, FilteredData *filtered_data) {
  WindGustState *state = &filter_state->wind_gust;

  // Only proceed if this is the correct time to sample the signal in order to
  // maintain the desired frequency.
  int64_t now = ClockGetUs();
  double interval = (double)(now - state->prev_sample_time_us) / 1e6;
  if (interval < state->desired_period) {
    return;
  }
  if (state->prev_sample_time_us != 0) {
    double avg_interval =
        UpdateCircularAveragingBuffer(interval, &state->interval_buffer);
    filtered_data->wind_gust.frequency = 1.0 / avg_interval;
  }
  state->prev_sample_time_us = now;

  // Selection of platform sensor is arbitrary, so we choose PlatformSensorA.
  uint16_t new_sequence;
  const uint8_t *buf =
      CvtPeek(kAioNodePlatformSensorsA, kMessageTypeGroundStationWeather,
              &new_sequence, NULL);

  // Break out of the function if the message is NULL or stale.
  double quiet_interval = (double)(now - state->prev_data_time_us) / 1e6;
  if (buf == NULL || (state->sequence == new_sequence)) {
    if (quiet_interval > state->timeout && filtered_data->wind_gust.valid) {
      InitWindGustState(filter_state, filtered_data);
    }
    return;
  }

  state->prev_data_time_us = now;

  // Report data even if we don't have a full history.
  filtered_data->wind_gust.valid = true;
  state->sequence = new_sequence;

  // Unpack message, calculate loop time, and update filtered data.
  GroundStationWeatherMessage msg;
  UnpackGroundStationWeatherMessage(buf, 1, &msg);
  double vel_u = msg.wind.wind_velocity[0];
  double vel_v = msg.wind.wind_velocity[1];
  double speed = hypot(vel_u, vel_v);

  double u_avg = UpdateCircularAveragingBuffer(vel_u, &state->u_buffer);
  double v_avg = UpdateCircularAveragingBuffer(vel_v, &state->v_buffer);
  double speed_avg = UpdateCircularAveragingBuffer(speed, &state->speed_buffer);

  double u_squared_avg =
      UpdateCircularAveragingBuffer(vel_u * vel_u, &state->u_squared_buffer);
  double v_squared_avg =
      UpdateCircularAveragingBuffer(vel_v * vel_v, &state->v_squared_buffer);
  double speed_squared_avg = UpdateCircularAveragingBuffer(
      speed * speed, &state->speed_squared_buffer);

  // Calculate and set the tke, remember variance is E[X^2] - (E[X])^2.
  double u_var = u_squared_avg - u_avg * u_avg;
  double v_var = v_squared_avg - v_avg * v_avg;
  double tke = .5 * (u_var + v_var);
  filtered_data->wind_gust.tke = tke;
  double speed_var = speed_squared_avg - speed_avg * speed_avg;
  if (speed_avg > MIN_WIND_SPEED_FOR_EFFECTIVE_GUST) {
    filtered_data->wind_gust.turbulence_intensity = Sqrt(speed_var) / speed_avg;
  } else {
    filtered_data->wind_gust.turbulence_intensity = 0.0;
  }
}
