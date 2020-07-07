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

#include "control/hover/hover_inject.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>

#include "common/c_math/vec3.h"
#include "common/c_math/waveform.h"
#include "control/control_params.h"
#include "control/control_types.h"
#include "control/hover/hover_frame.h"
#include "control/hover/hover_types.h"

void HoverInjectInit(const HoverInjectParams *params) {
  assert(params != NULL);
  assert(HoverInjectValidateParams(params));
}

bool HoverInjectValidateParams(const HoverInjectParams *params) {
  if (!(params->position_start_time.x <= params->position_stop_time.x &&
        params->position_start_time.y <= params->position_stop_time.y &&
        params->position_start_time.z <= params->position_stop_time.z)) {
    assert(!(
        bool)"position_start_time must be less than or equal to"
             " position_stop_time.");
    return false;
  }

  if (!(params->angles_start_time.x <= params->angles_stop_time.x &&
        params->angles_start_time.y <= params->angles_stop_time.y &&
        params->angles_start_time.z <= params->angles_stop_time.z)) {
    assert(!(
        bool)"angles_start_time must be less than or equal to"
             " angles_stop_time.");
    return false;
  }

  if (!(params->blown_flaps_start_time <= params->blown_flaps_stop_time)) {
    assert(!(
        bool)"blown_flaps_start_time must be less than or equal to"
             " blown_flaps_stop_time.");
    return false;
  }

  if (!(params->drag_flaps_start_time <= params->drag_flaps_stop_time)) {
    assert(!(
        bool)"blown_flaps_start_time must be less than or equal to"
             " blown_flaps_stop_time.");
    return false;
  }

  if (!(params->elevator_start_time <= params->elevator_stop_time)) {
    assert(!(
        bool)"elevator_start_time must be less than or equal to"
             " elevator_stop_time.");
    return false;
  }

  if (!(params->rudder_start_time <= params->rudder_stop_time)) {
    assert(!(
        bool)"rudder_start_time must be less than or equal to"
             " rudder_stop_time.");
    return false;
  }

  return true;
}

bool HoverInjectIsEnabled(const FlightStatus *flight_status,
                          const HoverInjectParams *params) {
  return ((*g_cont.flight_plan == kFlightPlanHoverInPlace &&
           flight_status->flight_mode == kFlightModeHoverFullLength) &&
          params->use_signal_injection);
}

// Returns the time since we entered a flight mode where the signal
// injection is intended to take place.
static double GetInjectTime(const FlightStatus *flight_status,
                            const HoverInjectParams *params) {
  return HoverInjectIsEnabled(flight_status, params)
             ? flight_status->flight_mode_time
             : 0.0;
}

// Returns an independent rectangle function for each field of a Vec3.
static void CalcVec3StepSignal(double t, const Vec3 *amplitude,
                               const Vec3 *start_time, const Vec3 *stop_time,
                               Vec3 *signal) {
  assert(amplitude != NULL && start_time != NULL && stop_time != NULL &&
         signal != NULL);
  assert(start_time->x <= stop_time->x && start_time->y <= stop_time->y &&
         start_time->z <= stop_time->z);

  signal->x = (start_time->x < t && t < stop_time->x) ? amplitude->x : 0.0;
  signal->y = (start_time->y < t && t < stop_time->y) ? amplitude->y : 0.0;
  signal->z = (start_time->z < t && t < stop_time->z) ? amplitude->z : 0.0;
}

void HoverInjectPositionSignal(const FlightStatus *flight_status,
                               const Vec3 *wing_pos_g_cmd,
                               const Vec3 *hover_origin_g,
                               const HoverInjectParams *params,
                               Vec3 *wing_pos_g_cmd_out) {
  double t = GetInjectTime(flight_status, params);

  Vec3 wing_pos_h_step;
  CalcVec3StepSignal(t, &params->position_amplitude,
                     &params->position_start_time, &params->position_stop_time,
                     &wing_pos_h_step);

  Vec3 wing_pos_g_step;
  RotateHToG(&wing_pos_h_step, wing_pos_g_cmd, hover_origin_g,
             &wing_pos_g_step);
  Vec3Add(wing_pos_g_cmd, &wing_pos_g_step, wing_pos_g_cmd_out);
}

void HoverInjectAnglesSignal(const FlightStatus *flight_status,
                             const Vec3 *angles_cmd,
                             const HoverInjectParams *params,
                             Vec3 *angles_cmd_out) {
  double t = GetInjectTime(flight_status, params);

  Vec3 angles_step;
  CalcVec3StepSignal(t, &params->angles_amplitude, &params->angles_start_time,
                     &params->angles_stop_time, &angles_step);
  Vec3Add(angles_cmd, &angles_step, angles_cmd_out);
}

void HoverInjectOutputSignal(const FlightStatus *flight_status,
                             const ControlOutput *control_output,
                             const HoverInjectParams *params,
                             ControlOutput *control_output_out) {
  *control_output_out = *control_output;

  double t = GetInjectTime(flight_status, params);

  if (params->blown_flaps_start_time < t && t < params->blown_flaps_stop_time) {
    double t_signal = t - params->blown_flaps_start_time;
    double delta = params->blown_flaps_amplitude *
                   TriangleWave(t_signal, params->blown_flaps_period);

    // Port inner aileron.
    control_output_out->flaps[kFlapA4] -= delta;

    // Starboard inner aileron.
    control_output_out->flaps[kFlapA5] += delta;
  }

  if (params->drag_flaps_start_time < t && t < params->drag_flaps_stop_time) {
    double t_signal = t - params->drag_flaps_start_time;

    // Here we use only the sign of the triangle wave.
    if (TriangleWave(t_signal, params->drag_flaps_period) < 0.0) {
      // Generate negative roll moment.
      control_output_out->flaps[kFlapA1] = params->drag_flaps_low_drag_pos;
      control_output_out->flaps[kFlapA8] = params->drag_flaps_high_drag_pos;
    } else {
      // Generate positive roll moment.
      control_output_out->flaps[kFlapA1] = params->drag_flaps_high_drag_pos;
      control_output_out->flaps[kFlapA8] = params->drag_flaps_low_drag_pos;
    }
  }

  if (params->elevator_start_time < t && t < params->elevator_stop_time) {
    double t_signal = t - params->elevator_start_time;
    // Multiply the period by two so only the positive portion of the
    // triangle wave is used.
    double period =
        2.0 * (params->elevator_stop_time - params->elevator_start_time);
    control_output_out->flaps[kFlapEle] +=
        params->elevator_amplitude * TriangleWave(t_signal, period);
  }

  if (params->rudder_start_time < t && t < params->rudder_stop_time) {
    double t_signal = t - params->rudder_start_time;
    double period = params->rudder_stop_time - params->rudder_start_time;
    control_output_out->flaps[kFlapRud] +=
        params->rudder_amplitude * TriangleWave(t_signal, period);
  }
}
