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

#include "control/crosswind/crosswind_mode.h"

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/crosswind/crosswind_util.h"

// Checks velocity, tension, and altitude limits for entering
// crosswind.
static bool IsReadyForCrosswindNormal(CrosswindPathType path_type,
                                      const FlightStatus *flight_status,
                                      const StateEstimate *state_est,
                                      const CrosswindModeParams *params) {
  int32_t normal_gates = 0;

  if (flight_status->flight_mode == kFlightModeTransIn) {
    if (Vec3Norm(&state_est->Vg) < params->min_wing_speed) {
      normal_gates |= (1 << kCrosswindNormalGateSpeed);
    }

    // TODO: This code currently ignores tether_force_b.valid.
    if (state_est->tether_force_b.tension_f < params->min_tension) {
      normal_gates |= (1 << kCrosswindNormalGateTension);
    }

    if (state_est->Xg.z > params->max_wing_pos_g_z) {
      normal_gates |= (1 << kCrosswindNormalGateAltitude);
    }
  } else if (flight_status->flight_mode == kFlightModeCrosswindPrepTransOut) {
    // The prep trans-out flight mode behaves almost identically to the
    // crosswind normal flight before the path type changes. Until this happens,
    // the controller is completely free to change back to crosswind normal.

    if (path_type == kCrosswindPathPrepareTransitionOut) {
      // During the final quarter loop, the controller can still potentially
      // abort the trans-out if it has enough airspeed.
      if (state_est->apparent_wind.sph_f.v <
          params->min_airspeed_return_to_crosswind) {
        normal_gates |= (1 << kCrosswindNormalGateAirspeed);
      }
    }
  } else {
    normal_gates |= (1 << kCrosswindNormalGateFlightMode);
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeCrosswindNormal] = normal_gates;

  return normal_gates == 0;
}

// There are currently no gates in place for CrosswindPrepTransOut,
// but the function is here to add gates if needed.
static bool IsReadyForCrosswindPrepTransOut(void) {
  int32_t prep_trans_out_gates = 0;

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeCrosswindPrepTransOut] =
      prep_trans_out_gates;

  return prep_trans_out_gates == 0;
}

static bool IsReadyForHoverTransOut(double time_in_flare,
                                    CrosswindPathType path_type,
                                    const FlightStatus *flight_status,
                                    const StateEstimate *state_est,
                                    const CrosswindModeParams *params) {
  int32_t hover_trans_out_gates = 0;

  if (flight_status->flight_mode == kFlightModeHoverAccel) {
    if (flight_status->flight_mode_time < params->min_time_in_accel ||
        (flight_status->flight_mode_time < params->max_time_in_accel &&
         state_est->acc_norm_f > params->acc_slow_down_threshold)) {
      hover_trans_out_gates |=
          (1 << kCrosswindHoverTransOutGateStillAccelerating);
    }
    // For DisengageEngage, we proceed immediately from HoverAccel to
    // HoverTransOut.
    if (*g_cont.flight_plan == kFlightPlanDisengageEngage) {
      hover_trans_out_gates = 0;
    }
  } else {
    // The controller must be in the final "prepare transition out" flight path.
    if (path_type != kCrosswindPathPrepareTransitionOut) {
      hover_trans_out_gates |= (1 << kCrosswindHoverTransOutGatePathType);
    }

    // If the vehicle somehow gets into a bad situation where the angle of
    // attack isn't increasing or airspeed isn't dropping despite the flare,
    // force the hand-off to the hover controller based on time.
    bool force_handoff = time_in_flare > params->transout_max_time_in_flare;

    // Wait for the airspeed to drop to the point where thrust can control the
    // vehicle's attitude.
    if (state_est->apparent_wind.sph_f.v > params->transout_airspeed &&
        !force_handoff) {
      hover_trans_out_gates |= (1 << kCrosswindHoverTransOutGateAirspeed);
    }

    if (state_est->apparent_wind.sph.alpha < params->transout_alpha &&
        !force_handoff) {
      hover_trans_out_gates |= (1 << kCrosswindHoverTransOutGateAlpha);
    }
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeHoverTransOut] = hover_trans_out_gates;

  return hover_trans_out_gates == 0;
}

bool CrosswindModeIsReadyFor(FlightMode proposed_flight_mode,
                             const FlightStatus *flight_status,
                             const StateEstimate *state_est,
                             double time_in_flare, CrosswindPathType path_type,
                             const CrosswindModeParams *params) {
  if (proposed_flight_mode == kFlightModeCrosswindNormal) {
    return IsReadyForCrosswindNormal(path_type, flight_status, state_est,
                                     params);
  } else if (proposed_flight_mode == kFlightModeCrosswindPrepTransOut) {
    return IsReadyForCrosswindPrepTransOut();
  } else if (proposed_flight_mode == kFlightModeHoverTransOut) {
    return IsReadyForHoverTransOut(time_in_flare, path_type, flight_status,
                                   state_est, params);
  }

  return false;
}
