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

#include "control/trans_in/trans_in_mode.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/system_params.h"
#include "control/trans_in/trans_in_types.h"

// TODO: Add parameter validation.

static bool IsReadyForTransIn(const FlightStatus *flight_status,
                              const StateEstimate *state_est,
                              const TransInModeParams *params) {
  int32_t trans_in_gates = 0;

  // Allow trans-in in HighHover to support a forced trans in.  HoverAccel is
  // still gated on flight plan so we won't get here unless it was forced.
  if (!(*g_cont.flight_plan == kFlightPlanStartDownwind ||
        *g_cont.flight_plan == kFlightPlanTurnKey ||
        *g_cont.flight_plan == kFlightPlanHighHover)) {
    trans_in_gates |= (1 << kTransInGateFlightPlan);
  }

  double dynamic_pressure =
      fmax(0.5 * g_sys.phys->rho * state_est->apparent_wind.sph_f.v *
               state_est->apparent_wind.sph_f.v,
           0.5 * g_sys.phys->rho * Vec3NormSquared(&state_est->Vg_f));
  if (dynamic_pressure < params->min_dynamic_pressure) {
    trans_in_gates |= (1 << kTransInGateMinDynamicPressure);
  }

  double accel_timer = (flight_status->flight_mode == kFlightModeHoverAccel)
                           ? flight_status->flight_mode_time
                           : 0.0;
  if (accel_timer < params->min_time_in_accel ||
      (accel_timer < params->max_time_keep_accelerating &&
       state_est->acc_norm_f > params->acc_stopped_accelerating_threshold)) {
    trans_in_gates |= (1 << kTransInGateStillAccelerating);
  }

  // The transition to trans-in flight is forced if the kite is
  // pitched too far forward.
  //
  // The kite is considered to be pitched forward if the body x-axis is
  // pointed at the ground station.  We test for this here by examining
  // [1; 0; 0]' * dcm_g2b * [X_g.x; X_g.y; 0].
  bool force_transition_pitched_forward =
      state_est->dcm_g2b.d[0][0] * state_est->Xg.x +
          state_est->dcm_g2b.d[0][1] * state_est->Xg.y <
      sin(params->min_pitch_angle) * hypot(state_est->Xg.x, state_est->Xg.y);

  // Do not force the transition to trans-in if this mode is inhibited
  // by flight plan.
  if (trans_in_gates & (1 << kTransInGateFlightPlan)) {
    force_transition_pitched_forward = false;
  }

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->flight_mode_gates[kFlightModeTransIn] = trans_in_gates;

  return force_transition_pitched_forward || trans_in_gates == 0;
}

bool TransInModeIsReadyFor(FlightMode proposed_flight_mode,
                           const FlightStatus *flight_status,
                           const StateEstimate *state_est,
                           const TransInModeParams *params) {
  if (proposed_flight_mode == kFlightModeTransIn) {
    return IsReadyForTransIn(flight_status, state_est, params);
  }

  return false;
}
