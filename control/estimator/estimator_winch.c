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

#include "control/estimator/estimator_winch.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "control/control_params.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_params.h"
#include "control/system_types.h"

void EstimatorWinchInit(FlightPlan flight_plan, GroundStationModel gs_model,
                        double drum_radius, EstimatorWinchState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));

  if (gs_model == kGroundStationModelGSv2) {
    state->position_perched =
        -(g_sys.tether->length -
          GetSystemParams()->ground_station.gs02.racetrack_tether_length) -
        2.0 * drum_radius * PI;
  } else {
    // This is the predicted value for the winch position when perched
    // based on the tether length and the 180 degree drum rotation
    // during the perch transformation.  It may differ from the actual
    // value due to tether stretch and other factors.
    state->position_perched = -g_sys.tether->length - drum_radius * PI;
  }

  switch (gs_model) {
    case kGroundStationModelGSv1:
    case kGroundStationModelGSv2:
      state->last_valid_position = state->position_perched;
      break;

    case kGroundStationModelForceSigned:
    case kNumGroundStationModels:
    default:
      assert(false);
    case kGroundStationModelTopHat:
      // Set the last valid position to fully payed-out in the remote
      // perch flight plan, so we can disable the winch sensor.
      state->last_valid_position = 0.0;
      break;
  }

  if (flight_plan == kFlightPlanHighHover ||
      flight_plan == kFlightPlanLaunchPerch ||
      flight_plan == kFlightPlanTurnKey) {
    state->last_valid_proximity = true;
  } else {
    state->last_valid_proximity = false;
  }
}

void EstimatorWinchSetPayout(double payout, const WinchParams *winch_params,
                             EstimatorWinchState *state) {
  assert(0.0 <= payout);
  assert(winch_params != NULL && state != NULL);
  state->position_perched = state->last_valid_position - payout;
  if (!(*g_cont.control_opt & kControlOptHardCodeInitialPayout)) {
    // We multiply by 1.1 here to allow for some margin in variation
    // on the winch position while on the perch.
    assert(state->position_perched >=
           -g_sys.tether->length - 1.1 * winch_params->r_drum * PI);
  }
}

void EstimatorWinchStep(double winch_pos, const FaultMask *winch_fault,
                        bool proximity, const FaultMask *proximity_fault,
                        bool perched, GroundStationModel gs_model,
                        const WinchParams *winch_params,
                        EstimatorWinchState *state, WinchEstimate *winch) {
  assert(winch_fault != NULL && proximity_fault != NULL);
  assert(state != NULL && winch != NULL);

  switch (gs_model) {
    case kGroundStationModelGSv1:
    case kGroundStationModelGSv2:
      winch->valid = !HasAnyFault(winch_fault);
      // During a fault, hold the last valid value.
      if (winch->valid) state->last_valid_position = winch_pos;
      if (perched) EstimatorWinchSetPayout(0.0, winch_params, state);

      // Handle proximity flag.
      //
      // TODO: When the proximity flag is not valid, we should
      // make use of payout as a back-up sensor.  Also, if proximity
      // sensors and payout disagree we need a good fall-back option.
      winch->proximity_valid = !HasAnyFault(proximity_fault);
      if (winch->proximity_valid) {
        state->last_valid_proximity = proximity;
      }
      winch->proximity = proximity;

      break;

    case kGroundStationModelForceSigned:
    case kNumGroundStationModels:
    default:
      assert(false);
    case kGroundStationModelTopHat:
      winch->valid = true;
      state->last_valid_position = 0.0;
      EstimatorWinchSetPayout(g_sys.tether->length, winch_params, state);

      winch->proximity_valid = false;
      winch->proximity = false;
      break;
  }

  winch->position = state->last_valid_position;

  double max_payout = g_sys.tether->length;
  if (gs_model == kGroundStationModelGSv2) {
    // Calculate how far the payout is when the payout is on the GSG (drum
    // rotates to racetrack_high).
    // Note the last half turn involves unwrapping the tether on the racetrack
    // and the racetrack has a varying wrap radius. To simplify the calculation,
    // we model the racetrack as if it keeps the same wrap radius as the drum.
    max_payout =
        GetSystemParams()->ground_station.gs02.drum_angles.racetrack_high *
            winch_params->r_drum -
        state->position_perched;
  }
  winch->payout = fmin(max_payout, winch->position - state->position_perched);
}
