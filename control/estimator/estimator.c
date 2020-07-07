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

#include "control/estimator/estimator.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/geometry.h"
#include "common/c_math/linalg_common.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_apparent_wind.h"
#include "control/estimator/estimator_encoders.h"
#include "control/estimator/estimator_experiment.h"
#include "control/estimator/estimator_ground_station.h"
#include "control/estimator/estimator_joystick.h"
#include "control/estimator/estimator_nav_kite.h"
#include "control/estimator/estimator_perch_azi.h"
#include "control/estimator/estimator_tether_anchor.h"
#include "control/estimator/estimator_tether_force.h"
#include "control/estimator/estimator_tether_ground_angles.h"
#include "control/estimator/estimator_types.h"
#include "control/estimator/estimator_weather.h"
#include "control/estimator/estimator_winch.h"
#include "control/estimator/estimator_wind.h"
#include "control/ground_frame.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

#define GS_UNPAUSE_TRANSFORM_CYCLES 10

void EstimatorInit(const SystemParams *system_params,
                   const EstimatorParams *params, EstimatorState *state) {
  assert(system_params != NULL && params != NULL && state != NULL);
  memset(state, 0, sizeof(*state));
  state->tether_release_latched = false;
  state->time = 0.0;
  EstimatorApparentWindInit(&state->apparent_wind);
  EstimatorEncodersInit(&state->encoders);
  EstimatorExperimentInit(&state->experiment);
  EstimatorGroundStationInit(&state->ground_station);
  EstimatorJoystickInit(&state->joystick);
  EstimatorNavKiteInit(&params->nav, system_params->ground_frame.heading,
                       &state->nav);
  EstimatorPerchAziInit(&state->perch_azi);
  EstimatorTetherAnchorPointInit(&state->tether_anchor);
  EstimatorTetherGroundAnglesInit(&state->tether_ground_angles);
  EstimatorTetherForceInit(&state->tether_force);
  EstimatorVesselInit(&state->vessel);
  EstimatorWeatherInit(&params->weather, &state->weather);
  EstimatorWinchInit(system_params->flight_plan, system_params->gs_model,
                     system_params->winch.r_drum, &state->winch);

  EstimatorWindInit(&state->wind);
  EstimatorWindInit(&state->wind_aloft);
  state->gs_unpause_transform_count = 0;
}

// Test that valid messages are simultaneously available for a critical
// list of sensors.  This must return true before the control system will
// advance out of the kInitializationStateWaitForValidData.
bool EstimatorIsDataReady(FlightPlan flight_plan, const FaultMask faults[]) {
  if (flight_plan == kFlightPlanManual) {
    return !HasAnyFault(&faults[kSubsysJoystick]);
  } else {
    if (HasAnyFault(&faults[kSubsysGsGpsPos])) return false;
    if (HasAnyFault(&faults[kSubsysJoystick])) return false;
    if (HasAnyFault(&faults[kSubsysWingGpsCrosswindPos])) return false;
    if (HasAnyFault(&faults[kSubsysWingGpsCrosswindVel])) return false;
    if (HasAnyFault(&faults[kSubsysWingGpsHoverPos])) return false;
    if (HasAnyFault(&faults[kSubsysWingGpsHoverVel])) return false;
    if (flight_plan == kFlightPlanHoverInPlace) {
      if (HasAnyFault(&faults[kSubsysGsCompass])) return false;
    }
  }

  return true;
}

void EstimatorStep(const FlightStatus *flight_status,
                   const ControlInput *control_input,
                   const SystemParams *system_params, const FaultMask faults[],
                   const EstimatorParams *params, EstimatorState *state,
                   StateEstimate *state_est) {
  assert(flight_status != NULL && control_input != NULL &&
         system_params != NULL && faults != NULL && params != NULL &&
         state != NULL && state_est != NULL);
  // Handle initialization.
  bool initializing = state->time < params->t_initialize;

  // Encoder angles.
  EncodersEstimate encoders;
  EstimatorEncodersStep(
      control_input->gsg, &faults[SUBSYS_GSG_A], &faults[SUBSYS_GSG_B],
      control_input->perch.levelwind_ele, &faults[kSubsysLevelwindEleA],
      &faults[kSubsysLevelwindEleB], control_input->perch.perch_azi,
      &faults[kSubsysPerchAziA], &faults[kSubsysPerchAziB], &state->encoders,
      &encoders);

  // Estimate air density.

  EstimatorWeatherStep(initializing, &control_input->weather,
                       &faults[kSubsysWeather], &params->weather,
                       &state->weather, &state_est->rho);

  // Sensor values required by the controller.
  state_est->stacking_state = control_input->stacking_state;

  EstimatorJoystickStep(&control_input->joystick, &faults[kSubsysJoystick],
                        &params->joystick, &state->joystick,
                        &state_est->joystick);

  EstimatorExperimentStep(state_est->joystick.pitch_f,
                          control_input->experiment_type,
                          control_input->experiment_case_id, &state->experiment,
                          &state_est->experiment);

  // Convert loadcell tensions to tether tension and tether roll angle.
  EstimatorTetherForceStep(control_input->loadcells, &faults[SUBSYS_LOADCELLS],
                           &system_params->wing, system_params->loadcells,
                           &params->tether_force, &state->tether_force,
                           &state_est->tether_force_b);

  // Determine whether the the tether has been released.
  state->tether_release_latched |= control_input->tether_released;
  state_est->tether_released = state->tether_release_latched;

  // Determine if the operator is commanding a gs mode or unpause.
  state_est->force_high_tension = control_input->force_high_tension;
  state_est->force_reel = control_input->force_reel;

  if (control_input->gs_unpause_transform) {
    state->gs_unpause_transform_count = GS_UNPAUSE_TRANSFORM_CYCLES;
  } else if (state->gs_unpause_transform_count > 0) {
    state->gs_unpause_transform_count--;
  }
  state_est->gs_unpause_transform = state->gs_unpause_transform_count > 0;

  // Estimate the heading and position of the ground station.
  GroundStationEstimate ground_station;
  EstimatorGroundStationStep(
      &control_input->gs_gps, &faults[kSubsysGsGpsPos],
      control_input->gs_sensors.mode, &faults[kSubsysGroundStation],
      control_input->gs_sensors.detwist_pos, &faults[kSubsysDetwist],
      control_input->gs_sensors.transform_stage, &system_params->ground_frame,
      &params->ground_station, &state->ground_station, &ground_station);

  state_est->gs_mode = ground_station.mode;
  state_est->gs_transform_stage = ground_station.transform_stage;

  // Move detwist through an extra revolution, if commanded.
  state_est->force_detwist_turn_once = control_input->force_detwist_turn_once;

  EstimatorPerchAziStep(encoders.perch_azi, encoders.perch_azi_valid,
                        &params->perch_azi, &state->perch_azi,
                        &state_est->perch_azi);

  EstimatorVesselStep(&control_input->ground_estimate, &state_est->perch_azi,
                      &faults[kSubsysGroundEstimatorPosition], &state->vessel,
                      &state_est->vessel);

  // Estimate the tether payout and winch position.
  EstimatorWinchStep(
      control_input->gs_sensors.winch_pos, &faults[kSubsysDrum],
      control_input->gs_sensors.proximity, &faults[kSubsysGroundStation],
      flight_status->flight_mode == kFlightModePerched, system_params->gs_model,
      &system_params->winch, &state->winch, &state_est->winch);

  // Estimate the wind direction.
  EstimatorWindStep(initializing, &control_input->wind_ws,
                    &faults[kSubsysWindSensor], &state_est->vessel,
                    &system_params->wind_sensor, &params->wind, &state->wind,
                    &state_est->wind_g);
  Vec3 acc_b;
  EstimatorNavKiteStep(
      initializing, control_input->imus, control_input->wing_gps,
      &control_input->gs_gps, control_input->pitots, &ground_station.pose,
      &state_est->wind_g, &encoders, &state_est->perch_azi,
      &state_est->tether_force_b, &state_est->winch, flight_status->flight_mode,
      faults, system_params, &params->nav, &state->nav, &state_est->pqr,
      &state_est->dcm_g2b, &state_est->Ag, &state_est->gps_active,
      &state_est->Vg, &state_est->Xg, &state_est->pqr_f, &state_est->Ab_f,
      &acc_b, &state_est->acc_norm_f, &state_est->Vb, &state_est->Vb_f,
      &state_est->Vg_f);

  ApparentWindSph apparent_wind_pitot;
  EstimatorApparentWindStep(
      control_input->pitots, &state_est->tether_force_b, &state_est->wind_g,
      &state_est->dcm_g2b, &state_est->pqr, &acc_b, &state_est->Ab_f,
      &state_est->Vg, faults, system_params, &params->apparent_wind,
      &state->apparent_wind, &state_est->apparent_wind, &apparent_wind_pitot);

  EstimatorWindAloftStep(initializing, &state_est->apparent_wind,
                         &apparent_wind_pitot, &state_est->Vg,
                         &state_est->dcm_g2b, &state_est->wind_g, &params->wind,
                         &state->wind_aloft, &state_est->wind_aloft_g);

  // Saturate wind aloft playbook direction within limits.
  double wind_aloft_dir_allow_start =
      Wrap(state_est->wind_g.dir_f_playbook -
               params->wind.playbook_aloft_azi_offset_max,
           -PI, PI);
  double wind_aloft_dir_allow_end =
      Wrap(state_est->wind_g.dir_f_playbook +
               params->wind.playbook_aloft_azi_offset_max,
           -PI, PI);

  state_est->wind_aloft_g.dir_f_playbook = SaturateWrapped(
      state_est->wind_aloft_g.dir_f_playbook, wind_aloft_dir_allow_start,
      wind_aloft_dir_allow_end, -PI, PI);

  if (state->time < params->t_initialize) state->time += *g_sys.ts;

  // Update telemetry.
  GetEstimatorTelemetry()->initializing = initializing;
  GetEstimatorTelemetry()->ground_station = ground_station;

  EstimatorTetherGroundAnglesStep(
      initializing, system_params->gs_model, &ground_station, &state_est->winch,
      &state_est->vessel, &system_params->winch, &params->tether_ground_angles,
      &encoders, system_params->ground_station.gs02.detwist_elevation,
      &state->tether_ground_angles, &state_est->tether_ground_angles);

  EstimatorTetherAnchorPointStep(initializing, &state_est->winch,
                                 &state_est->vessel, state_est->winch.payout,
                                 &params->tether_anchor, &state->tether_anchor,
                                 &state_est->tether_anchor);
}
