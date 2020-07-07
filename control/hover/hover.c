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

#include "control/hover/hover.h"

#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_playbook.h"
#include "control/hover/hover_altitude.h"
#include "control/hover/hover_angles.h"
#include "control/hover/hover_experiments.h"
#include "control/hover/hover_frame.h"
#include "control/hover/hover_inject.h"
#include "control/hover/hover_mode.h"
#include "control/hover/hover_output.h"
#include "control/hover/hover_path.h"
#include "control/hover/hover_position.h"
#include "control/hover/hover_tension.h"
#include "control/hover/hover_winch.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "control/system_types.h"

void HoverInit(const StateEstimate *state_est,
               const Vec3 *previous_perched_pos_g,
               double previous_int_yaw_angle, double previous_thrust_cmd,
               const double *flaps_z1, const Vec3 *previous_loop_center_v,
               const HoverParams *params, HoverState *state) {
  assert(state_est != NULL && params != NULL && state != NULL);

  memset(state, 0, sizeof(*state));
  HoverAltitudeInit(previous_thrust_cmd, &params->altitude, &state->altitude);
  HoverAnglesInit(&params->angles, &state->angles);
  HoverInjectInit(&params->inject);
  HoverExperimentsInit(&params->experiments, &state->experiment_state);
  HoverOutputInit(state_est->joystick.data.throttle, flaps_z1[kFlapEle],
                  previous_loop_center_v, &params->output, &state->output);
  HoverPathInit(&state_est->Xg, previous_perched_pos_g, &params->path,
                &state->path);
  HoverPositionInit(previous_int_yaw_angle, &params->position,
                    &state->position);
  HoverTensionInit(&params->tension, &state->tension);
  HoverWinchInit(&params->winch, &state->winch);
  state->thrust_moment_z1.thrust = 0.0;
  state->thrust_moment_z1.moment = kVec3Zero;
  state->tether_elevation.good_elevation_since = 0.0;
}

bool HoverIsReadyForMode(FlightMode proposed_flight_mode,
                         const FlightStatus *flight_status,
                         const StateEstimate *state_est,
                         const HoverParams *params, const HoverState *state) {
  assert(IsValidFlightMode(proposed_flight_mode));
  assert(flight_status != NULL &&
         IsValidFlightMode(flight_status->flight_mode) &&
         IsValidFlightMode(flight_status->last_flight_mode) &&
         flight_status->flight_mode_time >= 0.0);
  assert(state_est != NULL && params != NULL && state != NULL);

  bool ascent_complete = HoverPathIsAscentComplete(flight_status->flight_mode,
                                                   &params->path, state_est);

  bool gain_ramp_done = HoverOutputIsGainRampDone(&state->output);

  Vec3 angles_error;
  HoverAnglesGetAnglesError(&state->angles, &angles_error);

  Vec3 waypoint_g_error;
  HoverPathGetWaypointError(&state_est->Xg, &state->path, &waypoint_g_error);

  double waypoint_azi_error =
      HoverPathGetWaypointAzimuthError(&state_est->Xg, &state->path);

  double tether_elevation_error_for_transform =
      params->path.target_transform_tether_elevation -
      state_est->tether_ground_angles.elevation_p;

  return HoverModeIsReadyFor(
      proposed_flight_mode, flight_status, state_est, ascent_complete,
      gain_ramp_done, state->output.forcing_detwist_turn,
      state->tether_elevation.good_elevation_since,
      tether_elevation_error_for_transform, &angles_error,
      &state->path.accel_start_pos_g, waypoint_azi_error,
      &state->path.raw_wing_pos_g_cmd, &params->mode);
}

const ThrustMoment *HoverGetLastThrustMomentCmd(const HoverState *state) {
  return &state->thrust_moment_z1;
}

// Calculate the horizontal tension.
static double CalcHorizontalTension(const Mat3 *dcm_g2b,
                                    const Vec3 *tether_force_b) {
  Vec3 tether_force_g;
  Mat3TransVec3Mult(dcm_g2b, tether_force_b, &tether_force_g);

  const double horizontal_tension = Vec3XyNorm(&tether_force_g);
  return horizontal_tension;
}

// TODO: This function does not check:
//   - state_est->joystick.valid,
//   - state_est->winch.valid,
//   - state_est->wind_g.valid.
void HoverStep(const FlightStatus *flight_status,
               const StateEstimate *state_est, const Playbook *playbook,
               const HoverParams *params, HoverState *state,
               ControlOutput *control_output) {
  assert(flight_status != NULL &&
         IsValidFlightMode(flight_status->flight_mode) &&
         IsValidFlightMode(flight_status->last_flight_mode) &&
         flight_status->flight_mode_time >= 0.0);
  assert(state_est != NULL && params != NULL && state != NULL &&
         control_output != NULL);

  // The hover origin is the location at which we consider the tether to be
  // anchored, for the purpose of determining hover angles and similar
  // operations involving the hover frame.
  // TODO: We may wish to apply smoothing here. One easy way to
  // accomplish that may be to use tether_anchor.pos_g after smoothing is
  // applied to that quantity.
  const Vec3 *hover_origin_g = &state_est->vessel.pos_g;

  const double tether_elevation_cmd = HoverPathCalcTetherElevationCommand(
      flight_status, state_est->winch.payout, &params->path);

  double winch_vel_cmd = HoverWinchStep(flight_status->flight_mode, state_est,
                                        &params->winch, &state->winch);

  bool hold_horizontal_tension_cmd =
      (flight_status->flight_mode == kFlightModeHoverTransformGsUp ||
       flight_status->flight_mode == kFlightModeHoverTransformGsDown);

  // The outermost control loop selects the target wing position and
  // tether tension.
  // TODO(b/112041251): The winch velocity input is hard-coded to zero
  // here; consider removing this feature completely.
  Vec3 wing_pos_v_g;
  Vec3Sub(&state_est->Xg, hover_origin_g, &wing_pos_v_g);
  double horizontal_tension_cmd = HoverTensionSetHorizontalTension(
      &wing_pos_v_g, &state_est->wind_g.vector_f_slow, 0.0,
      hold_horizontal_tension_cmd, &params->tension, &state->tension);

  // Calculate the horizontal tension.
  GetHoverTelemetry()->horizontal_tension = CalcHorizontalTension(
      &state_est->dcm_g2b, &state_est->tether_force_b.vector);

  // For GSv2, check whether the GS is ready for transform.
  HoverModeCheckTetherElevationForGsTransform(
      &params->path, &params->mode, &state_est->tether_ground_angles,
      flight_status, &state->tether_elevation);

  bool gain_ramp_done = HoverOutputIsGainRampDone(&state->output);
  Vec3 wing_pos_g_cmd, wing_vel_g_cmd, crosswind_loop_center_pos_g;

  HoverPathStep(&state_est->tether_anchor.pos_g_f, &state_est->vessel,
                horizontal_tension_cmd, tether_elevation_cmd,
                &state_est->tether_ground_angles, state_est->winch.payout,
                &state_est->Xg, &state_est->Vg, &state_est->wind_g, playbook,
                state_est->winch.position, winch_vel_cmd, flight_status,
                &params->path, &state->path, &wing_pos_g_cmd, &wing_vel_g_cmd,
                &crosswind_loop_center_pos_g);

  // Run the outer control loop (position).

  // Tension is servoed to the commanded value using pitch as an
  // actuator.  The feed-forward pitch is taken as an input in order
  // to apply saturations on the total pitch command, and to hold the
  // integrator when saturated.
  double thrust_z1 = HoverOutputGetLastThrust(&state->output);
  bool below_half_thrust = thrust_z1 < g_sys.wing->m * g_sys.phys->g / 2.0;
  double wing_offset_from_perched_g_z =
      HoverPathGetZOffsetFromPerchedPosition(&state_est->Xg, &state->path);

  // Inject signals into the position command.
  if (HoverInjectIsEnabled(flight_status, &params->inject)) {
    HoverInjectPositionSignal(flight_status, &wing_pos_g_cmd, hover_origin_g,
                              &params->inject, &wing_pos_g_cmd);
  }

  double pitch_cmd = HoverTensionStep(
      horizontal_tension_cmd, tether_elevation_cmd, &state_est->tether_force_b,
      &state_est->Xg, hover_origin_g, &state_est->wind_g.vector_f_slow,
      state_est->winch.payout, winch_vel_cmd, below_half_thrust, flight_status,
      &state_est->joystick, &params->tension, &state->tension);

  // Calculate the current attitude in the hover frame.
  Vec3 angles;
  HoverAnglesGetAngles(&state_est->Xg, hover_origin_g, &state_est->dcm_g2b,
                       &angles);

  // Control the tangential position and provide velocity damping via
  // an attitude command.
  Vec3 angles_cmd;
  Vec3 pqr_cmd;
  HoverPositionStep(
      &wing_pos_g_cmd, &state_est->Xg, &wing_vel_g_cmd, &state_est->Vg_f,
      hover_origin_g, &state_est->pqr_f, &angles, pitch_cmd,
      state_est->wind_aloft_g.speed_f_playbook, &state_est->joystick.data,
      &state_est->dcm_g2b, state_est->winch.payout, flight_status,
      wing_offset_from_perched_g_z, &params->position, &state->position,
      &pqr_cmd, &angles_cmd);

  // Altitude is controlled by actuating on thrust.
  ThrustMoment thrust_moment;

  thrust_moment.thrust = HoverAltitudeStep(
      wing_pos_g_cmd.z, state_est->Xg.z, wing_vel_g_cmd.z, state_est->Vg_f.z,
      state_est->joystick.data.throttle, state_est->winch.payout, flight_status,
      gain_ramp_done, thrust_z1, &params->altitude, &state->altitude);

  // Run the inner control loop (attitude).

  // Inject signals into the angle command.
  if (HoverInjectIsEnabled(flight_status, &params->inject)) {
    HoverInjectAnglesSignal(flight_status, &angles_cmd, &params->inject,
                            &angles_cmd);
  }

  double elevator_pitch_moment, rudder_yaw_moment, blown_flaps_roll_moment;
  HoverAnglesStep(
      &angles_cmd, &angles, &pqr_cmd, &state_est->pqr_f, &state_est->winch,
      &state_est->vessel, thrust_z1, flight_status, &params->angles,
      &state_est->experiment, &state->angles, &thrust_moment.moment,
      &blown_flaps_roll_moment, &elevator_pitch_moment, &rudder_yaw_moment);

  HoverOutputStep(flight_status, state_est, crosswind_loop_center_pos_g,
                  &thrust_moment, blown_flaps_roll_moment,
                  elevator_pitch_moment, rudder_yaw_moment, winch_vel_cmd,
                  &params->output, &state->output, control_output);

  state->thrust_moment_z1 = thrust_moment;

  // Inject signals directly into the output.
  if (HoverInjectIsEnabled(flight_status, &params->inject)) {
    HoverInjectOutputSignal(flight_status, control_output, &params->inject,
                            control_output);
  }

  HoverElevatorExperimentsOutputSignal(
      flight_status, control_output, state_est, &params->experiments,
      &state->experiment_state, control_output);
}
