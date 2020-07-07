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

#include "control/crosswind/crosswind_output.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "avionics/common/plc_messages.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/estimator/estimator_types.h"
#include "control/ground_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/vessel_frame.h"

bool CrosswindOutputValidateParams(const CrosswindOutputParams *params) {
  // Sanity check individual values.
  for (int i = 0; i < ARRAYSIZE(params->rudder_limit_betas); ++i) {
    if (!(-PI / 2.0 < params->rudder_limit_betas[i] &&
          params->rudder_limit_betas[i] < PI / 2.0)) {
      assert(!(bool)"rudder_limit_betas must be sensible values in radians.");
      return false;
    }
  }

  for (int i = 0; i < ARRAYSIZE(params->rudder_limit_airspeeds); ++i) {
    if (!(0.0 <= params->rudder_limit_airspeeds[i] &&
          params->rudder_limit_airspeeds[i] <= 100.0)) {
      assert(!(bool)"rudder_limit_airspeeds must be sensible values in m/s.");
      return false;
    }
  }

  for (int i = 0; i < ARRAYSIZE(params->rudder_limit_airspeeds); ++i) {
    for (int j = 0; j < ARRAYSIZE(params->rudder_limit_betas); ++j) {
      if (!(-PI / 2.0 < params->rudder_limits_lower[i][j] &&
            params->rudder_limits_lower[i][j] < PI / 2.0)) {
        assert(
            !(bool)"rudder_limits_lower must be sensible values in radians.");
        return false;
      }
      if (!(-PI / 2.0 < params->rudder_limits_upper[i][j] &&
            params->rudder_limits_upper[i][j] < PI / 2.0)) {
        assert(
            !(bool)"rudder_limits_upper must be sensible values in radians.");
        return false;
      }
    }
  }

  // Check relationships between values.
  for (int i = 1; i < ARRAYSIZE(params->rudder_limit_betas); ++i) {
    if (!(params->rudder_limit_betas[i - 1] < params->rudder_limit_betas[i])) {
      assert(!(bool)"rudder_limit_betas must be monotonically increasing.");
      return false;
    }
  }

  for (int i = 1; i < ARRAYSIZE(params->rudder_limit_airspeeds); ++i) {
    if (!(params->rudder_limit_airspeeds[i - 1] <
          params->rudder_limit_airspeeds[i])) {
      assert(!(bool)"rudder_limit_airspeeds must be monotonically increasing.");
      return false;
    }
  }

  for (int i = 0; i < ARRAYSIZE(params->rudder_limit_airspeeds); ++i) {
    for (int j = 0; j < ARRAYSIZE(params->rudder_limit_betas); ++j) {
      if (!(params->rudder_limits_lower[i][j] <=
            params->rudder_limits_upper[i][j])) {
        assert(!(
            bool)"rudder_limits_lower must be less than or equal to "
                 "rudder_limits_upper at all points in the table.");
        return false;
      }
    }
  }

  return true;
}

void CrosswindOutputInit(const CrosswindOutputParams *params,
                         const HoverOutputState *hover_output_state,
                         CrosswindOutputState *state,
                         double previous_detwist_loop_angle,
                         int32_t previous_detwist_rev_count) {
  assert(params != NULL && hover_output_state != NULL && state != NULL);
  assert(CrosswindOutputValidateParams(params));

  memset(state, 0, sizeof(*state));

  // See GetDetwistCommand.
  state->detwist_loop_angle = previous_detwist_loop_angle;
  state->detwist_rev_count = previous_detwist_rev_count;

  // Initialize the delayed release states.
  state->prerelease_timer = 0.0;
  state->prerelease_flag = false;

  // Initialize the loop center position.
  state->path_center_v_f_z1 = hover_output_state->cw_loop_center_v_f_z1;
}

static void GetRudderLimits(double airspeed, double beta,
                            const CrosswindOutputParams *params,
                            double *rudder_lower_limit,
                            double *rudder_upper_limit) {
  assert(params != NULL && rudder_lower_limit != NULL &&
         rudder_upper_limit != NULL);
  const InterpOption opt = kInterpOptionSaturate;

  *rudder_lower_limit =
      Interp2(params->rudder_limit_betas, params->rudder_limit_airspeeds,
              ARRAYSIZE(params->rudder_limit_betas),
              ARRAYSIZE(params->rudder_limit_airspeeds),
              (const double *)params->rudder_limits_lower, beta, airspeed, opt);

  *rudder_upper_limit =
      Interp2(params->rudder_limit_betas, params->rudder_limit_airspeeds,
              ARRAYSIZE(params->rudder_limit_betas),
              ARRAYSIZE(params->rudder_limit_airspeeds),
              (const double *)params->rudder_limits_upper, beta, airspeed, opt);

  assert(*rudder_lower_limit <= *rudder_upper_limit);
}

// Gets the command for the detwist servo, in [0, TETHER_DETWIST_REVS * 2*PI).
//
// Small changes in loop_angle opposite the loop direction are assumed to be
// temporary bounce-backs and are ignored.
//
// A large (>PI) loop_angle jump opposite the loop direction is interpreted as
// completion of a revolution. A large (>PI) jump in the loop direction is
// assumed to be a bounce backwards across the periodic boundary and is ignored.
//
// Args:
//   loop_dir: Direction of loops.
//   loop_angle: The loop angle measured by the crosswind controller, in
//       [0, 2*PI).
//   rev_count: Revolution counter, from 0 to TETHER_DETWIST_REVS-1. This
//       increments for counterclockwise loops and decrements for clockwise
//       loops, in agreement with whether loop_angle is increasing or
//       decreasing.
//   detwist_loop_angle: The single-loop angle for the detwist servo.
//       This is in [-2*PI, 0) for clockwise loop direction, and [0, 2*PI) for
//       counter-clockwise loop direction, but should start at 0 in the latter
//       case, as not to falsely complete a loop.
static double GetDetwistCommand(LoopDirection loop_dir, double loop_angle,
                                int32_t *rev_count,
                                double *detwist_loop_angle) {
  if (loop_dir == kLoopDirectionCcw) {
    double difference = loop_angle - *detwist_loop_angle;
    if (difference < -PI) {
      *rev_count = WrapInt32(*rev_count + 1, 0, (int32_t)TETHER_DETWIST_REVS);
      *detwist_loop_angle = loop_angle;
    } else if (0.0 < difference && difference < PI) {
      *detwist_loop_angle = loop_angle;
    }
  } else if (loop_dir == kLoopDirectionCw) {
    double wrapped_angle = Wrap(loop_angle, -2.0 * PI, 0.0);
    double difference = wrapped_angle - *detwist_loop_angle;
    if (difference > PI) {
      *rev_count = WrapInt32(*rev_count - 1, 0, (int32_t)TETHER_DETWIST_REVS);
      *detwist_loop_angle = wrapped_angle;
    } else if (-PI < difference && difference < 0.0) {
      *detwist_loop_angle = wrapped_angle;
    }
  } else {
    assert(false);
  }

  return Wrap(2.0 * PI * (double)*rev_count + *detwist_loop_angle, 0.0,
              2.0 * PI * (double)TETHER_DETWIST_REVS);
}

void CrosswindOutputStep(LoopDirection loop_dir, double loop_angle,
                         bool allow_flare, const ThrustMoment *thrust_moment,
                         const Deltas *deltas, const StateEstimate *state_est,
                         const Vec3 *path_center_g,
                         const CrosswindOutputParams *params,
                         CrosswindOutputState *state,
                         ControlOutput *control_output,
                         Deltas *deltas_available) {
  assert(thrust_moment != NULL && deltas != NULL && state_est != NULL &&
         params != NULL && state != NULL && control_output != NULL &&
         deltas_available != NULL);

  // Calculate ground station azimuth command.
  double gs_azi_target_raw;
  {
    // TODO: Here we assume that the vessel is near enough to the
    // origin that we need not compensate for vessel_pos_g. This is a case where
    // a slowly-filtered version of the vessel estimate might be useful.
    // TODO: Observe vessel->valid.
    Vec3 path_center_v;
    Mat3Vec3Mult(&state_est->vessel.dcm_g2v, path_center_g, &path_center_v);
    control_output->gs_mode_request = kGroundStationModeHighTension;
    LpfVec3(&path_center_v, GetControlParams()->estimator.wind.fc_dir,
            *g_sys.ts, &state->path_center_v_f_z1);
    control_output->gs_azi_cmd.target =
        Wrap(VecVToAzimuth(&state->path_center_v_f_z1), 0.0, 2.0 * PI);
    gs_azi_target_raw = Wrap(VecVToAzimuth(&path_center_v), 0.0, 2.0 * PI);
  }

  // TODO: Replace this hard coded deadband with a parameter
  control_output->gs_azi_cmd.dead_zone = 1.0 * PI / 180.0;
  control_output->gs_unpause_transform = false;
  control_output->hold_gs_azi_cmd = false;

  // Compute flap limits and call MixFlaps.
  {
    double lower_flap_limits[kNumFlaps];
    double upper_flap_limits[kNumFlaps];

    for (int i = 0; i < kNumFlaps; ++i) {
      // Copy the default flap limits from the parameters.
      lower_flap_limits[i] = allow_flare ? params->lower_flap_limits_flare[i]
                                         : params->lower_flap_limits[i];
      upper_flap_limits[i] = params->upper_flap_limits[i];
      assert(lower_flap_limits[i] <= upper_flap_limits[i]);
    }

    // Overwrite the rudder limits.
    // TODO(b/113932478): We might want to use a more heavily
    // filtered airspeed and beta here to avoid impressing
    // high-frequency motion onto the rudder.
    GetRudderLimits(state_est->apparent_wind.sph_f.v,
                    state_est->apparent_wind.sph_f.beta, params,
                    &lower_flap_limits[kFlapRud], &upper_flap_limits[kFlapRud]);

    MixFlaps(deltas, params->flap_offsets, lower_flap_limits, upper_flap_limits,
             control_output->flaps, deltas_available);
  }

  ThrustMoment thrust_moment_avail;
  double v_app_locals[kNumMotors];
  double rotors[kNumMotors];
  MixRotors(thrust_moment, &params->thrust_moment_weights,
            state_est->apparent_wind.sph_f.v, &state_est->pqr_f,
            (StackingState)state_est->stacking_state, false, g_sys.phys->rho,
            g_sys.rotors, g_cont.rotor_control, rotors, &thrust_moment_avail,
            v_app_locals);

  // Deal speed command into motor representation.
  // TODO: Make common function for generating these across modes.
  for (int i = 0; i < kNumMotors; ++i) {
    control_output->motor_speed_upper_limit[i] = rotors[i];
    control_output->motor_speed_lower_limit[i] = rotors[i];
    control_output->motor_torque[i] = 0.0;
  }

  // Choose here between using the "optimal" or the deterministic (loop angle).
  if (params->adaptive_detwist_cmd) {
    control_output->detwist_cmd =
        state_est->tether_ground_angles.departure_detwist_angle;
  } else {
    control_output->detwist_cmd =
        GetDetwistCommand(loop_dir, loop_angle, &state->detwist_rev_count,
                          &state->detwist_loop_angle);
  }

  // Winch velocity is always zero in crosswind.
  control_output->winch_vel_cmd = 0.0;

  // Decide whether to release the tether.
  control_output->tether_release = false;
  if (state_est->joystick.valid && state_est->joystick.data.release) {
    state->prerelease_flag = true;
  }

  // Delay tether release to reduce angle of attack first.
  if (state->prerelease_flag) {
    // Reduce aileron commands prior to release to unload the wing.
    control_output->flaps[kFlapA1] = params->release_aileron_cmd;
    control_output->flaps[kFlapA2] = params->release_aileron_cmd;
    control_output->flaps[kFlapA4] = params->release_aileron_cmd;
    control_output->flaps[kFlapA5] = params->release_aileron_cmd;
    control_output->flaps[kFlapA7] = params->release_aileron_cmd;
    control_output->flaps[kFlapA8] = params->release_aileron_cmd;

    // Run release wait timer, send the release command.
    state->prerelease_timer += *g_sys.ts;
    if (state->prerelease_timer > params->release_wait_period) {
      control_output->tether_release = true;
    }
  }

  // Don't stop the motors.
  control_output->stop_motors = false;

  // These values are set by the outer control system.
  control_output->run_motors = false;
  control_output->sync.sequence = 0U;
  control_output->sync.flight_mode = -1;

  // Update telemetry.
  ControlTelemetry *ct = GetControlTelemetry();
  ct->deltas = *deltas;
  ct->deltas_avail = *deltas_available;
  ct->thrust_moment = *thrust_moment;
  ct->thrust_moment_avail = thrust_moment_avail;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    ct->v_app_locals[i] = v_app_locals[i];
    ct->gs_azi_target_raw = gs_azi_target_raw;
  }
}
