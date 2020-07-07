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

#include "control/crosswind/crosswind_curvature.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/control_params.h"
#include "control/crosswind/crosswind_types.h"
#include "control/crosswind/crosswind_util.h"
#include "control/simple_aero.h"
#include "control/system_params.h"
#include "control/system_types.h"

void CrosswindCurvatureInit(double tension, double alpha, double beta,
                            const CrosswindCurvatureParams *params,
                            CrosswindCurvatureState *state) {
  memset(state, 0, sizeof(*state));
  state->tension_hpf_z1 = 0.0;
  state->tension_z1 = tension;
  state->alpha_cmd_z1 =
      Saturate(alpha, params->alpha_cmd_min, params->alpha_cmd_max);
  state->beta_cmd_z1 =
      Saturate(beta, params->beta_cmd_min, params->beta_cmd_max);
  state->transout_flare_time = 0.0;
}

// Finds a tether roll, alpha, beta command that will meet the
// curvature command.  Trade-offs are made between these commands to
// limit tensions, improve curvature response, limit maximum AoA,
// sideslip, and tether roll, etc.  The equation which describes the
// trade-offs is in the comment to the CurvatureToTetherRoll equation
// above.
//
// Note: We have strict limits on maximum tether roll command at low
// tensions.  Here we are actually using tether tension as a proxy for
// situations where the tether pitch angle is negative (e.g. after
// trans-in before the tether becomes taught).  Under these
// situations, rolling the wing causes an adverse yaw moment from the
// tether and can lead to large sideslips.
//
// TODO: We use tension (rather than tether pitch) to
// limit the roll command here because W7 was not designed for a
// tether pitch measurement; however, we may want to reconsider this
// for the M600.
//
// TODO: Try a complementary fast/slow response approach.
// Use predominately CL and CY for the high-pass-filtered k_aero_cmd,
// and use predominately tether_roll for the low-pass-filtered
// k_aero_cmd.
//
// NOTE: Previous versions of the controller contained a
// trick that used pitch-down and side-slip to increase turning
// authority when large tether roll angles were commanded.  This trick
// was used often with W7, but its efficacy on the M600 was
// questionable, so we pulled it out.  The idea behind this trick is
// that large rolls are an indication of poor turning which is often
// caused by negative sideslip, and moving the bridle point forward in
// body coordinates should help reduce the adverse yaw moment when the
// bridle point is behind the center-of-mass.  We should put this
// trick on a firmer analytical footing and consider revisiting it in
// the future.
//
// Args:
//   k_aero_cmd: Aerodynamic curvature command.
//   tension: Tether tension.
//   airspeed: Measured airspeed.
//   flags: Flags are used to change behavior based on sensor faults
//       or extreme wind conditions.
//   params: Curvature loop parameters.
//   state: Curvature loop state.
//   flaring: True if the vehicle is latched in the final trans-out flare.
//   alpha_cmd: Output for the angle-of-attack set-point.
//   dCL_cmd: Output command for a change in lift coefficient from the
//       center flaps.  This is used to quickly change CL without
//       changing the angle-of-attack.
//   beta_cmd: Output for the sideslip set-point.
//   tether_roll_cmd: Output for the tether roll set-point.
static void SetAngles(double k_aero_cmd, double tension, double airspeed,
                      double alpha_nom, double beta_nom, double loop_angle,
                      CrosswindPathType path_type,
                      const FlightStatus *flight_status,
                      const CrosswindFlags *flags,
                      const CrosswindCurvatureParams *params,
                      CrosswindCurvatureState *state, bool *flaring,
                      double *alpha_cmd, double *dCL_cmd, double *beta_cmd,
                      double *tether_roll_cmd) {
  // Validate parameters.
  assert(params->alpha_cmd_rate_min <= params->alpha_cmd_rate_max);
  assert(params->alpha_cmd_rate_min <= 0.0);
  assert(params->alpha_cmd_rate_max >= 0.0);
  assert(params->beta_cmd_rate_min <= params->beta_cmd_rate_max);
  assert(params->beta_cmd_rate_min <= 0.0);
  assert(params->beta_cmd_rate_max >= 0.0);
  assert(params->dalpha_dairspeed < 0.0);

  assert(params->preptransout_alpha_rate >= 0.0);
  assert(params->transout_flare_alpha_rate >= 0.0);

  // Adjust angle-of-attack to limit tension.  We use airspeed, rather
  // than tension, as the independent variable because it is difficult
  // to control the tension loop while maintaining a reasonable
  // bandwidth.
  double alpha_nom_airspeed =
      params->alpha_min_airspeed -
      (params->alpha_min - alpha_nom) / params->dalpha_dairspeed;
  *alpha_cmd = Crossfade(alpha_nom, params->alpha_min, airspeed,
                         alpha_nom_airspeed, params->alpha_min_airspeed);
  *beta_cmd = beta_nom;

  // Trans-out flare state machine.
  *flaring = (flight_status->flight_mode == kFlightModeCrosswindPrepTransOut &&
              path_type == kCrosswindPathPrepareTransitionOut &&
              (airspeed < params->transout_flare_airspeed ||
               state->transout_flare_time > 0.0));
  if (*flaring) {
    state->transout_flare_time += *g_sys.ts;
  } else {
    state->transout_flare_time = 0.0;
  }

  // Calculate rate limits.
  double alpha_cmd_rate_min = params->alpha_cmd_rate_min;
  double alpha_cmd_rate_max = params->alpha_cmd_rate_max;
  double beta_cmd_rate_min = params->beta_cmd_rate_min;
  double beta_cmd_rate_max = params->beta_cmd_rate_max;

  if (flight_status->flight_mode == kFlightModeCrosswindPrepTransOut &&
      !*flaring) {
    // We're in PrepTransOut but not yet flaring.  Move to lower angle
    // of attack to dump lift before slowing down.
    *alpha_cmd = params->preptransout_alpha_cmd;
    alpha_cmd_rate_min = -params->preptransout_alpha_rate;
    alpha_cmd_rate_max = params->preptransout_alpha_rate;

  } else if (*flaring) {
    // Flare.
    *alpha_cmd = params->transout_flare_alpha_cmd;
    *beta_cmd = params->transout_flare_beta_cmd;

    // Impose alpha and beta limits as a proxy for body rotation rate limits.
    alpha_cmd_rate_min = -params->transout_flare_alpha_rate;
    alpha_cmd_rate_max = params->transout_flare_alpha_rate;
    beta_cmd_rate_min = -params->transout_flare_beta_rate;
    beta_cmd_rate_max = params->transout_flare_beta_rate;
  }

  // Apply rate limits.
  *alpha_cmd = RateLimit(*alpha_cmd, alpha_cmd_rate_min, alpha_cmd_rate_max,
                         *g_sys.ts, &state->alpha_cmd_z1);

  *beta_cmd = RateLimit(*beta_cmd, beta_cmd_rate_min, beta_cmd_rate_max,
                        *g_sys.ts, &state->beta_cmd_z1);

  // Remove high frequency tension noise with the flaps.  High pass
  // filter the tension and produce an appropriate change-in-CL
  // command (dCL_cmd).  Turn this feature off if there is any
  // indication of a fault in the tension measurement.  We currently
  // set the gain on this to 0 because the exact amount to use this
  // feature depends on unknown details of servo wear, etc...
  if (!flags->loadcell_fault) {
    double tension_hpf = Hpf(tension, params->fc_tension, *g_sys.ts,
                             &state->tension_hpf_z1, &state->tension_z1);
    *dCL_cmd =
        params->kp_tension_hf * -tension_hpf /
        fmax(0.5 * g_sys.phys->rho * g_sys.wing->A * airspeed * airspeed, 1.0);
  } else {
    *dCL_cmd = 0.0;
  }

  // Convert a curvature command to a tether roll command based on the
  // curvature equation in the function comment above.
  *tether_roll_cmd = CurvatureToTetherRoll(
      k_aero_cmd, AlphaToCL(*alpha_cmd, g_cont.simple_aero_model) + *dCL_cmd,
      BetaToCY(*beta_cmd, g_cont.simple_aero_model));

  // Add the feedforward tether roll command.
  *tether_roll_cmd += params->tether_roll_ff_amplitude *
                      cos(loop_angle + params->tether_roll_ff_phase);

  // Fade in tether roll command following transition-in.  This
  // greatly reduces the transients.
  if (flight_status->last_flight_mode == kFlightModeTransIn) {
    *tether_roll_cmd = Crossfade(0.0, *tether_roll_cmd,
                                 flight_status->flight_mode_time, 0.0, 2.0);
  }

  // Determine maximum tether roll angle excursion.  If any of the
  // loadcells have failed, do not trust the tension measurement and
  // default to the safer high roll angle case.
  double tether_roll_max_excursion;
  if (flags->loadcell_fault) {
    tether_roll_max_excursion = params->tether_roll_max_excursion_high;
  } else {
    tether_roll_max_excursion = Crossfade(
        params->tether_roll_max_excursion_low,
        params->tether_roll_max_excursion_high, tension,
        params->tether_roll_tension_low, params->tether_roll_tension_high);
  }

  // Saturate commands within reasonable limits.
  double alpha_cmd_max =
      *flaring ? params->alpha_cmd_max_flare : params->alpha_cmd_max;
  *alpha_cmd = Saturate(*alpha_cmd, params->alpha_cmd_min, alpha_cmd_max);
  *dCL_cmd = Saturate(*dCL_cmd, -params->dCL_cmd_max, params->dCL_cmd_max);
  *beta_cmd = Saturate(*beta_cmd, -params->beta_cmd_max, params->beta_cmd_max);
  *tether_roll_cmd = Saturate(
      *tether_roll_cmd, params->tether_roll_nom - tether_roll_max_excursion,
      params->tether_roll_nom + tether_roll_max_excursion);
}

double CrosswindCurvatureFlareTimer(const CrosswindCurvatureState *state) {
  return state->transout_flare_time;
}

// Finds the angular rate (in body coordinates) expected for a given
// curvature, tether roll, and speed.  The magnitude of the expected
// angular rate is: |pqr| = curvature / velocity.  The direction of
// the expected angular rate is perpendicular to the crosswind flight
// plane, so we need to know the pitch and roll angle of the wing
// relative to the flight plane.  Currently, we assume the pitch is
// small and the roll angle is given by:
//
//   body_roll_cmd = -tether_roll_cmd - (half cone angle)
//
// where the "half cone angle" for a circle radius R is defined by:
//
//   (half cone angle) = asin(R / tether_len)
//
// TODO: This does not take into account natural changes
// in the angular rate due to wind asymmetries.
static const Vec3 *SetRates(double k_geom_cmd, double tether_roll_cmd,
                            const Vec3 *Vg, Vec3 *pqr_cmd) {
  if (fabs(k_geom_cmd) < DBL_TOL) {
    *pqr_cmd = kVec3Zero;
  } else {
    double pqr_norm = k_geom_cmd * Vec3Norm(Vg);
    double body_roll_cmd =
        -Asin(1.0 / (k_geom_cmd * g_sys.tether->length)) - tether_roll_cmd;
    pqr_cmd->x = 0.0;
    pqr_cmd->y = pqr_norm * sin(body_roll_cmd);
    pqr_cmd->z = pqr_norm * cos(body_roll_cmd);
  }
  return pqr_cmd;
}

void CrosswindCurvatureStep(double k_aero_cmd, double k_geom_cmd,
                            double tension, double airspeed, double alpha_nom,
                            double beta_nom, double loop_angle,
                            CrosswindPathType path_type, const Vec3 *Vg,
                            const FlightStatus *flight_status,
                            const CrosswindFlags *flags,
                            const CrosswindCurvatureParams *params,
                            CrosswindCurvatureState *state, Vec3 *pqr_cmd,
                            bool *flaring, double *alpha_cmd, double *dCL_cmd,
                            double *beta_cmd, double *tether_roll_cmd) {
  assert(Vg != NULL && flags != NULL && params != NULL && state != NULL &&
         pqr_cmd != NULL && alpha_cmd != NULL && dCL_cmd != NULL &&
         beta_cmd != NULL && tether_roll_cmd != NULL);
  SetAngles(k_aero_cmd, tension, airspeed, alpha_nom, beta_nom, loop_angle,
            path_type, flight_status, flags, params, state, flaring, alpha_cmd,
            dCL_cmd, beta_cmd, tether_roll_cmd);
  SetRates(k_geom_cmd, *tether_roll_cmd, Vg, pqr_cmd);
}
