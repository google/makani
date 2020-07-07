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

#include "control/estimator/estimator_apparent_wind.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/simple_aero.h"
#include "control/system_params.h"
#include "control/system_types.h"

void EstimatorApparentWindInit(EstimatorApparentWindState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));
  state->sph_f_z1.v = 0.0;
  state->sph_f_z1.alpha = 0.0;
  state->sph_f_z1.beta = 0.0;
  state->apparent_wind_b_lpf_z1 = kVec3Zero;
  state->apparent_wind_b_hpf_z1 = kVec3Zero;
}

static void TetherForceToApparentWindSph(const Vec3 *tether_force_b,
                                         const Vec3 *acc, double v_app_norm,
                                         const SimpleAeroModelParams *params,
                                         ApparentWindSph *sph) {
  Vec3 centrifugal_force_b;
  Vec3Scale(acc, g_sys.wing->m, &centrifugal_force_b);
  Vec3 aero_and_rotor_force_b;
  Vec3Sub(&centrifugal_force_b, tether_force_b, &aero_and_rotor_force_b);
  Vec3 C_aero;
  Vec3Scale(&aero_and_rotor_force_b,
            1.0 / (0.5 * g_sys.phys->rho * g_sys.wing->A *
                   fmax(v_app_norm * v_app_norm, 1.0)),
            &C_aero);

  sph->v = v_app_norm;
  sph->alpha = CLToAlpha(-C_aero.z, params);
  sph->beta = CYToBeta(C_aero.y, params);
}

// Calculate the apparent wind from the Pitot accounting for upwash
// and mass balance tube aerodynamic interference.
//
// Args:
//   pitots: Array of kNumPitotSensors PitotData structures.
//   pqr: Estimated body rotation rates.
//   pitot_high_speed_faults: Array of kNumFaultDetectionPitotSignals faults.
//   pitot_low_speed_faults: Array of kNumFaultDetectionPitotSignals faults.
//   pitot_params: Array of kNumPitotSensors Pitot parameters.
//   params: Parameters.
//   sph: Output estimate.
//
// Returns:
//   False if the estimate should not be used.
static bool CalcPitotApparentWindSph(const PitotData pitots[], const Vec3 *pqr,
                                     const FaultMask *pitot_high_speed_faults,
                                     const FaultMask *pitot_low_speed_faults,
                                     const PitotParams *pitot_params,
                                     const EstimatorApparentWindParams *params,
                                     ApparentWindSph *sph) {
  const PitotDifferentialData *high_speed_sensors =
      &pitots[kPitotSensorHighSpeed].diff;
  const PitotDifferentialData *low_speed_sensors =
      &pitots[kPitotSensorLowSpeed].diff;

  // NOTE: This code explicitly makes of the fact that we have a single Pitot
  // tube connected to two sets of pressure sensors.
  bool use_high_speed =
      !HasAnyFault(&pitot_high_speed_faults[kFaultDetectionPitotSignalAlpha]) &&
      !HasAnyFault(&pitot_high_speed_faults[kFaultDetectionPitotSignalBeta]) &&
      !HasAnyFault(&pitot_high_speed_faults[kFaultDetectionPitotSignalDynamic]);

  // We only use the low pressure Pitot if it is unsaturated and unfaulted.
  const double high_pressure =
      pitot_params->sensors[kPitotSensorLowSpeed].max_pressure;
  bool use_low_speed =
      low_speed_sensors->dyn_press <= high_pressure &&
      !HasAnyFault(&pitot_low_speed_faults[kFaultDetectionPitotSignalAlpha]) &&
      !HasAnyFault(&pitot_low_speed_faults[kFaultDetectionPitotSignalBeta]) &&
      !HasAnyFault(&pitot_low_speed_faults[kFaultDetectionPitotSignalDynamic]);

  // If the high-speed pressure sensors are not faulted and disagree
  // with the low speed sensors by a large enough value to imply one
  // of the two sensors is out of spec, we default to the high speed
  // sensors.  We favor the high speed sensors as they cover the
  // entire crosswind flight envelope.  Also, large pressure differences
  // could be due to the low speed sensors saturating.
  double max_pressure_diff =
      0.02 * pitot_params->sensors[kPitotSensorHighSpeed].max_pressure;
  if (use_high_speed &&
      (fabs(low_speed_sensors->dyn_press - high_speed_sensors->dyn_press) >
           max_pressure_diff ||
       fabs(low_speed_sensors->alpha_press - high_speed_sensors->alpha_press) >
           max_pressure_diff ||
       fabs(low_speed_sensors->beta_press - high_speed_sensors->beta_press) >
           max_pressure_diff)) {
    use_low_speed = false;
  }

  PitotDifferentialData diff;
  if (use_high_speed && use_low_speed) {
    diff.dyn_press = Crossfade(
        low_speed_sensors->dyn_press, high_speed_sensors->dyn_press,
        high_speed_sensors->dyn_press, 0.5 * high_pressure, high_pressure);

    diff.alpha_press = Crossfade(
        low_speed_sensors->alpha_press, high_speed_sensors->alpha_press,
        high_speed_sensors->dyn_press, 0.5 * high_pressure, high_pressure);

    diff.beta_press = Crossfade(
        low_speed_sensors->beta_press, high_speed_sensors->beta_press,
        high_speed_sensors->dyn_press, 0.5 * high_pressure, high_pressure);
  } else if (use_high_speed) {
    diff = *high_speed_sensors;
  } else {
    diff = *low_speed_sensors;
  }

  PitotToApparentWindSph(&diff, pqr, pitot_params, sph);

  sph->v /= Sqrt(1.0 - pitot_params->local_pressure_coeff);
  sph->alpha -= params->pitot_upwash_alpha_bias;
  sph->alpha /= params->pitot_upwash_alpha_scale;

  return use_low_speed || use_high_speed;
}

static void FilterApparentWindSph(const ApparentWindSph *sph,
                                  const EstimatorApparentWindParams *params,
                                  ApparentWindSph *sph_f_z1,
                                  ApparentWindSph *sph_f) {
  sph_f->v = Lpf(sph->v, params->fc_v, *g_sys.ts, &sph_f_z1->v);
  sph_f->alpha = Lpf(sph->alpha, params->fc_alpha, *g_sys.ts, &sph_f_z1->alpha);
  sph_f->beta = Lpf(sph->beta, params->fc_beta, *g_sys.ts, &sph_f_z1->beta);
}

// Compute an estimate of the apparent wind vector tracking the Pitot solution
// at low frequencies and the kite's inertial motion at higher frequencies. This
// provides rejection of gusts without introducing a phase delay in the apparent
// wind estimate.
//
// Input arguments:
//
// apparent_wind_b:  Apparent wind estimate from the Pitot [m/s], body coords.
// Ab:               Kite acceleration vector [m/s^2], body coordinates.
// Vb:               Kite velocity vector [m/s], body coordinates.
// pqr:              Kite rotation rates [rad/s] about body axes.
// wind_b_slow:      Estimate of the mean wind velocity [m/s], for example as
//                   measured by the ground station, but in body coordinates.
// params:           Parameters structure.
// state:            State.
//
// Output arguments:
//
// apparent_wind_b_filtered:  Output of complementary filter.
static void ComplementaryApparentWindFilter(
    const ApparentWindSph *apparent_wind_pitot, const Vec3 *Ab, const Vec3 *Vb,
    const Vec3 *pqr, const Vec3 *wind_b_slow,
    const EstimatorApparentWindParams *params,
    EstimatorApparentWindState *state,
    ApparentWindSph *apparent_wind_filtered) {
  // The form of the complementary filter is:
  //
  // u = (w_c / (s + w_c)) apparent_wind_pitot +
  //       (s / (s + w_c)) apparent_wind_inertial
  //
  // where (wc / (s + w_c)) is a low-pass filter with cutoff frequency wc
  // [rad/s], and (s / (s + w_c)) is a high-pass filter; and apparent_wind_pitot
  // and apparent_wind_inertial are estimates of the apparent wind vector from
  // the pitot and from inertial sensors, respectively.  Note that these two
  // filters sum to unity, which is the definition of a complementary filter.
  //
  // It turns out that it is easier to directly compute the time derivative of
  // apparent_wind_inertial than it is to compute apparent_wind_inertial
  // directly.  It turns out that we can use a low-pass filter to effectively
  // integrate-and-high-pass this derivative by refactoring our computation as
  // follows:
  //
  // u = (w_c / (s + w_c)) apparent_wind_pitot +
  //     (w_c / (s + w_c)) (1 / w_c) (s * apparent_wind_inertial)
  //
  // where (s * apparent_wind_inertial) == apparent_wind_dot_inertial, and
  // (w_c / (s + w_c)) is the filter implemented by LpfVec3.
  //
  // Thus we find that we can apply our complementary filter by applying LpfVec3
  // to both apparent_wind_pitot and apparent_wind_dot_inertial, with a scale
  // factor of (1 / w_c) applied to the latter.

  // Apply low-pass filter to the Pitot solution.
  Vec3 apparent_wind_b;
  ApparentWindSphToCart(apparent_wind_pitot, &apparent_wind_b);

  LpfVec3(&apparent_wind_b, params->fc_comp, *g_sys.ts,
          &state->apparent_wind_b_lpf_z1);

  // Construct estimate of d/dt(apparent_wind_b) from other measurements:
  //
  //   d/dt(apparent_wind_b) = -Ab + (pqr x Vb) - (pqr x wind_b) + gusts
  Vec3 pqr_cross_uvw;
  Vec3Cross(pqr, Vb, &pqr_cross_uvw);

  Vec3 pqr_cross_wind_b_slow;
  Vec3Cross(pqr, wind_b_slow, &pqr_cross_wind_b_slow);

  Vec3 apparent_wind_b_dot_inertial;
  Vec3Sub(&pqr_cross_uvw, &pqr_cross_wind_b_slow,
          &apparent_wind_b_dot_inertial);
  Vec3Sub(&apparent_wind_b_dot_inertial, Ab, &apparent_wind_b_dot_inertial);

  double hpf_scale_factor = 1.0 / (params->fc_comp * 2.0 * PI);
  Vec3 scaled_apparent_wind_b_dot_inertial;
  Vec3Scale(&apparent_wind_b_dot_inertial, hpf_scale_factor,
            &scaled_apparent_wind_b_dot_inertial);

  LpfVec3(&scaled_apparent_wind_b_dot_inertial, params->fc_comp, *g_sys.ts,
          &state->apparent_wind_b_hpf_z1);

  // Sum both contributions to the complementary filter.
  Vec3 apparent_wind_b_filtered;
  Vec3Add(&state->apparent_wind_b_lpf_z1, &state->apparent_wind_b_hpf_z1,
          &apparent_wind_b_filtered);

  ApparentWindCartToSph(&apparent_wind_b_filtered, apparent_wind_filtered);
}

// There are two methods of estimating airspeed and four methods of
// estimating angle-of-attack and sideslip.  Airspeed may be measured
// directly with a pitot tube or be estimated by combining a ground
// wind speed measurement and an inertial velocity estimate.
// Likewise, the apparent wind angles may be measured directly with
// the pitot tube or be estimated by combining the ground wind speed
// measurement, an inertial velocity estimate, *and* an attitude
// estimate.  The third method of estimating angle-of-attack and
// sideslip assumes that we know the aerodynamic properties of the
// vehicle and calculates the apparent wind angles that are necessary
// to create the forces measured by the bridle loadcells and the IMU.
// Finally, the fourth method of estimating angle-of-attack and
// sideslip is simply to assume that when you are flying crosswind,
// your angle-of-attack and sideslip are near their nominal flight
// values.
//
// At high airspeeds and normal angles (< 15-25 deg), we use the pitot
// estimate (or the loadcell estimate if the pitot has faulted).  At
// low airspeeds, large angles, or if both the pitot and loadcell
// based estimates are faulted, we use the wind/vel/attitude based
// estimate.
void EstimatorApparentWindStep(
    const PitotData pitots[], const TetherForceEstimate *tether_force_b,
    const WindEstimate *wind_g, const Mat3 *dcm_g2b, const Vec3 *pqr,
    const Vec3 *acc, const Vec3 *Ab_f, const Vec3 *Vg, const FaultMask faults[],
    const SystemParams *system_params,
    const EstimatorApparentWindParams *params,
    EstimatorApparentWindState *state, ApparentWindEstimate *apparent_wind,
    ApparentWindSph *apparent_wind_pitot) {
  assert(pitots != NULL && tether_force_b != NULL && wind_g != NULL &&
         dcm_g2b != NULL && pqr != NULL && acc != NULL && Vg != NULL &&
         faults != NULL && system_params != NULL && params != NULL &&
         state != NULL && apparent_wind != NULL);
  // Use our current inertial velocity and wind sensor data to form
  // an estimate.

  // Ignore the vertical component of the wind sensor measurement.
  // TODO: This code does not check wind_g->valid.
  const Vec3 wind_g_xy = {wind_g->vector.x, wind_g->vector.y, 0.0};

  Vec3 V_app_est_g;
  Vec3Sub(&wind_g_xy, Vg, &V_app_est_g);

  Vec3 V_app_est_b;
  Mat3Vec3Mult(dcm_g2b, &V_app_est_g, &V_app_est_b);

  ApparentWindSph apparent_wind_est;
  ApparentWindCartToSph(&V_app_est_b, &apparent_wind_est);

  // Other apparent wind calculations.
  ApparentWindSph apparent_wind_tether;
  TetherForceToApparentWindSph(&tether_force_b->vector, acc,
                               apparent_wind_est.v, g_cont.simple_aero_model,
                               &apparent_wind_tether);

  bool use_pitot = CalcPitotApparentWindSph(
      pitots, pqr, &faults[SUBSYS_PITOT_SENSOR_HIGH_SPEED],
      &faults[SUBSYS_PITOT_SENSOR_LOW_SPEED], &system_params->pitot, params,
      apparent_wind_pitot);

  // Construct an apparent wind estimate by applying a complementary
  // filter to the Pitot solution (slow part) and kite inertial
  // measurements (fast part) as a gust-rejection measure.
  Vec3 wind_b_slow;
  Vec3 Vb;
  Mat3Vec3Mult(dcm_g2b, &wind_g->vector_f_slow, &wind_b_slow);
  Mat3Vec3Mult(dcm_g2b, Vg, &Vb);
  ApparentWindSph apparent_wind_complementary;
  ComplementaryApparentWindFilter(apparent_wind_pitot, Ab_f, &Vb, pqr,
                                  &wind_b_slow, params, state,
                                  &apparent_wind_complementary);

  // Choose which estimate is going to be your flying-like-an-airplane
  // estimate based on the fault conditions.  The pitot tube provides
  // the primary estimate; the loadcells provide a fallback alpha and
  // beta; the nominal flight angles provide the next fallback.
  //
  // Before Change-ID Ifff94318 there was logic here to compare the
  // Pitot derived airspeed and angle-of-attack to the wind sensor and
  // inertial velocity for fault detection.  For first crosswind
  // flights we've decided that these schemes are too risky, and we
  // will only trigger the use of the loadcell estimated AOA / AOS if
  // communications with the pressure sensors is completely lost.
  //
  // TODO: After the first crosswind flights, revive the
  // apparent wind fault tolerance.
  ApparentWindSph apparent_wind_fly;
  if (use_pitot) {
    if (wind_g->valid) {
      // TODO: Implement logic to turn this off.
      apparent_wind_fly = apparent_wind_complementary;
      apparent_wind->solution_type =
          (int32_t)kApparentWindSolutionTypeComplementary;
    } else {
      apparent_wind_fly = *apparent_wind_pitot;
      apparent_wind->solution_type = (int32_t)kApparentWindSolutionTypePitot;
    }
  } else if (tether_force_b->valid) {
    apparent_wind_fly = apparent_wind_tether;
    apparent_wind->solution_type = (int32_t)kApparentWindSolutionTypeLoadcell;
  } else {
    // TODO: Ideally, alpha and beta should be set to the
    // nominal flight angles here.  What's the best way of
    // communicating the nominal flight angles from the crosswind
    // controller to here?
    apparent_wind_fly.v = apparent_wind_est.v;
    apparent_wind_fly.alpha = 0.0;
    apparent_wind_fly.beta = 0.0;
    apparent_wind->solution_type =
        (int32_t)kApparentWindSolutionTypeFixedAngles;
  }

  // If either of the estimates report low speeds or high angles, then
  // revert to apparent_wind_est.
  double c_vel =
      Crossfade(0.0, 1.0, fmin(apparent_wind_est.v, apparent_wind_fly.v),
                params->v_low, params->v_high);
  // The estimated angles and "flying" angles have different
  // thresholds because the estimated angles can be pretty far off.
  double c_ang_est = Crossfade(1.0, 0.0, fmax(fabs(apparent_wind_est.alpha),
                                              fabs(apparent_wind_est.beta)),
                               params->ang_est_low, params->ang_est_high);
  double c_ang_fly = Crossfade(1.0, 0.0, fmax(fabs(apparent_wind_fly.alpha),
                                              fabs(apparent_wind_fly.beta)),
                               params->ang_fly_low, params->ang_fly_high);

  double c_all = c_vel * c_ang_est * c_ang_fly;

  // Record "inertial-and-wind" or "mixed" solution types.
  if (c_all <= 0.0) {
    apparent_wind->solution_type =
        (int32_t)kApparentWindSolutionTypeInertialAndWind;
  } else if (c_all < 1.0) {
    apparent_wind->solution_type = (int32_t)kApparentWindSolutionTypeMixed;
  }

  apparent_wind->sph.v = Mix(apparent_wind_est.v, apparent_wind_fly.v, c_all);
  apparent_wind->sph.alpha =
      Mix(apparent_wind_est.alpha, apparent_wind_fly.alpha, c_all);
  apparent_wind->sph.beta =
      Mix(apparent_wind_est.beta, apparent_wind_fly.beta, c_all);

  ApparentWindSphToCart(&apparent_wind->sph, &apparent_wind->vector);
  FilterApparentWindSph(&apparent_wind->sph, params, &state->sph_f_z1,
                        &apparent_wind->sph_f);

  // Update telemetry.
  EstimatorTelemetry *est = GetEstimatorTelemetry();
  est->apparent_wind_est = apparent_wind_est;
  est->apparent_wind_tether = apparent_wind_tether;
  est->apparent_wind_pitot = *apparent_wind_pitot;
  est->apparent_wind_cf = apparent_wind_complementary;
}
