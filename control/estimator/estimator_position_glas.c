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

#include "control/estimator/estimator_position_glas.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/common.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/perch_frame.h"
#include "control/sensor_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

bool EstimatorPositionGlasValidateParams(
    const EstimatorPositionGlasParams *params) {
  assert(params != NULL);

  if (params->min_relative_sigma_pos_g <= 0.0) {
    assert(!(bool)"min_relative_sigma_pos_g must be positive.");
    return false;
  }

  if (params->sigma_per_weight_over_tension_ratio <= 0.0) {
    assert(!(bool)"sigma_per_weight_over_tension_ratio must be positive.");
    return false;
  }

  if (params->max_weight_over_tension_ratio <= 0.0 ||
      params->max_weight_over_tension_ratio > 0.5) {
    assert(!(bool)"max_weight_over_tension_ratio is out of range.");
    return false;
  }

  if (params->gsg_bias_fc <= 0.0 || params->gsg_bias_fc >= 0.2) {
    assert(!(bool)"gsg_bias_fc out of range.");
    return false;
  }

  if (params->gsg_bias_tension_lim <= 0.0) {
    assert(!(bool)"gsg_bias_tension_lim must be positive.");
    return false;
  }

  if (params->bias_low.azi > 0.0 || params->bias_low.ele > 0.0) {
    assert(!(bool)"bias_low entries must be non-positive.");
    return false;
  }

  if (params->bias_high.azi < 0.0 || params->bias_high.ele < 0.0) {
    assert(!(bool)"bias_high entries must be non-negative.");
    return false;
  }

  return true;
}

void EstimatorPositionGlasInit(EstimatorPositionGlasState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));

  state->last_valid_wing_pos_g = kVec3Zero;
  state->gsg_bias.azi = 0.0;
  state->gsg_bias.ele = 0.0;
}

void EstimatorPositionGlasUpdateGsgBias(
    const Vec3 *Xg, const EncodersEstimate *encoders,
    const PerchAziEstimate *perch_azi,
    const TetherForceEstimate *tether_force_b, const WinchEstimate *winch,
    const GsgParams *gsg_params, const WinchParams *winch_params,
    const EstimatorPositionGlasParams *params,
    EstimatorPositionGlasState *state) {
  assert(Xg != NULL && encoders != NULL && perch_azi != NULL &&
         tether_force_b != NULL && winch != NULL && gsg_params != NULL &&
         winch_params != NULL && params != NULL && state != NULL);

  if (GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
      GetSystemParams()->gs_model == kGroundStationModelTopHat) {
    if (perch_azi->valid && encoders->gsg_azi_valid &&
        encoders->gsg_ele_valid && winch->valid && tether_force_b->valid &&
        tether_force_b->tension_f > params->gsg_bias_tension_lim) {
      // Invert the position into GSG angles.
      Vec3 Xp;
      GToP(Xg, perch_azi->angle, &Xp);
      GsgData gsg_est;
      XpToGsg(&Xp, WinchPosToDrumAngle(winch->position, winch_params),
              &gsg_est);

      Lpf(encoders->gsg.azi - gsg_est.azi, params->gsg_bias_fc, *g_sys.ts,
          &state->gsg_bias.azi);
      Lpf(encoders->gsg.ele - gsg_est.ele, params->gsg_bias_fc, *g_sys.ts,
          &state->gsg_bias.ele);

      state->gsg_bias.azi = Saturate(state->gsg_bias.azi, params->bias_low.azi,
                                     params->bias_high.azi);
      state->gsg_bias.ele = Saturate(state->gsg_bias.ele, params->bias_low.ele,
                                     params->bias_high.ele);
    }
  }
}

// Calculate the ratio of the tether weight (assuming full payout)
// over the tension with guards against division by zero.
static double CalcWeightOverTensionRatio(
    double tension, const EstimatorPositionGlasParams *params) {
  // TODO: This code could easily be extended to a vary with payout
  // if we add some safety checks.
  double tether_weight =
      g_sys.tether->length * g_sys.tether->linear_density * g_sys.phys->g;

  if (tether_weight >= params->max_weight_over_tension_ratio * tension) {
    return params->max_weight_over_tension_ratio;
  } else {
    return fmin(tether_weight / tension, params->max_weight_over_tension_ratio);
  }
}

static double AsinhXOverX(double x) {
  // If x is near zero, evaluate the first few terms of the Taylor
  // expansion to avoid division by zero.
  if (fabs(x) < 0.01) {
    double x_squared = x * x;
    return 1.0 - x_squared * (1.0 / 6.0) + x_squared * x_squared * (3.0 / 40.0);
  } else {
    return asinh(x) / x;
  }
}

// Estimate the position of the kite in the catenary frame.
//
// This implementation assumes that the kite is supporting the entire
// weight of the tether which requires ele >= 0.0 and weight_over_tension_ratio
// < 1.0.
//
// The catenary frame is identical to the GSG frame when GSG elevation
// is zero.
static void CalcXcat(double ele, double weight_over_tension_ratio,
                     double length, Vec3 *X_cat) {
  assert(0.0 <= ele && ele <= PI / 2.0);
  assert(0.0 < weight_over_tension_ratio && weight_over_tension_ratio < 1.0);
  assert(length > 0.0);

  // Approximate the angle the tension vector makes at the kite.
  //
  //             tension
  //                    ^    ^
  //                     \   |
  //                      \  |
  //      ele + delta   ,' \ |
  //                   .    \|
  //                    <----o
  //                         `
  //                          `
  //                          ,`
  //                      ele ;  `.
  //                               o
  //                               |
  //                               v -vertical_tension_at_gs
  //
  // The static catenary assumption yields:
  //
  //   tension * sin(ele + delta) - tension_gs * sin(ele) = tether_weight, (1)
  //
  //   tension * cos(ele + delta) = tension_gs * cos(ele).                 (2)
  //
  // From this we have:
  //
  //   sin(delta) = tether_weight * cos(ele) / tension.                    (3)
  //
  double delta = Asin(cos(ele) * weight_over_tension_ratio);

  // Assuming the catenary is static in a vertical plane, the position along
  // the ground is given by:
  //
  // x = tether_length * cos(ele + delta) * (tension / tether_weight) * (
  //         arcsinh(tan(ele + delta)) - arcsinh(tan(ele))),
  //
  //   = tether_length * cos(ele + delta) * (tension / tether_weight) * (
  //         arcsinh((sin(ele + delta) - sin(ele))
  //                  / cos(ele + delta) / cos(ele))).
  //
  // This is further simplified to give the formula below.
  //
  // When 0 <= ele <= PI / 2.0 and tether_weight < tension:
  //
  //     a. ele < ele + delta <= PI / 2.0, and
  //
  //     b. cos(delta) - sin(ele) * tether_weight / tension monotonically
  //        decreases with ele.
  //
  // We use these facts to guard against division by zero.
  double denominator = fmax(1.0 - weight_over_tension_ratio,
                            cos(delta) - weight_over_tension_ratio * sin(ele));

  double c = (1.0 + 1.0 / denominator) / fmax(1.0, 1.0 + cos(delta));

  X_cat->x = cos(ele + delta) * c * AsinhXOverX(c * weight_over_tension_ratio);

  X_cat->y = 0.0;

  // The formula for vertical position is given by:
  //
  // z = -tether_length * (tether_weight / tension) * (
  //     hypot(tension * sin(ele + delta), tension * cos(ele + delta))
  //     - hypot(tether_weight - tension * sin(ele + delta),
  //             tension * cos(ele + delta))).
  //
  // This is further simplified to give the formula below.
  X_cat->z = -sin(ele) -
             weight_over_tension_ratio * cos(ele) * cos(ele) /
                 fmax(1.0, 1.0 + cos(delta));

  Vec3Scale(X_cat, length, X_cat);
}

// Calculate line-angle sensor position measurement.
static bool CalcXgGlas(const GsgData *gsg, double perch_azi, double tension,
                       double drum_angle,
                       const EstimatorPositionGlasParams *params, Vec3 *Xg) {
  double length = g_sys.tether->length + g_sys.wing->bridle_rad;
  double weight_over_tension_ratio =
      CalcWeightOverTensionRatio(tension, params);

  Vec3 X_gsg;
  bool valid;
  // The GLAS estimate is currently restricted to be used only when
  // the GSG elevation angle is positive and the tether tension
  // exceeds the weight of the tether.
  if (gsg->ele >= 0.0 &&
      weight_over_tension_ratio < params->max_weight_over_tension_ratio) {
    // Temporarily store X_cat in X_gsg.
    CalcXcat(gsg->ele, weight_over_tension_ratio, length, &X_gsg);

    // Rotate the catenary vector above into the GSG frame.
    Mat3 dcm;
    AngleToDcm(0.0, gsg->ele, 0.0, kRotationOrderZyx, &dcm);
    Mat3Vec3Mult(&dcm, &X_gsg, &X_gsg);

    valid = true;
  } else {
    X_gsg.x = length;
    X_gsg.y = 0.0;
    X_gsg.z = 0.0;
    valid = false;
  }

  Vec3 Xp;
  XgsgToXp(gsg, &X_gsg, drum_angle, &Xp);
  PToG(&Xp, perch_azi, Xg);

  return valid;
}

// TODO: Prior to go/mkcl/11645, an estimation scheme was
// implemented here that attempted to estimate uncertainty in the GLAS
// estimate.  This should be revisited.
void EstimatorPositionGlasStep(const EncodersEstimate *encoders,
                               const PerchAziEstimate *perch_azi,
                               const TetherForceEstimate *tether_force_b,
                               const WinchEstimate *winch,
                               const SystemParams *system_params,
                               const EstimatorPositionGlasParams *params,
                               EstimatorPositionGlasState *state,
                               EstimatorPositionGlasEstimate *glas) {
  assert(encoders != NULL && perch_azi != NULL && tether_force_b != NULL &&
         winch != NULL && system_params != NULL && params != NULL &&
         state != NULL && glas != NULL);

  double drum_angle =
      WinchPosToDrumAngle(winch->position, &system_params->winch);

  glas->wing_pos_valid = false;
  if (GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
      GetSystemParams()->gs_model == kGroundStationModelTopHat) {
    // TODO: Make GLAS work for GSv2.
    if (encoders->gsg_azi_valid && encoders->gsg_ele_valid &&
        perch_azi->valid && tether_force_b->valid && winch->valid &&
        !IsLevelwindEngaged(drum_angle, &system_params->levelwind)) {
      GsgData gsg_biased = {encoders->gsg.azi - state->gsg_bias.azi,
                            encoders->gsg.ele - state->gsg_bias.ele};
      glas->wing_pos_valid =
          CalcXgGlas(&gsg_biased, perch_azi->angle, tether_force_b->tension_f,
                     drum_angle, params, &state->last_valid_wing_pos_g);
    }
  }
  glas->Xg = state->last_valid_wing_pos_g;

  // This GLAS uncertainty model is based on the position error of a
  // straight-line estimate of the kite's position compared to the
  // actual position using a catenary model.
  double weight_over_tension_ratio =
      CalcWeightOverTensionRatio(tether_force_b->tension_f, params);

  double sigma =
      g_sys.tether->length * fmax(params->min_relative_sigma_pos_g,
                                  params->sigma_per_weight_over_tension_ratio *
                                      weight_over_tension_ratio);
  glas->sigma_Xg.x = sigma;
  glas->sigma_Xg.y = sigma;
  glas->sigma_Xg.z = sigma;
}
