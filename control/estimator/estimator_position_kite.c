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

#include "control/estimator/estimator_position_kite.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/filter.h"
#include "common/c_math/kalman.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_position_baro.h"
#include "control/estimator/estimator_position_filter_kite.h"
#include "control/estimator/estimator_position_glas.h"
#include "control/estimator/estimator_position_gps.h"
#include "control/ground_frame.h"
#include "control/perch_frame.h"
#include "control/sensor_types.h"
#include "control/system_params.h"
#include "control/system_types.h"

bool EstimatorPositionKiteValidateParams(
    const EstimatorPositionParams *params) {
  assert(params != NULL);

  return EstimatorPositionBaroValidateParams(&params->baro) &&
         EstimatorPositionFilterValidateParams(&params->filter) &&
         EstimatorPositionGlasValidateParams(&params->glas) &&
         EstimatorPositionGpsValidateParams(&params->gps);
}

void EstimatorPositionKiteInit(const EstimatorPositionParams *params,
                               EstimatorPositionState *state) {
  assert(params != NULL && state != NULL);
  assert(EstimatorPositionKiteValidateParams(params));
  memset(state, 0, sizeof(*state));

  // On startup, default to the Hover receiver, as it is expected to perform
  // better in the initial (hover) flight modes.
  state->last_center_gps_receiver = kWingGpsReceiverHover;

  EstimatorPositionBaroInit(&params->baro, &state->baro);
  EstimatorPositionFilterInit(&params->filter, &state->filter);
  EstimatorPositionGlasInit(&state->glas);
  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    EstimatorPositionGpsInit(&state->gps[i]);
  }
}

// Factor in GS GPS sigma to position sigma if the kite has an RTK solution.
static void AddGsGpsSigma(const GpsData *gps_data, const GsGpsData *gs_gps_data,
                          const FaultMask faults[],
                          EstimatorPositionGpsEstimate *gps) {
  if (HasAnyFault(&faults[kSubsysGsGpsPos])) {
    // Don't make an adjustment is GS GPS is faulted.
    return;
  }

  double gs_axis_sigma;

  switch (gps_data->pos_sol_type) {
    case kGpsSolutionTypeRtkFloat:
    case kGpsSolutionTypeRtkInt:
    case kGpsSolutionTypeRtkIonoFreeFloat:
    case kGpsSolutionTypeRtkNarrowFloat:
    case kGpsSolutionTypeRtkNarrowInt:
    case kGpsSolutionTypeRtkWideInt:
      // Divide by sqrt(3) to produce an average per-axis sigma and preserve
      // the overall sigma magnitude.
      gs_axis_sigma = gs_gps_data->pos_sigma * sqrt(1.0 / 3.0);
      gps->sigma_Xg.x = hypot(gps->sigma_Xg.x, gs_axis_sigma);
      gps->sigma_Xg.y = hypot(gps->sigma_Xg.y, gs_axis_sigma);
      gps->sigma_Xg.z = hypot(gps->sigma_Xg.z, gs_axis_sigma);
      break;
    case kGpsSolutionTypeNone:
    case kGpsSolutionTypeDifferential:
    case kGpsSolutionTypeStandAlone:
      // Non RTK solution type: do nothing.
      break;
    case kGpsSolutionTypeFixedHeight:
    case kGpsSolutionTypeFixedPosition:
    case kGpsSolutionTypeUnsupported:
    case kGpsSolutionTypeForceSigned:
    case kNumGpsSolutionTypes:
    default:
      // Invalid solution type.
      assert(false);
      break;
  }
}

void EstimatorPositionKiteStep(
    const Vec3 *acc, const Vec3 *pqr, const Mat3 *dcm_g2b,
    const GpsData wing_gps[], const GsGpsData *gs_gps,
    const GroundStationPoseEstimate *ground_station,
    const EncodersEstimate *encoders, const PerchAziEstimate *perch_azi,
    const TetherForceEstimate *tether_force_b, const WinchEstimate *winch,
    const PitotData pitots[], FlightMode flight_mode, const FaultMask faults[],
    const SystemParams *system_params, const EstimatorPositionParams *params,
    EstimatorPositionState *state, Vec3 *Xg, Vec3 *Vg,
    EstimatorVelocitySolutionType *vel_type) {
  assert(acc != NULL && pqr != NULL && dcm_g2b != NULL && wing_gps != NULL &&
         gs_gps != NULL && ground_station != NULL && winch != NULL &&
         perch_azi != NULL && tether_force_b != NULL && encoders != NULL &&
         pitots != NULL && faults != NULL && system_params != NULL &&
         params != NULL && state != NULL && Xg != NULL && Vg != NULL);

  EstimatorPositionBaroEstimate baro;
  EstimatorPositionBaroStep(
      dcm_g2b, pitots, &faults[kSubsysPitotSensorHighSpeedStatic],
      &faults[kSubsysPitotSensorLowSpeedStatic], &system_params->pitot,
      &params->baro, &state->baro, &baro);

  EstimatorPositionGlasEstimate glas;
  EstimatorPositionGlasStep(encoders, perch_azi, tether_force_b, winch,
                            system_params, &params->glas, &state->glas, &glas);

  EstimatorPositionGpsEstimate gps_estimates[kNumWingGpsReceivers];
  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    EstimatorPositionGpsStep(&wing_gps[i], ground_station, dcm_g2b, pqr,
                             GetWingGpsPosFault(faults, i),
                             GetWingGpsVelFault(faults, i),
                             &system_params->wing_gps[i], &params->gps,
                             &state->gps[i], &gps_estimates[i]);
  }

  state->last_center_gps_receiver = EstimatorPositionGpsSelectCenterEstimate(
      flight_mode, state->last_center_gps_receiver, gps_estimates,
      &state->filter.pos_g, &params->gps);
  const EstimatorPositionGpsEstimate *gps_center =
      &gps_estimates[state->last_center_gps_receiver];

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    AddGsGpsSigma(&wing_gps[i], gs_gps, faults, &gps_estimates[i]);
  }

  EstimatorPositionCorrections correct;
  memset(&correct, 0, sizeof(correct));

  EstimatorPositionFilterKiteStep(dcm_g2b, acc, &baro, &glas, gps_center,
                                  &gps_estimates[kWingGpsReceiverPort],
                                  &gps_estimates[kWingGpsReceiverStar],
                                  flight_mode, &params->filter, &state->filter,
                                  &correct, Xg, Vg, vel_type);

  // When perched, bias the barometric altitude using GPS.
  // TODO: Integrate differential pressure measurement using
  // the ground station's weather station.
  if (flight_mode == kFlightModePerched && gps_center->wing_pos_valid) {
    EstimatorPositionBaroUpdateBias(gps_center->Xg.z, gps_center->sigma_Xg.z,
                                    &baro, &params->baro, &state->baro);
  }

  // Bias the GSG angles to track the current position.  This smooths
  // the transition between position estimates during a GPS dropout.
  if (flight_mode == kFlightModeCrosswindNormal) {
    EstimatorPositionGlasUpdateGsgBias(
        Xg, encoders, perch_azi, tether_force_b, winch, &system_params->gsg,
        &system_params->winch, &params->glas, &state->glas);
  }

  // Update telemetry.
  EstimatorTelemetry *est = GetEstimatorTelemetry();
  est->baro = baro;
  est->pos_baro_state = state->baro;
  est->glas = glas;
  est->gsg_bias = state->glas.gsg_bias;
  est->position_corrections = correct;
  est->current_gps_receiver = state->last_center_gps_receiver;
  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    est->gps[i] = gps_estimates[i];
  }
}
