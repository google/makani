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

#include "control/estimator/estimator_weather.h"

#include "common/c_math/filter.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"

void EstimatorWeatherInit(const EstimatorWeatherParams *params,
                          EstimatorWeatherState *state) {
  // Initialize state to reasonable values.
  state->last_valid_rho = g_sys.phys->rho;
  Lpf2Init(state->last_valid_rho, params->fc_rho, params->zeta_rho, *g_sys.ts,
           state->rho_zs);
}

void EstimatorWeatherStep(bool initializing, const GsWeather *weather,
                          const FaultMask *weather_station_fault,
                          const EstimatorWeatherParams *params,
                          EstimatorWeatherState *state, double *rho_estimate) {
  if (!HasAnyFault(weather_station_fault)) {
    bool rho_valid;
    double pending_rho = CalcAirDensity(weather->pressure, weather->temperature,
                                        weather->humidity, &rho_valid);
    if (rho_valid) {
      state->last_valid_rho = pending_rho;
    }
  }

  double rho = state->last_valid_rho;

  // Record the instantaneous (unfiltered) value in telemetry.
  GetEstimatorTelemetry()->rho_instantaneous = rho;

  // Saturate the measured density to a sensible range around the
  // hard-coded value.
  rho = Saturate(rho, 0.90 * g_sys.phys->rho, 1.10 * g_sys.phys->rho);

  // Apply filter.
  if (initializing) {
    Lpf2Init(rho, params->fc_rho, params->zeta_rho, *g_sys.ts, state->rho_zs);
  }
  rho = Lpf2(rho, params->fc_rho, params->zeta_rho, *g_sys.ts, state->rho_zs);

  *rho_estimate = rho;
}
