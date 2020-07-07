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

#ifndef CONTROL_ESTIMATOR_ESTIMATOR_WEATHER_H_
#define CONTROL_ESTIMATOR_ESTIMATOR_WEATHER_H_

#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void EstimatorWeatherInit(const EstimatorWeatherParams *params,
                          EstimatorWeatherState *state);

void EstimatorWeatherStep(bool initializing, const GsWeather *weather,
                          const FaultMask *weather_station_fault,
                          const EstimatorWeatherParams *params,
                          EstimatorWeatherState *state, double *rho_estimate);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_ESTIMATOR_ESTIMATOR_WEATHER_H_
