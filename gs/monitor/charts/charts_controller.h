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

#ifndef GS_MONITOR_CHARTS_CHARTS_CONTROLLER_H_
#define GS_MONITOR_CHARTS_CHARTS_CONTROLLER_H_

#include <stdint.h>

#include "gs/monitor/widgets/indicator_chart.h"

#ifdef __cplusplus
extern "C" {
#endif

void UpdateAirspeed(IndicatorChart *ich, int32_t init);
void UpdateAltitude(IndicatorChart *ich, int32_t init);
void UpdateCrosswindDeltas(IndicatorChart *ich, int32_t init);
void UpdateEngageAltitude(IndicatorChart *ich, int32_t init);
void UpdateEstimatorAttitudeDiff(IndicatorChart *ich, int32_t init);
void UpdateEstimatorTetherForce(IndicatorChart *ich, int32_t init);
void UpdateHoverAngles(IndicatorChart *ich, int32_t init);
void UpdateHoverPathCommand(IndicatorChart *ich, int32_t init);
void UpdateHoverPathErrors(IndicatorChart *ich, int32_t init);
void UpdateHoverPositionErrors(IndicatorChart *ich, int32_t init);
void UpdateHoverTension(IndicatorChart *ich, int32_t init);
void UpdateHoverVelocityErrors(IndicatorChart *ich, int32_t init);
void UpdateHoverWindDir(IndicatorChart *ich, int32_t init);
void UpdatePitotAngles(IndicatorChart *ich, int32_t init);
void UpdateTension(IndicatorChart *ich, int32_t init);
void UpdateWindDir(IndicatorChart *ich, int32_t init);
void UpdateWingPos(IndicatorChart *ich, int32_t init);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_CHARTS_CHARTS_CONTROLLER_H_
