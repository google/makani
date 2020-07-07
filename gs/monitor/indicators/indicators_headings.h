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

#ifndef GS_MONITOR_INDICATORS_INDICATORS_HEADINGS_H_
#define GS_MONITOR_INDICATORS_INDICATORS_HEADINGS_H_

#include <stdint.h>

#include "gs/monitor/widgets/indicator.h"

#ifdef __cplusplus
extern "C" {
#endif

void UpdateHeadingAio(Indicator *ind, int32_t init);
void UpdateHeadingArbitration(Indicator *ind, int32_t init);
void UpdateHeadingAvionics(Indicator *ind, int32_t init);
void UpdateHeadingBottomMotors(Indicator *ind, int32_t init);
void UpdateHeadingFlightController(Indicator *ind, int32_t init);
void UpdateHeadingGroundStation(Indicator *ind, int32_t init);
void UpdateHeadingGsCompass(Indicator *ind, int32_t init);
void UpdateHeadingGsGps(Indicator *ind, int32_t init);
void UpdateHeadingImu(Indicator *ind, int32_t init);
void UpdateHeadingPlatform(Indicator *ind, int32_t init);
void UpdateHeadingRedundantImus(Indicator *ind, int32_t init);
void UpdateHeadingServos(Indicator *ind, int32_t init);
void UpdateHeadingTopMotors(Indicator *ind, int32_t init);
void UpdateHeadingWinch(Indicator *ind, int32_t init);
void UpdateHeadingWind(Indicator *ind, int32_t init);
void UpdateHeadingWing(Indicator *ind, int32_t init);
void UpdateHeadingWingGpsA(Indicator *ind, int32_t init);
void UpdateHeadingWingGpsB(Indicator *ind, int32_t init);
void UpdateHeadingWingSensors(Indicator *ind, int32_t init);
void UpdateHeadingWingState(Indicator *ind, int32_t init);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_INDICATORS_INDICATORS_HEADINGS_H_
