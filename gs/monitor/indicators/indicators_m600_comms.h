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

#ifndef GS_MONITOR_INDICATORS_INDICATORS_M600_COMMS_H_
#define GS_MONITOR_INDICATORS_INDICATORS_M600_COMMS_H_

#include <stdint.h>

#include "gs/monitor/widgets/indicator.h"

#ifdef __cplusplus
extern "C" {
#endif

void UpdateAioStatusUpdatedControllers(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedCoreSwitchGs(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedCoreSwitchWing(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedDrumSensors(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedFlightComputers(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedGsCompass(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedGsGps(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedJoystick(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedLoadcells(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedMotorsBottom(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedMotorsTop(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedPlatformSensors(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedSelfTest(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedServosPort(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedServosStar(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedServosTail(Indicator *ind, int32_t init);
void UpdateAioStatusUpdatedWinchPlc(Indicator *ind, int32_t init);
void UpdateMaxBoardTemperature(Indicator *ind, int32_t init);
void UpdateCsAPortTraffic(Indicator *ind, int32_t init);
void UpdateCsBPortTraffic(Indicator *ind, int32_t init);
void UpdateCsGsAPortTraffic(Indicator *ind, int32_t init);
void UpdateCsGsBPortTraffic(Indicator *ind, int32_t init);
void UpdateMotorsNetworkAErrors(Indicator *ind, int32_t init);
void UpdateMotorsNetworkBErrors(Indicator *ind, int32_t init);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_INDICATORS_INDICATORS_M600_COMMS_H_
