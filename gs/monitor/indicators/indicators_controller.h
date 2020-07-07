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

#ifndef GS_MONITOR_INDICATORS_INDICATORS_CONTROLLER_H_
#define GS_MONITOR_INDICATORS_INDICATORS_CONTROLLER_H_

#include <stdint.h>

#include "control/control_types.h"
#include "gs/monitor/widgets/indicator.h"

#ifdef __cplusplus
extern "C" {
#endif

void UpdateAccelStart(Indicator *ind, int32_t init);
void UpdateApparentWind(Indicator *ind, int32_t init);
void UpdateAutoglide(Indicator *ind, int32_t init);
void UpdateControlTime(Indicator *ind, int32_t init);
void UpdateControllerTiming(Indicator *ind, int32_t init);
void UpdateEstimatorAccBDiff(Indicator *ind, int32_t init);
void UpdateEstimatorCurrentGpsReceiver(Indicator *ind, int32_t init);
void UpdateEstimatorGpsDiff(Indicator *ind, int32_t init);
void UpdateEstimatorGsPosEcef(Indicator *ind, int32_t init);
void UpdateEstimatorGsgBias(Indicator *ind, int32_t init);
void UpdateEstimatorGyroBias(Indicator *ind, int32_t init);
void UpdateEstimatorGyroDiff(Indicator *ind, int32_t init);
void UpdateEstimatorMagnetometerDiff(Indicator *ind, int32_t init);
void UpdateEstimatorWinchDisagreement(Indicator *ind, int32_t init);
void UpdateFdAllActive(Indicator *ind, int32_t init);
void UpdateFdDisabled(Indicator *ind, int32_t init);
void UpdateFdGps(Indicator *ind, int32_t init);
void UpdateFdGsCompass(Indicator *ind, int32_t init);
void UpdateFdGsGps(Indicator *ind, int32_t init);
void UpdateFdGsgA(Indicator *ind, int32_t init);
void UpdateFdGsgB(Indicator *ind, int32_t init);
void UpdateFdImuA(Indicator *ind, int32_t init);
void UpdateFdImuB(Indicator *ind, int32_t init);
void UpdateFdImuC(Indicator *ind, int32_t init);
void UpdateFdJoystick(Indicator *ind, int32_t init);
void UpdateFdLevelwindEleA(Indicator *ind, int32_t init);
void UpdateFdLevelwindEleB(Indicator *ind, int32_t init);
void UpdateFdLoadcells(Indicator *ind, int32_t init);
void UpdateFdMotors(Indicator *ind, int32_t init);
void UpdateFdPerchAziA(Indicator *ind, int32_t init);
void UpdateFdPerchAziB(Indicator *ind, int32_t init);
void UpdateFdPitotHighSpeed(Indicator *ind, int32_t init);
void UpdateFdPitotLowSpeed(Indicator *ind, int32_t init);
void UpdateFdProximitySensor(Indicator *ind, int32_t init);
void UpdateFdServos(Indicator *ind, int32_t init);
void UpdateFdWinch(Indicator *ind, int32_t init);
void UpdateFdWindSensor(Indicator *ind, int32_t init);
void UpdateFlightMode(Indicator *ind, int32_t init);
void UpdateFlightModeGates(Indicator *ind, int32_t init);
void UpdateFlightPlan(Indicator *ind, int32_t init);
void UpdateFlightTimer(Indicator *ind, int32_t init);
void UpdateHeightAgl(Indicator *ind, int32_t init);
void UpdateHoverAngleCommand(Indicator *ind, int32_t init);
void UpdateHoverGainRampScale(Indicator *ind, int32_t init);
void UpdateHoverThrustMoment(Indicator *ind, int32_t init);
void UpdateJoystick(Indicator *ind, int32_t init);
void UpdateManualFlaps(Indicator *ind, int32_t init);
void UpdateManualState(Indicator *ind, int32_t init);
void UpdatePayout(Indicator *ind, int32_t init);
void UpdatePerchAzi(Indicator *ind, int32_t init);
void UpdateProximityFlagValid(Indicator *ind, int32_t init);
void UpdateProximitySensor(Indicator *ind, int32_t init);
void UpdateTensionComponents(Indicator *ind, int32_t init);
void UpdateTensionValid(Indicator *ind, int32_t init);
void UpdateThrottle(Indicator *ind, int32_t init);
void UpdateVersion(Indicator *ind, int32_t init);
void UpdateWinchPos(Indicator *ind, int32_t init);
void UpdateWind(Indicator *ind, int32_t init);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_INDICATORS_INDICATORS_CONTROLLER_H_
