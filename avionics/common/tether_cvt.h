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

#ifndef AVIONICS_COMMON_TETHER_CVT_H_
#define AVIONICS_COMMON_TETHER_CVT_H_

#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/controller_arbitration.h"
#include "avionics/common/tether_cvt_rtcm.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/common/tether_op.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"

typedef struct {
  // Timestamps.
  int64_t gps_time;
  // Operator command reply queue.
  TetherOpQueue command_reply;
  // Controller arbitration input state.
  ControllerArbitrationState controller_arbitration;
  // Arbitrated leader of the controllers.
  ControllerLabel lead_controller;
  // Cache node status.
  TetherNodeStatus node_status[kNumAioNodes];
} TetherDownCvtState;

typedef struct {
  // Timestamps.
  int64_t gps_time;
  // Operator command queue.
  TetherOpQueue operator_command;
  // GPS RTCM state.
  TetherUpCvtRtcmState rtcm;
} TetherUpCvtState;

void TetherDownCvtInit(int64_t now_usec, TetherDownCvtState *ts);
void TetherDownCvtQuery(int64_t now_usec, uint16_t frame_index,
                        TetherDownCvtState *ts, TetherDownMessage *out);

void TetherUpCvtInit(TetherUpCvtState *ts);
void TetherUpCvtQuery(int64_t now_usec, uint16_t frame_index,
                      TetherUpCvtState *ts, TetherUpMessage *out);

void TetherCvtQueryBatteryStatusMessage(AioNode source,
                                        TetherNodeStatus *node_status,
                                        TetherBatteryStatus *out);

void TetherCvtQueryControllerCommandMessage(int64_t now_usec,
                                            ControllerArbitrationState *arb,
                                            ControllerLabel *lead_controller,
                                            TetherControlCommand *out);

void TetherCvtQueryDrumSensorsMessage(AioNode source, TetherDrum *out);

void TetherCvtQueryFlightComputerSensor(AioNode source,
                                        TetherNodeStatus *node_status,
                                        TetherFlightComputer *out);

void TetherCvtQueryGpsTimeMessage(int64_t now_usec, AioNode source,
                                  int64_t *last_usec, TetherGpsTime *out);

void TetherCvtQueryGroundStationPlcStatusMessage(AioNode source,
                                                 TetherPlc *plc);

void TetherCvtQueryGroundStationStatusMessage(
    AioNode source, TetherGroundStation *ground_station);

void TetherCvtQueryGroundStationWeatherMessage(AioNode source, TetherWind *wind,
                                               TetherWeather *weather);

void TetherCvtQueryJoystickStatusMessage(AioNode source, TetherJoystick *out);

void TetherCvtQueryLoadcellMessage(AioNode source,
                                   TetherNodeStatus *node_status,
                                   TetherReleaseStatus *out);

void TetherCvtQueryMotorStatusMessage(AioNode source,
                                      TetherNodeStatus *node_status,
                                      TetherMotorStatus *out);

void TetherCvtQueryMvlvStatusMessage(AioNode source,
                                     TetherNodeStatus *node_status,
                                     TetherMvlvStatus *out);

void TetherCvtQueryNovAtelCompassMessage(AioNode source,
                                         TetherGsGpsCompass *out);

void TetherCvtQueryNovAtelSolutionMessage(AioNode source,
                                          TetherGpsStatus *status,
                                          TetherGsGpsPosition *position);

void TetherCvtQueryPlatformSensorsMessage(AioNode source, TetherPlatform *out);

void TetherCvtQueryRecorderStatusMessage(AioNode source,
                                         TetherNodeStatus *node_status);

void TetherCvtQuerySelfTestMessage(AioNode source,
                                   TetherNodeStatus *node_status);

void TetherCvtQuerySeptentrioSolutionMessage(AioNode source,
                                             TetherGpsStatus *status);

void TetherCvtQueryServoStatusMessage(AioNode source,
                                      TetherNodeStatus *node_status,
                                      TetherServoStatus *out);

void TetherCvtQueryShortStackStatusMessage(AioNode source,
                                           TetherNodeStatus *node_status);

void TetherCvtQuerySlowStatusMessage(AioNode source,
                                     TetherNodeStatus *node_status);

void TetherCvtQuerySmallControlTelemetryMessage(AioNode source,
                                                TetherControlTelemetry *out);

void TetherCvtGetTetherDownMergeInputs(TetherDownMergeState *state);
void TetherCvtPeekTetherDownMergeInputs(TetherDownMergeState *state);

void TetherCvtGetTetherUpMergeInputs(TetherUpMergeState *state);
void TetherCvtPeekTetherUpMergeInputs(TetherUpMergeState *state);

#endif  // AVIONICS_COMMON_TETHER_CVT_H_
