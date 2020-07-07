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

#include "avionics/common/tether_cvt.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/controller_arbitration.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_avionics_messages.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_cvt_op.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/common/tether_op.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "common/macros.h"
#include "system/labels.h"
#include "system/labels_util.h"

static bool ValidateControllerCommandMessage(
    const ControllerCommandMessage *in) {
  return isfinite(in->detwist_position);
}

static void QueryNextNodeStatus(uint16_t frame_index, TetherDownCvtState *state,
                                TetherNodeStatus *out) {
  // Cycle through wing AioNodes for each long range radio transmission.
  uint8_t node = out->node;
  if (frame_index % TETHER_NODE_STATUS_DECIMATION == 0) {
    node = (uint8_t)((node + 1U) % ARRAYSIZE(state->node_status));
  }
  while (!IsWingNode(node)) {
    node = (uint8_t)((node + 1U) % ARRAYSIZE(state->node_status));
  }
  TetherNodeStatusToOutputTetherNodeStatus(
      (AioNode)node, &state->node_status[node], out);
}

void TetherDownCvtInit(int64_t now_usec, TetherDownCvtState *ts) {
  memset(ts, 0, sizeof(*ts));

  // Timestamps.
  ts->gps_time = INT32_MIN;

  // Operator command reply queue.
  TetherOpInit(&ts->command_reply);

  // Controller arbitration input state.
  ControllerArbitrationInit(now_usec, &ts->controller_arbitration);
  ts->lead_controller = kControllerA;

  // Node status cache.
  for (int32_t i = 0; i < ARRAYSIZE(ts->node_status); ++i) {
    ts->node_status[i].no_update_count = INT32_MAX;
  }
}

void TetherDownCvtQuery(int64_t now_usec, uint16_t frame_index,
                        TetherDownCvtState *ts, TetherDownMessage *out) {
  out->frame_index = frame_index;

  TetherCvtQueryBatteryStatusMessage(kAioNodeBattA,
                                     &ts->node_status[kAioNodeBattA],
                                     &out->batt_a);

  TetherCvtQueryBatteryStatusMessage(kAioNodeBattB,
                                     &ts->node_status[kAioNodeBattB],
                                     &out->batt_b);

  TetherCvtQueryControllerCommandMessage(
      now_usec, &ts->controller_arbitration, &ts->lead_controller,
      &out->control_command);

  for (int32_t i = 0; i < kNumFlightComputers; ++i) {
    AioNode aio_node =
        FlightComputerLabelToFlightComputerAioNode((FlightComputerLabel)i);
    TetherCvtQueryFlightComputerSensor(aio_node, &ts->node_status[aio_node],
                                       &out->flight_computers[i]);
  }

  TetherCvtQueryGpsTimeMessage(now_usec, kAioNodeFcB, &ts->gps_time,
                               &out->gps_time);

  for (int32_t i = 0; i < kNumLights; ++i) {
    AioNode aio_node = LightLabelToLightAioNode((LightLabel)i);
    TetherCvtQueryFlightComputerSensor(aio_node, &ts->node_status[aio_node],
                                       NULL);
  }

  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    AioNode aio_node =
        LoadcellNodeLabelToLoadcellNodeAioNode((LoadcellNodeLabel)i);
    TetherCvtQueryLoadcellMessage(aio_node, &ts->node_status[aio_node],
                                  &out->release_statuses[i]);
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    AioNode aio_node = MotorLabelToMotorAioNode(i);
    TetherCvtQueryMotorStatusMessage(
        aio_node, &ts->node_status[aio_node], &out->motor_statuses[i]);
  }

  TetherCvtQueryMvlvStatusMessage(kAioNodeMvlv,
                                  &ts->node_status[kAioNodeMvlv],
                                  &out->mvlv);

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    WingGpsReceiverLabel label = (WingGpsReceiverLabel)i;
    AioNode aio_node = WingGpsReceiverLabelToAioNode(label);
    GpsReceiverType type = WingGpsReceiverLabelToGpsReceiverType(label);
    TetherGpsStatus *gps_status = &out->gps_statuses[i];

    switch (type) {
      case kGpsReceiverTypeNovAtel:
        TetherCvtQueryNovAtelSolutionMessage(aio_node, gps_status, NULL);
        break;
      case kGpsReceiverTypeSeptentrio:
        TetherCvtQuerySeptentrioSolutionMessage(aio_node, gps_status);
        break;
      case kGpsReceiverTypeNone:
      case kGpsReceiverTypeForceSigned:
      case kGpsReceiverTypeForceSize:
      case kNumGpsReceiverTypes:
      default:
        assert(false);
        break;
    }
  }

  for (int32_t i = 0; i < kNumRecorderTms570s; ++i) {
    AioNode aio_node = RecorderTms570LabelToRecorderTms570AioNode(i);
    TetherCvtQueryRecorderStatusMessage(aio_node, &ts->node_status[aio_node]);
  }

  for (int32_t i = 0; i < kNumShortStacks; ++i) {
    AioNode aio_node = ShortStackLabelToShortStackAioNode(i);
    TetherCvtQueryShortStackStatusMessage(aio_node, &ts->node_status[aio_node]);
  }

  for (int32_t i = 0; i < ARRAYSIZE(ts->node_status); ++i) {
    TetherCvtQuerySelfTestMessage((AioNode)i, &ts->node_status[i]);
  }

  for (int32_t i = 0; i < kNumServos; ++i) {
    AioNode aio_node = ServoLabelToServoAioNode(i);
    TetherCvtQueryServoStatusMessage(
        aio_node, &ts->node_status[aio_node], &out->servo_statuses[i]);
  }

  for (int32_t i = 0; i < ARRAYSIZE(ts->node_status); ++i) {
    TetherCvtQuerySlowStatusMessage((AioNode)i, &ts->node_status[i]);
  }

  TetherCvtQuerySmallControlTelemetryMessage(
      ControllerLabelToControllerAioNode(ts->lead_controller),
      &out->control_telemetry);

  QueryNextNodeStatus(frame_index, ts, &out->node_status);
}

void TetherUpCvtInit(TetherUpCvtState *ts) {
  memset(ts, 0, sizeof(*ts));

  // Timestamps.
  ts->gps_time = INT32_MIN;

  // Operator command queue.
  TetherOpInit(&ts->operator_command);

  // GPS RTCM state.
  TetherUpCvtRtcmInit(&ts->rtcm);
}

void TetherUpCvtQuery(int64_t now_usec, uint16_t frame_index,
                      TetherUpCvtState *ts, TetherUpMessage *out) {
  out->frame_index = frame_index;

  TetherCvtQueryDrumSensorsMessage(kAioNodeDrumSensorsA, &out->drum_a);

  TetherCvtQueryDrumSensorsMessage(kAioNodeDrumSensorsB, &out->drum_b);

  TetherCvtQueryGpsTimeMessage(now_usec, kAioNodeGpsBaseStation,
                               &ts->gps_time, &out->gps_time);

  TetherCvtQueryGroundStationPlcStatusMessage(kAioNodePlcTophat, &out->plc);

  TetherCvtQueryGroundStationStatusMessage(kAioNodePlcGs02,
                                           &out->ground_station);

  TetherCvtQueryGroundStationWeatherMessage(
      kAioNodePlatformSensorsA, &out->wind, &out->weather);

  TetherCvtQueryJoystickStatusMessage(kAioNodeJoystickA, &out->joystick);

  TetherCvtQueryNovAtelCompassMessage(
      kAioNodeGpsBaseStation, &out->gps_compass);

  TetherCvtQueryNovAtelSolutionMessage(
      kAioNodeGpsBaseStation, &out->gps_status, &out->gps_position);

  TetherCvtQueryPlatformSensorsMessage(
      kAioNodePlatformSensorsA, &out->platform_a);

  TetherCvtQueryPlatformSensorsMessage(
      kAioNodePlatformSensorsB, &out->platform_b);
}

void TetherCvtQueryBatteryStatusMessage(AioNode source,
                                        TetherNodeStatus *node_status,
                                        TetherBatteryStatus *out) {
  BatteryStatusMessage in;
  if (CvtGetBatteryStatusMessage(source, &in, NULL, NULL)) {
    BatteryStatusMessageToTetherBatteryStatus(&in, node_status, out);
  } else {
    BatteryStatusMessageToTetherBatteryStatus(NULL, node_status, out);
  }
}

void TetherCvtQueryControllerCommandMessage(int64_t now_usec,
                                            ControllerArbitrationState *arb,
                                            ControllerLabel *lead_controller,
                                            TetherControlCommand *out) {
  ControllerArbitrationUpdateFromCvt(arb);
  const ControllerCommandMessage *in = ControllerArbitrationGetCommand(
      now_usec, ValidateControllerCommandMessage, arb, lead_controller);
  if (in != NULL) {
    ControllerCommandMessageToTetherControlCommand(
        *lead_controller, in, arb->sequence_numbers[*lead_controller], out);
  } else {
    ControllerCommandMessageToTetherControlCommand(
        *lead_controller, NULL, 0U, out);
  }
}

void TetherCvtQueryDrumSensorsMessage(AioNode source, TetherDrum *out) {
  DrumSensorsMessage in;
  uint16_t sequence;
  if (CvtGetDrumSensorsMessage(source, &in, &sequence, NULL)) {
    DrumSensorsMessageToTetherDrum(&in, sequence, out);
  } else {
    DrumSensorsMessageToTetherDrum(NULL, 0U, out);
  }
}

void TetherCvtQueryFlightComputerSensor(AioNode source,
                                        TetherNodeStatus *node_status,
                                        TetherFlightComputer *out) {
  FlightComputerSensorMessage in;
  if (CvtGetFlightComputerSensorMessage(source, &in, NULL, NULL)) {
    // Variable out is NULL when querying the light nodes.
    if (out != NULL) {
      FlightComputerSensorMessageToTetherFlightComputer(&in, out);
    }
    FlightComputerSensorMessageToTetherNodeStatus(&in, node_status);
  } else {
    if (out != NULL) {
      FlightComputerSensorMessageToTetherFlightComputer(NULL, out);
    }
    FlightComputerSensorMessageToTetherNodeStatus(NULL, node_status);
  }
}

void TetherCvtQueryGpsTimeMessage(int64_t now_usec, AioNode source,
                                  int64_t *last_usec, TetherGpsTime *out) {
  GpsTimeMessage in;
  int64_t timestamp_usec;
  if (CvtGetGpsTimeMessage(source, &in, NULL, &timestamp_usec)) {
    int64_t latency_usec = now_usec - timestamp_usec;
    GpsTimeMessageToTetherGpsTime(&in, latency_usec, out);
  } else {
    int64_t dt_usec = now_usec - *last_usec;
    GpsTimeMessageToTetherGpsTime(NULL, dt_usec, out);
  }
  *last_usec = now_usec;
}

void TetherCvtQueryGroundStationPlcStatusMessage(AioNode source,
                                                 TetherPlc *plc) {
  GroundStationPlcStatusMessage in;
  uint16_t sequence;
  if (CvtGetGroundStationPlcStatusMessage(source, &in, &sequence, NULL)) {
    GroundStationPlcStatusMessageToTetherPlc(&in, sequence, plc);
  } else {
    GroundStationPlcStatusMessageToTetherPlc(NULL, 0U, plc);
  }
}

void TetherCvtQueryGroundStationStatusMessage(
    AioNode source, TetherGroundStation *ground_station) {
  GroundStationStatusMessage in;
  uint16_t sequence;
  if (CvtGetGroundStationStatusMessage(source, &in, &sequence, NULL)) {
    GroundStationStatusMessageToTetherGroundStation(&in, sequence,
                                                    ground_station);
  } else {
    GroundStationStatusMessageToTetherGroundStation(NULL, 0U, ground_station);
  }
}

void TetherCvtQueryGroundStationWeatherMessage(AioNode source, TetherWind *wind,
                                               TetherWeather *weather) {
  GroundStationWeatherMessage in;
  uint16_t sequence;
  if (CvtGetGroundStationWeatherMessage(source, &in, &sequence, NULL)) {
    GroundStationWeatherMessageToTetherWind(&in, sequence, wind);
    GroundStationWeatherMessageToTetherWeather(&in, sequence, weather);
  } else {
    GroundStationWeatherMessageToTetherWind(NULL, 0U, wind);
    GroundStationWeatherMessageToTetherWeather(NULL, 0U, weather);
  }
}

void TetherCvtQueryJoystickStatusMessage(AioNode source, TetherJoystick *out) {
  JoystickStatusMessage in;
  uint16_t sequence;
  if (CvtGetJoystickStatusMessage(source, &in, &sequence, NULL)) {
    JoystickStatusMessageToTetherJoystick(&in, sequence, out);
  } else {
    JoystickStatusMessageToTetherJoystick(NULL, 0U, out);
  }
}

void TetherCvtQueryLoadcellMessage(AioNode source,
                                   TetherNodeStatus *node_status,
                                   TetherReleaseStatus *out) {
  LoadcellMessage in;
  if (CvtGetLoadcellMessage(source, &in, NULL, NULL)) {
    LoadcellMessageToTetherReleaseStatus(&in, node_status, out);
  } else {
    LoadcellMessageToTetherReleaseStatus(NULL, node_status, out);
  }
}

void TetherCvtQueryMotorStatusMessage(AioNode source,
                                      TetherNodeStatus *node_status,
                                      TetherMotorStatus *out) {
  MotorStatusMessage in;
  if (CvtGetMotorStatusMessage(source, &in, NULL, NULL)) {
    MotorStatusMessageUpdateTetherMotorStatus(&in, node_status, out);
  } else {
    MotorStatusMessageUpdateTetherMotorStatus(NULL, node_status, out);
  }
}

void TetherCvtQueryMvlvStatusMessage(AioNode source,
                                     TetherNodeStatus *node_status,
                                     TetherMvlvStatus *out) {
  MvlvStatusMessage in;
  if (CvtGetMvlvStatusMessage(source, &in, NULL, NULL)) {
    MvlvStatusMessageUpdateTetherMvlvStatus(&in, node_status, out);
  } else {
    MvlvStatusMessageUpdateTetherMvlvStatus(NULL, node_status, out);
  }
}

void TetherCvtQueryNovAtelCompassMessage(AioNode source,
                                         TetherGsGpsCompass *out) {
  NovAtelCompassMessage in;
  uint16_t sequence;
  if (CvtGetNovAtelCompassMessage(source, &in, &sequence, NULL)) {
    NovAtelCompassMessageToTetherGsGpsCompass(&in, sequence, out);
  } else {
    NovAtelCompassMessageToTetherGsGpsCompass(NULL, 0U, out);
  }
}

void TetherCvtQueryNovAtelSolutionMessage(AioNode source,
                                          TetherGpsStatus *status,
                                          TetherGsGpsPosition *position) {
  NovAtelSolutionMessage in;
  uint16_t sequence;
  if (CvtGetNovAtelSolutionMessage(source, &in, &sequence, NULL)) {
    if (status != NULL) {
      NovAtelSolutionMessageToTetherGpsStatus(&in, sequence, status);
    }
    if (position != NULL) {
      NovAtelSolutionMessageToTetherGpsPosition(&in, sequence, position);
    }
  } else {
    if (status != NULL) {
      NovAtelSolutionMessageToTetherGpsStatus(NULL, 0U, status);
    }
    if (position != NULL) {
      NovAtelSolutionMessageToTetherGpsPosition(NULL, 0U, position);
    }
  }
}

void TetherCvtQueryPlatformSensorsMessage(AioNode source, TetherPlatform *out) {
  PlatformSensorsMessage in;
  uint16_t sequence;
  if (CvtGetPlatformSensorsMessage(source, &in, &sequence, NULL)) {
    PlatformSensorsMessageToTetherPlatform(&in, sequence, out);
  } else {
    PlatformSensorsMessageToTetherPlatform(NULL, 0U, out);
  }
}

void TetherCvtQueryRecorderStatusMessage(AioNode source,
                                         TetherNodeStatus *node_status) {
  RecorderStatusMessage in;
  if (CvtGetRecorderStatusMessage(source, &in, NULL, NULL)) {
    RecorderStatusMessageUpdateTetherRecorderStatus(&in, node_status);
  } else {
    RecorderStatusMessageUpdateTetherRecorderStatus(NULL, node_status);
  }
}

void TetherCvtQuerySelfTestMessage(AioNode source,
                                   TetherNodeStatus *node_status) {
  SelfTestMessage in;
  if (CvtGetSelfTestMessage(source, &in, NULL, NULL)) {
    SelfTestMessageUpdateTetherNodeStatus(&in, node_status);
  } else {
    SelfTestMessageUpdateTetherNodeStatus(NULL, node_status);
  }
}

void TetherCvtQuerySeptentrioSolutionMessage(AioNode source,
                                             TetherGpsStatus *status) {
  SeptentrioSolutionMessage in[2];  // Silence stack protector warning w/ array.
  uint16_t sequence;
  if (CvtGetSeptentrioSolutionMessage(source, &in[0], &sequence, NULL)) {
    SeptentrioSolutionMessageToTetherGpsStatus(&in[0], sequence, status);
  } else {
    SeptentrioSolutionMessageToTetherGpsStatus(NULL, 0U, status);
  }
}

void TetherCvtQueryServoStatusMessage(AioNode source,
                                      TetherNodeStatus *node_status,
                                      TetherServoStatus *out) {
  ServoStatusMessage in;
  if (CvtGetServoStatusMessage(source, &in, NULL, NULL)) {
    ServoStatusMessageUpdateTetherServoStatus(&in, node_status, out);
  } else {
    ServoStatusMessageUpdateTetherServoStatus(NULL, node_status, out);
  }
}

void TetherCvtQueryShortStackStatusMessage(AioNode source,
                                           TetherNodeStatus *node_status) {
  ShortStackStatusMessage in;
  if (CvtGetShortStackStatusMessage(source, &in, NULL, NULL)) {
    ShortStackStatusMessageUpdateTetherShortStackStatus(&in, node_status);
  } else {
    ShortStackStatusMessageUpdateTetherShortStackStatus(NULL, node_status);
  }
}

void TetherCvtQuerySlowStatusMessage(AioNode source,
                                     TetherNodeStatus *node_status) {
  SlowStatusMessage in;
  if (CvtGetSlowStatusMessage(source, &in, NULL, NULL)) {
    SlowStatusMessageUpdateTetherNodeStatus(&in, node_status);
  } else {
    SlowStatusMessageUpdateTetherNodeStatus(NULL, node_status);
  }
}

void TetherCvtQuerySmallControlTelemetryMessage(AioNode source,
                                                TetherControlTelemetry *out) {
  SmallControlTelemetryMessage in;
  uint16_t sequence;
  if (CvtGetSmallControlTelemetryMessage(source, &in, &sequence, NULL)) {
    SmallControlTelemetryMessageToTetherControlTelemetry(&in, sequence, out);
  } else {
    SmallControlTelemetryMessageToTetherControlTelemetry(NULL, 0U, out);
  }
}

void TetherCvtGetTetherDownMergeInputs(TetherDownMergeState *state) {
  for (int32_t i = 0; i < ARRAYSIZE(state->input_messages); ++i) {
    AioNode aio_node = TetherDownSourceToAioNode((TetherDownSource)i);
    state->input_updated[i] = CvtGetTetherDownMessage(
        aio_node, &state->input_messages[i], &state->input_sequence_numbers[i],
        &state->input_timestamps[i]);
  }

  state->controller_updated = CvtGetControllerCommandMessage(
      kAioNodeOperator, &state->controller_message,
      &state->controller_sequence_number, &state->controller_timestamp);
}

void TetherCvtPeekTetherDownMergeInputs(TetherDownMergeState *state) {
  uint16_t sequence;
  int64_t timestamp;
  const uint8_t *buf;

  for (int32_t i = 0; i < ARRAYSIZE(state->input_messages); ++i) {
    buf = CvtPeek(TetherDownSourceToAioNode((TetherDownSource)i),
                  kMessageTypeTetherDown, &sequence, &timestamp);
    if (buf != NULL && sequence != state->input_sequence_numbers[i]) {
      UnpackTetherDownMessage(buf, 1, &state->input_messages[i]);
      state->input_sequence_numbers[i] = sequence;
      state->input_timestamps[i] = timestamp;
      state->input_updated[i] = true;
    } else {
      state->input_updated[i] = false;
    }
  }

  // ControllerCommand
  buf = CvtPeek(kAioNodeOperator, kMessageTypeControllerCommand, &sequence,
                &timestamp);
  if (buf != NULL && sequence != state->controller_sequence_number) {
    UnpackControllerCommandMessage(buf, 1, &state->controller_message);
    state->controller_sequence_number = sequence;
    state->controller_timestamp = timestamp;
    state->controller_updated = true;
  } else {
    state->controller_updated = false;
  }
}

void TetherCvtGetTetherUpMergeInputs(TetherUpMergeState *state) {
  // TetherUpMessages.
  for (int32_t i = 0; i < ARRAYSIZE(state->input_messages); ++i) {
    AioNode aio_node = TetherUpSourceToAioNode((TetherUpSource)i);
    state->input_updated[i] = CvtGetTetherUpMessage(
        aio_node, &state->input_messages[i], &state->input_sequence_numbers[i],
        &state->input_timestamps[i]);
  }

  // JoystickStatus.
  state->joystick_updated = CvtGetJoystickStatusMessage(
      kAioNodeJoystickA, &state->joystick_message,
      &state->joystick_sequence_number, &state->joystick_timestamp);
}

void TetherCvtPeekTetherUpMergeInputs(TetherUpMergeState *state) {
  uint16_t sequence;
  int64_t timestamp;
  const uint8_t *buf;

  // TetherUpMessages.
  for (int32_t i = 0; i < ARRAYSIZE(state->input_messages); ++i) {
    buf = CvtPeek(TetherUpSourceToAioNode((TetherUpSource)i),
                  kMessageTypeTetherUp, &sequence, &timestamp);
    if (buf != NULL && sequence != state->input_sequence_numbers[i]) {
      UnpackTetherUpMessage(buf, 1, &state->input_messages[i]);
      state->input_sequence_numbers[i] = sequence;
      state->input_timestamps[i] = timestamp;
      state->input_updated[i] = true;
    } else {
      state->input_updated[i] = false;
    }
  }

  // JoystickStatus.
  buf = CvtPeek(kAioNodeJoystickA, kMessageTypeJoystickStatus, &sequence,
                &timestamp);
  if (buf != NULL && sequence != state->joystick_sequence_number) {
    UnpackJoystickStatusMessage(buf, 1, &state->joystick_message);
    state->joystick_sequence_number = sequence;
    state->joystick_timestamp = timestamp;
    state->joystick_updated = true;
  } else {
    state->joystick_updated = false;
  }
}
