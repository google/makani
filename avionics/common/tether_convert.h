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

#ifndef AVIONICS_COMMON_TETHER_CONVERT_H_
#define AVIONICS_COMMON_TETHER_CONVERT_H_

#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/firmware/monitors/cs_types.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"

#ifdef __cplusplus
extern "C" {
#endif

// Status message functions.

// In all functions, set 'in' to NULL to mark the message as invalid.

void BatteryStatusMessageToTetherBatteryStatus(
    const BatteryStatusMessage *in, TetherNodeStatus *node_status,
    TetherBatteryStatus *out);

void ControllerCommandMessageToTetherControlCommand(
    ControllerLabel lead_controller, const ControllerCommandMessage *in,
    uint16_t sequence, TetherControlCommand *out);

// Argument long_range_rssi represents the received signal strength [dBm],
// which is a negative value. Set this value to zero to indicate no link.
void CoreSwitchStatsToTetherCommsStatus(
    AioNode source, const CoreSwitchStats *in, int16_t long_range_rssi,
    TetherCommsStatus *out);

void CsMonitorDataToTetherNodeStatus(
    const CsMonitorData *in, TetherNodeStatus *node_status);

void DrumSensorsMessageToTetherDrum(
    const DrumSensorsMessage *in, uint16_t sequence, TetherDrum *out);

void FlightComputerSensorMessageToTetherFlightComputer(
    const FlightComputerSensorMessage *in, TetherFlightComputer *out);

void FlightComputerSensorMessageToTetherNodeStatus(
    const FlightComputerSensorMessage *in, TetherNodeStatus *node_status);

void GpsTimeMessageToTetherGpsTime(
    const GpsTimeMessage *in, int64_t latency_usec, TetherGpsTime *out);

void GroundStationPlcStatusMessageToTetherPlc(
    const GroundStationPlcStatusMessage *in, uint16_t sequence,
    TetherPlc *out);

void GroundStationStatusMessageToTetherGroundStation(
    const GroundStationStatusMessage *in, uint16_t sequence,
    TetherGroundStation *out);

void GroundStationWeatherMessageToTetherWeather(
    const GroundStationWeatherMessage *in, uint16_t sequence,
    TetherWeather *out);

void GroundStationWeatherMessageToTetherWind(
    const GroundStationWeatherMessage *in, uint16_t sequence,
    TetherWind *out);

void JoystickStatusMessageToTetherJoystick(
    const JoystickStatusMessage *in, uint16_t sequence, TetherJoystick *out);

void LoadcellMessageToTetherReleaseStatus(
    const LoadcellMessage *in, TetherNodeStatus *node_status,
    TetherReleaseStatus *out);

void MotorStatusMessageUpdateTetherMotorStatus(
    const MotorStatusMessage *in, TetherNodeStatus *node_status,
    TetherMotorStatus *out);

void MvlvStatusMessageUpdateTetherMvlvStatus(
    const MvlvStatusMessage *in, TetherNodeStatus *node_status,
    TetherMvlvStatus *out);

void NovAtelCompassMessageToTetherGsGpsCompass(
    const NovAtelCompassMessage *in, uint16_t sequence,
    TetherGsGpsCompass *out);

void NovAtelSolutionMessageToTetherGpsPosition(
    const NovAtelSolutionMessage *in, uint16_t sequence,
    TetherGsGpsPosition *out);

void NovAtelSolutionMessageToTetherGpsStatus(
    const NovAtelSolutionMessage *in, uint16_t sequence, TetherGpsStatus *out);

void PlatformSensorsMessageToTetherPlatform(
    const PlatformSensorsMessage *in, uint16_t sequence, TetherPlatform *out);

void RecorderStatusMessageUpdateTetherRecorderStatus(
    const RecorderStatusMessage *in, TetherNodeStatus *node_status);

void SelfTestMessageUpdateTetherNodeStatus(
    const SelfTestMessage *in, TetherNodeStatus *node_status);

void SeptentrioSolutionMessageToTetherGpsStatus(
    const SeptentrioSolutionMessage *in, uint16_t sequence,
    TetherGpsStatus *out);

void ServoStatusMessageUpdateTetherServoStatus(
    const ServoStatusMessage *in, TetherNodeStatus *node_status,
    TetherServoStatus *out);

void ShortStackStatusMessageUpdateTetherShortStackStatus(
    const ShortStackStatusMessage *in, TetherNodeStatus *node_status);

void SlowStatusMessageUpdateTetherNodeStatus(
    const SlowStatusMessage *in, TetherNodeStatus *node_status);

void SmallControlTelemetryMessageToTetherControlTelemetry(
    const SmallControlTelemetryMessage *in, uint16_t sequence,
    TetherControlTelemetry *out);

void TetherNodeStatusToOutputTetherNodeStatus(
    AioNode source, TetherNodeStatus *node_status, TetherNodeStatus *out);

// Helper functions.

int32_t TetherNoUpdateCountToMilliseconds(int32_t no_update_count);

AioNode TetherDownSourceToAioNode(TetherDownSource source);
AioNode TetherUpSourceToAioNode(TetherUpSource source);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_TETHER_CONVERT_H_
