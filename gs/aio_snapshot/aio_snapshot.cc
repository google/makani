// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gs/aio_snapshot/aio_snapshot.h"

#include <glog/logging.h>
#include <stdint.h>
#include <string.h>

#include <functional>
#include <vector>

#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/linux/aio.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "control/avionics/avionics_sim.h"
#include "control/control_telemetry.h"
#include "control/cvt_control_telemetry.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "gs/aio_snapshot/pack_aio_telemetry.h"
#include "gs/monitor/monitor_params.h"
#include "lib/udpio/udpio.h"
#include "sim/cvt_sim_messages.h"
#include "system/labels.h"
#include "system/labels_util.h"

COMPILE_ASSERT(PACK_AIOTELEMETRY1_SIZE <= UDP_BUFLEN,
               aio_telemetry_1_packet_is_too_large_for_udpio_buffer);
COMPILE_ASSERT(PACK_AIOTELEMETRY2_SIZE <= UDP_BUFLEN,
               aio_telemetry_2_packet_is_too_large_for_udpio_buffer);
COMPILE_ASSERT(PACK_AIOTELEMETRY3_SIZE <= UDP_BUFLEN,
               aio_telemetry_3_packet_is_too_large_for_udpio_buffer);

int main() {
  ground_station::AioSnapshotMain snapshot_main(GetSystemParams()->ts,
                                                GetSystemParams()->comms);
  snapshot_main.SetupComms();
  snapshot_main.Run();
  snapshot_main.CloseComms();
  return 0;
}

namespace ground_station {

// Initialize the snapshots.
AioSnapshotMain::AioSnapshotMain(double snapshot_period,
                                 const CommsParams &params)
    : snapshot_period_(snapshot_period),
      params_(params),
      aio_nodes_(),
      controller_nodes_(),
      core_switch_nodes_(),
      drum_nodes_(),
      flight_computer_nodes_(),
      loadcell_nodes_(),
      motor_nodes_(),
      platform_nodes_(),
      recorder_q7_nodes_(),
      recorder_tms570_nodes_(),
      servo_nodes_(),
      tether_down_nodes_(),
      winch_plc_node_(kAioNodePlcTophat),
      wing_gps_nodes_{WingGpsReceiverLabelToAioNode(kWingGpsReceiverCrosswind),
                      WingGpsReceiverLabelToAioNode(kWingGpsReceiverHover)},
      aio_1_udpio_config_(),
      aio_telemetry_1_(),
      aio_2_udpio_config_(),
      aio_telemetry_2_(),
      aio_3_udpio_config_(),
      aio_telemetry_3_() {
  // Zero out the AIO telemetry structures (has the effect of setting
  // updated flags to false and the sequence number to zero).
  memset(&aio_telemetry_1_, 0, sizeof(aio_telemetry_1_));
  memset(&aio_telemetry_2_, 0, sizeof(aio_telemetry_2_));
  memset(&aio_telemetry_3_, 0, sizeof(aio_telemetry_3_));

  // Generate AioNode values for each class of status message.
  for (int32_t i = 0; i < kNumAioNodes; ++i) {
    aio_nodes_[i] = static_cast<AioNode>(i);
  }

  for (int32_t i = 0; i < kNumControllers; ++i) {
    controller_nodes_[i] =
        ControllerLabelToControllerAioNode(static_cast<ControllerLabel>(i));
  }

  for (int32_t i = 0; i < kNumCoreSwitches; ++i) {
    core_switch_nodes_[i] =
        CoreSwitchLabelToCoreSwitchAioNode(static_cast<CoreSwitchLabel>(i));
  }

  for (int32_t i = 0; i < kNumDrums; ++i) {
    drum_nodes_[i] = DrumLabelToDrumAioNode(static_cast<DrumLabel>(i));
  }

  for (int32_t i = 0; i < kNumFlightComputers; ++i) {
    flight_computer_nodes_[i] = FlightComputerLabelToFlightComputerAioNode(
        static_cast<FlightComputerLabel>(i));
  }

  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    loadcell_nodes_[i] = LoadcellNodeLabelToLoadcellNodeAioNode(
        static_cast<LoadcellNodeLabel>(i));
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    motor_nodes_[i] = MotorLabelToMotorAioNode(static_cast<MotorLabel>(i));
  }

  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    platform_nodes_[i] =
        PlatformLabelToPlatformAioNode(static_cast<PlatformLabel>(i));
  }

  for (int32_t i = 0; i < kNumRecorderQ7s; ++i) {
    recorder_q7_nodes_[i] =
        RecorderQ7LabelToRecorderQ7AioNode(static_cast<RecorderQ7Label>(i));
  }

  for (int32_t i = 0; i < kNumRecorderTms570s; ++i) {
    recorder_tms570_nodes_[i] = RecorderTms570LabelToRecorderTms570AioNode(
        static_cast<RecorderTms570Label>(i));
  }

  for (int32_t i = 0; i < kNumServos; ++i) {
    servo_nodes_[i] = ServoLabelToServoAioNode(static_cast<ServoLabel>(i));
  }

  assert(tether_down_nodes_.size() == 2);
  tether_down_nodes_[0] = kAioNodeCsA;
  tether_down_nodes_[1] = kAioNodeCsGsA;
}

// Setup a UDP connection to send status messages to.
//
// Args:
//   params: Parameter structure for UDPIO connections.
void AioSnapshotMain::SetupComms() {
  udp_setup_sender(&aio_1_udpio_config_,
                   params_.udpio.aio_telemetry_remote_addr,
                   params_.udpio.aio_telemetry_1_remote_port, 1);
  udp_setup_sender(&aio_2_udpio_config_,
                   params_.udpio.aio_telemetry_remote_addr,
                   params_.udpio.aio_telemetry_2_remote_port, 1);
  udp_setup_sender(&aio_3_udpio_config_,
                   params_.udpio.aio_telemetry_remote_addr,
                   params_.udpio.aio_telemetry_3_remote_port, 1);
}

// Close the UDP connection.
void AioSnapshotMain::CloseComms() {
  udp_close(&aio_1_udpio_config_);
  udp_close(&aio_2_udpio_config_);
  udp_close(&aio_3_udpio_config_);
}

// Register what AIO messages will be listened for, and begin the AIO main loop.
void AioSnapshotMain::Run() {
  std::vector<MessageType> subscribe_types = {
      kMessageTypeControllerCommand,
      kMessageTypeControlSlowTelemetry,
      kMessageTypeControlTelemetry,
      kMessageTypeCoreSwitchSlowStatus,
      kMessageTypeCoreSwitchStatus,
      kMessageTypeDrumSensors,
      kMessageTypeFlightComputerSensor,
      kMessageTypeGroundStationWeather,
      kMessageTypeGroundStationWinchStatus,
      kMessageTypeJoystickStatus,
      kMessageTypeJoystickMonitorStatus,
      kMessageTypeLoadcell,
      kMessageTypeMotorStatus,
      kMessageTypeNovAtelCompass,
      kMessageTypeNovAtelObservations,
      kMessageTypeNovAtelSolution,
      kMessageTypePlatformSensors,
      kMessageTypeQ7SlowStatus,
      kMessageTypeRecorderStatus,
      kMessageTypeSelfTest,
      kMessageTypeSeptentrioSolution,
      kMessageTypeSeptentrioObservations,
      kMessageTypeServoStatus,
      kMessageTypeSlowStatus,
      kMessageTypeTetherDown};

  if (GetMonitorParams()->v1_monitor_use_sim_sensors) {
    subscribe_types.push_back(kMessageTypeSimSensor);
  }

  AioLoopStart(kAioNodeTelemetrySnapshot, params_.aio_port,
               subscribe_types.data(),
               static_cast<int32_t>(subscribe_types.size()),
               ground_station::AioSnapshotMain::AioCallback, this,
               static_cast<int32_t>(snapshot_period_ * 1000000));
  AioClose();
}

// Receive and unpack messages for a set of AioNodes to an array of messages.
//
// Args:
//   get_func: Function that fetches the structure (unpacked) from the CVT.
//   nodes: Array of nodes to receive messages of MessageType type from.
//   elements: Array of num_elts elements to be written.
//   elements_updated: Array of num_elts flags written 1 indicating a new
//       message was received and 0 otherwise.
template <class T, int32_t num_elts>
void AioSnapshotMain::TakeArraySnapshot(
    const std::function<bool(AioNode, T *, uint16_t *, int64_t *)> &get_func,
    const std::array<AioNode, num_elts> &nodes, T elements[],
    bool elements_updated[]) {
  for (int32_t i = 0; i < num_elts; ++i) {
    elements_updated[i] = get_func(nodes[i], &elements[i], nullptr, nullptr);
  }
}

// Poll AIO for any new messages and store the results.
void AioSnapshotMain::TakeSnapshots() {
  // Unpack SimSensors messages.
  if (GetMonitorParams()->v1_monitor_use_sim_sensors) {
    uint16_t sequence = 0U;
    int64_t timestamp = 0;
    SimSensorMessage sensor_message;
    if (CvtGetSimSensorMessage(kAioNodeSimulator, &sensor_message, &sequence,
                               &timestamp)) {
      UpdateControllerCvtFromSimSensorMessage(&sensor_message,
                                              &GetSystemParams()->hitl.config,
                                              sequence, timestamp);
    }
  }

  TakeArraySnapshot<ControlTelemetry, kNumControllers>(
      &CvtGetControlTelemetry, controller_nodes_,
      aio_telemetry_1_.control_telemetries,
      aio_telemetry_1_.control_telemetries_updated);

  TakeArraySnapshot<ControlSlowTelemetry, kNumControllers>(
      &CvtGetControlSlowTelemetry, controller_nodes_,
      aio_telemetry_1_.control_slow_telemetries,
      aio_telemetry_1_.control_slow_telemetries_updated);

  TakeArraySnapshot<ControllerCommandMessage, kNumControllers>(
      &CvtGetControllerCommandMessage, controller_nodes_,
      aio_telemetry_1_.controller_commands,
      aio_telemetry_1_.controller_commands_updated);

  TakeArraySnapshot<Q7SlowStatusMessage, kNumControllers>(
      &CvtGetQ7SlowStatusMessage, controller_nodes_,
      aio_telemetry_2_.controller_q7_slow_statuses,
      aio_telemetry_2_.controller_q7_slow_statuses_updated);

  TakeArraySnapshot<CoreSwitchStatusMessage, kNumCoreSwitches>(
      &CvtGetCoreSwitchStatusMessage, core_switch_nodes_,
      aio_telemetry_1_.core_switch_statuses,
      aio_telemetry_1_.core_switch_statuses_updated);

  TakeArraySnapshot<CoreSwitchSlowStatusMessage, kNumCoreSwitches>(
      &CvtGetCoreSwitchSlowStatusMessage, core_switch_nodes_,
      aio_telemetry_2_.core_switch_slow_statuses,
      aio_telemetry_2_.core_switch_slow_statuses_updated);

  TakeArraySnapshot<DrumSensorsMessage, kNumDrums>(
      &CvtGetDrumSensorsMessage, drum_nodes_, aio_telemetry_1_.drum_sensors,
      aio_telemetry_1_.drum_sensors_updated);

  TakeArraySnapshot<FlightComputerSensorMessage, kNumFlightComputers>(
      &CvtGetFlightComputerSensorMessage, flight_computer_nodes_,
      aio_telemetry_1_.flight_computer_sensors,
      aio_telemetry_1_.flight_computer_sensors_updated);

  aio_telemetry_1_.ground_station_weather_updated =
      CvtGetGroundStationWeatherMessage(
          gs_weather_node_, &aio_telemetry_1_.ground_station_weather, nullptr,
          nullptr);

  aio_telemetry_1_.gs_gps_compass_updated = CvtGetNovAtelCompassMessage(
      gs_gps_node_, &aio_telemetry_1_.gs_gps_compass, nullptr, nullptr);

  aio_telemetry_1_.gs_gps_observations_updated =
      CvtGetNovAtelObservationsMessage(gs_gps_node_,
                                       &aio_telemetry_1_.gs_gps_observations,
                                       nullptr, nullptr);

  aio_telemetry_1_.gs_gps_solution_updated = CvtGetNovAtelSolutionMessage(
      gs_gps_node_, &aio_telemetry_1_.gs_gps_solution, nullptr, nullptr);

  aio_telemetry_1_.joystick_updated = CvtGetJoystickStatusMessage(
      joystick_node_, &aio_telemetry_1_.joystick, nullptr, nullptr);

  aio_telemetry_1_.joystick_monitor_updated =
      CvtGetJoystickMonitorStatusMessage(
          joystick_node_, &aio_telemetry_1_.joystick_monitor, nullptr, nullptr);

  TakeArraySnapshot<LoadcellMessage, kNumLoadcellNodes>(
      &CvtGetLoadcellMessage, loadcell_nodes_,
      aio_telemetry_1_.loadcell_statuses,
      aio_telemetry_1_.loadcell_statuses_updated);

  TakeArraySnapshot<MotorStatusMessage, kNumMotors>(
      &CvtGetMotorStatusMessage, motor_nodes_, aio_telemetry_1_.motor_statuses,
      aio_telemetry_1_.motor_statuses_updated);

  TakeArraySnapshot<PlatformSensorsMessage, kNumPlatforms>(
      &CvtGetPlatformSensorsMessage, platform_nodes_,
      aio_telemetry_1_.platform_sensors,
      aio_telemetry_1_.platform_sensors_updated);

  TakeArraySnapshot<Q7SlowStatusMessage, kNumRecorderQ7s>(
      &CvtGetQ7SlowStatusMessage, recorder_q7_nodes_,
      aio_telemetry_2_.recorder_q7_slow_statuses,
      aio_telemetry_2_.recorder_q7_slow_statuses_updated);

  TakeArraySnapshot<RecorderStatusMessage, kNumRecorderTms570s>(
      &CvtGetRecorderStatusMessage, recorder_tms570_nodes_,
      aio_telemetry_1_.recorder_statuses,
      aio_telemetry_1_.recorder_statuses_updated);

  TakeArraySnapshot<ServoStatusMessage, kNumServos>(
      &CvtGetServoStatusMessage, servo_nodes_, aio_telemetry_1_.servo_statuses,
      aio_telemetry_1_.servo_statuses_updated);

  assert(2 == tether_down_nodes_.size());
  assert(2 == ARRAYSIZE(aio_telemetry_1_.tether_down));
  assert(2 == ARRAYSIZE(aio_telemetry_1_.tether_down_updated));
  TakeArraySnapshot<TetherDownMessage, 2>(
      &CvtGetTetherDownMessage, tether_down_nodes_,
      aio_telemetry_1_.tether_down, aio_telemetry_1_.tether_down_updated);

  aio_telemetry_1_.winch_plc_updated = CvtGetGroundStationWinchStatusMessage(
      winch_plc_node_, &aio_telemetry_1_.winch_plc, nullptr, nullptr);

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    auto gps_label = static_cast<WingGpsReceiverLabel>(i);
    GpsReceiverType gps_type = WingGpsReceiverLabelToGpsReceiverType(gps_label);
    if (gps_type == kGpsReceiverTypeNovAtel) {
      aio_telemetry_1_.wing_gps_novatel_observations_updated[i] =
          CvtGetNovAtelObservationsMessage(
              wing_gps_nodes_[i],
              &aio_telemetry_1_.wing_gps_novatel_observations[i], nullptr,
              nullptr);
      aio_telemetry_1_.wing_gps_novatel_solutions_updated[i] =
          CvtGetNovAtelSolutionMessage(
              wing_gps_nodes_[i],
              &aio_telemetry_1_.wing_gps_novatel_solutions[i], nullptr,
              nullptr);
    } else if (gps_type == kGpsReceiverTypeSeptentrio) {
      aio_telemetry_1_.wing_gps_septentrio_observations_updated[i] =
          CvtGetSeptentrioObservationsMessage(
              wing_gps_nodes_[i],
              &aio_telemetry_1_.wing_gps_septentrio_observations[i], nullptr,
              nullptr);
      aio_telemetry_1_.wing_gps_septentrio_solutions_updated[i] =
          CvtGetSeptentrioSolutionMessage(
              wing_gps_nodes_[i],
              &aio_telemetry_1_.wing_gps_septentrio_solutions[i], nullptr,
              nullptr);
    } else {
      LOG(FATAL) << "Unsupported GPS type: " << static_cast<int32_t>(gps_type);
    }
  }

  TakeArraySnapshot<SelfTestMessage, kNumAioNodes>(
      &CvtGetSelfTestMessage, aio_nodes_, aio_telemetry_3_.self_test,
      aio_telemetry_3_.self_test_updated);

  TakeArraySnapshot<SlowStatusMessage, kNumAioNodes>(
      &CvtGetSlowStatusMessage, aio_nodes_, aio_telemetry_2_.slow_statuses,
      aio_telemetry_2_.slow_statuses_updated);

  aio_telemetry_1_.seq++;
  aio_telemetry_2_.seq++;
  aio_telemetry_3_.seq++;
}

// Transmit snapshot over UDP.
void AioSnapshotMain::SendSnapshots() {
  UDP_SEND_PACKED(&aio_1_udpio_config_, PackAioTelemetry1, &aio_telemetry_1_);
  UDP_SEND_PACKED(&aio_2_udpio_config_, PackAioTelemetry2, &aio_telemetry_2_);
  UDP_SEND_PACKED(&aio_3_udpio_config_, PackAioTelemetry3, &aio_telemetry_3_);
}

}  // namespace ground_station
