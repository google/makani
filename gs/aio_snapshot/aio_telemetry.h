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

#ifndef GS_AIO_SNAPSHOT_AIO_TELEMETRY_H_
#define GS_AIO_SNAPSHOT_AIO_TELEMETRY_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "control/control_telemetry.h"
#include "control/system_types.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: Determine if packing / unpacking messages in the
// snapshot program presents a performance hit or risks obscuring the
// state of the network.
typedef struct {
  uint32_t seq;

  // Telemetry messages from the controller.
  bool control_telemetries_updated[kNumControllers];
  ControlTelemetry control_telemetries[kNumControllers];

  // Slow telemetry messages from the controllers.
  bool control_slow_telemetries_updated[kNumControllers];
  ControlSlowTelemetry control_slow_telemetries[kNumControllers];

  // Command messages from the controller.
  bool controller_commands_updated[kNumControllers];
  ControllerCommandMessage controller_commands[kNumControllers];

  // Core switch messages.
  bool core_switch_statuses_updated[kNumCoreSwitches];
  CoreSwitchStatusMessage core_switch_statuses[kNumCoreSwitches];

  // Drum sensors messages.
  bool drum_sensors_updated[kNumDrums];
  DrumSensorsMessage drum_sensors[kNumDrums];

  // Flight computer messages.
  bool flight_computer_sensors_updated[kNumFlightComputers];
  FlightComputerSensorMessage flight_computer_sensors[kNumFlightComputers];

  // Ground station weather messages.
  bool ground_station_weather_updated;
  GroundStationWeatherMessage ground_station_weather;

  // Ground station GPS messages.
  bool gs_gps_compass_updated;
  NovAtelCompassMessage gs_gps_compass;

  bool gs_gps_observations_updated;
  NovAtelObservationsMessage gs_gps_observations;

  bool gs_gps_solution_updated;
  NovAtelSolutionMessage gs_gps_solution;

  // Joystick messages.
  bool joystick_updated;
  JoystickStatusMessage joystick;

  bool joystick_monitor_updated;
  JoystickMonitorStatusMessage joystick_monitor;

  // Loadcell messages.
  bool loadcell_statuses_updated[kNumLoadcellNodes];
  LoadcellMessage loadcell_statuses[kNumLoadcellNodes];

  // Motor messages.
  bool motor_statuses_updated[kNumMotors];
  MotorStatusMessage motor_statuses[kNumMotors];

  // Platform sensors messages.
  bool platform_sensors_updated[kNumPlatforms];
  PlatformSensorsMessage platform_sensors[kNumPlatforms];

  // Recorder TMS570 status messages.
  bool recorder_statuses_updated[kNumRecorderTms570s];
  RecorderStatusMessage recorder_statuses[kNumRecorderTms570s];

  // Servo messages.
  bool servo_statuses_updated[kNumServos];
  ServoStatusMessage servo_statuses[kNumServos];

  // TetherDown messages.
  // Index 0=CsA, 1=CsGsA.
  bool tether_down_updated[2];
  TetherDownMessage tether_down[2];

  // Winch PLC status messages.
  bool winch_plc_updated;
  GroundStationWinchStatusMessage winch_plc;

  // Wing GPS messages. Fields for each GPS type are allocated for each
  // WingGpsReceiverLabel to simplify switching between GPS types.
  bool wing_gps_novatel_observations_updated[kNumWingGpsReceivers];
  NovAtelObservationsMessage
      wing_gps_novatel_observations[kNumWingGpsReceivers];

  bool wing_gps_novatel_solutions_updated[kNumWingGpsReceivers];
  NovAtelSolutionMessage wing_gps_novatel_solutions[kNumWingGpsReceivers];

  bool wing_gps_septentrio_observations_updated[kNumWingGpsReceivers];
  SeptentrioObservationsMessage
      wing_gps_septentrio_observations[kNumWingGpsReceivers];

  bool wing_gps_septentrio_solutions_updated[kNumWingGpsReceivers];
  SeptentrioSolutionMessage wing_gps_septentrio_solutions[kNumWingGpsReceivers];
} AioTelemetry1;

// AIO telemetry is sharded into multiple structures to stay below the limit on
// UDP packet length.
//
// Fields that are tightly coupled in the monitors should be kept within the
// same shard.
typedef struct {
  uint32_t seq;

  // Slow status messages.
  bool slow_statuses_updated[kNumAioNodes];
  SlowStatusMessage slow_statuses[kNumAioNodes];

  // Controller Q7 slow status message.
  bool controller_q7_slow_statuses_updated[kNumControllers];
  Q7SlowStatusMessage controller_q7_slow_statuses[kNumControllers];

  // Core switch slow status messages.
  bool core_switch_slow_statuses_updated[kNumCoreSwitches];
  CoreSwitchSlowStatusMessage core_switch_slow_statuses[kNumCoreSwitches];

  // Recorder Q7 slow status message.
  bool recorder_q7_slow_statuses_updated[kNumRecorderQ7s];
  Q7SlowStatusMessage recorder_q7_slow_statuses[kNumRecorderQ7s];
} AioTelemetry2;

typedef struct {
  uint32_t seq;

  // Self-test messages.
  bool self_test_updated[kNumAioNodes];
  SelfTestMessage self_test[kNumAioNodes];
} AioTelemetry3;

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_AIO_SNAPSHOT_AIO_TELEMETRY_H_
