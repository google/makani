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

#ifndef SIM_MODELS_FULL_SYSTEM_H_
#define SIM_MODELS_FULL_SYSTEM_H_

#include <memory>
#include <vector>

#include "avionics/common/avionics_messages.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/actuator.h"
#include "sim/models/actuators/ground_station_v2.h"
#include "sim/models/actuators/rotor.h"
#include "sim/models/actuators/servo.h"
#include "sim/models/actuators/tether_release.h"
#include "sim/models/actuators/winch.h"
#include "sim/models/base_system_model.h"
#include "sim/models/environment.h"
#include "sim/models/high_voltage_harness.h"
#include "sim/models/imu_mount.h"
#include "sim/models/perch.h"
#include "sim/models/power_sys.h"
#include "sim/models/rigid_bodies/buoy.h"
#include "sim/models/rigid_bodies/platform.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/sea.h"
#include "sim/models/sensors/ground_station_v2_sensors.h"
#include "sim/models/spring_constraint.h"
#include "sim/models/tether.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/state.h"

class FullSystem : public BaseSystemModel {
 public:
  FullSystem(const SystemParams &system_params, const SimParams &sim_params,
             FaultSchedule *faults,
             const AvionicsPackets &external_avionics_packets,
             SimSensorMessage *sensor_message);
  ~FullSystem() {}

  const DynoCommandMessage &dyno_command_packet() {
    return dyno_command_packet_.val();
  }

  static void SetGsgAngles(const Buoy &buoy__, const Tether &tether__,
                           GroundStationV2Base *gs02__,
                           GroundStationV2Sensors *gs02_sensors_,
                           SimTelemetry *sim_telem_);

 private:
  void Init();
  void InitGroundStationV1();
  void InitGroundStationV2();
  void InitPerchWinchTether(bool initialize_in_crosswind_config, Perch *perch,
                            Winch *winch, Tether *tether) const;
  void InitGs02Tether(const Gs02SimParams &gs02_sim, GroundStationV2Base *gs02,
                      Tether *tether) const;

  Tether::CalcBridlePointFunc GetCalcBridlePointCallback() const;

  const AvionicsPackets &avionics_packets() { return avionics_packets_.val(); }

  void DiscreteStepHelper(double /*t*/) override{};

  // Sub-models.
  Environment environment_;
  GroundFrame ground_frame_;
  ImuMount wing_imu_mount_;
  ImuMount gs_imu_mount_;
  Wing wing_;
  Tether tether_;
  std::vector<std::unique_ptr<RotorBase>> rotors_;
  std::vector<std::unique_ptr<ServoBase>> servos_;
  std::unique_ptr<TetherReleaseBase> tether_release_;
  std::unique_ptr<Winch> winch_;
  std::unique_ptr<Perch> perch_;
  std::unique_ptr<Platform> platform_;
  std::unique_ptr<GroundStationV2Base> gs02_;
  std::unique_ptr<Buoy> buoy_;
  std::unique_ptr<Sea> sea_;
  std::unique_ptr<PowerSys> power_sys_;
  std::unique_ptr<SpringConstraint> constraint_;
  std::unique_ptr<HighVoltageHarness> high_voltage_harness_;

  // Inputs.
  //
  // avionics_packets_ is populated by the data in external_avionics_packets_
  // via a connection. Doing so allows avionics_packets_ to be handled uniformly
  // with other states.
  const AvionicsPackets &external_avionics_packets_;
  State<AvionicsPackets> avionics_packets_;
  SimSensorMessage *sensor_message_;

  // Outputs.
  //
  // dyno_command_packet_ is populated with aero torques from the rotor models
  // and, if desired, broadcast over the network.
  State<DynoCommandMessage> dyno_command_packet_;

  DISALLOW_COPY_AND_ASSIGN(FullSystem);
};

#endif  // SIM_MODELS_FULL_SYSTEM_H_
