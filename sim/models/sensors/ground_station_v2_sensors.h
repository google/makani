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

#ifndef SIM_MODELS_SENSORS_GROUND_STATION_V2_SENSORS_H_
#define SIM_MODELS_SENSORS_GROUND_STATION_V2_SENSORS_H_

#include "avionics/common/avionics_messages.h"
#include "avionics/common/plc_messages.h"
#include "sim/models/sensors/sensor.h"
#include "sim/sim_messages.h"

class GroundStationV2Sensors : public Sensor {
 public:
  GroundStationV2Sensors();
  virtual ~GroundStationV2Sensors() {}

  void AddInternalConnections(ConnectionStore *connections) override;

  void UpdateSensorOutputs(SimSensorMessage * /*sensor_message*/,
                           TetherUpMessage *tether_up) const override;

  void Publish() const override;

  void set_mode(GroundStationMode val) { mode_.set_val(val); }
  void set_mode_cmd(GroundStationMode val) { mode_cmd_.set_val(val); }
  void set_transform_stage(uint8_t val) { transform_stage_.set_val(val); }
  void set_platform_azi(double val) { platform_azi_.set_val(val); }
  void set_drum_angle(double val) { drum_angle_.set_val(val); }
  void set_nominal_levelwind_ele(double val) {
    nominal_levelwind_ele_.set_val(val);
  }
  void set_detwist_angle(double val) {
    detwist_angle_.set_val(Wrap(val, -PI, PI));
  }
  void set_nominal_gsg_termination(double val) {
    nominal_gsg_termination_.set_val(val);
  }
  void set_nominal_gsg_yoke(double val) { nominal_gsg_yoke_.set_val(val); }
  void set_prox_sensor_active(bool val) { prox_sensor_active_.set_val(val); }

 private:
  void DiscreteStepHelper(double /*t*/) override {}

  void UpdateTetherPlatform(TetherPlatform *platform) const;
  void UpdateTetherDrum(TetherDrum *drum) const;
  double WrapAngle(double angle) const;

  double gsg_yoke() const;
  double gsg_termination() const;
  double levelwind_ele() const;
  double detwist_angle() const;
  bool tether_engaged() const { return tether_engaged_.val(); }

  State<GroundStationMode> mode_;
  State<GroundStationMode> mode_cmd_;
  State<uint8_t> transform_stage_;

  // TODO: Separate into actual_platform_azi_ for the true azimuth
  // position and platform_azis_ for the two imperfect encoder signals. (See how
  // perch_azimuths_ is handled in the old GSG model.)
  State<double> platform_azi_;

  State<double> drum_angle_;
  State<double> nominal_levelwind_ele_;
  State<double> detwist_angle_;
  State<double> nominal_gsg_termination_, nominal_gsg_yoke_;
  State<bool> tether_engaged_;
  State<bool> prox_sensor_active_;

  DISALLOW_COPY_AND_ASSIGN(GroundStationV2Sensors);
};

#endif  // SIM_MODELS_SENSORS_GROUND_STATION_V2_SENSORS_H_
