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

#include "sim/models/sensors/ground_station_v2_sensors.h"

#include "avionics/common/tether_message_types.h"
#include "common/c_math/util.h"
#include "sim/sim_telemetry.h"

GroundStationV2Sensors::GroundStationV2Sensors()
    : Sensor("GS02 sensors"),
      mode_(new_input_value(), "mode"),
      mode_cmd_(new_input_value(), "mode_cmd"),
      transform_stage_(new_input_value(), "transform_stage"),
      platform_azi_(new_input_value(), "platform_azi"),
      drum_angle_(new_input_value(), "drum_angle"),
      nominal_levelwind_ele_(new_input_value(), "nominal_levelwind_ele"),
      detwist_angle_(new_input_value(), "detwist_angle"),
      nominal_gsg_termination_(new_input_value(), "nominal_gsg_termination"),
      nominal_gsg_yoke_(new_input_value(), "nominal_gsg_yoke"),
      tether_engaged_(new_input_value(), "tether_engaged"),
      prox_sensor_active_(new_input_value(), "prox_sensor_active") {
  SetupDone();
}

// TODO(b/113276040): Currently the "true" state of the GS02 model is
// copied to the sensor outputs.  Instead, add sensor models.  These
// models should also enforce the angle wrapping behavior pertinent to
// the sensor.  For example, in the simulator, rotational axes are
// generally defined over a continuous multiturn domain in which the
// angle can increase or decrease without bound.  The reported sensor
// values need to be wrapped to [-PI, PI] or otherwise match the
// behavior of the installed hardware.
//
// Wrap an angle to the interval [-PI, PI].
double GroundStationV2Sensors::WrapAngle(double angle) const {
  return Wrap(angle, -PI, PI);
}

double GroundStationV2Sensors::gsg_termination() const {
  return tether_engaged() ? 0.0 : nominal_gsg_termination_.val();
}

double GroundStationV2Sensors::gsg_yoke() const {
  return tether_engaged() ? 0.0 : nominal_gsg_yoke_.val();
}

double GroundStationV2Sensors::levelwind_ele() const {
  return tether_engaged() ? nominal_levelwind_ele_.val() : 0.0;
}

double GroundStationV2Sensors::detwist_angle() const {
  return detwist_angle_.val();
}

void GroundStationV2Sensors::AddInternalConnections(
    ConnectionStore *connections) {
  connections->Add(6, [this](double /*t*/) {
    if (mode_.val() == kGroundStationModeReel ||
        mode_.val() == kGroundStationModeManual) {
      tether_engaged_.set_val(true);
    } else if (mode_.val() == kGroundStationModeHighTension) {
      tether_engaged_.set_val(false);
    } else if (mode_cmd_.val() == kGroundStationModeReel) {
      tether_engaged_.set_val(transform_stage_.val() == 4);
    } else if (mode_cmd_.val() == kGroundStationModeHighTension) {
      tether_engaged_.set_val(transform_stage_.val() == 0 ||
                              transform_stage_.val() == 4);
    } else {
      CHECK(false) << "Invalid GS mode command.";
    }
  });
}

void GroundStationV2Sensors::UpdateTetherPlatform(
    TetherPlatform *platform) const {
  platform->sequence = static_cast<uint16_t>((platform->sequence + 1) %
                                             TETHER_SEQUENCE_ROLLOVER);
  platform->no_update_count = 0;
  platform->flags = 0x0;
  platform->perch_azi = static_cast<float>(WrapAngle(platform_azi_.val()));
  platform->levelwind_ele = static_cast<float>(WrapAngle(levelwind_ele()));
}

void GroundStationV2Sensors::UpdateTetherDrum(TetherDrum *drum) const {
  drum->sequence =
      static_cast<uint16_t>((drum->sequence + 1) % TETHER_SEQUENCE_ROLLOVER);
  drum->no_update_count = 0;
  drum->flags = 0x0;
  drum->gsg_axis1 = static_cast<float>(WrapAngle(gsg_yoke()));
  drum->gsg_axis2 = static_cast<float>(WrapAngle(gsg_termination()));
}

void GroundStationV2Sensors::UpdateSensorOutputs(
    SimSensorMessage * /*sensor_message*/, TetherUpMessage *tether_up) const {
  TetherGroundStation &gs = tether_up->ground_station;
  gs.sequence =
      static_cast<uint16_t>((gs.sequence + 1U) % TETHER_SEQUENCE_ROLLOVER);
  gs.no_update_count = 0;
  gs.mode = static_cast<uint8_t>(mode_.val());
  gs.transform_stage = transform_stage_.val();
  gs.drum_angle = static_cast<float>(drum_angle_.val());  // Multi-turn angle.
  gs.detwist_angle = static_cast<float>(WrapAngle(detwist_angle_.val()));
  gs.proximity = static_cast<uint8_t>(prox_sensor_active_.val());
  gs.tether_engaged = tether_engaged_.val();

  TetherPlc &plc = tether_up->plc;
  plc.sequence =
      static_cast<uint16_t>((plc.sequence + 1U) % TETHER_SEQUENCE_ROLLOVER);
  plc.no_update_count = 0;
  plc.drum_angle = static_cast<float>(drum_angle_.val());  // Multi-turn angle.
  plc.flags &= static_cast<uint8_t>(~kTetherPlcFlagDrumFault);

  UpdateTetherPlatform(&tether_up->platform_a);
  UpdateTetherPlatform(&tether_up->platform_b);

  UpdateTetherDrum(&tether_up->drum_a);
  UpdateTetherDrum(&tether_up->drum_b);
}

void GroundStationV2Sensors::Publish() const {
  sim_telem.gsg.gsg_yoke = gsg_yoke();
  sim_telem.gsg.gsg_termination = gsg_termination();
  sim_telem.gsg.levelwind_ele = levelwind_ele();
  sim_telem.gsg.twist = detwist_angle();
}
