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

#include "sim/models/sensors/gsg.h"

#include <math.h>
#include <stdint.h>
#include <vector>

#include "avionics/common/tether_message_types.h"
#include "common/c_math/filter.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/perch_frame.h"
#include "control/sensor_util.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/signals/measurement.h"
#include "sim/models/tether.h"
#include "sim/sim_messages.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"

Gsg::Gsg(const Perch *perch, const Tether &tether, const Wing &wing,
         const GsgParams &gsg_params, const GsgSimParams &gsg_sim_params,
         const PerchParams &perch_params,
         const PerchSimParams &perch_sim_params,
         const LevelwindParams &levelwind_params, FaultSchedule *faults)
    : Sensor("GSG"),
      gsg_params_(gsg_params),
      gsg_sim_params_(gsg_sim_params),
      perch_params_(perch_params),
      levelwind_params_(levelwind_params),
      perch_(perch),
      tether_(tether),
      wing_(wing),
      actual_elevation_(new_discrete_state(), "actual_elevation", 0.0, 0.0),
      actual_azimuth_(new_discrete_state(), "actual_azimuth", 0.0, 0.0),
      actual_twist_(new_discrete_state(), "actual_twist", 0.0, 0.0),
      actual_perch_azimuth_(new_discrete_state(), "actual_perch_azimuth", 0.0,
                            0.0),
      actual_levelwind_elevation_(new_discrete_state(),
                                  "actual_levelwind_elevation", 0.0, 0.0),
      prev_tether_force_g_(new_discrete_state(), "prev_tether_force_g",
                           {-1.0, 0.0, 0.0}),
      elevations_(),
      azimuths_(),
      twists_(),
      perch_azimuths_(),
      levelwind_elevations_() {
  // Drum sensors.
  azimuths_.reserve(kNumDrums);
  elevations_.reserve(kNumDrums);
  twists_.reserve(kNumDrums);

  for (int32_t i = 0; i < kNumDrums; ++i) {
    azimuths_.emplace_back(
        full_name(), "azimuth[" + std::to_string(i) + "]", gsg_sim_params.ts,
        std::vector<SensorModelParams>({gsg_sim_params.gsg_azi_sensor[i]}),
        actual_azimuth_, faults);
    add_sub_model(&azimuths_[i]);

    elevations_.emplace_back(
        full_name(), "elevation[" + std::to_string(i) + "]", gsg_sim_params.ts,
        std::vector<SensorModelParams>({gsg_sim_params.gsg_ele_sensor[i]}),
        actual_elevation_, faults);
    add_sub_model(&elevations_[i]);

    twists_.emplace_back(
        full_name(), "twist[" + std::to_string(i) + "]", gsg_sim_params.ts,
        std::vector<SensorModelParams>({gsg_sim_params.gsg_twist_sensor[i]}),
        actual_twist_, faults);
    add_sub_model(&twists_[i]);
  }

  // Platform sensors.
  perch_azimuths_.reserve(kNumPlatforms);
  levelwind_elevations_.reserve(kNumPlatforms);

  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    perch_azimuths_.emplace_back(
        full_name(), "perch_azi[" + std::to_string(i) + "]",
        perch_sim_params.ts,
        std::vector<SensorModelParams>({perch_sim_params.perch_azi_sensor[i]}),
        actual_perch_azimuth_, faults);
    add_sub_model(&perch_azimuths_[i]);

    levelwind_elevations_.emplace_back(
        full_name(), "levelwind_ele[" + std::to_string(i) + "]",
        perch_sim_params.ts, std::vector<SensorModelParams>(
                                 {perch_sim_params.levelwind_ele_sensor[i]}),
        actual_levelwind_elevation_, faults);
    add_sub_model(&levelwind_elevations_[i]);
  }

  SetupDone();
}

void Gsg::DiscreteStepHelper(double t) {
  Vec3 tether_force_g, tether_force_p;
  Mat3 dcm_g2p = kMat3Identity;
  double drum_angle = 0.0;

  double elevation__ = actual_elevation_.val();
  double azimuth__ = actual_azimuth_.val();
  double twist__ = actual_twist_.val();
  double perch_azimuth__ = actual_perch_azimuth_.val();
  double levelwind_elevation__ = actual_levelwind_elevation_.val();

  if (perch_) {
    dcm_g2p = perch_->dcm_g2p();
    perch_azimuth__ = perch_->theta_perch();
    drum_angle = perch_->theta_winch();
  }

  // Calculate the tether direction in the perch frame.  In lieu of a
  // GSG dynamics model, hold GSG position until tension returns.
  tether_force_g = tether_.start_force_moment().force;
  if (Vec3Norm(&tether_force_g) < 1.0 ||
      Vec3Dot(&tether_force_g, &prev_tether_force_g_.val()) < 0.0) {
    tether_force_g = prev_tether_force_g_.val();
  }
  Mat3Vec3Mult(&dcm_g2p, &tether_force_g, &tether_force_p);

  // Only update GSG readings if levelwind is not engaged.
  if (!perch_ || !perch_->levelwind_engaged()) {
    // TODO: Add messed up model.
    double dum1, dum2;
    QuatToAngle(&wing_.q(), kRotationOrderZyx, &twist__, &dum1, &dum2);
    twist__ += M_PI;

    Vec3 tether_force_wd;
    RotPToWd(&tether_force_p, drum_angle, &tether_force_wd);
    TetherDirectionWdToGsgAziEle(&tether_force_wd, &azimuth__, &elevation__);

    levelwind_elevation__ = levelwind_params_.elevation_nominal;
  } else if (perch_->levelwind_engaged()) {
    Vec3 tether_force_lw;
    RotPToLw(&tether_force_p, 0.0, &tether_force_lw);

    // If there is no tension, then the spring-loaded levelwind
    // returns to its home position.
    double levelwind_elevation_perfect;
    if (Vec3Norm(&tether_force_lw) < DBL_TOL) {
      levelwind_elevation_perfect = levelwind_params_.elevation_nominal;
    } else {
      levelwind_elevation_perfect =
          atan2(-tether_force_lw.x, -tether_force_lw.z);
    }

    if (*g_sim.sim_opt & kSimOptImperfectSensors) {
      levelwind_elevation__ = Backlash(levelwind_elevation_perfect,
                                       levelwind_params_.elevation_backlash,
                                       &levelwind_elevation__);
    } else {
      levelwind_elevation__ = levelwind_elevation_perfect;
    }
  }

  actual_elevation_.DiscreteUpdate(t, elevation__);
  actual_azimuth_.DiscreteUpdate(t, azimuth__);
  actual_twist_.DiscreteUpdate(t, twist__);

  actual_perch_azimuth_.DiscreteUpdate(t, perch_azimuth__);
  actual_levelwind_elevation_.DiscreteUpdate(t, levelwind_elevation__);

  prev_tether_force_g_.DiscreteUpdate(t, tether_force_g);
}

void Gsg::UpdateTetherDrum(DrumLabel label, TetherDrum *drum) const {
  drum->sequence =
      static_cast<uint16_t>((drum->sequence + 1) % TETHER_SEQUENCE_ROLLOVER);
  drum->no_update_count = 0;
  drum->flags = 0x0;
  drum->gsg_axis1 = static_cast<float>(azimuth(label));
  drum->gsg_axis2 = static_cast<float>(elevation(label));
}

void Gsg::UpdateTetherPlatform(PlatformLabel label,
                               TetherPlatform *platform) const {
  platform->sequence = static_cast<uint16_t>((platform->sequence + 1) %
                                             TETHER_SEQUENCE_ROLLOVER);
  platform->no_update_count = 0;
  platform->flags = 0x0;
  platform->perch_azi = static_cast<float>(perch_azimuth(label));
  platform->levelwind_ele = static_cast<float>(levelwind_elevation(label));
}

void Gsg::UpdateSensorOutputs(SimSensorMessage * /*sensor_message*/,
                              TetherUpMessage *tether_up) const {
  UpdateTetherDrum(kDrumSensorsA, &tether_up->drum_a);
  UpdateTetherDrum(kDrumSensorsB, &tether_up->drum_b);

  UpdateTetherPlatform(kPlatformSensorsA, &tether_up->platform_a);
  UpdateTetherPlatform(kPlatformSensorsB, &tether_up->platform_b);
}

void Gsg::Publish() const {
  sim_telem.gsg.ele = actual_elevation_.val();
  sim_telem.gsg.azi = actual_azimuth_.val();
  sim_telem.gsg.twist = actual_twist_.val();
  sim_telem.gsg.perch_azi = actual_perch_azimuth_.val();
  sim_telem.gsg.levelwind_ele = actual_levelwind_elevation_.val();
}
