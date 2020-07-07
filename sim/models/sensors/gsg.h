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

#ifndef SIM_MODELS_SENSORS_GSG_H_
#define SIM_MODELS_SENSORS_GSG_H_

#include <vector>

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/perch.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/measurement.h"
#include "sim/models/tether.h"
#include "sim/sim_messages.h"

// TODO: Consider renaming this class to
// GroundStationEncoders or something similar.
class Gsg : public Sensor {
  friend class GsgTest;

 public:
  Gsg(const Perch *perch, const Tether &tether, const Wing &wing,
      const GsgParams &gsg_params, const GsgSimParams &gsg_sim_params,
      const PerchParams &perch_params, const PerchSimParams &perch_sim_params,
      const LevelwindParams &levelwind_params, FaultSchedule *faults);
  ~Gsg() {}

  void UpdateSensorOutputs(SimSensorMessage * /*sensor_message*/,
                           TetherUpMessage *tether_up) const override;
  void Publish() const override;

 private:
  void DiscreteStepHelper(double t) override;
  void UpdateTetherDrum(DrumLabel label, TetherDrum *drum) const;
  void UpdateTetherPlatform(PlatformLabel label,
                            TetherPlatform *platform) const;

  double elevation(int32_t i) const { return elevations_[i].recorded(); }
  double azimuth(int32_t i) const { return azimuths_[i].recorded(); }
  double twist(int32_t i) const { return twists_[i].recorded(); }

  double perch_azimuth(int32_t i) const {
    return perch_azimuths_[i].recorded();
  }
  double levelwind_elevation(int32_t i) const {
    return levelwind_elevations_[i].recorded();
  }

  // Parameters.
  const GsgParams &gsg_params_;
  const GsgSimParams &gsg_sim_params_;
  const PerchParams &perch_params_;
  const LevelwindParams &levelwind_params_;

  // Connections to other models.
  const Perch *perch_;
  const Tether &tether_;
  const Wing &wing_;

  // Discrete state.
  DiscreteState<double> actual_elevation_, actual_azimuth_, actual_twist_;
  DiscreteState<double> actual_perch_azimuth_, actual_levelwind_elevation_;
  DiscreteState<Vec3> prev_tether_force_g_;

  // Sub-models.
  std::vector<Measurement<double>> elevations_, azimuths_, twists_;
  std::vector<Measurement<double>> perch_azimuths_, levelwind_elevations_;

  DISALLOW_COPY_AND_ASSIGN(Gsg);
};

#endif  // SIM_MODELS_SENSORS_GSG_H_
