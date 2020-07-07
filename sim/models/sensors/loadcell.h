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

#ifndef SIM_MODELS_SENSORS_LOADCELL_H_
#define SIM_MODELS_SENSORS_LOADCELL_H_

#include <gsl/gsl_vector.h>
#include <stdint.h>
#include <functional>
#include <vector>

#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"

namespace sim {

void TetherForceToLoadcells(const WingParams &wing_params,
                            const LoadcellParams loadcell_params[],
                            const Vec3 &tether_force_b,
                            double loadcell_forces[]);

}  // namespace sim

class Loadcell : public Sensor {
  friend class LoadcellTest;

 public:
  Loadcell(const LoadcellParams *loadcell_params,
           const LoadcellSimParams &loadcell_sim_params,
           const WingParams &wing_params, FaultSchedule *faults);
  ~Loadcell() {}

  void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                           TetherUpMessage * /*tether_up*/) const override;
  void Publish() const override;

  void set_Fb_tether(const Vec3 &val) { Fb_tether_.set_val(val); }
  void set_tether_released(bool val) { tether_released_.set_val(val); }

 private:
  void DiscreteStepHelper(double t) override;

  double tensions(int32_t i) const { return tensions_[i].recorded(); }
  const Vec3 &Fb_tether() const { return Fb_tether_.val(); }

  // Loadcell parameters.
  const LoadcellParams *loadcell_params_;
  const LoadcellSimParams &loadcell_sim_params_;
  const WingParams &wing_params_;

  // Input states.
  State<Vec3> Fb_tether_;
  State<bool> tether_released_;

  // Discrete state.
  std::vector<DiscreteState<double>> actual_tensions_;

  // Sub-models.
  std::vector<Measurement<double>> tensions_;

  DISALLOW_COPY_AND_ASSIGN(Loadcell);
};

#endif  // SIM_MODELS_SENSORS_LOADCELL_H_
