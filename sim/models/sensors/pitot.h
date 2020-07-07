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

#ifndef SIM_MODELS_SENSORS_PITOT_H_
#define SIM_MODELS_SENSORS_PITOT_H_

#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/environment.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/models/sensors/sensor.h"
#include "sim/models/signals/measurement.h"
#include "sim/sim_messages.h"
#include "sim/sim_types.h"

class Pitot : public Sensor {
  friend class PitotTest;

 public:
  Pitot(const Environment &environment, const Wing &wing,
        PitotSensorLabel label, FlightComputerLabel fc_label,
        const PitotParams &pitot_params, const PitotSimParams &pitot_sim_params,
        FaultSchedule *faults);
  ~Pitot() {}

  void UpdateSensorOutputs(SimSensorMessage *sensor_message,
                           TetherUpMessage * /*tether_up*/) const override;
  void Publish() const override;

  void set_total_rotor_thrust(double val) { total_rotor_thrust_.set_val(val); }

 private:
  void DiscreteStepHelper(double t) override;

  double P_dyn() const { return P_dyn_.recorded(); }
  double P_stat() const { return P_stat_.recorded(); }
  double P_alpha() const { return P_alpha_.recorded(); }
  double P_beta() const { return P_beta_.recorded(); }
  double total_rotor_thrust() const { return total_rotor_thrust_.val(); }

  const PitotSensorLabel label_;

  // DCM for converting body coordinates to the actual measurement
  // coordinates.
  Mat3 dcm_b2actual_;

  const PitotParams &pitot_params_;
  const PitotSimParams &pitot_sim_params_;
  const FlightComputerLabel fc_label_;
  const Wing &wing_;
  const Environment &environment_;

  // Input states.

  // Combined thrust [N] from all the rotors.  This can affect the
  // induced velocity at the pitot tube if include_rotor_inflow is
  // true.
  State<double> total_rotor_thrust_;

  // Discrete state.
  DiscreteState<double> actual_P_dyn_, actual_P_stat_;
  DiscreteState<double> actual_P_alpha_, actual_P_beta_;

  // Sub-models.
  Measurement<double> P_dyn_, P_stat_, P_alpha_, P_beta_;

  DISALLOW_COPY_AND_ASSIGN(Pitot);
};

#endif  // SIM_MODELS_SENSORS_PITOT_H_
