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

#ifndef ANALYSIS_FLIGHT_DATA_DYNAMICS_SIM_H_
#define ANALYSIS_FLIGHT_DATA_DYNAMICS_SIM_H_

#include <stdlib.h>

#include <string>
#include <vector>

#include "sim/models/actuators/rotor.h"
#include "sim/models/base_system_model.h"
#include "sim/models/environment.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/physics/aero_frame.h"
#include "sim/physics/ground_frame.h"
#include "sim/sim_telemetry.h"

DynamicsReplayMessage GenerateDynamics(const SystemParams &system_params,
                                       const SimTelemetry &sim_telemetry,
                                       const Vec3 &Ab, const Vec3 &omega_b,
                                       const Vec3 &omega_b_dot);

// DynamicsSystem is a system model used to run the simulator's dynamics
// models using input states drawn externally.
// In particular, note that there is no integration.
//
// Many connections are copied directly from the standard kite system model
// (FullSystem). We use the exact same connection levels where appropriate, as
// the Wing and Rotor models define their own internal connections at predefined
// levels.
class DynamicsSystem : public BaseSystemModel {
 public:
  DynamicsSystem(const std::string &name, const SystemParams &system_params,
                 const SimParams &sim_params, FaultSchedule *faults);
  virtual ~DynamicsSystem() {}

 protected:
  virtual void DiscreteStepHelper(double /*t*/) {}
  virtual Vec3 Xg() = 0;
  virtual Vec3 Vb() = 0;
  virtual Vec3 pqr() = 0;
  virtual Mat3 dcm_g2b() = 0;
  virtual Vec3 wind_g() = 0;
  virtual const double *flaps() = 0;
  virtual const double *rotors() = 0;
  virtual Vec3 tether_force_b() = 0;
  virtual Vec3 bridle_moment_b() = 0;
  virtual Vec3 knot_pos_b() = 0;

  // Sub-models. Use of HitlRotor is slightly hacky, but its functionality is so
  // close to what we need that defining a new rotor class would be excessive.
  Environment environment_;
  GroundFrame ground_frame_;
  Wing wing_;
  std::vector<std::unique_ptr<HitlRotor>> rotors_;

  // Store the current estimator state for reprocessing.
  FlightMode flight_mode_;
};

#endif  // ANALYSIS_FLIGHT_DATA_DYNAMICS_SIM_H_
