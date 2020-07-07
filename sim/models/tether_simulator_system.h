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

#ifndef SIM_MODELS_TETHER_SIMULATOR_SYSTEM_H_
#define SIM_MODELS_TETHER_SIMULATOR_SYSTEM_H_

#include <memory>
#include <string>
#include <vector>

#include "sim/math/interp.h"
#include "sim/models/base_system_model.h"
#include "sim/models/environment.h"
#include "sim/models/tether.h"
#include "sim/physics/ground_frame.h"

// Interpolates data from a flight log for access at arbitrary time points. Data
// requested before or after the time span of the log is provided by zero-order
// extrapolation.
class FlightLogInterpolator {
 public:
  explicit FlightLogInterpolator(const std::vector<std::string> &filenames);
  ~FlightLogInterpolator();

  // Kite states.
  Vec3 Xg(double t);
  Vec3 Vb(double t);
  Vec3 omega(double t);
  Mat3 dcm_g2b(double t);

  // GSG/perch states.
  double perch_azi(double t);
  double gsg_azi(double t);
  double gsg_ele(double t);

  inline double EarliestTime() { return t_[0]; }
  inline double LatestTime() { return t_.back(); }

  // Indicates whether tether release has been commanded at or before the input
  // time.
  bool TetherReleaseCommanded(double t) { return t >= tether_release_time_; }

 private:
  sim::TimeseriesInterpolator *interp_;
  std::vector<double> t_;

  // Kite states.
  std::vector<Vec3> Xg_;
  std::vector<Vec3> Vb_;
  std::vector<Vec3> omega_;
  std::vector<Vec3> eulers_;

  // GSG/perch states.
  std::vector<double> perch_azi_;
  std::vector<double> gsg_azi_;
  std::vector<double> gsg_ele_;

  // Time at which tether release was commanded.
  double tether_release_time_;

  DISALLOW_COPY_AND_ASSIGN(FlightLogInterpolator);
};

class TetherSimulatorSystem : public BaseSystemModel {
 public:
  TetherSimulatorSystem(FlightLogInterpolator *log_interpolator,
                        const SystemParams &system_params,
                        const SimParams &sim_params, FaultSchedule *faults);
  ~TetherSimulatorSystem() {}

  void Publish() const override;

 private:
  void Init();

  void DiscreteStepHelper(double /*t*/) override {}

  // Calculates the position and velocity (g-frame) of the tether start
  // (ground-side termination).
  void CalcTetherStartPosVel(double perch_azi, double gsg_azi, double gsg_ele,
                             Vec3 *tether_start_pos_g,
                             Vec3 *tether_start_vel_g);

  // Sub-models.
  Environment environment_;
  GroundFrame ground_frame_;
  Tether tether_;

  FlightLogInterpolator *log_interpolator_;

  State<Vec3> Xg_wing_;
  State<Vec3> Vb_wing_;
  State<Vec3> omega_wing_b_;
  State<Mat3> dcm_g2b_;

  State<double> perch_azi_;
  State<double> gsg_azi_;
  State<double> gsg_ele_;

  State<bool> tether_release_commanded_;

  DISALLOW_COPY_AND_ASSIGN(TetherSimulatorSystem);
};

#endif  // SIM_MODELS_TETHER_SIMULATOR_SYSTEM_H_
