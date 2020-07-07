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

#ifndef SIM_MODELS_SEA_H_
#define SIM_MODELS_SEA_H_

#include "sim/math/util.h"
#include "sim/models/environment.h"
#include "sim/models/model.h"
#include "sim/physics/ground_frame.h"

// This class implements a sea model.
// Document describing the sea model can be found here:
// docs.google.com/document/d/10--gfN41OCA1k3tjxwAdSkv7BPQm0VIaORuc5npkUWk

class Sea : public Model {
  friend class SeaTest;

 public:
  Sea(const Environment &environment, const GroundFrame &ground_frame,
      const SeaSimParams &sea_sim_params, double msl_pos_z_g);
  ~Sea() {}

  void Publish() const override;
  double GetSeaElevation(double t, double x, double y) const;
  double GetWaterDensity() const { return sea_sim_params_.water_density; }
  void GetWaveKinematics(double t, Vec3 *pos_g, Vec3 *velocity_g,
                         Vec3 *accel_g) const;
  double GetMeanSeaLevelPosZ() { return msl_pos_z_g_; }

 private:
  void Init();
  void UpdateDerivedStates();
  void UpdateStepHelper(double t);
  void CalcDerivHelper(double t) override;
  void DiscreteStepHelper(double t) override;
  double JonswapSpectrum(double w);
  double TrapzIntegral(double x_start, double x_end, double (Sea::*f)(double),
                       int n_samples);
  double GetWaveHeight(double t, double x, double y) const;

  // Sea parameters
  const SeaSimParams &sea_sim_params_;
  const Environment &environment_;
  const GroundFrame &ground_frame_;
  double *w_i;                // Wave frequency [rad/s].
  double *H_i;                // Wave amplitude [m].
  double *c_i;                // Wavenumber [rad/m].
  double *phi_i;              // Wave phase [rad].
  double msl_pos_z_g_;        // Mean Sea Level in ground frame [m].
  double *wave_transl_coord;  // Wave translational coordinate [m],
                              // for visualization.
  double *wave_x_coord;       // Wave planar x coordinate [m],
                              // for visualization.
  double *wave_y_coord;       // Wave planar y coordinate [m],
                              // for visualization.
  double *wave_elev_g;        // Wave elevation in ground frame [m],
                              // for visualization.

  NamedRandomNumberGenerator rng_;

  // Inputs.

  // Discrete states.

  // Continuous states.

  // Outputs.

  // Derived values.

  DISALLOW_COPY_AND_ASSIGN(Sea);
};

#endif  // SIM_MODELS_SEA_H_
