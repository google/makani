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

#ifndef SIM_MODELS_ENVIRONMENT_H_
#define SIM_MODELS_ENVIRONMENT_H_

#include <stdint.h>

#include <memory>
#include <stack>
#include <string>

#include "common/c_math/force_moment.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/math/util.h"
#include "sim/models/model.h"
#include "sim/physics/reference_frame.h"
#include "sim/physics/wind.h"
#include "sim/sim_types.h"

class Environment : public Model {
  friend class EnvironmentTest;

 public:
  Environment(const IecSimParams &iec_sim_params,
              const PhysSimParams &phys_sim_params,
              const PhysParams &phys_params,
              const WindSensorParams &wind_sensor_params,
              const GroundFrameParams &ground_frame_params);
  ~Environment();

  void CalcWind(const Vec3 &pos_g, Vec3 *wind_g, Vec3 *wind_omega_g) const;
  void CalcWind(const Vec3 &pos_g, Vec3 *wind_g) const;

  const ReferenceFrame &ned_frame() const { return ned_frame_; }
  double air_density() const { return phys_sim_params_.air_density; }
  double dynamic_viscosity() const {
    return phys_sim_params_.dynamic_viscosity;
  }
  double g() const { return phys_params_.g; }
  const Vec3 &g_ned() const { return phys_params_.g_g; }
  const Vec3 &mag_ned() const { return phys_params_.mag_ned; }
  double pressure(double altitude) const {
    return phys_params_.P_atm - air_density() * g() * altitude;
  }

  // This function is deprecated.  Use g_ned() followed by a rotation
  // to the local reference frame instead.
  const Vec3 &g_g() const { return phys_params_.g_g; }

 private:
  void DiscreteStepHelper(double t) override;

  void UpdateWindSpeed(double t);

  // Dryden turbulence model.
  void CalcDrydenTurbulence(double h, Vec3 *wind_turbulence_mw) const;
  void UpdateDrydenTurbulence(double t, double h, double V_app_norm);
  void CalcDrydenTurbulenceLengthScale(double h_ft, Vec3 *length_scale) const;
  void CalcDrydenTurbulenceIntensities(double h_ft, double wind20_ft,
                                       Vec3 *sigma) const;

  // IEC test case.
  void CalcIecMeanWind(double t, const Vec3 &pos_mw, IecDesignLoadCase iec_case,
                       Vec3 *mean_wind_mw) const;

  // Calculate a white-noise component to be added to the wind-field at all
  // points.  The scaling of this noise is based on numbers from the IEC
  // specification, but the there is no spatial variation and the temporal
  // variation does not correspond to the specification.
  void CalcIecSurrogateTurbulence(IecDesignLoadCase iec_case,
                                  Vec3 *wind_turbulence_mw) const;

  // Get the height above ground level of the wind sensor.
  // Returns the height of the wind sensor above ground level.
  // TODO: Replace the use of the wind sensor as reference height
  // with a more general reference height + reference wind pair.
  double GetMeasuredWindHeightAgl() const;

  const Vec3 &noise() const { return noise_.val(); }
  const Vec3 &noise_f_z1() const { return noise_f_z1_.val(); }
  const Vec3 &noise2_f_z1() const { return noise2_f_z1_.val(); }

  // Parameters.
  const IecSimParams &iec_sim_params_;
  const PhysParams &phys_params_;
  const PhysSimParams &phys_sim_params_;
  const WindSensorParams &wind_sensor_params_;
  const double ground_z_;

  // DCM from the mean wind frame to the ground frame.
  Mat3 dcm_mw2g_;

  NamedRandomNumberGenerator rng_;

  // Finite difference step size [m] for calculating wind vorticity.
  // Equation (7) in the McGrath reference mentioned in the vorticity
  // calculation section of the environment.cc file recommends using 0.85b (22m)
  // for the roll and yaw directions, and horizontal-tail-to-quarter-chord
  // length (8m) for pitch.
  // Using a blanket 10 m step for now until we can properly reformulate the
  // vorticity calculation method to equation (7): b/78169176.
  static constexpr double kFiniteDifferenceStep = 1e1;

  // Wind model for IEC shear scenarios.
  const sim::physics::wind::IecWindModel iec_wind_model_;
  std::unique_ptr<sim::physics::wind::FrozenTurbulenceWindDatabase>
      wind_database_;

  // North-east-down reference frame.
  const ReferenceFrame ned_frame_;

  DiscreteState<double> wind_speed_;
  DiscreteState<double> wind_speed_target_;
  std::stack<WindSpeedOffset> wind_speed_offsets_;
  DiscreteState<Vec3> noise_, noise_f_z1_, noise2_f_z1_;

  DISALLOW_COPY_AND_ASSIGN(Environment);
};

#endif  // SIM_MODELS_ENVIRONMENT_H_
