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

#ifndef SIM_MODELS_RIGID_BODIES_BUOY_H_
#define SIM_MODELS_RIGID_BODIES_BUOY_H_

#include "sim/models/environment.h"
#include "sim/models/model.h"
#include "sim/models/rigid_bodies/rigid_body.h"
#include "sim/models/sea.h"
#include "sim/physics/ground_frame.h"

// This class implements a rigid body "buoy".

class Buoy : public RigidBody {
 public:
  Buoy(const Environment &environment, const GroundFrame &ground_frame,
       const BuoyParams &buoy_params, const BuoySimParams &buoy_sim_param,
       Sea *sea);
  ~Buoy() {}

  void Publish() const override;
  void SetTetherForceMoment(const ForceMomentPos &val) {
    fmx_tether_g_.set_val(val);
  }

  void set_dcm_g2v(const Mat3 &val) { dcm_g2v_.set_val(val); }
  void clear_dcm_g2v() { dcm_g2v_.Clear(); }

  // Returns a general, accelerating reference frame attached to the vessel's
  // coordinates.
  void CalcAcceleratingFrame(ReferenceFrame *accelerating_frame) const override;

  const ReferenceFrame &vessel_frame() const { return vessel_frame_.val(); }
  const Mat3 &dcm_g2v() const { return dcm_g2v_.val(); }
  const Vec3 &Xg() const { return Xg_.val(); }
  const Vec3 &Vg() const { return Vg_.val(); }
  const double &wet_height() const { return wet_height_.val(); }
  const double &wet_volume() const { return wet_volume_.val(); }
  const Vec3 &buoyancy_center_v() const { return buoyancy_center_v_.val(); }
  const Vec3 &water_line_v() const { return water_line_v_.val(); }

 private:
  void Init(const Vec3 &Xg_0, const Vec3 &Vg_0, const Quat &q_0,
            const Vec3 &omega_0);
  void ManageUncertainties();
  void UpdateDerivedStates();
  void DiscreteStepHelper(double t) override;
  void CalcDerivHelper(double t) override;
  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdateDerivedPositionAndAttitudeStates();
  void UpdateWetVolume(double time);
  void UpdateForceMoments(double time);
  void CalcHydroForceMoment(ForceMoment *fm_hydro, double time);
  void CalcTetherForceMoment(ForceMoment *fm_tether);
  void CalcGravityForceMoment(ForceMoment *fm_gravity);
  void CalcMooringForceMoment(ForceMoment *fm_mooring);
  void GetBuoyancyForce(Vec3 *buoyancy_force_g);
  void CalcWaveForce(ForceMoment *fm_wave, double time);
  void GetExternalForceMoment(ForceMoment *fm_hydro, ForceMoment *fm_tether,
                              ForceMoment *fm_gravity, ForceMoment *fm_mooring,
                              ForceMoment *fm_total, double time);

  // Buoy parameters
  const BuoyParams &buoy_params_;
  const BuoySimParams &buoy_sim_params_;
  double mass_;
  Mat3 inertia_tensor_;
  Vec3 center_of_mass_pos_;
  const Environment &environment_;
  const GroundFrame &ground_frame_;
  const Sea *sea_;
  BuoyHydrodynamicsSimParams hydrodynamics_sim_params_;
  BuoyMooringLineSimParams mooring_sim_params_;

  // Inputs.

  // Tether ForceMomentPos in ground coordinates acting at Xg_start.
  State<ForceMomentPos> fmx_tether_g_;

  // Discrete states.

  // Continuous states.

  // Attitude [quaternion].
  ContinuousState<Quat> q_;

  // Angular rate vector [rad/s] relative to the parent (ground) frame,
  // expressed in vessel coordinates.
  ContinuousState<Vec3> omega_;

  // Position [m] of the buoy center of mass, expressed in ground coordinates.
  ContinuousState<Vec3> Xg_center_of_mass_;

  // Velocity [m/s] of the buoy center of mass expressed in ground coordinates.
  ContinuousState<Vec3> Vg_center_of_mass_;

  // Outputs.
  const Quat &q() const { return q_.val(); }
  const Vec3 &omega() const { return omega_.val(); }
  const Vec3 &domega() const { return omega_.deriv(); }
  const Vec3 &Xg_center_of_mass() const { return Xg_center_of_mass_.val(); }
  const Vec3 &Vg_center_of_mass() const { return Vg_center_of_mass_.val(); }
  const Vec3 &dVg_center_of_mass() const { return Vg_center_of_mass_.deriv(); }

  const ForceMoment &fm_hydro() const { return fm_hydro_.val(); }
  const ForceMoment &fm_tether() const { return fm_tether_.val(); }
  const ForceMoment &fm_gravity() const { return fm_gravity_.val(); }
  const ForceMoment &fm_mooring() const { return fm_mooring_.val(); }
  const ForceMomentPos &tether_force_moment_g() const {
    return fmx_tether_g_.val();
  }
  const ForceMoment &fm_total() const { return fm_total_.val(); }

  // Derived values.

  // Reference frame of the buoy (vessel frame). Note that this is *not* an
  // accelerating frame, so it should not be used by the IMU to determine
  // accelerations. See CalcAcceleratingFrame.
  State<ReferenceFrame> vessel_frame_;

  // Position [m] of the vessel frame origin in ground coordinates.
  State<Vec3> Xg_;

  // Velocity [m/s] of the vessel frame origin relative to the ground frame,
  // expressed in ground coordinates.
  State<Vec3> Vg_;

  // Direction cosine matrix between ground and vessel coordinates.
  State<Mat3> dcm_g2v_;

  // Wet height and wet volume.
  State<double> wet_height_;
  State<double> wet_volume_;
  State<Vec3> buoyancy_center_v_;
  State<Vec3> water_line_v_;

  // Yaw angle.
  State<double> yaw_angle_from_eq_;

  // External applied forces and moments.
  State<ForceMoment> fm_hydro_;
  State<ForceMoment> fm_tether_;
  State<ForceMoment> fm_gravity_;
  State<ForceMoment> fm_mooring_;
  State<ForceMoment> fm_total_;

  DISALLOW_COPY_AND_ASSIGN(Buoy);
};

#endif  // SIM_MODELS_RIGID_BODIES_BUOY_H_
