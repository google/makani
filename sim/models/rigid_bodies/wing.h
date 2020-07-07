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

#ifndef SIM_MODELS_RIGID_BODIES_WING_H_
#define SIM_MODELS_RIGID_BODIES_WING_H_

#include <stdint.h>

#include <string>
#include <unordered_map>
#include <vector>

#include "common/c_math/force_moment.h"
#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/faults/faults.h"
#include "sim/models/environment.h"
#include "sim/models/rigid_bodies/rigid_body.h"
#include "sim/physics/aero.h"
#include "sim/physics/aero_types.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"

class Wing : public RigidBody {
  friend class WingTest;

 public:
  Wing(const Environment &environment, const ReferenceFrame &ground_frame,
       const WingParams &wing_params, const AeroSimParams &aero_sim_params,
       const WingSimParams &wing_sim_params, FaultSchedule *faults);
  ~Wing() {}

  double CalcFlapMoment(FlapLabel flap_label) const;

  // Returns the local apparent wind at a position on the kite,
  // accounting for angular rate, local pressure coefficient (if
  // necessary), and upwash variations.  The upwash is
  // modeled by assuming a linear vorticity source runs along the span
  // of the wing whose strength is given by the Kutta-Joukowski
  // theorem:
  //
  //   L = rho V Gamma b.
  //
  void CalcLocalApparentWindB(const Vec3 &pos, double Cp,
                              Vec3 *local_apparent_wind_b) const;

  // Returns a general, accelerating reference frame attached to the
  // wing's body coordinates.
  void CalcAcceleratingFrame(ReferenceFrame *accelerating_frame) const override;

  static void CalcBridlePivot(const WingParams &wing_params,
                              Vec3 *Xg_bridle_pivot);
  static void CalcBridlePoint(const Vec3 &Xg, const Vec3 &Vb, const Vec3 &omega,
                              const Mat3 &dcm_g2b,
                              const WingParams &wing_params,
                              const Vec3 &Xg_last_tether_node,
                              const Vec3 &Vg_bridle_point_last_tether_node,
                              Vec3 *Xg_bridle_point, Vec3 *Vg_bridle_point);

  // The proboscis is a reference position on the wing, to which the
  // constraint system is attached, and which must remain within an
  // allowed region while operating under constraints.
  void CalcProboscisPos(Vec3 *Xg_proboscis) const;

  // Calculates the total external force, not including aerodynamic
  // forces, in ground coordinates applied to the wing.  This is used,
  // in a hack, to calculate the total force on the perch.  See
  // b/19891678.
  void CalcTotalExternalForce(Vec3 *Fg_external) const;

  // Returns the sum of the rotor forces in body coordinates.
  void CalcTotalRotorForce(Vec3 *rotor_force_b) const;

  void Publish() const override;

  // External force setup.
  int32_t AddExternalForce(const std::string &force_name);
  void FinalizeExternalForces();

  // Inputs.
  void set_wind_g(const Vec3 &wind_g__) { wind_g_.set_val(wind_g__); }
  void set_wind_omega_g(const Vec3 &wind_omega_g__) {
    wind_omega_g_.set_val(wind_omega_g__);
  }
  void set_flap_angles(FlapLabel flap_label, double angle) {
    flap_angles_[flap_label].set_val(angle);
  }
  void set_external_forces(int32_t i, const ForceMomentPos &force) {
    external_forces_[i].set_val(force);
  }

  // Outputs.
  const Vec3 &Xg() const { return Xg_.val(); }
  const Vec3 &Vb() const { return Vb_.val(); }
  const Quat &q() const { return q_.val(); }
  const Vec3 &omega() const { return omega_.val(); }

  const Vec3 &dXg_center_of_mass() const { return Xg_center_of_mass_.deriv(); }
  const Vec3 &dVb_center_of_mass() const { return Vb_center_of_mass_.deriv(); }
  const Quat &dq() const { return q_.deriv(); }
  const Vec3 &domega() const { return omega_.deriv(); }

  const ReferenceFrame &frame() const { return frame_.val(); }

  const Mat3 &dcm_g2b() const { return dcm_g2b_.val(); }
  const Vec3 &Vg() const { return Vg_.val(); }
  const Vec3 &omega_hat() const { return omega_hat_.val(); }
  double reynolds_number() const { return reynolds_number_.val(); }

  double alpha() const { return alpha_.val(); }
  double beta() const { return beta_.val(); }
  double v_app_norm() const { return v_app_norm_.val(); }
  double q_bar() const { return q_bar_.val(); }
  const ForceMoment &aero_force_moment_coeffs() const {
    return aero_force_moment_coeffs_.val();
  }
  double lift_coeff() const;
  double side_force_coeff() const;

  const ForceMomentPos &fmx_aero() const { return fmx_aero_.val(); }
  const ForceMomentPos &fmx_gravity() const { return fmx_gravity_.val(); }
  const ForceMomentPos &fmx_disturb() const { return fmx_disturb_.val(); }
  const ForceMomentPos &fmx_total() const { return fmx_total_.val(); }

  const RawAeroCoeffs &raw_aero_coeffs() const {
    return raw_aero_coeffs_.val();
  }
  double total_rotor_thrust_coeff() const {
    return total_rotor_thrust_coeff_.val();
  }

  // Inject states from flight log data. Used for dynamics replay.
  void InjectStates(const Vec3 &Xg_wing, const Vec3 &Vb_wing,
                    const Vec3 &omega_b, const Mat3 &dcm_g2b__);

  void set_total_rotor_thrust_coeff(double val) {
    total_rotor_thrust_coeff_.set_val(val);
  }

 private:
  void Init(const Vec3 &Xg_0, const Vec3 &Vb_0, const Quat &q_0,
            const Vec3 &omega_0);

  // Manage wing uncertainties.
  void ManageUncertainties();

  // Calculates the derivatives of the continuous states
  // (Xg_center_of_mass_, Vb_center_of_mass_, omega_, and q_) based on
  // a force-moment applied at the center of mass.
  void CalcDerivFromForceMoment(const ForceMoment &fm);

  // Calculates the aerodynamic coefficients for the kite.
  void UpdateAeroCoeffs(double thrust_coeff_z1);

  // Sums all the forces on the wing, including gravity, aerodynamic
  // forces, and all the external forces such as the tether and rotor
  // forces.  The total force-moment is referenced to the center of mass.
  // This also updates related states CFM, CFM_avl, and avl_coeffs.
  void UpdateForceMomentPos(double t);

  // Calculates the aerodynamic force on the wing.
  void CalcAeroForceMomentPos(ForceMomentPos *fm) const;

  // Calculates the gravitational force on the wing.
  void CalcGravityForceMomentPos(ForceMomentPos *fm) const;

  // Calculates the force on the wing from the disturbance faults.
  void CalcDisturbForceMomentPos(double t, ForceMomentPos *fm) const;

  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdatePositionAndAttitudeStates();
  void UpdateAeroStates();

  void CalcDerivHelper(double t) override;
  void DiscreteStepHelper(double /*t*/) override;

  int32_t external_force_index(const std::string &force_name) const {
    DCHECK(external_force_indices_.find(force_name) !=
           external_force_indices_.end());
    return external_force_indices_.at(force_name);
  }

  const Vec3 &Xg_center_of_mass() const { return Xg_center_of_mass_.val(); }
  const Vec3 &Vb_center_of_mass() const { return Vb_center_of_mass_.val(); }
  const Vec3 &wind_g() const { return wind_g_.val(); }
  const Vec3 &wind_omega_g() const { return wind_omega_g_.val(); }
  double flap_angles(FlapLabel label) const {
    return flap_angles_[label].val();
  }
  const ForceMomentPos &external_forces(int32_t i) const {
    return external_forces_[i].val();
  }

  // Parameters.
  const WingParams &wing_params_;
  const Aero aero_;
  const WingSimParams &wing_sim_params_;
  Vec3 center_of_mass_pos_;
  double mass_;
  Mat3 moment_of_inertia_;
  Mat3 moment_of_inertia_inv_;
  const Environment &environment_;

  // The reference frame that the wing frame is embedded in.  This is
  // currently assumed to be the inertial frame ground-station frame.
  const ReferenceFrame &ground_frame_;

  const FaultSchedule::FaultFunc fault_func_;

  // Indicates if FinalizeExternalForces has been called.
  bool external_forces_finalized_;
  std::unordered_map<std::string, int32_t> external_force_indices_;

  // Input states.

  // Wind speed [m/s] and wind angular velocity [rad/s] in ground
  // coordinates.
  State<Vec3> wind_g_, wind_omega_g_;

  // Deflections [rad] of the flaps.
  std::vector<State<double>> flap_angles_;

  // A force [N], moment [N-m], position [m] triple describing an
  // external force applied to the wing, e.g. the tether force.
  std::vector<State<ForceMomentPos>> external_forces_;

  // Continuous states.

  // Xg_center_of_mass_ and Vb_center_of_mass refer to the position
  // and velocity of the center-of-mass rather than the aerodynamic
  // reference position.
  ContinuousState<Vec3> Xg_center_of_mass_, Vb_center_of_mass_, omega_;
  ContinuousState<Quat> q_;

  // Output states.

  // Reference frame attached to the wing.  Note that this is *not* an
  // accelerating frame, so it should not be used by the IMU to
  // determine accelerations.  See CalcAcceleratingFrame.
  State<ReferenceFrame> frame_;

  // Velocity [m/s] of the body coordinate center, i.e. not the
  // center-of-mass, in body coordinates.
  State<Vec3> Vb_;

  // Position [m] of the body coordinate center, i.e. not the
  // center-of-mass, in ground coordinates.
  State<Vec3> Xg_;

  // Direction cosine matrix, derived from the quaternion state, that
  // takes vectors in ground coordinates to body coordinates.
  State<Mat3> dcm_g2b_;

  // Velocity [m/s] of the body coordinate center, i.e. not the
  // center-of-mass, in ground coordinates.
  State<Vec3> Vg_;

  // Dimensionless angular rate [#].
  State<Vec3> omega_hat_;

  // Angle-of-attack [rad], sideslip [rad], apparent wind speed [m/s],
  // dynamic pressure [Pa], and Reynolds number [#].
  State<double> alpha_, beta_, v_app_norm_, q_bar_, reynolds_number_;

  // Aerodynamic force and moment coefficients [#].
  State<ForceMoment> aero_force_moment_coeffs_;

  // Forces and moments.
  State<ForceMomentPos> fmx_aero_;
  State<ForceMomentPos> fmx_gravity_;
  State<ForceMomentPos> fmx_disturb_;
  State<ForceMomentPos> fmx_total_;

  // Raw coefficients [#] from the AVL and DVL databases.  These are
  // only for diagnostic purposes in the telemetry and should not be
  // used elsewhere in the class.
  State<RawAeroCoeffs> raw_aero_coeffs_;

  // String labels of external force indices.
  std::vector<std::string> rotor_force_labels_;
  std::vector<std::string> blown_wing_force_labels_;

  // States for rotor thrust coefficient. We need a State to ingest the quantity
  // from the rotor models, and a DiscreteState to record the value to use at
  // the next time step in order to avoid an algebraic loop.
  State<double> total_rotor_thrust_coeff_;
  DiscreteState<double> total_rotor_thrust_coeff_z1_;

  DISALLOW_COPY_AND_ASSIGN(Wing);
};

#endif  // SIM_MODELS_RIGID_BODIES_WING_H_
