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

#include "sim/models/rigid_bodies/wing.h"

#include <glog/logging.h>
#include <math.h>

#include <exception>
#include <string>
#include <vector>

#include "common/c_math/mat3.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/c_math/waveform.h"
#include "control/sensor_util.h"
#include "sim/faults/faults.h"
#include "sim/models/actuators/servo.h"
#include "sim/models/environment.h"
#include "sim/models/rigid_bodies/rigid_body.h"
#include "sim/physics/aero.h"
#include "sim/physics/aero_frame.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "system/labels.h"

Wing::Wing(const Environment &environment, const ReferenceFrame &ground_frame,
           const WingParams &wing_params, const AeroSimParams &aero_sim_params,
           const WingSimParams &wing_sim_params, FaultSchedule *faults)
    : RigidBody("Wing"),
      wing_params_(wing_params),
      aero_(aero_sim_params),
      wing_sim_params_(wing_sim_params),
      center_of_mass_pos_(wing_params_.center_of_mass_pos),
      mass_(wing_params_.m),
      moment_of_inertia_(wing_params_.I),
      moment_of_inertia_inv_(wing_params_.I_inv),
      environment_(environment),
      ground_frame_(ground_frame),
      fault_func_(faults->ClaimFaultFunc(
          full_name(),
          {// Parameters are force x, y, z components [N] and frequency [Hz].
           {kSimFaultDisturbanceBodyForceSine, 4U},
           // Parameters are force x, y, z components [N].
           {kSimFaultDisturbanceBodyForceStep, 3U},
           // Parameters are torque x, y, z components [N-m] and frequency [Hz].
           {kSimFaultDisturbanceBodyTorqueSine, 4U},
           // Parameters are torque x, y, z components [N-m].
           {kSimFaultDisturbanceBodyTorqueStep, 3U}})),
      external_forces_finalized_(false),
      external_force_indices_(),
      wind_g_(new_derived_value(), "wind_g"),
      wind_omega_g_(new_derived_value(), "wind_omega_g"),
      flap_angles_(),
      external_forces_(),
      Xg_center_of_mass_(new_continuous_state(), "Xg_center_of_mass"),
      Vb_center_of_mass_(new_continuous_state(), "Vb_center_of_mass"),
      omega_(new_continuous_state(), "omega", wing_sim_params_.omega_0),
      q_(new_continuous_state(), "q", wing_sim_params_.q_0),
      frame_(new_derived_value(), "frame"),
      Vb_(new_derived_value(), "Vb"),
      Xg_(new_derived_value(), "Xg"),
      dcm_g2b_(new_derived_value(), "dcm_g2b"),
      Vg_(new_derived_value(), "Vg"),
      omega_hat_(new_derived_value(), "omega_hat"),
      alpha_(new_derived_value(), "alpha"),
      beta_(new_derived_value(), "beta"),
      v_app_norm_(new_derived_value(), "v_app_norm"),
      q_bar_(new_derived_value(), "q_bar"),
      reynolds_number_(new_derived_value(), "reynolds_number"),
      aero_force_moment_coeffs_(new_derived_value(),
                                "aero_force_moment_coeffs"),
      fmx_aero_(new_derived_value(), "fmx_aero"),
      fmx_gravity_(new_derived_value(), "fmx_gravity"),
      fmx_disturb_(new_derived_value(), "fmx_disturb"),
      fmx_total_(new_derived_value(), "fmx_total"),
      raw_aero_coeffs_(new_derived_value(), "raw_aero_coeffs"),
      rotor_force_labels_(),
      blown_wing_force_labels_(),
      total_rotor_thrust_coeff_(new_derived_value(), "total_rotor_thrust_coeff",
                                0.0),
      total_rotor_thrust_coeff_z1_(new_discrete_state(),
                                   "total_rotor_thrust_coeff_z1", 0.0) {
  ManageUncertainties();

  for (int32_t i = 0; i < kNumMotors; ++i) {
    rotor_force_labels_.push_back("motors[" + std::to_string(i) + "]");
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    blown_wing_force_labels_.push_back("blown_wing[" + std::to_string(i) + "]");
  }

  Init(wing_sim_params_.Xg_0, wing_sim_params_.Vb_0, wing_sim_params_.q_0,
       wing_sim_params_.omega_0);
}

void Wing::ManageUncertainties() {
  // Scale mass.
  mass_ *= wing_sim_params_.mass_prop_uncertainties.mass_scale;

  // Offset center of mass.
  Vec3Add(&wing_sim_params_.mass_prop_uncertainties.center_of_mass_offset,
          &center_of_mass_pos_, &center_of_mass_pos_);

  // Scale inertia tensor.
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      moment_of_inertia_.d[i][j] *= Sqrt(
          wing_sim_params_.mass_prop_uncertainties.moment_of_inertia_scale[i] *
          wing_sim_params_.mass_prop_uncertainties.moment_of_inertia_scale[j]);
      moment_of_inertia_inv_.d[i][j] /= Sqrt(
          wing_sim_params_.mass_prop_uncertainties.moment_of_inertia_scale[i] *
          wing_sim_params_.mass_prop_uncertainties.moment_of_inertia_scale[j]);
    }
  }
}

double Wing::lift_coeff() const {
  Vec3 CF_w;
  RotBToW(&aero_force_moment_coeffs().force, alpha(), beta(), &CF_w);
  return -CF_w.z;
}

double Wing::side_force_coeff() const {
  Vec3 CF_w;
  RotBToW(&aero_force_moment_coeffs().force, alpha(), beta(), &CF_w);
  return CF_w.y;
}

// The servo-flap system is modeled by servo.cc, Wing::CalcServoMoment, and
// Wing::ServosToFlaps.
double Wing::CalcFlapMoment(FlapLabel flap_label) const {
  DCHECK_LT(flap_label, flap_angles_.size())
      << "Flap label index is larger than flap_angles_ vector size.";

  double angle_of_incidence = (flap_label == kFlapRud) ? beta() : alpha();
  double flap_angle = flap_angles(flap_label);
  double aero_moment =
      q_bar() * wing_params_.A * wing_params_.c *
      aero_.CalcHingeCM(flap_label, angle_of_incidence, flap_angle);

  // Apply a spring force once the flap reaches an unrealistic angle
  // to prevent it from wrapping around.  The angle limit [rad] and
  // the angular spring constant [N-m/rad] here are not tied to
  // physical values; they are simply intended to keep the flap from
  // moving to an unrealistic angle.
  constexpr double angle_limit = 0.9 * PI;
  constexpr double angular_spring_const = 100000.0;
  double spring_moment = -angular_spring_const * Sign(flap_angle) *
                         fmax(fabs(flap_angle) - angle_limit, 0.0);

  return aero_moment + spring_moment;
}

// TODO: Reconsider the order of local flow transformations
// and match/invert the order in the state estimator.
void Wing::CalcLocalApparentWindB(const Vec3 &pos, double Cp,
                                  Vec3 *local_apparent_wind_b) const {
  ReferenceFrame local_frame(frame(), pos);
  local_frame.TransformFrom(ground_frame_, ReferenceFrame::kVelocity, wind_g(),
                            local_apparent_wind_b);

  // Apply pressure coefficient for any local flow effects.
  Vec3Scale(local_apparent_wind_b, Sqrt(1.0 - Cp), local_apparent_wind_b);

  // Calculate the wing's bound vortex circulation due to freestream.
  double gamma =
      0.5 * v_app_norm() * wing_params_.c * Saturate(lift_coeff(), 0.0, 3.75);

  double d = hypot(pos.x, pos.z);
  DCHECK_LT(DBL_EPSILON, d);
  double theta_1 = atan2(d, pos.y + wing_params_.b / 2.0);
  double theta_2 = atan2(d, pos.y - wing_params_.b / 2.0);

  // Houghton & Carpenter Eq. 5.10.
  double induced_vel = (cos(theta_1) - cos(theta_2)) * gamma / (4.0 * M_PI * d);

  local_apparent_wind_b->x += induced_vel * pos.z / d;
  local_apparent_wind_b->z -= induced_vel * pos.x / d;

  // Calculate the induced flow from the bound vortex on the pylon lifting
  // surface (see b/78480146).

  // Takes total side force and assumes equal contributions from the pylons and
  // the vertical tail to get the per-pylon side force coefficient.
  int32_t num_pylons = kNumMotors / 2;
  int32_t num_vertical_tails = 1;
  double CY_pylon = Saturate(side_force_coeff(), -0.5, 0.5) /
                    (num_pylons + num_vertical_tails);

  for (int32_t i = 0; i < num_pylons; ++i) {
    // Scales the side force coefficient to the pylon reference area for the
    // circulation calculation. Pylon ref chord cancels out of the equation.
    double gamma_s =
        (0.5 * v_app_norm() * CY_pylon * wing_params_.A / wing_params_.b_pylon);
    double d_s = hypot(pos.x - wing_params_.pylon_pos[i].x,
                       pos.z - wing_params_.pylon_pos[i].z);
    double theta_1_s = atan2(d_s, pos.z - (wing_params_.pylon_pos[i].z +
                                           wing_params_.b_pylon / 2.0));
    double theta_2_s = atan2(d_s, pos.z - (wing_params_.pylon_pos[i].z -
                                           wing_params_.b_pylon / 2.0));
    double induced_vel_s =
        (cos(theta_1_s) - cos(theta_2_s)) * gamma_s / (4.0 * M_PI * d_s);
    local_apparent_wind_b->x +=
        (induced_vel_s * (pos.y - wing_params_.pylon_pos[i].y) / d_s);
    local_apparent_wind_b->y +=
        (-induced_vel_s * (pos.x - wing_params_.pylon_pos[i].x) / d_s);
  }
}

// TODO: This is calculated with a function, rather than
// treated as a state, because it depends on values calculated in
// CalcDeriv (Vb_center_of_mass, domega). Another possibility would be to run
// this in DiscreteUpdate, but that has no ordering between the models.
void Wing::CalcAcceleratingFrame(ReferenceFrame *accelerating_frame) const {
  Vec3 omega_g, domega_g, Vg_center_of_mass;
  frame().RotateTo(ground_frame_, omega(), &omega_g);
  frame().RotateTo(ground_frame_, domega(), &domega_g);
  frame().RotateTo(ground_frame_, Vb_center_of_mass(), &Vg_center_of_mass);

  // Calculate the acceleration in the parent frame.  This calculation
  // is placing velocities in the position field and accelerations in
  // the velocity field because it is using the reference frames to
  // transform a derivative.  This is equivalent to:
  //
  //   Ag = dcm_b2g * (omega_b x Vb + dVb/dt)
  //
  ReferenceFrame rotating_frame(ground_frame_, kVec3Zero, kVec3Zero, dcm_g2b(),
                                omega_g);
  ReferenceFrame velocity_offset_frame(rotating_frame, Vb_center_of_mass());
  Vec3 Ag_center_of_mass;
  velocity_offset_frame.TransformTo(ground_frame_, ReferenceFrame::kVelocity,
                                    dVb_center_of_mass(), &Ag_center_of_mass);

  // Build the accelerating center-of-mass frame.
  ReferenceFrame acc_frame_com(ground_frame_, Xg_center_of_mass(),
                               Vg_center_of_mass, Ag_center_of_mass, dcm_g2b(),
                               omega_g, domega_g);
  Vec3 acc_frame_pos;
  Vec3Scale(&center_of_mass_pos_, -1.0, &acc_frame_pos);

  // Transform to the standard accelerating frame.
  *accelerating_frame = ReferenceFrame(acc_frame_com, acc_frame_pos);
}

void Wing::CalcBridlePivot(const WingParams &wing_params,
                           Vec3 *Xb_bridle_pivot) {
  Vec3Scale(Vec3Add(&wing_params.bridle_pos[kBridlePort],
                    &wing_params.bridle_pos[kBridleStar], Xb_bridle_pivot),
            0.5, Xb_bridle_pivot);
  Xb_bridle_pivot->y += wing_params.bridle_y_offset;
}

void Wing::CalcBridlePoint(const Vec3 &Xg, const Vec3 &Vb, const Vec3 &omega,
                           const Mat3 &dcm_g2b, const WingParams &wing_params,
                           const Vec3 &Xg_last_tether_node,
                           const Vec3 &Vg_last_tether_node,
                           Vec3 *Xg_bridle_point, Vec3 *Vg_bridle_point) {
  Vec3 Xg_bridle_pivot, Rg_bridle_pivot_to_tether, Rg_knot_vec;
  Vec3 Xb_bridle_pivot;
  CalcBridlePivot(wing_params, &Xb_bridle_pivot);

  Vec3 dcm_g2b_y = {dcm_g2b.d[1][0], dcm_g2b.d[1][1], dcm_g2b.d[1][2]};

  // Calculate the unit vector between the bridle pivot line and the
  // last tether node in ground coordinates.
  Vec3Add(&Xg, Mat3TransVec3Mult(&dcm_g2b, &Xb_bridle_pivot, &Xg_bridle_pivot),
          &Xg_bridle_pivot);
  Vec3Sub(&Xg_last_tether_node, &Xg_bridle_pivot, &Rg_bridle_pivot_to_tether);
  Vec3LinComb(1.0, &Rg_bridle_pivot_to_tether,
              -Vec3Dot(&dcm_g2b_y, &Rg_bridle_pivot_to_tether), &dcm_g2b_y,
              &Rg_knot_vec);
  const double pivot_to_tether_body_xz_norm =
      fmax(Vec3Norm(&Rg_knot_vec), 1e-9);
  Vec3Normalize(&Rg_knot_vec, &Rg_knot_vec);

  // Calculate the bridle point, assuming a fixed bridle radius.
  Vec3Add(&Xg_bridle_pivot,
          Vec3Scale(&Rg_knot_vec, wing_params.bridle_rad, Xg_bridle_point),
          Xg_bridle_point);

  // Calculate the bridle point in body coordinates.
  Vec3 Xb_bridle;
  Mat3Vec3Mult(&dcm_g2b, Vec3Sub(Xg_bridle_point, &Xg, &Xb_bridle), &Xb_bridle);

  // Calculate the vector from bridle pivot to last tether node
  // in body coordinates.
  Vec3 Rb_bridle_pivot_to_tether;
  Mat3Vec3Mult(&dcm_g2b, &Rg_bridle_pivot_to_tether,
               &Rb_bridle_pivot_to_tether);

  // Calculate the velocity of the bridle pivot in body coordinates.
  Vec3 Vb_bridle_pivot;
  Vec3Add(&Vb, Vec3Cross(&omega, &Xb_bridle_pivot, &Vb_bridle_pivot),
          &Vb_bridle_pivot);

  // Calculate the body x-z plane component of the relative velocity of the
  // last tether node w.r.t. the bridle pivot.
  Vec3 Vb_tether_from_pivot;
  Vec3Sub(Mat3Vec3Mult(&dcm_g2b, &Vg_last_tether_node, &Vb_tether_from_pivot),
          &Vb_bridle_pivot, &Vb_tether_from_pivot);
  Vec3 Vb_tether_from_pivot_plane = Vb_tether_from_pivot;
  Vb_tether_from_pivot_plane.y = 0.0;

  // Calculate the bridle velocity in body coordinates.
  Vec3 Rb_knot_vec;
  Mat3Vec3Mult(&dcm_g2b, &Rg_knot_vec, &Rb_knot_vec);
  Vec3 Vb_bridle_point;
  // Remove the velocity component in the direction the bridle cannot move.
  Vec3LinComb(1.0, &Vb_tether_from_pivot_plane,
              -Vec3Dot(&Rb_knot_vec, &Vb_tether_from_pivot_plane), &Rb_knot_vec,
              &Vb_bridle_point);
  Vec3LinComb(1.0, &Vb_bridle_pivot,
              wing_params.bridle_rad / pivot_to_tether_body_xz_norm,
              &Vb_bridle_point, &Vb_bridle_point);

  // Rotate the bridle velocity to ground coordinates.
  Mat3TransVec3Mult(&dcm_g2b, &Vb_bridle_point, Vg_bridle_point);
}

void Wing::CalcProboscisPos(Vec3 *Xg_proboscis) const {
  frame().TransformTo(ground_frame_, ReferenceFrame::kPosition,
                      wing_params_.proboscis_pos, Xg_proboscis);
}

void Wing::CalcTotalExternalForce(Vec3 *Fg_external) const {
  *Fg_external = kVec3Zero;
  for (int32_t i = 0; i < static_cast<int32_t>(external_forces_.size()); ++i) {
    Vec3 force_g;
    frame().RotateTo(ground_frame_, external_forces(i).force, &force_g);
    Vec3Add(Fg_external, &force_g, Fg_external);
  }
}

void Wing::CalcTotalRotorForce(Vec3 *rotor_force_b) const {
  *rotor_force_b = kVec3Zero;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    int32_t force_index = external_force_index(rotor_force_labels_[i]);
    Vec3Add(rotor_force_b, &external_forces(force_index).force, rotor_force_b);
  }
}

void Wing::Publish() const {
  sim_telem.wing.Xg = Xg();
  sim_telem.wing.Vb = Vb();
  sim_telem.wing.Vg = Vg();
  ReferenceFrame wing_accelerating_frame;
  CalcAcceleratingFrame(&wing_accelerating_frame);
  wing_accelerating_frame.TransformFrom(environment_.ned_frame(),
                                        ReferenceFrame::kAcceleration,
                                        kVec3Zero, &sim_telem.wing.Ab);
  Vec3Scale(&sim_telem.wing.Ab, -1.0, &sim_telem.wing.Ab);
  sim_telem.wing.dVb_center_of_mass = dVb_center_of_mass();
  sim_telem.wing.omega = omega();
  sim_telem.wing.domega = domega();
  sim_telem.wing.q = q();
  sim_telem.wing.dcm_g2b = dcm_g2b();
  sim_telem.wing.reynolds_number = reynolds_number();

  Vec3 eulers;
  QuatToAngle(&q(), kRotationOrderZyx, &eulers.z, &eulers.y, &eulers.x);
  sim_telem.wing.eulers = eulers;

  for (int32_t i = 0; i < kNumFlaps; ++i) {
    sim_telem.wing.flaps[i] = flap_angles(static_cast<FlapLabel>(i));
  }

  sim_telem.wing.wind_g = wind_g();
  sim_telem.wing.wind_omega_g = wind_omega_g();

  // Compute rotor force-moments and blown wing force-moments.
  ForceMomentPos fmx_rotors = {kVec3Zero, kVec3Zero, center_of_mass_pos_};
  ForceMomentPos fmx_blown_wing = {kVec3Zero, kVec3Zero, center_of_mass_pos_};
  for (int32_t i = 0; i < kNumMotors; ++i) {
    int32_t force_index = external_force_index(rotor_force_labels_[i]);
    ForceMomentPosAdd(&fmx_rotors, &external_forces(force_index), &fmx_rotors);
    int32_t force_index_bw = external_force_index(blown_wing_force_labels_[i]);
    ForceMomentPosAdd(&fmx_blown_wing, &external_forces(force_index_bw),
                      &fmx_blown_wing);
  }

  sim_telem.wing.rotor_thrust_moment.thrust =
      // TODO: With the inclusion of edgewise forces, rotor thrust
      // is no longer Vec3Norm of all rotor forces.
      Sign(fmx_rotors.force.x) * Vec3Norm(&fmx_rotors.force);
  sim_telem.wing.rotor_thrust_moment.moment = fmx_rotors.moment;

  // Force and moment outputs, all relative to the center of mass.
  ForceMomentPos fmx_out;
  fmx_out.pos = center_of_mass_pos_;

  ForceMomentPosRef(&fmx_aero(), &fmx_out);
  sim_telem.wing.fm_aero.force = fmx_out.force;
  sim_telem.wing.fm_aero.moment = fmx_out.moment;

  ForceMomentPosRef(&fmx_gravity(), &fmx_out);
  sim_telem.wing.fm_gravity.force = fmx_out.force;
  sim_telem.wing.fm_gravity.moment = fmx_out.moment;

  ForceMomentPosRef(&external_forces(external_force_index("tether")), &fmx_out);
  sim_telem.wing.fm_tether.force = fmx_out.force;
  sim_telem.wing.fm_tether.moment = fmx_out.moment;

  ForceMomentPosRef(&fmx_rotors, &fmx_out);
  sim_telem.wing.fm_rotors.force = fmx_out.force;
  sim_telem.wing.fm_rotors.moment = fmx_out.moment;

  ForceMomentPosRef(&fmx_blown_wing, &fmx_out);
  sim_telem.wing.fm_blown_wing.force = fmx_out.force;
  sim_telem.wing.fm_blown_wing.moment = fmx_out.moment;

  ForceMomentPosRef(&fmx_disturb(), &fmx_out);
  sim_telem.wing.fm_disturb.force = fmx_out.force;
  sim_telem.wing.fm_disturb.moment = fmx_out.moment;

  ForceMomentPosRef(&fmx_total(), &fmx_out);
  sim_telem.wing.fm_total.force = fmx_out.force;
  sim_telem.wing.fm_total.moment = fmx_out.moment;

  // Apparent wind.
  sim_telem.wing.apparent_wind_b.v = v_app_norm();
  sim_telem.wing.apparent_wind_b.alpha = alpha();
  sim_telem.wing.apparent_wind_b.beta = beta();

  const Vec3 &tether_force_b =
      external_forces(external_force_index("tether")).force;
  if (tether_force_b.z > 0.0) {
    TetherForceCartToSph(&tether_force_b, &sim_telem.wing.tether_force_b);
  } else {
    // If the body z tether force is negative, recognize that the last tether
    // element was in compression and adjust the calculations to avoid angles
    // not physically possible.
    // NOTE: Consider using the position of the bridle knot and the
    // last tether node to calculate the tether angles.
    sim_telem.wing.tether_force_b.tension = -Vec3Norm(&tether_force_b);
    sim_telem.wing.tether_force_b.roll =
        atan2(tether_force_b.y, Vec3XzNorm(&tether_force_b));
    sim_telem.wing.tether_force_b.pitch =
        atan2(-tether_force_b.x, -tether_force_b.z);
  }

  sim_telem.wing.constraint_tension =
      (*g_sim.sim_opt & kSimOptConstraintSystem)
          ? Vec3Norm(&external_forces(external_force_index("constraint")).force)
          : 0.0;

  Vec3 proboscis_pos_g;
  CalcProboscisPos(&proboscis_pos_g);
  sim_telem.wing.proboscis_pos_g = proboscis_pos_g;

  // Lift and drag coefficients.
  Vec3 CF_w;
  RotBToW(&raw_aero_coeffs().force_moment_coeff.force, alpha(), beta(), &CF_w);
  sim_telem.wing.CD = -CF_w.x;
  sim_telem.wing.CL = -CF_w.z;
}

void Wing::InjectStates(const Vec3 &Xg_wing, const Vec3 &Vb_wing,
                        const Vec3 &omega_b, const Mat3 &dcm_g2b__) {
  {
    Vec3 cg_offset_g;
    Mat3TransVec3Mult(&dcm_g2b__, &center_of_mass_pos_, &cg_offset_g);

    Vec3 Xg_cg;
    Vec3Add(&Xg_wing, &cg_offset_g, &Xg_cg);
    Xg_center_of_mass_.set_val(Xg_cg);
  }

  {
    Vec3 Vb_cg;
    Vec3Cross(&omega_b, &center_of_mass_pos_, &Vb_cg);
    Vec3Add(&Vb_wing, &Vb_cg, &Vb_cg);
    Vb_center_of_mass_.set_val(Vb_cg);
  }

  omega_.set_val(omega_b);

  Quat q__;
  DcmToQuat(&dcm_g2b__, &q__);
  q_.set_val(q__);
}

void Wing::UpdatePositionAndAttitudeStates() {
  // Update dcm_g2b.
  // NOTE(b/111892140): The simulator uses error feedback to keep
  // the quaternion representing the attitude state normalized.
  // Here we check the norm of that quaternion.
  const double quat_norm_difference_from_unity = fabs(QuatMod(&q()) - 1.0);
  if (quat_norm_difference_from_unity > 1e-3) {
    LOG(WARNING) << "|QuatMod(q) - 1.0| == " << quat_norm_difference_from_unity;
  }

  // On the theory that this mostly happens when the ODE solver
  // takes too large of a time step, here we ask the ODE solver to
  // backtrack.
  if (quat_norm_difference_from_unity > 1e-3) {
    throw std::runtime_error("|QuatMod(q) - 1.0| too large");
  }

  Mat3 dcm_g2b__;
  QuatToDcm(&q(), &dcm_g2b__);
  dcm_g2b_.set_val(dcm_g2b__);

  Vec3 Xg__, center_of_mass_offset;
  Mat3TransVec3Mult(&dcm_g2b(), &center_of_mass_pos_, &center_of_mass_offset);
  Vec3Sub(&Xg_center_of_mass(), &center_of_mass_offset, &Xg__);
  Xg_.set_val(Xg__);

  Vec3 Vb__;
  Vec3Cross(&center_of_mass_pos_, &omega(), &Vb__);
  Vec3Add(&Vb_center_of_mass(), &Vb__, &Vb__);
  Vb_.set_val(Vb__);

  Vec3 Vg__;
  Vg_.set_val(*Mat3TransVec3Mult(&dcm_g2b(), &Vb(), &Vg__));

  Vec3 omega_g;
  Mat3TransVec3Mult(&dcm_g2b(), &omega(), &omega_g);
  frame_.set_val(ReferenceFrame(ground_frame_, Xg(), Vg(), dcm_g2b(), omega_g));
}

void Wing::UpdateAeroStates() {
  Vec3 wind_b, V_app_b;
  Mat3Vec3Mult(&dcm_g2b(), &wind_g(), &wind_b);
  Vec3Sub(&Vb(), &wind_b, &V_app_b);

  double v_app_norm__ = Vec3NormBound(&V_app_b, 1e-4);
  v_app_norm_.set_val(v_app_norm__);
  alpha_.set_val(atan2(V_app_b.z, V_app_b.x));
  beta_.set_val(asin(V_app_b.y / v_app_norm__));
  q_bar_.set_val(0.5 * environment_.air_density() * v_app_norm__ *
                 v_app_norm__);
  reynolds_number_.set_val(v_app_norm__ * wing_params_.c *
                           environment_.air_density() /
                           environment_.dynamic_viscosity());

  const Vec3 length_scale = {wing_params_.b, wing_params_.c, wing_params_.b};
  Vec3 omega_hat__;
  // Add the wind-induced apparent rates, then normalize.
  Mat3Vec3Mult(&dcm_g2b(), &wind_omega_g(), &omega_hat__);
  Vec3Add(&omega(), &omega_hat__, &omega_hat__);
  Vec3Mult(&omega_hat__, &length_scale, &omega_hat__);
  Vec3Scale(&omega_hat__, 1.0 / v_app_norm__ / 2.0, &omega_hat__);
  omega_hat_.set_val(omega_hat__);

  UpdateAeroCoeffs(total_rotor_thrust_coeff_z1_.val());
}

void Wing::DiscreteStepHelper(double t) {
  total_rotor_thrust_coeff_z1_.DiscreteUpdate(t, total_rotor_thrust_coeff());
}

void Wing::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(1,
                   [this](double /*t*/) { UpdatePositionAndAttitudeStates(); });

  connections->Add(2, [this](double /*t*/) { UpdateAeroStates(); });

  connections->Add(5, [this](double t) {
    // Calculate all the forces on the wing.
    UpdateForceMomentPos(t);
  });
}

void Wing::CalcDerivHelper(double /*t*/) {
  // Compute accelerations.
  ForceMoment fm_tot = {fmx_total().force, fmx_total().moment};
  CalcDerivFromForceMoment(fm_tot);
}

void Wing::Init(const Vec3 &Xg_0, const Vec3 &Vb_0, const Quat &q_0,
                const Vec3 &omega_0) {
  // Construct an initial reference frame, which is used to convert
  // the initial conditions in body coordinates to equivalent values
  // for the center-of-mass.
  Mat3 dcm_g2b__;
  QuatToDcm(&q_0, &dcm_g2b__);
  Vec3 Vg_0;
  Mat3TransVec3Mult(&dcm_g2b__, &Vb_0, &Vg_0);

  Vec3 omega_g;
  Mat3TransVec3Mult(&dcm_g2b__, &omega_0, &omega_g);

  ReferenceFrame frame_avl(ground_frame_, Xg_0, Vg_0, dcm_g2b__, omega_g);
  ReferenceFrame frame_cg(frame_avl, center_of_mass_pos_);

  Vec3 Xg_center_of_mass__;
  frame_cg.TransformOriginTo(ground_frame_, ReferenceFrame::kPosition,
                             &Xg_center_of_mass__);
  Xg_center_of_mass_.set_val(Xg_center_of_mass__);

  Vec3 Vb_center_of_mass__;
  frame_cg.TransformOriginFrom(ground_frame_, ReferenceFrame::kVelocity,
                               &Vb_center_of_mass__);
  Vec3Scale(&Vb_center_of_mass__, -1.0, &Vb_center_of_mass__);
  Vb_center_of_mass_.set_val(Vb_center_of_mass__);

  flap_angles_.reserve(kNumFlaps);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    flap_angles_.emplace_back(new_derived_value(),
                              "flap_angles[" + std::to_string(i) + "]", 0.0);
  }

  // This is required to setup values such as dcm_g2b_
  // used in initializing the perch.
  UpdatePositionAndAttitudeStates();
}

int32_t Wing::AddExternalForce(const std::string &force_name) {
  CHECK(!external_forces_finalized_);
  CHECK(external_force_indices_.find(force_name) ==
        external_force_indices_.end());
  int32_t index = static_cast<int32_t>(external_force_indices_.size());
  external_force_indices_[force_name] = index;
  return index;
}

void Wing::FinalizeExternalForces() {
  CHECK(!external_forces_finalized_);

  std::vector<std::string> ordered_force_names(external_force_indices_.size());
  for (const auto &name_and_index : external_force_indices_) {
    ordered_force_names[name_and_index.second] = name_and_index.first;
  }

  external_forces_.reserve(external_force_indices_.size());
  for (const std::string &force_name : ordered_force_names) {
    external_forces_.emplace_back(new_derived_value(),
                                  "external_forces/" + force_name,
                                  kForceMomentPosZero);
  }

  // Clear the derived values set in Init() and update the class again
  // now that it includes the external forces.
  ClearDerivedValues();
  UpdatePositionAndAttitudeStates();

  external_forces_finalized_ = true;
  SetupDone();
}

void Wing::CalcDerivFromForceMoment(const ForceMoment &fm) {
  // Calculate the derivatives of position and velocity states.
  Vec3 dXg_center_of_mass__, dVb_center_of_mass__;
  {
    //
    // If the position and velocity states were both kept in ground coordinates,
    // this would be as simple as:
    //
    //   d/dt Xg_cm = Vg_cm
    //   d/dt Vg_cm = Ag_cm, where Ag_cm = (1/m) f_g.
    //
    // Because the velocity state is instead tracked in body coordinates, we
    // incur a term due to the changing attitude, given by the "transport
    // theorem"[*]:
    //
    //   ^B d/dt v = ^G d/dt v + omega_(g/b) x v
    //
    // where ^B d/dt indicates the time derivative of an abstract vector v taken
    // "as viewed from" the b-frame, and omega_(g/b) = -omega_(b/g) has the
    // opposite sign of our usual body rates vector omega.
    //
    // This gives our state derivatives:
    //
    //   d/dt Xg_cm = dcm_g2b^T Vb_cm
    //   d/dt Vb_cm = Vb_cm x omega + Ab_cm, where Ab_cm = (1/m) f_b.
    //
    // [*]: https://en.wikipedia.org/wiki/Rotating_reference_frame
    //      #Time_derivatives_in_the_two_frames

    Vec3 Ab_center_of_mass;
    Vec3 Vb_cross_omega;
    Mat3TransVec3Mult(&dcm_g2b(), &Vb_center_of_mass(), &dXg_center_of_mass__);
    Vec3Scale(&fm.force, 1.0 / mass_, &Ab_center_of_mass);

    Vec3Cross(&Vb_center_of_mass(), &omega(), &Vb_cross_omega);
    Vec3Add(&Ab_center_of_mass, &Vb_cross_omega, &dVb_center_of_mass__);
  }

  // Calculate the quaternion derivative.
  Quat dq__;
  {
    // Apply the differential equations governing rigid body motion.  This obeys
    // equation 1.3-35 in Stevens and Lewis:
    //
    //   (d/dt) q = (1/2) q * omega
    //
    // where the omega vector, representing the body rotation rates, is embedded
    // into the vector part of a quaternion to make it compatible with
    // quaternionic multiplication.

    Quat q_omega = {0.0, 0.5 * omega().x, 0.5 * omega().y, 0.5 * omega().z};
    Quat dq_phys;
    QuatMultiply(&q(), &q_omega, &dq_phys);

    // Apply error-feedback to keep the quaternion normalized.  An explanation
    // of this method may be found at https://stackoverflow.com/a/12934750.
    const double k_quat = 1.0;
    Quat dq_numeric;
    QuatScale(&q(), k_quat * (1.0 - QuatModSquared(&q())), &dq_numeric);

    // Add the two contributions to the quaternion derivative.
    QuatAdd(&dq_phys, &dq_numeric, &dq__);
  }

  // Calculate the rate of change of the angular velocity vector omega due to
  // the moments acting on the body.
  Vec3 domega__;
  {
    // Here we apply Euler's equation to obtain the rate of change of the body
    // rate vector "omega".
    //
    // I (d/dt omega) + omega ⨯ (I omega) = moment_ext        [Euler's equation]
    // I (d/dt omega) = moment_ext - omega x (I omega)
    //    d/dt omega = Inv(I) (moment_ext - omega x (I omega))
    //    d/dt omega = Inv(I) (moment_ext + (I omega) x omega)

    Vec3 Iomega, moment_gyro, moment_tot;
    Mat3Vec3Mult(&moment_of_inertia_, &omega(), &Iomega);
    Vec3Cross(&Iomega, &omega(), &moment_gyro);
    Vec3Add(&fm.moment, &moment_gyro, &moment_tot);
    Mat3Vec3Mult(&moment_of_inertia_inv_, &moment_tot, &domega__);
  }

  // If the wing is tied down, set all of the derivatives to zero.
  if (*g_sim.sim_opt & kSimOptTiedDown) {
    dXg_center_of_mass__ = kVec3Zero;
    dVb_center_of_mass__ = kVec3Zero;
    dq__ = kQuatZero;
    domega__ = kVec3Zero;
  }

  Xg_center_of_mass_.set_deriv(dXg_center_of_mass__);
  Vb_center_of_mass_.set_deriv(dVb_center_of_mass__);
  q_.set_deriv(dq__);
  omega_.set_deriv(domega__);
}

void Wing::UpdateAeroCoeffs(double thrust_coeff) {
  ForceMoment aero_force_moment_coeffs__;
  RawAeroCoeffs raw_aero_coeffs__;
  VEC_INIT(8, flaps,
           {flap_angles(kFlapA1), flap_angles(kFlapA2), flap_angles(kFlapA4),
            flap_angles(kFlapA5), flap_angles(kFlapA7), flap_angles(kFlapA8),
            flap_angles(kFlapEle), flap_angles(kFlapRud)});
  aero_.CalcForceMomentCoeff(alpha(), beta(), omega_hat(), flaps,
                             reynolds_number(), &aero_force_moment_coeffs__,
                             thrust_coeff, &raw_aero_coeffs__);

  aero_force_moment_coeffs_.set_val(aero_force_moment_coeffs__);
  raw_aero_coeffs_.set_val(raw_aero_coeffs__);
}

void Wing::UpdateForceMomentPos(double t) {
  ForceMomentPos fmx_aero__, fmx_grav__, fmx_disturb__;
  // Calculate the internal forces.
  // Aero.
  CalcAeroForceMomentPos(&fmx_aero__);
  fmx_aero_.set_val(fmx_aero__);

  // Gravity.
  CalcGravityForceMomentPos(&fmx_grav__);
  fmx_gravity_.set_val(fmx_grav__);

  // Disturbances.
  CalcDisturbForceMomentPos(t, &fmx_disturb__);
  fmx_disturb_.set_val(fmx_disturb__);

  // Sum forces and moments on wing about center of mass.
  ForceMomentPos fmx_total__;
  fmx_total__.pos = center_of_mass_pos_;
  ForceMomentPosAdd3(&fmx_aero__, &fmx_grav__, &fmx_disturb__, &fmx_total__);

  // Add external forces, expressed in body coordinates.
  for (int32_t i = 0; i < static_cast<int32_t>(external_forces_.size()); ++i) {
    ForceMomentPosAdd(&fmx_total__, &external_forces(i), &fmx_total__);
  }
  fmx_total_.set_val(fmx_total__);
}

void Wing::CalcAeroForceMomentPos(ForceMomentPos *fm_aero) const {
  // Prepare output.
  const Vec3 length_scale = {wing_params_.b, wing_params_.c, wing_params_.b};
  Vec3Scale(&aero_force_moment_coeffs().force, q_bar() * wing_params_.A,
            &fm_aero->force);
  Vec3Mult(&aero_force_moment_coeffs().moment, &length_scale, &fm_aero->moment);
  Vec3Scale(&fm_aero->moment, q_bar() * wing_params_.A, &fm_aero->moment);
  fm_aero->moment.y =
      fm_aero->moment.y + aero_.CalcEmpiricalPitchingMoment(alpha());
  fm_aero->pos = kVec3Zero;
}

void Wing::CalcGravityForceMomentPos(ForceMomentPos *fm) const {
  Mat3Vec3Mult(&dcm_g2b(), &environment_.g_g(), &fm->force);
  Vec3Scale(&fm->force, mass_, &fm->force);
  fm->moment = kVec3Zero;
  fm->pos = center_of_mass_pos_;
}

void Wing::CalcDisturbForceMomentPos(double t, ForceMomentPos *fm) const {
  std::vector<double> parameters;
  fm->force = kVec3Zero;
  fm->moment = kVec3Zero;
  fm->pos = center_of_mass_pos_;

  if (fault_func_(t, kSimFaultDisturbanceBodyForceSine, &parameters)) {
    Vec3 f_sine = {parameters[0], parameters[1], parameters[2]};
    Vec3LinComb(1.0, &fm->force, sin(2.0 * M_PI * parameters[3] * t), &f_sine,
                &fm->force);
  }

  if (fault_func_(t, kSimFaultDisturbanceBodyForceStep, &parameters)) {
    Vec3 f_step = {parameters[0], parameters[1], parameters[2]};
    Vec3Add(&fm->force, &f_step, &fm->force);
  }

  if (fault_func_(t, kSimFaultDisturbanceBodyTorqueSine, &parameters)) {
    Vec3 m_sine = {parameters[0], parameters[1], parameters[2]};
    Vec3LinComb(1.0, &fm->moment, sin(2.0 * M_PI * parameters[3] * t), &m_sine,
                &fm->moment);
  }

  if (fault_func_(t, kSimFaultDisturbanceBodyTorqueStep, &parameters)) {
    Vec3 m_step = {parameters[0], parameters[1], parameters[2]};
    Vec3Add(&fm->moment, &m_step, &fm->moment);
  }
}
