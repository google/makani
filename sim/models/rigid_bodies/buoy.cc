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

#include "sim/models/rigid_bodies/buoy.h"
#include "common/c_math/linalg.h"
#include "sim/models/environment.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"

Buoy::Buoy(const Environment &environment, const GroundFrame &ground_frame,
           const BuoyParams &buoy_params, const BuoySimParams &buoy_sim_params,
           Sea *sea)
    : RigidBody("Buoy"),
      buoy_params_(buoy_params),
      buoy_sim_params_(buoy_sim_params),
      mass_(buoy_params_.mass),
      inertia_tensor_(buoy_params_.inertia_tensor),
      center_of_mass_pos_(buoy_params_.center_of_mass_pos),
      environment_(environment),
      ground_frame_(ground_frame),
      sea_(sea),
      hydrodynamics_sim_params_(),
      mooring_sim_params_(),
      fmx_tether_g_(new_derived_value(), "tether_force_moment_position_g",
                    {kVec3Zero, kVec3Zero, kVec3Zero}),
      q_(new_continuous_state(), "q"),
      omega_(new_continuous_state(), "omega", buoy_sim_params_.omega_0),
      Xg_center_of_mass_(new_continuous_state(), "Xg_center_of_mass"),
      Vg_center_of_mass_(new_continuous_state(), "Vg_center_of_mass"),
      vessel_frame_(new_derived_value(), "vessel_frame"),
      Xg_(new_derived_value(), "Xg"),
      Vg_(new_derived_value(), "Vg"),
      dcm_g2v_(new_derived_value(), "dcm_g2v"),
      wet_height_(new_derived_value(), "wet_height"),
      wet_volume_(new_derived_value(), "wet_volume"),
      buoyancy_center_v_(new_derived_value(), "buoyancy_center_v"),
      water_line_v_(new_derived_value(), "water_line_v"),
      yaw_angle_from_eq_(new_derived_value(), "yaw_angle"),
      fm_hydro_(new_derived_value(), "fm_hydro"),
      fm_tether_(new_derived_value(), "fm_tether"),
      fm_gravity_(new_derived_value(), "fm_gravity"),
      fm_mooring_(new_derived_value(), "fm_mooring"),
      fm_total_(new_derived_value(), "fm_total") {
  ManageUncertainties();
  Init(buoy_sim_params_.Xg_0, buoy_sim_params_.Vg_0, buoy_sim_params_.q_0,
       buoy_sim_params_.omega_0);
  SetupDone();
}

void Buoy::Init(const Vec3 &Xg_0, const Vec3 &Vg_0, const Quat &q_0,
                const Vec3 &omega_0) {
  // Replace initial yaw angle with the equilibrium yaw angle.
  // NOTE: This makes the buoy always at rest in yaw at the initial time.
  double yaw_0, pitch_0, roll_0;
  QuatToAngle(&q_0, kRotationOrderZyx, &yaw_0, &pitch_0, &roll_0);
  double yaw_equilibrium_heading_g =
      mooring_sim_params_.yaw_equilibrium_heading - ground_frame_.heading();
  Quat q_0_equilibrium;
  AngleToQuat(yaw_equilibrium_heading_g, pitch_0, roll_0, kRotationOrderZyx,
              &q_0_equilibrium);
  q_.set_val(q_0_equilibrium);
  Mat3 dcm_g2v__;
  QuatToDcm(&q_0_equilibrium, &dcm_g2v__);

  Vec3 omega_g;
  Mat3TransVec3Mult(&dcm_g2v__, &omega_0, &omega_g);

  ReferenceFrame frame__(ground_frame_, Xg_0, Vg_0, dcm_g2v__, omega_g);
  ReferenceFrame frame_cg(frame__, center_of_mass_pos_);

  Vec3 Xg_center_of_mass__;
  frame_cg.TransformOriginTo(ground_frame_, ReferenceFrame::kPosition,
                             &Xg_center_of_mass__);
  Xg_center_of_mass_.set_val(Xg_center_of_mass__);

  Vec3 Vg_center_of_mass__;
  frame_cg.TransformOriginTo(ground_frame_, ReferenceFrame::kVelocity,
                             &Vg_center_of_mass__);
  Vg_center_of_mass_.set_val(Vg_center_of_mass__);

  UpdateDerivedPositionAndAttitudeStates();
  UpdateWetVolume(0.0);
  UpdateForceMoments(0.0);
}

void Buoy::ManageUncertainties() {
  // Mass properties.

  // Scale buoy mass.
  mass_ *= buoy_sim_params_.mass_prop_uncertainties.mass_scale;

  // Offset center of mass.
  Vec3Add(&buoy_sim_params_.mass_prop_uncertainties.center_of_mass_offset,
          &center_of_mass_pos_, &center_of_mass_pos_);

  // Scale inertia tensor.
  for (int32_t i = 0; i < 3; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      inertia_tensor_.d[i][j] *= Sqrt(
          buoy_sim_params_.mass_prop_uncertainties.moment_of_inertia_scale[i] *
          buoy_sim_params_.mass_prop_uncertainties.moment_of_inertia_scale[j]);
    }
  }

  // Hydrodynamic model parameters.
  hydrodynamics_sim_params_.torsional_damping_x =
      buoy_sim_params_.hydrodynamics.torsional_damping_x *
      buoy_sim_params_.hydrodynamics.uncertainties.torsional_damping_x_scale;
  hydrodynamics_sim_params_.torsional_damping_y =
      buoy_sim_params_.hydrodynamics.torsional_damping_y *
      buoy_sim_params_.hydrodynamics.uncertainties.torsional_damping_y_scale;
  hydrodynamics_sim_params_.torsional_damping_z =
      buoy_sim_params_.hydrodynamics.torsional_damping_z *
      buoy_sim_params_.hydrodynamics.uncertainties.torsional_damping_z_scale;
  hydrodynamics_sim_params_.buoyancy_damping_coeff =
      buoy_sim_params_.hydrodynamics.buoyancy_damping_coeff *
      buoy_sim_params_.hydrodynamics.uncertainties.buoyancy_damping_coeff_scale;
  hydrodynamics_sim_params_.Cd = buoy_sim_params_.hydrodynamics.Cd;
  hydrodynamics_sim_params_.Ca =
      buoy_sim_params_.hydrodynamics.Ca *
      buoy_sim_params_.hydrodynamics.uncertainties.Ca_scale;
  hydrodynamics_sim_params_.Dh =
      buoy_sim_params_.hydrodynamics.Dh *
      buoy_sim_params_.hydrodynamics.uncertainties.Dh_scale;
  hydrodynamics_sim_params_.ki =
      buoy_sim_params_.hydrodynamics.ki *
      buoy_sim_params_.hydrodynamics.uncertainties.ki_scale;

  // Mooring line model parameters.
  mooring_sim_params_.yaw_equilibrium_heading =
      Wrap(buoy_sim_params_.mooring_lines.yaw_equilibrium_heading +
               DegToRad(buoy_sim_params_.mooring_lines.uncertainties
                            .yaw_equilibrium_heading_delta),
           -PI, PI);
  mooring_sim_params_.torsional_stiffness_z =
      buoy_sim_params_.mooring_lines.torsional_stiffness_z *
      buoy_sim_params_.mooring_lines.uncertainties.torsional_stiffness_z_scale;
  mooring_sim_params_.mooring_attach_v.x =
      buoy_sim_params_.mooring_lines.mooring_attach_v.x +
      buoy_sim_params_.mooring_lines.uncertainties.mooring_attach_pos_x_delta;
  mooring_sim_params_.mooring_attach_v.y =
      buoy_sim_params_.mooring_lines.mooring_attach_v.y +
      buoy_sim_params_.mooring_lines.uncertainties.mooring_attach_pos_y_delta;
  mooring_sim_params_.mooring_attach_v.z =
      buoy_sim_params_.mooring_lines.mooring_attach_v.z +
      buoy_sim_params_.mooring_lines.uncertainties.mooring_attach_pos_z_delta;
  mooring_sim_params_.kt0 =
      buoy_sim_params_.mooring_lines.kt0 *
      buoy_sim_params_.mooring_lines.uncertainties.kt0_scale;
  mooring_sim_params_.kt1 =
      buoy_sim_params_.mooring_lines.kt1 *
      buoy_sim_params_.mooring_lines.uncertainties.kt1_scale;
  mooring_sim_params_.ct =
      buoy_sim_params_.mooring_lines.ct *
      buoy_sim_params_.mooring_lines.uncertainties.ct_scale;
}

void Buoy::UpdateDerivedPositionAndAttitudeStates() {
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

  Mat3 dcm_g2v__;
  QuatToDcm(&q(), &dcm_g2v__);
  dcm_g2v_.set_val(dcm_g2v__);

  Vec3 Xg__, center_of_mass_offset;
  Mat3TransVec3Mult(&dcm_g2v(), &center_of_mass_pos_, &center_of_mass_offset);
  Vec3Sub(&Xg_center_of_mass(), &center_of_mass_offset, &Xg__);
  Xg_.set_val(Xg__);

  Vec3 Vv, Vv_center_of_mass;
  Mat3Vec3Mult(&dcm_g2v(), &Vg_center_of_mass(), &Vv_center_of_mass);
  Vec3Cross(&omega(), &center_of_mass_pos_, &Vv);
  Vec3Sub(&Vv_center_of_mass, &Vv, &Vv);

  Vec3 Vg__;
  Vg_.set_val(*Mat3TransVec3Mult(&dcm_g2v(), &Vv, &Vg__));

  Vec3 omega_g;
  Mat3TransVec3Mult(&dcm_g2v(), &omega(), &omega_g);
  vessel_frame_.set_val(
      ReferenceFrame(ground_frame_, Xg(), Vg(), dcm_g2v(), omega_g));
}

void Buoy::UpdateWetVolume(double time) {
  if (sea_ == nullptr) {
    wet_height_.set_val(0.0);
    wet_volume_.set_val(0.0);
    buoyancy_center_v_.set_val(kVec3Zero);
    water_line_v_.set_val(kVec3Zero);
    return;
  }

  // Get buoy geometry.
  Vec3 buoy_bottom_g, buoy_top_g;
  Vec3 buoy_bottom_v = {0.0, 0.0, buoy_params_.bottom_deck_pos_z_v};
  Vec3 buoy_top_v = {0.0, 0.0, buoy_params_.top_deck_pos_z_v};
  Mat3TransVec3Mult(&dcm_g2v(), &buoy_bottom_v, &buoy_bottom_g);
  Mat3TransVec3Mult(&dcm_g2v(), &buoy_top_v, &buoy_top_g);
  Vec3Add(&buoy_bottom_g, &Xg(), &buoy_bottom_g);
  Vec3Add(&buoy_top_g, &Xg(), &buoy_top_g);

  // The water level is obtained at a point on the buoy that matches the
  // position of the water line when the buoy is at rest. The water level
  // above/below this point should be very close to the water level at the water
  // line for reasonably expected waves and buoy motion. This simplification
  // allows for a fast evaluation of the buoyancy force because GetSeaElevation
  // gets called only once.
  // TODO: add a check for the buoy fully under the water level.
  double water_density = sea_->GetWaterDensity();
  double rest_wet_height;  // Wet height when buoy is at rest.
  rest_wet_height =
      mass_ / PI / Square(buoy_params_.spar_diameter / 2.0) / water_density;
  Vec3 bottom_to_top_g;  // Vector from the bottom deck to the top of the spar.
  Vec3Sub(&buoy_top_g, &buoy_bottom_g, &bottom_to_top_g);
  Vec3Normalize(&bottom_to_top_g, &bottom_to_top_g);
  Vec3 rest_water_line_g;
  Vec3LinComb(1.0, &buoy_bottom_g, rest_wet_height, &bottom_to_top_g,
              &rest_water_line_g);
  double water_level_g;
  water_level_g =
      sea_->GetSeaElevation(time, rest_water_line_g.x, rest_water_line_g.y);
  Vec3 water_plane_normal_g = {
      0., 0., -1.};  // TODO: Make this anti-gravity.
  Vec3 water_plane_pos_g = {0., 0., water_level_g};

  // Get the wet height and volume of the spar, as well as the location of the
  // buoyancy center. This is a simplified implementation, in which the
  // submerged volume is approximated by a cylinder, and the buoyancy center is
  // the center of this submerged cylinder.
  Vec3 bottom_to_top_unit_g;  // Unit vector from the bottom deck to the top of
                              // the spar.
  Vec3Sub(&buoy_top_g, &buoy_bottom_g, &bottom_to_top_unit_g);
  Vec3Normalize(&bottom_to_top_unit_g, &bottom_to_top_unit_g);
  Vec3 water_buoy_intersect_g, bottom_to_water_line_g;
  Vec3 origin_to_buoyancy_center_g, buoyancy_center_g, buoyancy_center_v;
  Vec3 water_line_g, water_line_v;
  double wet_height;
  if (IntersectLinePlane(&water_plane_pos_g, &water_plane_normal_g,
                         &buoy_bottom_g, &bottom_to_top_unit_g,
                         &water_buoy_intersect_g)) {
    // The buoy intersects the water surface and we can find the wet height.
    Vec3Sub(&water_buoy_intersect_g, &buoy_bottom_g, &bottom_to_water_line_g);
    wet_height = Vec3Norm(&bottom_to_water_line_g);
    // Get the buoyancy center.
    Vec3Add(&buoy_bottom_g, Vec3Scale(&bottom_to_top_unit_g, wet_height / 2.0,
                                      &buoyancy_center_g),
            &buoyancy_center_g);
    Vec3Sub(&buoyancy_center_g, &Xg(), &origin_to_buoyancy_center_g);
    Mat3Vec3Mult(&dcm_g2v(), &origin_to_buoyancy_center_g, &buoyancy_center_v);
    Vec3Sub(&water_buoy_intersect_g, &Xg(), &water_line_g);
    Mat3Vec3Mult(&dcm_g2v(), &water_line_g, &water_line_v);
  } else {
    // Something bad has happened and the buoy is parallel to the sea level.
    // Since it is probably not flying, it is probably sunken.
    wet_height = buoy_params_.spar_height;
    buoyancy_center_v = kVec3Zero;
    water_line_v = kVec3Zero;
  }
  double wet_volume;
  wet_volume = PI * Square(buoy_params_.spar_diameter / 2.0) * wet_height;

  // Set the state values.
  wet_height_.set_val(wet_height);
  wet_volume_.set_val(wet_volume);
  buoyancy_center_v_.set_val(buoyancy_center_v);
  water_line_v_.set_val(water_line_v);
}

void Buoy::UpdateForceMoments(double time) {
  if (sea_ == nullptr) {
    fm_hydro_.set_val(kForceMomentZero);
    fm_tether_.set_val(kForceMomentZero);
    fm_gravity_.set_val(kForceMomentZero);
    fm_mooring_.set_val(kForceMomentZero);
    fm_total_.set_val(kForceMomentZero);
    yaw_angle_from_eq_.set_val(0.0);
    return;
  }

  ForceMoment fm_hydro, fm_tether, fm_gravity, fm_mooring, fm_total;
  GetExternalForceMoment(&fm_hydro, &fm_tether, &fm_gravity, &fm_mooring,
                         &fm_total, time);
  fm_hydro_.set_val(fm_hydro);
  fm_tether_.set_val(fm_tether);
  fm_gravity_.set_val(fm_gravity);
  fm_mooring_.set_val(fm_mooring);
  fm_total_.set_val(fm_total);
}

void Buoy::Publish() const {
  // Publish buoy telemetry.
  sim_telem.buoy.Xg = Xg();
  sim_telem.buoy.Vg = Vg();
  sim_telem.buoy.Xg_center_of_mass = Xg_center_of_mass();
  sim_telem.buoy.Vg_center_of_mass = Vg_center_of_mass();
  sim_telem.buoy.q = q();
  sim_telem.buoy.omega = omega();
  sim_telem.buoy.dcm_g2v = dcm_g2v();
  sim_telem.buoy.fm_hydro = fm_hydro();
  sim_telem.buoy.fm_tether = fm_tether();
  sim_telem.buoy.fm_gravity = fm_gravity();
  sim_telem.buoy.fm_mooring = fm_mooring();
  sim_telem.buoy.fm_total = fm_total();
  sim_telem.buoy.water_line_pos_z_v = water_line_v().z;
  sim_telem.buoy.yaw_angle_from_eq = yaw_angle_from_eq_.val();

  // Log the acceleration of the origin of the vessel frame.
  ReferenceFrame vessel_origin;
  CalcAcceleratingFrame(&vessel_origin);
  Vec3 vessel_origin_acc_g;
  vessel_origin.TransformOriginTo(ground_frame_, ReferenceFrame::kAcceleration,
                                  &vessel_origin_acc_g);
  sim_telem.buoy.vessel_origin_accel_g = vessel_origin_acc_g;
}

void Buoy::DiscreteStepHelper(double /*t*/) {
  // Place-holder.
}

void Buoy::CalcDerivHelper(double /*t*/) {
  if (sea_ == nullptr) {
    q_.set_deriv(kQuatZero);
    omega_.set_deriv(kVec3Zero);
    Xg_center_of_mass_.set_deriv(kVec3Zero);
    Vg_center_of_mass_.set_deriv(kVec3Zero);
    return;
  }

  // Quaternion kinematics.
  // Calculate the change in attitude (q) due to the derived angular velocity
  // (omega).  This obeys equation 1.3-35 in Stevens and Lewis:
  //
  //   q_dot = (1/2) q * omega
  //
  // where the omega vector is embedded into the vector part of a
  // quaternion to make it compatible with quaternionic
  // multiplication.
  Quat q_omega = {0.0, 0.5 * omega().x, 0.5 * omega().y, 0.5 * omega().z};
  Quat dq_phys;
  QuatMultiply(&q(), &q_omega, &dq_phys);

  // The following implements error feedback to keep the quaternion
  // normalized.  Add this correction into the total change of the
  // attitude quaternion, dq__.
  const double k_quat = 1.0;
  Quat dq_numeric;
  QuatScale(&q(), k_quat * (1.0 - QuatModSquared(&q())), &dq_numeric);

  Quat dq__;
  QuatAdd(&dq_phys, &dq_numeric, &dq__);

  // Transport the total moment vector acting on the buoy to its center of mass.
  Vec3 disp_g, disp_v;
  Mat3Vec3Mult(&dcm_g2v(), Vec3Sub(&Xg_center_of_mass(), &Xg(), &disp_g),
               &disp_v);
  ForceMoment fm_total_center_of_mass;
  ForceMomentRef(&fm_total(), &disp_v, &fm_total_center_of_mass);

  // Get mass properties.
  double water_density = sea_->GetWaterDensity();

  // The equations of motion can be expressed as:
  //
  //   [M + A]*vel_v_dot + [S*M + CA]*vel_v = F_v
  //
  // where:
  //   M is the structural mass matrix, in the vessel frame.
  //   A is the added mass matrix, in the vessel frame.
  //   S is the velocity cross product matrix, defined as:
  //
  //           [ (w)x   0  ]
  //       S = [           ], where (a)x is the skew-symmetric matrix of a.
  //           [   0  (w)x ]
  //
  //   w is the rotation rate of the vessel wrt the ground frame, in vessel
  //       frame.
  //   CA is the hydrodynamic Coriolis and centripetal matrix.
  //   vel_v_dot is the vessel frame time derivative of the generalized
  //       velocity of the buoy CM wrt the ground frame, expressed in vessel
  //       frame.
  //   vel_v is the generalized velocity of the buoy CM wrt the ground frame,
  //       expressed in vessel frame.
  //   F_v is the generalized external force, in vessel frame.
  //
  //  See the derivation of the equations of motion in:
  //  docs.google.com/document/d/1jWqC9o-Yt4zTOIJNOOfksZaf65bHfcanh4u3ItlXRFU

  // Create the structural mass matrix M: struct_mass.
  MAT_INIT(6, 6, struct_mass, {{0.}});
  for (int i = 0; i < 3; i++) {
    *MatPtr(&struct_mass, i, i) = mass_;
  }
  MAT_CLONE(3, 3, inertia, &inertia_tensor_);  // Transform Mat3 into Mat.
  MatSubmatSet(&inertia, 0, 0, 3, 3, 3, 3, &struct_mass);

  // Create the added mass matrix A: added_mass.
  MAT_INIT(6, 6, added_mass, {{0.}});
  Vec3 com_to_buoyancy_center_v;
  Vec3Sub(&buoyancy_center_v(), &center_of_mass_pos_,
          &com_to_buoyancy_center_v);
  double b = Vec3Norm(&com_to_buoyancy_center_v);
  double D = buoy_params_.spar_diameter;
  double Iadd = water_density * wet_volume() *
                (b * b + D * D / 16.0 + wet_height() * wet_height() / 12.0);

  double Ca = hydrodynamics_sim_params_.Ca;
  *MatPtr(&added_mass, 0, 0) = Ca * water_density * wet_volume();
  *MatPtr(&added_mass, 1, 1) = Ca * water_density * wet_volume();
  *MatPtr(&added_mass, 2, 2) =
      Ca / 12.0 * water_density * PI * ThirdPower(hydrodynamics_sim_params_.Dh);
  *MatPtr(&added_mass, 3, 3) = Ca * hydrodynamics_sim_params_.ki * Iadd;
  *MatPtr(&added_mass, 4, 4) = Ca * hydrodynamics_sim_params_.ki * Iadd;
  *MatPtr(&added_mass, 0, 4) = -b * Ca * water_density * wet_volume();
  *MatPtr(&added_mass, 4, 0) = -b * Ca * water_density * wet_volume();
  *MatPtr(&added_mass, 1, 3) = b * Ca * water_density * wet_volume();
  *MatPtr(&added_mass, 3, 1) = b * Ca * water_density * wet_volume();

  // Obtain the total system mass matrix [M + A]: total_mass.
  MAT(6, 6, total_mass);
  MatAdd(&struct_mass, &added_mass, &total_mass);

  // Build the generalized force vector F_v: generalized_force_v.
  VEC(6, generalized_force_v);
  *VecPtr(&generalized_force_v, 0) = fm_total_center_of_mass.force.x;
  *VecPtr(&generalized_force_v, 1) = fm_total_center_of_mass.force.y;
  *VecPtr(&generalized_force_v, 2) = fm_total_center_of_mass.force.z;
  *VecPtr(&generalized_force_v, 3) = fm_total_center_of_mass.moment.x;
  *VecPtr(&generalized_force_v, 4) = fm_total_center_of_mass.moment.y;
  *VecPtr(&generalized_force_v, 5) = fm_total_center_of_mass.moment.z;

  // Get the generalized velocity vel_v: generalized_vel_v.
  Vec3 Vv_center_of_mass;
  Mat3Vec3Mult(&dcm_g2v(), &Vg_center_of_mass(), &Vv_center_of_mass);
  VEC_INIT(6, generalized_vel_v, {0.});
  *VecPtr(&generalized_vel_v, 0) = Vv_center_of_mass.x;
  *VecPtr(&generalized_vel_v, 1) = Vv_center_of_mass.y;
  *VecPtr(&generalized_vel_v, 2) = Vv_center_of_mass.z;
  *VecPtr(&generalized_vel_v, 3) = omega().x;
  *VecPtr(&generalized_vel_v, 4) = omega().y;
  *VecPtr(&generalized_vel_v, 5) = omega().z;

  // Get the generalized velocity skew-symmetric matrix S: generalized_vel_skw.
  Mat3 omega_skw;
  Mat3Cross(&omega(), &omega_skw);
  MAT_INIT(6, 6, generalized_vel_skw, {{0.}});
  MAT_CLONE(3, 3, omega_skw_, &omega_skw);  // Transform Mat3 into Mat.
  MatSubmatSet(&omega_skw_, 0, 0, 3, 3, 0, 0, &generalized_vel_skw);
  MatSubmatSet(&omega_skw_, 0, 0, 3, 3, 3, 3, &generalized_vel_skw);

  // Get the hydrodynamic Coriolis and centripetal matrix CA.
  // The CA matrix can be parameterized by:
  //
  //        [        0         -(A11v + A12w)x ]   [  0    CA12 ]
  //   CA = [                                  ] = [            ]
  //        [ -(A11v + A12w)x  -(A21v + A22w)x ]   [ CA12  CA22 ]
  //
  // where:
  //   (a)x is the skew-symmetric matrix of a.
  //   w is the rotation rate wrt the ground frame, in vessel frame.
  //   v is the velocity wrt the ground frame, in vessel frame.
  //   A11, A12, A21, A22 are sub-matrices of the added mass matrix A:
  //
  //           [ A11  A12 ]
  //       A = [          ]
  //           [ A21  A22 ]
  //
  // For an additional reference, see http://www.fossen.biz/wiley/Ch3.pdf.
  MAT_INIT(3, 3, A11_, {{0.}});
  MAT_INIT(3, 3, A12_, {{0.}});
  MAT_INIT(3, 3, A21_, {{0.}});
  MAT_INIT(3, 3, A22_, {{0.}});
  MatSubmatSet(&added_mass, 0, 0, 3, 3, 0, 0, &A11_);
  MatSubmatSet(&added_mass, 3, 0, 3, 3, 0, 0, &A12_);
  MatSubmatSet(&added_mass, 0, 3, 3, 3, 0, 0, &A21_);
  MatSubmatSet(&added_mass, 3, 3, 3, 3, 0, 0, &A22_);
  Mat3 A11, A12, A21, A22;
  memcpy(A11.d, A11_.d, 9 * sizeof(double));
  memcpy(A12.d, A12_.d, 9 * sizeof(double));
  memcpy(A21.d, A21_.d, 9 * sizeof(double));
  memcpy(A22.d, A22_.d, 9 * sizeof(double));

  // Obtain CA12.
  Vec3 A11v, A12w, CA12_;
  Mat3Vec3Mult(&A11, &Vv_center_of_mass, &A11v);
  Mat3Vec3Mult(&A12, &omega(), &A12w);
  Vec3Add(&A11v, &A12w, &CA12_);
  Vec3Scale(&CA12_, -1.0, &CA12_);
  Mat3 CA12_skw;
  Mat3Cross(&CA12_, &CA12_skw);
  MAT_CLONE(3, 3, CA12, &CA12_skw);

  // Obtain CA22.
  Vec3 A21v, A22w, CA22_;
  Mat3Vec3Mult(&A21, &Vv_center_of_mass, &A21v);
  Mat3Vec3Mult(&A22, &omega(), &A22w);
  Vec3Add(&A21v, &A22w, &CA22_);
  Vec3Scale(&CA22_, -1.0, &CA22_);
  Mat3 CA22_skw;
  Mat3Cross(&CA22_, &CA22_skw);
  MAT_CLONE(3, 3, CA22, &CA22_skw);

  // Build the final CA matrix.
  MAT_INIT(6, 6, CA, {{0.}});
  MatSubmatSet(&CA12, 0, 0, 3, 3, 0, 3, &CA);
  MatSubmatSet(&CA12, 0, 0, 3, 3, 3, 0, &CA);
  MatSubmatSet(&CA22, 0, 0, 3, 3, 3, 3, &CA);

  // Get the total generalized force. This is the right-hand side of the
  // equation of motion: F_v - [S*M + CA]*vel_v.
  MAT(6, 6, gyroscopic_matrix);
  MAT(6, 6, total_gyroscopic_matrix);
  MatMult(&generalized_vel_skw, &struct_mass, &gyroscopic_matrix);
  MatAdd(&gyroscopic_matrix, &CA, &total_gyroscopic_matrix);
  VEC(6, gyro_force_total);
  MatVecMult(&total_gyroscopic_matrix, &generalized_vel_v, &gyro_force_total);
  VEC(6, generalized_force_total_v);
  VecSub(&generalized_force_v, &gyro_force_total, &generalized_force_total_v);

  // Compute the generalized acceleration vel_v_dot: generalized_accel_v.
  VEC(6, generalized_accel_v);
  MatVecLeftDivide(&total_mass, &generalized_force_total_v,
                   &generalized_accel_v);

  // Obtain the rotational acceleration in vessel frame, the three last
  // components of generalized_accel_v.
  Vec3 domega__;
  domega__.x = VecGet(&generalized_accel_v, 3);
  domega__.y = VecGet(&generalized_accel_v, 4);
  domega__.z = VecGet(&generalized_accel_v, 5);

  // Get inertial translational acceleration in ground frame.
  // For this, we solve accel_i = [M + A]^-1 * F_v, and then we express the
  // first three components of the solution in the ground frame.
  VEC(6, generalized_accel_i);
  MatVecLeftDivide(&total_mass, &generalized_force_v, &generalized_accel_i);
  Vec3 accel_i, dVg_center_of_mass__;
  accel_i.x = VecGet(&generalized_accel_i, 0);
  accel_i.y = VecGet(&generalized_accel_i, 1);
  accel_i.z = VecGet(&generalized_accel_i, 2);
  Mat3TransVec3Mult(&dcm_g2v(), &accel_i, &dVg_center_of_mass__);

  // Translational kinematics.
  Vec3 dXg_center_of_mass__ = Vg_center_of_mass();

  // Set derivatives.
  q_.set_deriv(dq__);
  omega_.set_deriv(domega__);
  Xg_center_of_mass_.set_deriv(dXg_center_of_mass__);
  Vg_center_of_mass_.set_deriv(dVg_center_of_mass__);
}

void Buoy::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(
      1, [this](double /*t*/) { UpdateDerivedPositionAndAttitudeStates(); });
  connections->Add(4, [this](double t) { UpdateWetVolume(t); });
  connections->Add(5, [this](double t) { UpdateForceMoments(t); });
}

void Buoy::GetExternalForceMoment(ForceMoment *fm_hydro, ForceMoment *fm_tether,
                                  ForceMoment *fm_gravity,
                                  ForceMoment *fm_mooring,
                                  ForceMoment *fm_total, double time) {
  // Returns external forces and moments in the vessel frame.
  CalcHydroForceMoment(fm_hydro, time);
  CalcTetherForceMoment(fm_tether);
  CalcGravityForceMoment(fm_gravity);
  CalcMooringForceMoment(fm_mooring);
  ForceMomentAdd(ForceMomentAdd(ForceMomentAdd(fm_hydro, fm_tether, fm_total),
                                fm_gravity, fm_total),
                 fm_mooring, fm_total);
}

void Buoy::CalcHydroForceMoment(ForceMoment *fm_hydro, double time) {
  // Returns the external torque acting at the center of mass of the buoy due to
  // hydrostatic (buoyancy) and hydrodynamic (water motion) forces.

  // Hydrostatic force and moment.
  Vec3 buoyancy_force_g;
  ForceMoment fm_buoyancy_v, fm_buoyancy_origin_v;
  GetBuoyancyForce(&buoyancy_force_g);
  Mat3Vec3Mult(&dcm_g2v(), &buoyancy_force_g, &fm_buoyancy_v.force);
  fm_buoyancy_v.moment = kVec3Zero;
  Vec3 buoyancy_center_to_origin_v;
  Vec3Scale(&buoyancy_center_v(), -1.0, &buoyancy_center_to_origin_v);
  ForceMomentRef(&fm_buoyancy_v, &buoyancy_center_to_origin_v,
                 &fm_buoyancy_origin_v);

  // Damping torque.
  Vec3 damp_coeffs = {hydrodynamics_sim_params_.torsional_damping_x,
                      hydrodynamics_sim_params_.torsional_damping_y,
                      hydrodynamics_sim_params_.torsional_damping_z};

  double omega_normal_norm = Vec3XyNorm(&omega());
  Vec3 omega_abs = {omega_normal_norm, omega_normal_norm, fabs(omega().z)};

  ForceMoment fm_damping_v;
  Vec3Mult(&damp_coeffs, &omega(), &fm_damping_v.moment);
  Vec3Mult(&omega_abs, &fm_damping_v.moment, &fm_damping_v.moment);
  fm_damping_v.force = kVec3Zero;

  // Get hydrodynamic force and moment.
  ForceMoment fm_wave;
  CalcWaveForce(&fm_wave, time);

  // Total force and moment at the vessel frame origin.
  ForceMomentAdd(&fm_buoyancy_origin_v, &fm_damping_v, fm_hydro);
  ForceMomentAdd(fm_hydro, &fm_wave, fm_hydro);
}

void Buoy::CalcTetherForceMoment(ForceMoment *fm_tether) {
  // Calculate the tether force and moment about the vessel frame origin.
  ForceMomentPos fmx_tether_g, fmx_tether_v;
  fmx_tether_g.pos = Xg();
  ForceMomentPosRef(&tether_force_moment_g(), &fmx_tether_g);
  ForceMomentPosPoseTransform(&dcm_g2v(), &Xg(), &fmx_tether_g, &fmx_tether_v);
  fm_tether->force = fmx_tether_v.force;
  fm_tether->moment = fmx_tether_v.moment;
}

void Buoy::CalcGravityForceMoment(ForceMoment *fm_gravity) {
  Vec3 center_of_mass_to_origin_v;
  Vec3Scale(&center_of_mass_pos_, -1.0, &center_of_mass_to_origin_v);

  Vec3 force_gravity_g;
  Vec3Scale(&environment_.g_g(), mass_, &force_gravity_g);

  ForceMoment fm_gravity_cm;
  fm_gravity_cm.moment = kVec3Zero;
  Mat3Vec3Mult(&dcm_g2v(), &force_gravity_g, &fm_gravity_cm.force);
  ForceMomentRef(&fm_gravity_cm, &center_of_mass_to_origin_v, fm_gravity);
}

void Buoy::CalcMooringForceMoment(ForceMoment *fm_mooring) {
  // Obtain position and velocity of the mooring attachment point on the buoy
  // with respect the ground frame expressed in ground frame coordinates.
  Vec3 Xg_mooring_attach, Vg_mooring_attach;
  ReferenceFrame mooring_attach_frame(vessel_frame(),
                                      mooring_sim_params_.mooring_attach_v);
  mooring_attach_frame.TransformOriginTo(
      ground_frame_, ReferenceFrame::kPosition, &Xg_mooring_attach);
  mooring_attach_frame.TransformOriginTo(
      ground_frame_, ReferenceFrame::kVelocity, &Vg_mooring_attach);

  // Obtain the restoring force.
  double kt = mooring_sim_params_.kt0 +
              mooring_sim_params_.kt1 * Vec3XyNorm(&Xg_mooring_attach);
  Vec3 mooring_force_restore_g;
  Vec3Scale(&Xg_mooring_attach, -kt, &mooring_force_restore_g);
  mooring_force_restore_g.z = 0.;

  // Obtain the damping force.
  Vec3 mooring_force_damp_g;
  Vec3Scale(&Vg_mooring_attach,
            -mooring_sim_params_.ct * Vec3XyNorm(&Vg_mooring_attach),
            &mooring_force_damp_g);
  mooring_force_damp_g.z = 0.;

  // Obtain the restoring axial (yaw) torque due to mooring lines.
  // The yaw equilibrium heading is the heading (clockwise from North) of the
  // +X axis of the vessel frame when the buoy is at rest.
  Vec3 restoring_axial_coeffs = {0.0, 0.0,
                                 mooring_sim_params_.torsional_stiffness_z};
  Vec3 forward_v = kVec3X;  // Forward direction defined in the +X direction.
  Vec3 forward_g;
  Mat3TransVec3Mult(&dcm_g2v(), &forward_v, &forward_g);
  Vec3 forward_g_horizontal = {forward_g.x, forward_g.y, 0.0};
  Vec3 yaw_equilibrium_dir_ned = {
      cos(mooring_sim_params_.yaw_equilibrium_heading),
      sin(mooring_sim_params_.yaw_equilibrium_heading), 0.0};
  Mat3 dcm_ned2g = ground_frame_.CalcDcmNedToG();
  Vec3 yaw_equilibrium_dir_g;
  Mat3Vec3Mult(&dcm_ned2g, &yaw_equilibrium_dir_ned, &yaw_equilibrium_dir_g);
  Vec3 up = {0.0, 0.0, -1.0};
  double yaw_angle_from_eq;  // Clockwise from yaw_equilibrium_dir_g.
  // NOTE: The computation of yaw_angle_from_eq will work well only for angles
  // from equilibrium below 180 deg, because it is derived from the angle
  // between two vectors, and we assume the angle is always the smallest.
  // Accomodating larger angles will require using the yaw history to determine
  // what angle is the right one, but angles above 180 deg are not expected
  // nor desired.
  yaw_angle_from_eq = SignedAngleBetweenVectors(&forward_g_horizontal,
                                                &yaw_equilibrium_dir_g, &up);
  Vec3 yaw_rot = {0.0, 0.0, -yaw_angle_from_eq};
  Vec3 restoring_moment_axial;
  Vec3Mult(&restoring_axial_coeffs, &yaw_rot, &restoring_moment_axial);

  // Obtain the total force applied by the mooring lines.
  Vec3 mooring_force_total_g;
  Vec3Add(&mooring_force_restore_g, &mooring_force_damp_g,
          &mooring_force_total_g);

  // Build ForceMoment variable at the mooring line attachment point.
  ForceMoment fm_mooring_attach_v;
  Mat3Vec3Mult(&dcm_g2v(), &mooring_force_total_g, &fm_mooring_attach_v.force);
  fm_mooring_attach_v.moment = restoring_moment_axial;

  // Transfer the force and moment at the mooring line attachment point to the
  // origin of the vessel frame.
  Vec3 mooring_attach_to_origin_v;
  Vec3Scale(&mooring_sim_params_.mooring_attach_v, -1.0,
            &mooring_attach_to_origin_v);
  ForceMomentRef(&fm_mooring_attach_v, &mooring_attach_to_origin_v, fm_mooring);

  // Set yaw angle.
  yaw_angle_from_eq_.set_val(yaw_angle_from_eq);
}

void Buoy::GetBuoyancyForce(Vec3 *buoyancy_force_g) {
  // Obtain the buoyancy force.
  double water_density = sea_->GetWaterDensity();
  Vec3Scale(&environment_.g_g(), -water_density * wet_volume(),
            buoyancy_force_g);

  // The buoyancy damping model is quadratic with upward velocity.
  // TODO: Make this in the anti-gravity direction.
  Vec3 buoyancy_damping_g = kVec3Zero;
  buoyancy_damping_g.z = -hydrodynamics_sim_params_.buoyancy_damping_coeff *
                         Vg_center_of_mass().z * abs(Vg_center_of_mass().z);

  // Total buoyancy force.
  Vec3Add(buoyancy_force_g, &buoyancy_damping_g, buoyancy_force_g);
}

void Buoy::CalcWaveForce(ForceMoment *fm_wave, double time) {
  // Compute hydrodynamic force and moment due to waves.

  // Initialize to zero the total ForceMoment.
  *fm_wave = kForceMomentZero;

  // Get some important variables.
  double water_density = sea_->GetWaterDensity();
  Vec3 water_line_g;
  Vec3Add(&Xg(), Mat3TransVec3Mult(&dcm_g2v(), &water_line_v(), &water_line_g),
          &water_line_g);
  Vec3 buoy_bottom_g;
  Vec3 buoy_bottom_v = {0.0, 0.0, buoy_params_.bottom_deck_pos_z_v};
  Vec3Add(&Xg(), Mat3TransVec3Mult(&dcm_g2v(), &buoy_bottom_v, &buoy_bottom_g),
          &buoy_bottom_g);

  Vec3 axial_unit_g;  // Unit vector along the buoy axis.
  Vec3Normalize(Vec3Sub(&water_line_g, &buoy_bottom_g, &axial_unit_g),
                &axial_unit_g);

  // Define the buoy segment.
  int n_segments = 10;  // TODO: Make this a parameter.
  double segment_height = wet_height() / n_segments;
  double segment_area = PI * Square(buoy_params_.spar_diameter / 2.0);

  // For each segment compute the wave ForceMoment and accummulate.
  Vec3 segment_center_g;
  Vec3 fluid_vel_g, fluid_acc_g, fluid_perp_acc_g, perp_rel_vel_g;
  Vec3 origin_to_segment_g, origin_to_segment_v, vel_segment_v, vel_segment_g;
  Vec3 segment_rel_vel_g, inertial_force_segment_g, drag_force_segment_g,
      total_force_segment_g;
  ForceMoment fm_segment_v, fm_segment_origin_v;
  for (int i = 0; i < n_segments; i++) {
    // Find segment center.
    double relative_segment_pos =
        static_cast<double>(i) / n_segments + 1.0 / 2.0 / n_segments;
    CrossfadeVec3(&buoy_bottom_g, &water_line_g, relative_segment_pos, 0.0, 1.0,
                  &segment_center_g);

    // Get wave particle velocity and acceleration at the segment center.
    sea_->GetWaveKinematics(time, &segment_center_g, &fluid_vel_g,
                            &fluid_acc_g);

    // Get wave particle acceleration perpendicular to the buoy axis.
    // See https://math.stackexchange.com/a/1225525.
    Vec3Sub(&fluid_acc_g,
            Vec3Scale(&axial_unit_g, Vec3Dot(&fluid_acc_g, &axial_unit_g),
                      &fluid_perp_acc_g),
            &fluid_perp_acc_g);

    // Get segment velocity.
    Vec3Sub(&segment_center_g, &Xg(), &origin_to_segment_g);
    Mat3Vec3Mult(&dcm_g2v(), &origin_to_segment_g, &origin_to_segment_v);
    Vec3Cross(&omega(), &origin_to_segment_v, &vel_segment_v);
    Mat3TransVec3Mult(&dcm_g2v(), &vel_segment_v, &vel_segment_g);
    Vec3Add(&Vg(), &vel_segment_g, &vel_segment_g);

    // Get fluid relative velocity.
    Vec3Sub(&fluid_vel_g, &vel_segment_g, &segment_rel_vel_g);

    // Get the fluid relative velocity perpendicular to buoy axis.
    Vec3Sub(&segment_rel_vel_g,
            Vec3Scale(&axial_unit_g, Vec3Dot(&segment_rel_vel_g, &axial_unit_g),
                      &perp_rel_vel_g),
            &perp_rel_vel_g);

    // Obtain inertial wave force acting on the segment.
    Vec3Scale(&fluid_perp_acc_g, water_density * segment_height * segment_area *
                                     (1.0 + hydrodynamics_sim_params_.Ca),
              &inertial_force_segment_g);

    // Obtain the drag force acting on the segment.
    Vec3Scale(&perp_rel_vel_g,
              0.5 * water_density * segment_height *
                  buoy_params_.spar_diameter * hydrodynamics_sim_params_.Cd *
                  Vec3Norm(&perp_rel_vel_g),
              &drag_force_segment_g);

    // Obtain the total force acting on the segment.
    Vec3Add(&inertial_force_segment_g, &drag_force_segment_g,
            &total_force_segment_g);
    Mat3Vec3Mult(&dcm_g2v(), &total_force_segment_g, &fm_segment_v.force);
    fm_segment_v.moment = kVec3Zero;

    // Find vector from segment to origin of the vessel frame.
    Vec3 center_to_origin_g, center_to_origin_v;
    Vec3Sub(&Xg(), &segment_center_g, &center_to_origin_g);
    Mat3Vec3Mult(&dcm_g2v(), &center_to_origin_g, &center_to_origin_v);

    // Translate ForceMoment to origin of the vessel frame and add to overall
    // ForceMoment fm_wave.
    ForceMomentRef(&fm_segment_v, &center_to_origin_v, &fm_segment_origin_v);
    ForceMomentAdd(&fm_segment_origin_v, fm_wave, fm_wave);
  }
}

void Buoy::CalcAcceleratingFrame(ReferenceFrame *accelerating_frame) const {
  Vec3 omega_g, domega_g;
  vessel_frame().RotateTo(ground_frame_, omega(), &omega_g);
  vessel_frame().RotateTo(ground_frame_, domega(), &domega_g);

  // Build an accelerating frame attached to the buoy and located at its
  // center-of-mass.
  ReferenceFrame acc_frame_com(ground_frame_, Xg_center_of_mass(),
                               Vg_center_of_mass(), dVg_center_of_mass(),
                               dcm_g2v(), omega_g, domega_g);

  // Offset the accelerating frame to the origin of the vessel.
  Vec3 acc_frame_pos;
  Vec3Scale(&center_of_mass_pos_, -1.0, &acc_frame_pos);
  *accelerating_frame = ReferenceFrame(acc_frame_com, acc_frame_pos);
}
