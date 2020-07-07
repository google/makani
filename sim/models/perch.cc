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

#include "sim/models/perch.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include <limits>
#include <vector>

#include "common/c_math/mat2.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "control/perch_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/models/ground_station_util.h"
#include "sim/models/rigid_bodies/buoy.h"
#include "sim/physics/contactor.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"

namespace internal {

// Returns true if the ray starting at ray_root and extending in the
// direction ray_direction intersects the circle specified by
// circle_center and circle_radius.
bool RayIntersectsCircle(const Vec2 &ray_root, const Vec2 &ray_direction,
                         const Vec2 &circle_center, double circle_radius) {
  // Ray direction must be a unit vector.
  DCHECK_GE(3.0 * std::numeric_limits<double>::epsilon(),
            1.0 - Vec2Norm(&ray_direction));
  // Calculate the vector that is perpendicular to ray_direction and
  // intersects the circle center.  This is the shortest distance
  // between the ray and the circle.
  Vec2 a;
  Vec2Sub(&circle_center, &ray_root, &a);
  double t = Vec2Dot(&ray_direction, &a);
  Vec2LinComb(1.0, &a, -t, &ray_direction, &a);

  return t >= 0.0 && Vec2Norm(&a) < circle_radius;
}

}  // namespace internal

Perch::Perch(const ReferenceFrame &ground_frame, const Buoy &buoy,
             const PerchParams &perch_params,
             const LevelwindParams &levelwind_params,
             const WinchParams &winch_params,
             const PerchSimParams &perch_sim_params)
    : Model("Perch"),
      perch_params_(perch_params),
      levelwind_params_(levelwind_params),
      winch_params_(winch_params),
      perch_sim_params_(perch_sim_params),
      ground_frame_(ground_frame),
      buoy_(buoy),
      external_force_g_(new_derived_value(), "external_force_g"),
      winch_torque_(new_derived_value(), "winch_torque"),
      external_force_g_z1_(new_discrete_state(), "external_force_g_z1",
                           kVec3Zero),
      levelwind_engaged_z1_(new_discrete_state(), "levelwind_engaged_z1",
                            false),
      theta_(new_continuous_state(), "theta",
             {perch_sim_params.theta_p_0, 0.0}),
      omega_(new_continuous_state(), "omega", kVec2Zero),
      frame_(new_derived_value(), "frame"),
      panel_frame_(new_derived_value(), "panel_frame"),
      surface_(this->panel_frame_.val_unsafe(), nullptr),
      anchor_pos_g_(new_derived_value(), "anchor_pos_g"),
      anchor_vel_g_(new_derived_value(), "anchor_vel_g"),
      dcm_g2p_(new_derived_value(), "dcm_g2p"),
      levelwind_engaged_(new_derived_value(), "levelwind_engaged") {
  surface_.set_collision_func([this, perch_sim_params](
                                  const Vec3 &contactor_pos,
                                  Vec3 *collision_pos) -> bool {
    return ContactPerchPanel(contactor_pos, this->frame(), this->panel_frame(),
                             perch_sim_params.panel, collision_pos);
  });
  UpdateDerivedStates();

  SetupDone();
}

// Sets the initial winch drum angle.  This is separated from the rest
// of the initialization because the winch drum angle depends on the
// wing position, levelwind position, and perch angle in a somewhat
// complicated manner, and it is easiest to find it iteratively by
// searching for the zero tether force position.
//
// Args:
//   theta_winch_drum: Initial winch drum angle [rad].
void Perch::Init(double theta_winch_drum) {
  Vec2 theta_vec = {theta_perch(), theta_winch_drum};
  theta_.Clear();
  theta_.set_val(theta_vec);

  ClearDerivedValues();
  UpdateDerivedStates();
}

// Change the free length of the tether when the levelwind engages.
// There are two components to the difference in the free length of
// the tether.  First, there is the distance between engagement point
// of the levelwind and the GSG:
//
//   dist_1 = || gsg_pos_p(engaged) - levelwind_pos_p ||
//          = || dcm_wd2p * gsg_pos_wd(engaged) - levelwind_pos_p ||
//
// Second, there is the amount of the tether that the winch has reeled
// in, which is simply:
//
//   dist_2 = r_drum * (theta_winch - theta_winch(engaged))
//
double Perch::CalcTetherFreeLength() const {
  double tether_free_length = g_sys.tether->length;

  // Subtract the length of tether that is wound around the winch
  // drum.  This intentionally does not include the first half
  // rotation of the winch drum, i.e. pulley_engage_drum_angle, which
  // is used to rotate the perch rather than wind the tether.
  tether_free_length -=
      winch_params_.r_drum *
      fmax((levelwind_params_.pulley_engage_drum_angle - theta_winch()), 0.0);

  // If the levelwind is engaged, subtract the distance between the
  // levelwind origin and the contact point on the winch drum.
  if (levelwind_engaged()) {
    // TODO: If the levelwind engages with the drum rotated
    // past the pulley engage angle this difference in length is not
    // quite accurate and the wing jumps.
    Mat3 dcm_wd2p_engaged;
    AngleToDcm(-levelwind_params_.pulley_engage_drum_angle, 0.0, 0.0,
               kRotationOrderZyx, &dcm_wd2p_engaged);
    Vec3 gsg_engaged_pos_p;
    Mat3Vec3Mult(&dcm_wd2p_engaged, &perch_params_.gsg_pos_wd,
                 &gsg_engaged_pos_p);
    Vec3Add(&gsg_engaged_pos_p, &perch_params_.winch_drum_origin_p,
            &gsg_engaged_pos_p);

    double dist_gsg_to_levelwind_at_engage = Vec3Norm(
        Vec3Sub(&gsg_engaged_pos_p, &perch_params_.levelwind_origin_p_0,
                &gsg_engaged_pos_p));

    tether_free_length -= dist_gsg_to_levelwind_at_engage;
  }

  return tether_free_length;
}

void Perch::Publish() const {
  sim_telem.perch.theta_p = theta_perch();
  sim_telem.perch.omega_p = omega_perch();
  sim_telem.perch.theta_wd = theta_winch();
  sim_telem.perch.omega_wd = omega_winch();
  sim_telem.perch.levelwind_engaged = levelwind_engaged();
  sim_telem.perch.tether_free_length = CalcTetherFreeLength();
  sim_telem.perch.anchor_pos_g = anchor_pos_g();
  Vec3 unused_variable;
  CalcGsgPosVelG(&sim_telem.perch.gsg_pos_g, &unused_variable);
  CalcLevelwindPosVelG(&sim_telem.perch.levelwind_pos_g, &unused_variable);
}

void Perch::CalcGsgPosVelG(Vec3 *gsg_pos_g, Vec3 *gsg_vel_g) const {
  // Compute the position of the GSG in the perch frame.
  Mat3 dcm_p2wd;
  Vec3 gsg_pos_p;
  AngleToDcm(theta_winch(), 0.0, 0.0, kRotationOrderZyx, &dcm_p2wd);
  Mat3TransVec3Mult(&dcm_p2wd, &perch_params_.gsg_pos_wd, &gsg_pos_p);

  // Attach a reference frame to the GSG, derived from a pure translation of the
  // perch frame. The drum rotation is ignored since the winch stops once the
  // anchor of the tether is on the GSG.
  ReferenceFrame gsg_frame(frame(), gsg_pos_p);

  // Calculate the position and velocity of the gsg frame origin relative to the
  // ground frame.
  gsg_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kPosition,
                              gsg_pos_g);
  gsg_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kVelocity,
                              gsg_vel_g);
}

void Perch::CalcLevelwindPosVelG(Vec3 *levelwind_pos_g,
                                 Vec3 *levelwind_vel_g) const {
  // Attach a reference frame to the levelwind.
  Vec3 levelwind_pos_p = perch_params_.levelwind_origin_p_0;
  levelwind_pos_p.z +=
      levelwind_params_.drum_angle_to_vertical_travel * theta_winch();
  ReferenceFrame levelwind_frame(frame(), levelwind_pos_p);

  // Calculate the position and velocity of the levelwind frame origin relative
  // to the ground frame.
  levelwind_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kPosition,
                                    levelwind_pos_g);
  levelwind_frame.TransformOriginTo(ground_frame_, ReferenceFrame::kVelocity,
                                    levelwind_vel_g);
}

bool Perch::IsLevelwindEngaged(double drum_angle, bool levelwind_engaged_z1__,
                               const Vec3 &external_force_g__) const {
  // For large drum angles assume that we are engaged.  For drum
  // angles slightly larger than the engage drum angle, assume we are
  // engaged if we were engaged on the previous time step to avoid
  // bouncing on and off the levelwind.
  //
  // NOTE: You can get large jumps in tension if reel-in
  // is too fast and the large drum angle criterion is used to flip
  // back to the levelwind being engaged.
  if (drum_angle < 4.0 * levelwind_params_.pulley_engage_drum_angle ||
      (drum_angle < levelwind_params_.pulley_engage_drum_angle &&
       levelwind_engaged_z1__)) {
    return true;
  }

  // Determine if the tether intersects the levelwind.  This
  // approximate condition tests if the ray that starts at the GSG and
  // points in the direction of the external force intersects the
  // levelwind circle.  This calculation is done after projecting the
  // positions and vectors into the perch xy-plane, and neglects
  // elevation mismatch between the levelwind and GSG.
  Vec3 external_force_p;
  ground_frame_.RotateTo(frame(), external_force_g__, &external_force_p);
  Vec3 gsg_pos_p;
  WdToP(&perch_params_.gsg_pos_wd, drum_angle, &gsg_pos_p);

  // Project both vectors into the xy-plane.
  Vec2 center = {perch_sim_params_.levelwind_hub_p.x,
                 perch_sim_params_.levelwind_hub_p.y};
  Vec2 root = {gsg_pos_p.x, gsg_pos_p.y};
  Vec2 direction = {external_force_p.x, external_force_p.y};
  // The levelwind won't engage in low tensions.
  if (Vec2Norm(&direction) < perch_sim_params_.levelwind_engage_min_tension) {
    return false;
  } else {
    Vec2Normalize(&direction, &direction);
    return drum_angle < levelwind_params_.pulley_engage_drum_angle &&
           internal::RayIntersectsCircle(root, direction, center,
                                         perch_sim_params_.levelwind_radius);
  }
}

void Perch::CalcAnchorPoint(bool levelwind_engaged__, Vec3 *anchor_pos_g__,
                            Vec3 *anchor_vel_g__) const {
  if (levelwind_engaged__) {
    // Set the anchor point to be the levelwind origin and compute the position
    // and velocity relative to the ground frame.
    CalcLevelwindPosVelG(anchor_pos_g__, anchor_vel_g__);
  } else {
    // NOTE: On the real system, it is possible for the
    // drum surface to act as anchor point, but this is a situation we
    // should avoid.

    // Set the anchor point to be the GSG, compute position and velocity
    // relative to the ground frame.
    CalcGsgPosVelG(anchor_pos_g__, anchor_vel_g__);
  }
}

// The winch angle is zero when the winch drum is in the crosswind
// position, and ground station construction is such that a positive
// winch angular speed reels in the tether. Positive rotation is
// defined by the z-down winch drum frame.  Therefore, winch position
// is positive when the vehicle is on the perch.  Note that the
// controller may not yet follow this sign convention.
void Perch::CalcDerivHelper(double /*t*/) {
  // Find joint torques from tether tension.
  Vec3 anchor_pos;
  Vec3Sub(&anchor_pos_g(), &buoy_.Xg(), &anchor_pos);
  Vec3 tau_g, tau_p;
  Vec3Cross(&anchor_pos, &external_force_g(), &tau_g);
  Mat3Vec3Mult(&dcm_g2p(), &tau_g, &tau_p);
  // Approximate kinetic friction via a smooth function to avoid
  // hurting ODE performance.  The coefficient of 350.0 applies 95% of
  // the frictional force at a rotation rate of approximately 0.005
  // [rad/s].
  tau_p.z += -perch_params_.kinetic_friction_perch * tanh(omega().x * 350.0);

  // Find the torque applied to the winch by the tether.  When the
  // levelwind is engaged, tension is applied tangent to the winch
  // drum.
  Vec3 tau_w = kVec3Zero;
  if (levelwind_engaged()) {
    tau_w.z = Vec3Norm(&external_force_g()) * winch_params_.r_drum;
  } else {
    Vec3 winch_drum_origin_g, dum;
    Mat3TransVec3Mult(&dcm_g2p(), &perch_params_.winch_drum_origin_p,
                      &winch_drum_origin_g);
    Vec3Add(&winch_drum_origin_g, &buoy_.Xg(), &winch_drum_origin_g);

    Vec3Cross(Vec3Sub(&anchor_pos_g(), &winch_drum_origin_g, &dum),
              &external_force_g(), &tau_w);
  }

  // Find the winch and perch accelerations.  Add joint torques from
  // the winch servo and the externally applied forces. Note that
  // centrifugal and Coriolis forces are zero (and the A-matrix is
  // constant) as long as the winch drum CG lies on the winch drum
  // center-of-rotation.  This code makes that simplifying assumption.
  Vec2 A_omega, B_torque, domega__;
  Vec2 joint_torques = {tau_p.z, tau_w.z + winch_torque()};
  // domega = A*omega + B_joint*tau_ctrl + B_force*tau_force
  // where, omega = [omega_perch omega_winch]'
  Vec2Add(Mat2Vec2Mult(&perch_sim_params_.A, &omega(), &A_omega),
          Mat2Vec2Mult(&perch_sim_params_.B, &joint_torques, &B_torque),
          &domega__);

  if (*g_sim.sim_opt & kSimOptTiedDown) {
    domega__ = kVec2Zero;
  }

  omega_.set_deriv(domega__);
  theta_.set_deriv(omega());  // theta = [theta_perch theta_winch]'
}

void Perch::DiscreteStepHelper(double t) {
  external_force_g_z1_.DiscreteUpdate(t, external_force_g());
  levelwind_engaged_z1_.DiscreteUpdate(t, levelwind_engaged());
}

void Perch::UpdateDerivedStates() {
  // Find DCMs.
  Mat3 dcm_v2p, dcm_g2p__;
  AngleToDcm(theta_perch(), 0.0, 0.0, kRotationOrderZyx, &dcm_v2p);
  Mat3Mat3Mult(&dcm_v2p, &buoy_.dcm_g2v(), &dcm_g2p__);
  dcm_g2p_.set_val(dcm_g2p__);

  // Update the reference frame of the perch.  Note that omega_p_vec
  // is equivalent to omega_v_vec because the z-axes are collinear.
  Vec3 omega_p_vec = {0.0, 0.0, omega_perch()};
  frame_.set_val(ReferenceFrame(buoy_.vessel_frame(), kVec3Zero, kVec3Zero,
                                dcm_v2p, omega_p_vec));

  // Update the panel frame.
  panel_frame_.set_val(ReferenceFrame(frame_.val(), kVec3Zero,
                                      perch_sim_params_.panel.dcm_p2panel));

  // Use a simple levelwind engagement strategy.  Note that effects
  // like the levelwind "bouncing" off of the tether are not
  // captured.
  levelwind_engaged_.set_val(IsLevelwindEngaged(
      theta_winch(), levelwind_engaged_z1(), external_force_g_z1()));

  // Determine the tether anchor position and velocity.  The anchor
  // point may either be the GSG or the levelwind depending on the
  // angles of the winch drum and perch.
  Vec3 anchor_pos_g__, anchor_vel_g__;
  CalcAnchorPoint(levelwind_engaged(), &anchor_pos_g__, &anchor_vel_g__);
  anchor_pos_g_.set_val(anchor_pos_g__);
  anchor_vel_g_.set_val(anchor_vel_g__);
}

void Perch::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(1, [this](double /*t*/) { UpdateDerivedStates(); });
}
