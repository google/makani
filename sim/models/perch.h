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

#ifndef SIM_MODELS_PERCH_H_
#define SIM_MODELS_PERCH_H_

#include <stdint.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/models/model.h"
#include "sim/models/rigid_bodies/buoy.h"
#include "sim/physics/contactor.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"

class Perch : public Model {
  friend class PerchTest;

 public:
  Perch(const ReferenceFrame &ground_frame, const Buoy &buoy,
        const PerchParams &perch_params,
        const LevelwindParams &levelwind_params,
        const WinchParams &winch_params,
        const PerchSimParams &perch_sim_params);
  ~Perch() {}

  void Init(double theta);

  void Publish() const override;
  double CalcTetherFreeLength() const;

  void set_external_force_g(const Vec3 &val) { external_force_g_.set_val(val); }
  void set_winch_torque(double val) { winch_torque_.set_val(val); }

  double theta_perch() const { return theta_.val().x; }
  // TODO: Change winch sign convention.
  double theta_winch() const { return theta_.val().y; }
  double omega_winch() const { return omega_.val().y; }

  const Vec3 &anchor_pos_g() const { return anchor_pos_g_.val(); }
  const Vec3 &anchor_vel_g() const { return anchor_vel_g_.val(); }
  const Mat3 &dcm_g2p() const { return dcm_g2p_.val(); }
  bool levelwind_engaged() const { return levelwind_engaged_.val(); }

  const ContactSurface &surface() const { return surface_; }
  const ReferenceFrame &frame() const { return frame_.val(); }

 private:
  // Calculates the position and the velocity of the GSG in ground coordinates.
  void CalcGsgPosVelG(Vec3 *gsg_pos_g, Vec3 *gsg_vel_g) const;

  // Calculates the position and velocity of the origin of the levelwind
  // coordinate system in ground coordinates. This accounts for the vertical
  // travel of the levelwind, the perch angular velocity and the motion of the
  // vessel frame relative to the ground frame.
  void CalcLevelwindPosVelG(Vec3 *levelwind_pos_g, Vec3 *levelwind_vel_g) const;

  // Returns true if the levelwind should be engaged with the tether
  // based on the drum angle and the angle between the force vector of
  // the tether and the levelwind.
  bool IsLevelwindEngaged(double drum_angle, bool levelwind_engaged_z1__,
                          const Vec3 &external_force_g__) const;

  // Calculates the position and velocity of the anchor point where
  // the tether attaches to the perch.
  void CalcAnchorPoint(bool levelwind_engaged__, Vec3 *anchor_pos_g__,
                       Vec3 *anchor_vel_g__) const;

  void CalcDerivHelper(double t) override;
  void AddInternalConnections(ConnectionStore *connections) override;
  void UpdateDerivedStates();
  void DiscreteStepHelper(double /*t*/) override;

  const Vec3 &external_force_g() const { return external_force_g_.val(); }
  const Vec3 &external_force_g_z1() const { return external_force_g_z1_.val(); }
  bool levelwind_engaged_z1() const { return levelwind_engaged_z1_.val(); }
  double winch_torque() const { return winch_torque_.val(); }
  const Vec2 &omega() const { return omega_.val(); }
  double omega_perch() const { return omega_.val().x; }
  const ReferenceFrame &panel_frame() const { return panel_frame_.val(); }

  // Perch parameters.
  const PerchParams &perch_params_;
  const LevelwindParams &levelwind_params_;
  const WinchParams &winch_params_;
  const PerchSimParams &perch_sim_params_;

  const ReferenceFrame &ground_frame_;
  const Buoy &buoy_;

  // Input states.

  // Force vector [N], typically from the tether, acting on the lever
  // arm of the perch.
  State<Vec3> external_force_g_;

  // Torque [N-m] from the winch given the angle [rad] and angular
  // rate [rad/s] of the winch.
  State<double> winch_torque_;

  // Discrete state.

  // NOTE: This discrete state should not be necessary,
  // because it should be possible to arrange the levels in
  // full_system.cc such that we can use the input external_force_g.
  // Practically, it is much easier to use the previous value.
  //
  // Force vector [N] from the previous time step.  This is used to
  // determine when the levelwind has engaged.
  DiscreteState<Vec3> external_force_g_z1_;

  // Whether the levelwind was engaged on the last time step.
  DiscreteState<bool> levelwind_engaged_z1_;

  // Continuous state.

  // Azimuth angle [rad] and rotation rate [rad/s] of the perch (x)
  // and winch drum (y).  The winch drum angle is defined to be 0 rad
  // in the crosswind configuration and it increases during reel-in
  // (see comment about changing sign convention above.).
  ContinuousState<Vec2> theta_, omega_;

  // Output states.

  // Reference frame of the perch.
  State<ReferenceFrame> frame_;

  // This frame is a child of the perch frame. Its origin is at the top of the
  // panels along the bookseam (seam between the panels). It is pitched and
  // rolled so that the z-axis points downward along the bookseam.
  State<ReferenceFrame> panel_frame_;

  ContactSurface surface_;

  // Derived values.

  // Position [m] and velocity [m/s] of the tether anchor point.  This
  // can either be the GSG, the drum surface, or the levelwind
  State<Vec3> anchor_pos_g_, anchor_vel_g_;

  // Rotation matrix between ground and perch coordinates.
  State<Mat3> dcm_g2p_;

  // True if the levelwind is engaged and thus determines the tether
  // anchor point.
  State<bool> levelwind_engaged_;

  DISALLOW_COPY_AND_ASSIGN(Perch);
};

#endif  // SIM_MODELS_PERCH_H_
