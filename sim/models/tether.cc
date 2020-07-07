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

#include "sim/models/tether.h"

#include <glog/logging.h>
#include <math.h>
#include <stdint.h>

#include <string>

#include "common/c_math/vec3.h"
#include "control/ground_frame.h"
#include "sim/models/environment.h"
#include "sim/sim_telemetry.h"

namespace internal {

// Calculates the curvature of the circle passing through an arbitrary point p,
// p + v1, and p + v1 + v2.
//
// First, we calculate the radius vector, r, from the center of the circle to p
// + v1. This is determined by the following conditions:
//   (1) r \dot (v1 \cross v2) = 0
//   (2) r \dot v1 = 1/2 |v1|^2
//   (3) r \dot v2 = -1/2 |v2|^2.
// Condition (2) is produced by observing that when the circle is drawn,
// r - v1/2 is the perpendicular bisector of the chord corresponding to v1.
// Condition (3) is analogous.
//
// Then, the curvature vector has magnitude 1 / |r| and is in the direction of
// v1 \cross v2. The sign is determined by the direction in which the three
// points p, p + v1, and p + v1 + v2 traverse the circle.
//
// Returns true if it successfully calculates a curvature and false if it
// detects an edge case, rather than making assumptions about intent.
// The caller must decide how to handle edge cases.
bool CalcCurvature(const Vec3 &v1, const Vec3 &v2, Vec3 *curvature) {
  Vec3 cross;
  Vec3Cross(&v1, &v2, &cross);

  // If either v1 or v2 are zero return false and don't calculate a curvature.
  if ((Vec3Norm(&v1) < 1e-9) || (Vec3Norm(&v2) < 1e-9)) {
    return false;
  }
  // If v1 and v2 are parallel and in the same direction, the curvature is
  // zero. If they are parallel and in opposite directions, return false.
  // We could calculate a limiting scalar value of the curvature, but not a
  // direction in this case.
  if (Vec3Norm(&cross) < 1e-6) {    // parallel
    if (Vec3Dot(&v1, &v2) > 0.0) {  // in same direction
      *curvature = kVec3Zero;
      return true;
    } else {  // in opposite directions
      return false;
    }
  }

  // Create and solve the linear system for r. If we've reached this point, v1
  // and v2 are not parallel, so the system must be nonsingular.
  VEC_INIT(3, b, {0.0, Vec3Dot(&v1, &v1) / 2.0, -Vec3Dot(&v2, &v2) / 2.0});
  MAT_INIT(
      3, 3, A,
      {{cross.x, cross.y, cross.z}, {v1.x, v1.y, v1.z}, {v2.x, v2.y, v2.z}});
  VEC_INIT(3, r, {0.0});
  if (MatVecLeftDivide(&A, &b, &r) == kLinalgErrorNone) {
    Vec3Scale(&cross, 1.0 / Vec3Norm(&cross) / VecNorm(&r), curvature);
    return true;
  } else {
    return false;
  }
}
}  // namespace internal

namespace {
// Calculates the future position of a node at small time step, assuming it
// continues moving at its current velocity
void CalcXgStep(const Vec3 &xg, const Vec3 &vg, const double &t_step,
                Vec3 *xg_step) {
  Vec3 dxg;
  Vec3Scale(&vg, t_step, &dxg);
  Vec3Add(&xg, &dxg, xg_step);
}

}  // namespace

Tether::Tether(const Environment &environment,
               const TetherParams &tether_params,
               const TetherSimParams &tether_sim_params)
    : Model("Tether"),
      tether_params_(tether_params),
      tether_sim_params_(tether_sim_params),
      num_nodes_(tether_sim_params.num_nodes),
      L_seg_0_(tether_params.length / tether_sim_params.num_nodes),
      c_damp_staged_(
          tether_sim_params.longitudinal_damping_ratio_staged *
          sqrt(2.0 * tether_params.tensile_stiffness *
               tether_params.linear_density)),  // Longitudinal damping
      c_damp_active_(
          tether_sim_params.longitudinal_damping_ratio_active *
          sqrt(2.0 * tether_params.tensile_stiffness *
               tether_params.linear_density)),  // Longitudinal damping
      // NOTE: bend_moment_damping =
      //   d_damp_coeff_active_ * L_seg_0 * L_seg_0 * curvature_rate
      d_damp_coeff_active_(
          tether_sim_params.bending_damping_ratio_active *
          sqrt(tether_params_.bending_stiffness *
               tether_params_.linear_density)),  // Bending damping
      environment_(environment),
      ground_surface_(nullptr),
      Xg_start_(new_derived_value(), "Xg_start"),
      Vg_start_(new_derived_value(), "Vg_start"),
      Xg_end_(new_derived_value(), "Xg_end"),
      Vg_end_(new_derived_value(), "Vg_end"),
      free_length_(new_derived_value(), "free_length"),
      Xg_nodes_(),
      Vg_nodes_(),
      start_ind_(new_derived_value(), "start_ind"),
      end_ind_(new_derived_value(), "end_ind"),
      L_start_(new_derived_value(), "L_start"),
      L_end_(new_derived_value(), "L_end"),
      tether_start_dir_g_(new_derived_value(), "tether_start_dir_g"),
      start_force_moment_(new_derived_value(), "start_force_moment"),
      end_force_moment_(new_derived_value(), "end_force_moment"),
      F_aero_nodes_(),
      F_long_spring_nodes_(),
      F_long_damp_nodes_(),
      F_bend_spring_damp_nodes_(),
      F_ground_nodes_(),
      F_teth_nodes_(),
      curvature_nodes_(),
      curvature_rate_nodes_() {
  CHECK_GE(num_nodes_, 0) << "The number of nodes should be non-negative.";
  CHECK_GT(tether_params_.length, 0.0)
      << "The tether length should be positive.";

  Xg_nodes_.reserve(num_nodes_);
  Vg_nodes_.reserve(num_nodes_);
  F_aero_nodes_.reserve(num_nodes_);
  F_long_spring_nodes_.reserve(num_nodes_);
  F_long_damp_nodes_.reserve(num_nodes_);
  F_bend_spring_damp_nodes_.reserve(num_nodes_);
  F_ground_nodes_.reserve(num_nodes_);
  F_teth_nodes_.reserve(num_nodes_);
  curvature_nodes_.reserve(num_nodes_);
  curvature_rate_nodes_.reserve(num_nodes_);

  for (int32_t i = 0; i < num_nodes_; ++i) {
    Xg_nodes_.emplace_back(new_continuous_state(),
                           "Xg_nodes[" + std::to_string(i) + "]", kVec3Zero);
    Vg_nodes_.emplace_back(new_continuous_state(),
                           "Vg_nodes[" + std::to_string(i) + "]", kVec3Zero);
    F_aero_nodes_.emplace_back(new_derived_value(),
                               "F_aero_nodes[" + std::to_string(i) + "]",
                               kVec3Zero);
    F_long_spring_nodes_.emplace_back(
        new_derived_value(), "F_long_spring_nodes[" + std::to_string(i) + "]",
        kVec3Zero);
    F_long_damp_nodes_.emplace_back(
        new_derived_value(), "F_long_damp_nodes[" + std::to_string(i) + "]",
        kVec3Zero);
    F_bend_spring_damp_nodes_.emplace_back(
        new_derived_value(),
        "F_bend_spring_damp_nodes[" + std::to_string(i) + "]", kVec3Zero);
    F_ground_nodes_.emplace_back(new_derived_value(),
                                 "F_ground_nodes[" + std::to_string(i) + "]",
                                 kVec3Zero);
    F_teth_nodes_.emplace_back(new_derived_value(),
                               "F_teth_nodes[" + std::to_string(i) + "]",
                               kVec3Zero);
    curvature_nodes_.emplace_back(new_derived_value(),
                                  "curvature_nodes[" + std::to_string(i) + "]",
                                  kVec3Zero);
    curvature_rate_nodes_.emplace_back(
        new_derived_value(), "curvature_rate_nodes[" + std::to_string(i) + "]",
        kVec3Zero);
  }
  SetupDone();
}

// Initializes the continuous state of the tether iteratively because
// the endpoints of the tether can depend on the tether state.
//
// TODO: Look for a way of initializing without passing
// in a pointer to the wing model.
void Tether::Init(CalcBridlePointFunc calc_bridle_point, const Vec3 &Xg_start__,
                  const Vec3 &Vg_start__, double free_length__) {
  const int32_t num_iter_init_nodes = 2;
  for (int32_t i = 0; i < num_iter_init_nodes; ++i) {
    ClearDerivedValues();
    set_free_length(free_length__);
    set_Xg_start(Xg_start__);
    set_Vg_start(Vg_start__);
    UpdateStartAndEndIndices();
    Vec3 pos, vel;
    calc_bridle_point(Xg_last_node(), Vg_last_node(), &pos, &vel);
    set_Xg_end(pos);
    set_Vg_end(vel);
    UpdateForceMoments();
    ClearContinuousStates();
    InitTetherNodes(Xg_start(), Xg_end(), free_length());
  }
  UpdateTetherStartDirG();
}

void Tether::Publish() const {
  sim_telem.tether.start_ind = start_ind();
  sim_telem.tether.end_ind = end_ind();
  sim_telem.tether.L_start = L_start();
  sim_telem.tether.L_end = L_end();
  sim_telem.tether.Xg_start = Xg_start();
  sim_telem.tether.Vg_start = Vg_start();
  sim_telem.tether.Xg_end = Xg_end();
  sim_telem.tether.Vg_end = Vg_end();
  sim_telem.tether.aero_power = 0.0;
  for (int32_t i = 0; i < num_nodes_; ++i) {
    sim_telem.tether.Xg_nodes[i] = Xg_nodes(i);
    sim_telem.tether.Vg_nodes[i] = Vg_nodes(i);
    sim_telem.tether.Fg_nodes[i] = F_teth_nodes(i);
    sim_telem.tether.Fg_aero_nodes[i] = F_aero_nodes(i);
    sim_telem.tether.aero_power += Vec3Dot(&Vg_nodes(i), &F_aero_nodes(i));
  }
  sim_telem.tether.start_force_g = start_force_moment().force;
  sim_telem.tether.end_force_g = end_force_moment().force;
}

void Tether::UpdateTetherStartDirG() {
  const Vec3 &base_node = Xg_start();
  Vec3 starting_node = Xg_end();

  if (start_ind() < num_nodes_) {
    starting_node = Xg_nodes(start_ind());
  }

  Vec3 tether_start_dir_g__;
  Vec3Sub(&starting_node, &base_node, &tether_start_dir_g__);
  tether_start_dir_g_.set_val(tether_start_dir_g__);
}

void Tether::UpdateStartAndEndIndices() {
  double tether_len = free_length();
  double L_start__, L_end__;
  start_ind_.set_val(GetStartIndex(tether_len, &L_start__));
  L_start_.set_val(L_start__);
  end_ind_.set_val(GetEndIndex(tether_len, &L_end__));
  L_end_.set_val(L_end__);
}

// Calculates the curvature and rate of change of curvature at each node.
// The curvature is calculated by fitting a circle through the current node and
// the adjacent nodes. The approach is similar to that of other industry dynamic
// cable simulators, which make a small-angle assumption that allows for a
// direct calculation instead of solving a set of equations. (IE, Orcaflex:
// https://www.orcina.com/SoftwareProducts/OrcaFlex/Documentation/Help/
// Theory → Line Theory → Calculation Stage 2 Bend Moments)
// The rate of change of the curvature is calculated based on a finite
// difference method, using the curvature calculated at the current time, and
// the curvature calculated using the node positions projected forward in time
// by a small step based on Vg_nodes.
// Curvature and rate of curvature are assumed to be zero at start and end of
// tether based on pinned end conditions, which neglects bearing friction and
// termination inertia, so do not need to be calculated seperately.
// The curvatures and curvature rates need to be available to the
// CalcBendingSpringDampForceSeg method no matter if called through
// UpdateTetherForces or
// CalcEndForceMomentPos.
void Tether::UpdateCurvatures() {
  // Declare semi-arbitrary timestep [s] to use for projecting nodes forward.
  double t_step = 1e-6;
  // Precalculate the coordinates of current node at t+t_step
  std::vector<Vec3> Xg_nodes_step(num_nodes_);
  for (int32_t i = 0; i < num_nodes_; ++i) {
    CalcXgStep(Xg_nodes(i), Vg_nodes(i), t_step, &Xg_nodes_step[i]);
  }
  for (int32_t i = 0; i < num_nodes_; ++i) {
    Vec3 curvature_node, curvature_rate_node, Xg_pre, Xg_pre_step, Xg_post,
        Xg_post_step, Xg_seg_pre, Xg_seg_post;
    if (i > end_ind()          // Handle unused nodes.
        || i < start_ind()) {  // Handle inactive nodes.
      curvature_node = kVec3Zero;
      curvature_rate_node = kVec3Zero;
    } else {
      // Handle coordinates for start and end cases.
      if (i == start_ind()) {
        Xg_pre = Xg_start();
        CalcXgStep(Xg_start(), Vg_start(), t_step, &Xg_pre_step);
      } else {
        Xg_pre = Xg_nodes(i - 1);
        Xg_pre_step = Xg_nodes_step[i - 1];
      }
      if (i == end_ind()) {
        Xg_post = Xg_end();
        CalcXgStep(Xg_end(), Vg_end(), t_step, &Xg_post_step);
      } else {
        Xg_post = Xg_nodes(i + 1);
        Xg_post_step = Xg_nodes_step[i + 1];
      }

      // Calculate the segment vector from pre to i and i to post.
      Vec3Sub(&Xg_nodes(i), &Xg_pre, &Xg_seg_pre);
      Vec3Sub(&Xg_post, &Xg_nodes(i), &Xg_seg_post);
      // Calculate curvature at current time.
      if (!internal::CalcCurvature(Xg_seg_pre, Xg_seg_post, &curvature_node)) {
        // If the CalcCurvature fails to evaluate the curvature, assume a
        // curvature and curvature rate of zero.
        // This technically could lead to interesting behavior if we ever get a
        // tether perfectly folded back on itself, since it would go from an
        // extremely high curvature and bending moment at one time step to zero
        // in the next time step, but this is very unlikely to ever happen.
        curvature_node = kVec3Zero;
        curvature_rate_node = kVec3Zero;
      } else {
        // Calculate the segment vector from pre to i and i to post at t+t_step.
        Vec3Sub(&Xg_nodes_step[i], &Xg_pre_step, &Xg_seg_pre);
        Vec3Sub(&Xg_post_step, &Xg_nodes_step[i], &Xg_seg_post);
        // Calculate curvature at current time + t_step.
        // Store intermediate values in curvature_rate_node.
        if (!internal::CalcCurvature(Xg_seg_pre, Xg_seg_post,
                                     &curvature_rate_node)) {
          // If CalcCurvature fails at this step, only assume a curvature
          // rate of zero. Just assuming the curvature is zero at this time step
          // could lead to calculating a very high curvature rate and hence
          // spontanously very high damping forces.
          curvature_rate_node = kVec3Zero;
        } else {
          // Calculate the curvature rate of change by finite difference.
          Vec3Sub(&curvature_rate_node, &curvature_node, &curvature_rate_node);
          Vec3Scale(&curvature_rate_node, 1.0 / t_step, &curvature_rate_node);
        }
      }
    }
    curvature_nodes_[i].set_val(curvature_node);
    curvature_rate_nodes_[i].set_val(curvature_rate_node);
  }
}

void Tether::UpdateForceMoments() {
  ForceMomentPos start_force_moment__, end_force_moment__;
  UpdateCurvatures();
  CalcStartForceMomentPos(&start_force_moment__);
  CalcEndForceMomentPos(&end_force_moment__);
  start_force_moment_.set_val(start_force_moment__);
  end_force_moment_.set_val(end_force_moment__);
  UpdateTetherForces();
}

void Tether::AddInternalConnections(ConnectionStore *connections) {
  connections->Add(2, [this](double /*t*/) { UpdateStartAndEndIndices(); });
  connections->Add(3, [this](double /*t*/) { UpdateForceMoments(); });
  connections->Add(4, [this](double /*t*/) { UpdateTetherStartDirG(); });
}

void Tether::CalcGroundForceNode(int32_t i, Vec3 *F_ground) const {
  if (ground_surface_ == nullptr) {
    *F_ground = kVec3Zero;
    return;
  }

  ContactorParams contactor_params =
      tether_sim_params_.ground_contactor_template;
  contactor_params.pos = Xg_nodes(i);
  Contactor contactor(contactor_params, ground_surface_->frame(),
                      *ground_surface_);
  ForceMomentPos fmx;
  contactor.CalcContactForceMomentPos(&fmx);
  *F_ground = fmx.force;
}

// Calculates the forces on all tether nodes. For nodes which are inactive
// or have non-physical dynamics (like next staging node), forces are set
// to zero.
//
// This function is separate from CalcDerivHelper because the forces are derived
// states and must be able to be computed without calling CalcDerivHelper.
void Tether::UpdateTetherForces() {
  // Iterate through each node to calculate all the forces that act on it.
  for (int32_t i = 0; i < num_nodes_; ++i) {
    Vec3 F_long_spring, F_long_damp, F_bend_spring_damp, F_aero, F_teth,
        F_ground;
    if (i > end_ind()               // Handle unused nodes.
        || i <= start_ind() - 2) {  // Handle inactive nodes.
      // Inactive and unused nodes are zeroed, the forces are not used in the
      // state derivative.
      // Derived values for the next staging node are set to zero because the
      // dynamics are not physical and forces are not used for state derivative.
      F_long_spring = kVec3Zero;
      F_long_damp = kVec3Zero;
      F_bend_spring_damp = kVec3Zero;
      F_aero = kVec3Zero;
      F_teth = kVec3Zero;
      F_ground = kVec3Zero;
    } else {  // Derivatives for active and the staging node are physical.
      // Forces for the active and staging nodes are composed of linear-elastic
      // (i.e. spring), damping, aerodynamic, and gravitational components.
      CalcLongSpringForceNode(i, &F_long_spring);
      CalcLongDampingForceNode(i, &F_long_damp);
      CalcBendingSpringDampForceNode(i, &F_bend_spring_damp);
      CalcAeroForceNode(i, &F_aero);
      Vec3Add3(&F_long_spring, &F_long_damp, &F_bend_spring_damp, &F_teth);
      Vec3Add(&F_aero, &F_teth, &F_teth);

      // Add gravity.
      Vec3 F_grav;
      Vec3Scale(&environment_.g_g(), CalcNodeMass(i), &F_grav);
      Vec3Add(&F_grav, &F_teth, &F_teth);

      // Add ground contact force.
      CalcGroundForceNode(i, &F_ground);
      Vec3Add(&F_ground, &F_teth, &F_teth);

      // TODO: Without this saturation it is possible to
      // get infinite tether forces, which can then result in NaNs in
      // the tether state.  We should solve the real issue of poor
      // node initialization.
      SaturateVec3ByScalar(&F_teth, -1e9, 1e9, &F_teth);
    }

    F_aero_nodes_[i].set_val(F_aero);
    F_long_spring_nodes_[i].set_val(F_long_spring);
    F_long_damp_nodes_[i].set_val(F_long_damp);
    F_bend_spring_damp_nodes_[i].set_val(F_bend_spring_damp);
    F_ground_nodes_[i].set_val(F_ground);
    F_teth_nodes_[i].set_val(F_teth);
  }
}

// Calculates the position and velocity derivatives for the tether nodes.
void Tether::CalcDerivHelper(double /*t*/) {
  for (int32_t i = 0; i < num_nodes_; ++i) {
    Vec3 dXg_nodes, dVg_nodes;
    if (i > end_ind()              // Handle unused nodes.
        || i < start_ind() - 2) {  // Handle inactive nodes.
      // The nodal points beyond end_ind() are not used. They are only
      // considered here so that the derivatives actually get set to a real
      // number. Nodes with indices less than start_ind() - 2 are inactive and
      // can be ignored.
      dXg_nodes = kVec3Zero;
      dVg_nodes = kVec3Zero;
    } else if (i == start_ind() - 2) {  // Position the next staging node.
      // Have the next staging node asymptotically approach the midpoint between
      // the current staging node and the tether start location. When the
      // current staging node activates, the next staging node has an
      // equilibrium near this location. Choosing a different initial condition
      // such as Xg_start() results in a stiffer problem and more oscillation.

      // Xg_next_staging and Vg_next_staging contain the position and velocity
      // at the midpoint between the staging node and the tether start location.
      Vec3 Xg_next_staging, Vg_next_staging;
      Vec3LinComb(0.5, &Xg_nodes(start_ind() - 1), 0.5, &Xg_start(),
                  &Xg_next_staging);
      Vec3LinComb(0.5, &Vg_nodes(start_ind() - 1), 0.5, &Vg_start(),
                  &Vg_next_staging);

      // The differences between the midpoint and the next staging node states
      // are then used in a proportional feedback loop to independently drive
      // the position and velocity to the desired values using the time constant
      // time_const.
      double typ_pay_out_speed = 3.0;  // [m/s]
      double time_const = L_seg_0_ / typ_pay_out_speed;
      Vec3 Xg_err, Vg_err;
      Vec3Sub(&Xg_next_staging, &Xg_nodes(i), &Xg_err);
      Vec3Sub(&Vg_next_staging, &Vg_nodes(i), &Vg_err);
      Vec3Scale(&Xg_err, 20.0 / time_const, &dXg_nodes);
      Vec3Scale(&Vg_err, 20.0 / time_const, &dVg_nodes);
    } else {  // Derivatives for active and the staging node.
      // Derivative of velocity is total force / mass.
      Vec3Scale(&F_teth_nodes(i), 1.0 / CalcNodeMass(i), &dVg_nodes);
      dXg_nodes = Vg_nodes(i);
    }

    Xg_nodes_[i].set_deriv(dXg_nodes);
    Vg_nodes_[i].set_deriv(dVg_nodes);
  }
}

// Place the nodes evenly between Xg_start and Xg_end.  For reeled-in
// tethers, the nodes continue linearly outward.
void Tether::InitTetherNodes(const Vec3 &Xg_start__, const Vec3 &Xg_end__,
                             double free_length__) {
  double cum_len;
  int32_t end_ind__ = GetEndIndex(free_length__, &cum_len);
  for (int32_t i = end_ind__; i >= 0; --i) {
    double c = cum_len / free_length__;
    Vec3 Xg_node;
    Vec3LinComb(1.0 - c, &Xg_end__, c, &Xg_start__, &Xg_node);
    Xg_nodes_[i].set_val(Xg_node);
    Vg_nodes_[i].set_val(kVec3Zero);
    cum_len += L_seg_0_;
  }
}

// Returns index of the first active node and the length of the first
// segment.
int32_t Tether::GetStartIndex(double tether_len, double *L_start__) const {
  double cum_len;
  int32_t start_ind__ = GetEndIndex(tether_len, &cum_len);
  while (cum_len + L_seg_0_ < tether_len) {
    start_ind__--;
    cum_len += L_seg_0_;
  }

  *L_start__ = tether_len - cum_len;
  if (*L_start__ < tether_sim_params_.stiff_len_lim) {
    // Do not add an extra segment length when deleting the last node.
    if (cum_len > L_seg_0_ / 2.0 + tether_sim_params_.stiff_len_lim) {
      *L_start__ += L_seg_0_;
    }
    start_ind__++;
  }

  DCHECK_GE(start_ind__, 0)
      << "The tether node start index must be non-negative.";
  return start_ind__;
}

// Returns index of the last active node (always last node now) and
// the length of the last segment.
int32_t Tether::GetEndIndex(double tether_len, double *L_end__) const {
  *L_end__ = fmin(L_seg_0_ / 2.0, tether_len);
  return num_nodes_ - 1;
}

double Tether::CalcNodeMass(int32_t i) const {
  double mass_fac = 1.0;
  if (i == start_ind()) {
    mass_fac = 0.5 + L_start() / L_seg_0_;
  }
  return mass_fac * tether_params_.length * tether_params_.linear_density /
         num_nodes_;
}

// Forces are calculated for the start node so it's easier to pull out GSG
// reaction forces from telemetry.
void Tether::CalcStartForceMomentPos(ForceMomentPos *fmx) const {
  CalcLongSpringForceSeg(Xg_start(), Xg_first_node(), L_first_seg(), false,
                         &fmx->force);
  Vec3 F_long_damp;
  CalcLongDampingForceSeg(Xg_start(), Vg_start(), Xg_first_node(),
                          Vg_first_node(), &F_long_damp);
  // (TODO: Do we need to include bending forces, if these are
  // minuscule relative to tension loads?
  Vec3Add(&F_long_damp, &fmx->force, &fmx->force);
  fmx->moment = kVec3Zero;
  fmx->pos = Xg_start();
}

// TODO: We should technically add aerodynamic forces as well.
// TODO: Determine how best to include effects of point mass at bridle
// knot. In order to properly include it, we can't just use a straight-line from
// last node to bridle pitch axis to determine bridle knot position.
void Tether::CalcEndForceMomentPos(ForceMomentPos *fmx) const {
  Vec3 F_long_spring, F_long_damp, F_bend_spring_damp, Xg_seg_pre;
  CalcLongSpringForceSeg(Xg_end(), Xg_last_node(), L_last_seg(), false,
                         &F_long_spring);
  CalcLongDampingForceSeg(Xg_end(), Vg_end(), Xg_last_node(), Vg_last_node(),
                          &F_long_damp);
  Vec3Sub(&Xg_end(), &Xg_last_node(), &Xg_seg_pre);
  CalcBendingSpringDampForceSeg(
      curvature_nodes(end_ind()), curvature_rate_nodes(end_ind()), kVec3Zero,
      kVec3Zero, Xg_seg_pre, L_last_seg(), &F_bend_spring_damp);
  // There is no post segment, so no shear contribution;
  // but need to use consistant sign convention and 'subtract' the pre segment.
  Vec3Scale(&F_bend_spring_damp, -1.0, &F_bend_spring_damp);
  Vec3Add3(&F_long_spring, &F_long_damp, &F_bend_spring_damp, &fmx->force);
  fmx->moment = kVec3Zero;
  fmx->pos = Xg_end();
}

// Calculates the longitudinal spring force on the i-th interior node of the
// tether. An elastic model found in CalcLongSpringForceSeg(...) is used to get
// the tension in tether segments adjacent to the ith node. The vector sum of
// the two forces are returned in F_long_spring.
//
// The neighboring nodes used for the calculation of tension depend on whether
// the node is active or not. Active nodes will use other active nodes, the
// start node (Xg_start()), or the end node (Xg_end()). The staging node (i.e.
// the node where i == start_ind() - 1) is an inactive node and always uses
// Xg_start()) as the i-1 node. See the diagram below.
//                                                     _- X
//                                                  _-`   |
// Start Node                                    _-`  Xg_nodes(start_ind() + 1)
// Xg_start()                                 _-`     or Xg_end()
//   |                                     _-`
//   X _-------------------------------_ X
//      ``--__             ____----````  |
//            ``' X ---````            First Active Node
//                |                    Xg_nodes(start_ind())
//             Staging Node
//             Xg_nodes(start_ind() - 1)
//
const Vec3 *Tether::CalcLongSpringForceNode(int32_t i,
                                            Vec3 *F_long_spring) const {
  Vec3 F_pre, F_post;
  const Vec3 *Xg_pre, *Xg_post;
  double L_pre, L_post;

  if (i < end_ind()) {
    Xg_post = &Xg_nodes(i + 1);
    L_post = L_seg_0_;
  } else {
    Xg_post = &Xg_end();
    L_post = L_end();
  }

  // Artificial dynamics that improve numerical behavior are allowed for staging
  // nodes only.
  bool allow_artificial_dynamics = false;
  if (i > start_ind()) {  // Interior and end of tether.
    Xg_pre = &Xg_nodes(i - 1);
    L_pre = L_seg_0_;
  } else if (i == start_ind()) {  // First active node.
    Xg_pre = &Xg_start();
    L_pre = L_start();
  } else {  // Staging node.
    // For inactive nodes, the previous node is reset to the start and the
    // unstretched segment lengths are recalculated as seen below.
    Xg_pre = &Xg_start();
    double L_first_seg_0 =
        (start_ind() <= end_ind()) ? L_seg_0_ : 0.5 * L_seg_0_;
    L_pre = fmax(0.5 * tether_sim_params_.stiff_len_lim,
                 L_first_seg() - L_first_seg_0);
    L_post = L_first_seg() - L_pre;

    allow_artificial_dynamics = true;
  }

  CalcLongSpringForceSeg(*Xg_pre, Xg_nodes(i), L_pre, allow_artificial_dynamics,
                         &F_pre);
  CalcLongSpringForceSeg(Xg_nodes(i), *Xg_post, L_post,
                         allow_artificial_dynamics, &F_post);
  Vec3Sub(&F_post, &F_pre, F_long_spring);

  return F_long_spring;
}

// Calculates the longitudinal spring force on a node caused by the extension of
// a segment. The force is assumed to be aligned with the segment. For
// positive strains, the spring force is given by Hooke's law:
//
//   F_long_spring = E * A * strain
//
// For negative strains, it is assumed that the force is zero.
//
// Args:
//   Xg_node_i: Position of the initial node.
//   Xg_node_f: Position of the final node.
//   L_seg_0: Resting length of the segment.
//   allow_artificial_dynamics: Whether artificial dynamics are allowed. This
//       should only be true if the final node is inactive.
//   F_seg: Force along the segment.
//
// Returns:
//   See F_seg.
const Vec3 *Tether::CalcLongSpringForceSeg(const Vec3 &Xg_node_i,
                                           const Vec3 &Xg_node_f,
                                           double L_seg_0,
                                           bool allow_artificial_dynamics,
                                           Vec3 *F_seg) const {
  DCHECK_GT(L_seg_0, 0.0)
      << "The unstretched tether segment length must be positive.";

  Vec3 Rg_seg;
  Vec3Sub(&Xg_node_f, &Xg_node_i, &Rg_seg);
  double L_seg = Vec3Norm(&Rg_seg);

  if (L_seg > L_seg_0) {
    double strain_seg = (L_seg - L_seg_0) / L_seg_0;
    double EA = tether_params_.tensile_stiffness;

    if (allow_artificial_dynamics) {
      // Artificially reduce EA for a short segment to better satisfy the CFL
      // condition.
      //
      // For x'' = -kx, the eigenvalues (wave speeds) of the corresponding
      // first-order system are +-i * sqrt(k). The CFL condition says that we
      // should maintain
      //     sqrt(k) * dt / dx <= 1.
      // Since k is proportional to 1/dx, this implies that for fixed dt, k must
      // be O(dx^3).
      //
      // Trying to sharply satisfy the CFL condition for the simulation's sample
      // time isn't worthwhile, as the ODE solver takes intermediate time steps
      // for other reasons. But this quick and dirty approach noticeably reduces
      // tether oscillations and improves performance.
      if (L_seg_0 < tether_sim_params_.stiff_len_lim) {
        EA *= pow(L_seg_0 / tether_sim_params_.stiff_len_lim, 3.0);
      }
    }

    // The L_seg here normalizes Rg_seg.
    Vec3Scale(&Rg_seg, EA * strain_seg / L_seg, F_seg);
  } else {
    *F_seg = kVec3Zero;
  }

  return F_seg;
}

// Calculates a nodal damping force. For active nodes, linear damping is applied
// tangentially along tether segments. For the staging node, the damping force
// is in the opposite direction of nodal velocity.
const Vec3 *Tether::CalcLongDampingForceNode(int i, Vec3 *F_long_damp) const {
  Vec3 F_pre, F_post;
  const Vec3 *Xg_pre, *Vg_pre, *Xg_post, *Vg_post;

  if (i >= start_ind()) {  // Active nodes.
    if (i > start_ind()) {
      Xg_pre = &Xg_nodes(i - 1);
      Vg_pre = &Vg_nodes(i - 1);
    } else {
      Xg_pre = &Xg_start();
      Vg_pre = &Vg_start();
    }

    if (i < end_ind()) {
      Xg_post = &Xg_nodes(i + 1);
      Vg_post = &Vg_nodes(i + 1);
    } else {
      Xg_post = &Xg_end();
      Vg_post = &Vg_end();
    }

    CalcLongDampingForceSeg(*Xg_pre, *Vg_pre, Xg_nodes(i), Vg_nodes(i), &F_pre);
    CalcLongDampingForceSeg(Xg_nodes(i), Vg_nodes(i), *Xg_post, *Vg_post,
                            &F_post);
    Vec3Sub(&F_post, &F_pre, F_long_damp);
  } else {  // Staging node.
    // By applying gravity and linear-elastic forces to the staging node without
    // affecting adjacent nodes, the problem actually becomes fairly stiff. A
    // relatively small longitudinal perturbation in the staging node can cause
    // one of the segments to go to zero tension and send the staging node
    // flying off in the opposite direction. The node's position remains
    // bounded, but artificial damping is needed to allow it to settle to the
    // equilibrium point.
    // Formulation for numerical damping is similar to other dynamic cable
    // simulators, see the comments in the CalcLongDampingForceSeg function.
    // c_damp is positive. Adding minus sign so that F_long_damp opposes the
    // velocity direction.
    Vec3Scale(&Vg_nodes(i), -c_damp_staged_, F_long_damp);
  }
  return F_long_damp;
}

// Calculates a longitudinal damping force that is proportional to the mean
// longitudinal velocity in a segment. This is an artificial term that models
// internal damping in the tether and external damping from drag, sound, etc.
const Vec3 *Tether::CalcLongDampingForceSeg(const Vec3 &Xg_node_i,
                                            const Vec3 &Vg_node_i,
                                            const Vec3 &Xg_node_f,
                                            const Vec3 &Vg_node_f,
                                            Vec3 *F_seg) const {
  Vec3 Xg_seg, dVg_seg, Xg_seg_hat;
  Vec3Sub(&Xg_node_f, &Xg_node_i, &Xg_seg);

  double longitudinal_vel =
      Vec3Dot(&Xg_seg, Vec3Sub(&Vg_node_f, &Vg_node_i, &dVg_seg)) /
      Vec3NormBound(&Xg_seg, 1e-9);
  // Formulation for damping is similar to other industry dynamic cable
  // simulators, (ie, Orcaflex:
  // https://www.orcina.com/SoftwareProducts/OrcaFlex/Documentation/Help/
  // Theory → Line Theory → Calculation Stage 1 Tension Forces)
  // the full term for the longitudinal damping force in a segment is:
  // F_long_damp = damping_ratio*sqrt(2*EA*linear_density)*dl/dt ;
  // dl/dt is the rate of change of the length of the segment.
  Vec3Scale(&Xg_seg, 1.0 / Vec3NormBound(&Xg_seg, 1e-9), &Xg_seg_hat);
  Vec3Scale(&Xg_seg_hat, c_damp_active_ * longitudinal_vel, F_seg);
  return F_seg;
}

// Calculates the nodal forces from bending stiffness and damping for the
// i-th interior node of the tether. An elastic and viscous damping model
// found in CalcBendingSpringDampForceSeg(...) is used to get the lateral shear
// force in tether segments adjacent to the ith node due to the curvature and
// rate of curvature of the nodes of either end of the segment. The vector sum
// of the two lateral forces are returned in F_bend_spring_damp.
//
// The neighboring nodes used for the bending calculations depend on whether
// the node is active or not. See comment and diagram for
// CalcLongSpringForceNode.
// For the staging node, no bending forces are applied.
const Vec3 *Tether::CalcBendingSpringDampForceNode(
    int i, Vec3 *F_bend_spring_damp) const {
  Vec3 F_pre, F_post, Xg_seg_pre, Xg_seg_post;
  const Vec3 *Xg_pre, *Xg_post, *curvature_node_pre, *curvature_rate_node_pre,
      *curvature_node_post, *curvature_rate_node_post;
  double L_pre, L_post;    // Initial, unstretched lengths
  if (i >= start_ind()) {  // Active nodes.
    // Determine which nodes are adjacent to the node of interest and set the
    // needed variables.
    if (i > start_ind()) {
      Xg_pre = &Xg_nodes(i - 1);
      L_pre = L_seg_0_;
      curvature_node_pre = &curvature_nodes(i - 1);
      curvature_rate_node_pre = &curvature_rate_nodes(i - 1);
    } else {
      Xg_pre = &Xg_start();
      L_pre = L_start();
      curvature_node_pre = &kVec3Zero;
      curvature_rate_node_pre = &kVec3Zero;
    }
    if (i < end_ind()) {
      Xg_post = &Xg_nodes(i + 1);
      L_post = L_seg_0_;
      curvature_node_post = &curvature_nodes(i + 1);
      curvature_rate_node_post = &curvature_rate_nodes(i + 1);
    } else {
      Xg_post = &Xg_end();
      L_post = L_end();
      curvature_node_post = &kVec3Zero;
      curvature_rate_node_post = &kVec3Zero;
    }

    // Calculate adjacent segment vectors
    Vec3Sub(&Xg_nodes(i), Xg_pre, &Xg_seg_pre);
    Vec3Sub(Xg_post, &Xg_nodes(i), &Xg_seg_post);

    CalcBendingSpringDampForceSeg(*curvature_node_pre, *curvature_rate_node_pre,
                                  curvature_nodes(i), curvature_rate_nodes(i),
                                  Xg_seg_pre, L_pre, &F_pre);
    CalcBendingSpringDampForceSeg(
        curvature_nodes(i), curvature_rate_nodes(i), *curvature_node_post,
        *curvature_rate_node_post, Xg_seg_post, L_post, &F_post);
    Vec3Sub(&F_post, &F_pre, F_bend_spring_damp);  // Handles sign convention.
  } else {                                         // Staging node.
    // We are not including bending moment in staging nodes, for now.
    // TODO: Explore if adding bending effects, especially damping, to
    // staging nodes might have a benefit, such as faster settling times.
    *F_bend_spring_damp = kVec3Zero;
  }
  return F_bend_spring_damp;
}

// Calculates a lateral shear force that acts on the nodes on either side of a
// segment due to the rate of change in bending moment along its length. The
// bending moments at each node are calculated from the local curvature and the
// damping moments are calculated from the rate of change of curvature.
const Vec3 *Tether::CalcBendingSpringDampForceSeg(
    const Vec3 &curvature_i, const Vec3 &curvature_rate_i,
    const Vec3 &curvature_f, const Vec3 &curvature_rate_f, const Vec3 &Xg_seg,
    const double L_seg_0, Vec3 *F_seg) const {
  DCHECK_GT(L_seg_0, 0.0)
      << "The unstretched tether segment length must be positive.";
  Vec3 curvature_hat, bend_moment_i, bend_moment_f, bend_moment_damping,
      bend_moment_diff, Xg_seg_hat;
  double curvature_norm;

  // Formulation for bending moments from stiffness and damping is similar to
  // other industry dynamic cable simulators, (ie, Orcaflex:
  // https://www.orcina.com/SoftwareProducts/OrcaFlex/Documentation/Help/
  // Theory → Line Theory → Calculation Stage 2 Bend Moments
  // & Calculation Stage 3 Shear Forces)
  // The 'damping coefficient' is precalculated in the tether constructor to
  // save the square root calculation each time, but L_seg_0 is different for
  // end segments so the full d_damp term needs to be calculated here.
  double d_damp_active = L_seg_0 * L_seg_0 * d_damp_coeff_active_;

  // Using the vectors directly should work and make physical sense, but the
  // OrcaFlex formulation uses the magnitudes of the stiffness and damping terms
  // and the direction from the curvature vector, so sticking with that
  // formulation.

  // Calculate total bending moment vector for initial node.
  curvature_norm = Vec3NormBound(&curvature_i, 1e-9);
  Vec3Scale(&curvature_i, 1.0 / curvature_norm, &curvature_hat);
  Vec3Scale(&curvature_i, tether_params_.bending_stiffness, &bend_moment_i);
  Vec3Scale(&curvature_hat,
            d_damp_active * Vec3NormBound(&curvature_rate_i, 1e-9),
            &bend_moment_damping);
  Vec3Add(&bend_moment_i, &bend_moment_damping, &bend_moment_i);

  // Calculate total bending moment vector for final node.
  curvature_norm = Vec3NormBound(&curvature_f, 1e-9);
  Vec3Scale(&curvature_f, 1.0 / curvature_norm, &curvature_hat);
  Vec3Scale(&curvature_f, tether_params_.bending_stiffness, &bend_moment_f);
  Vec3Scale(&curvature_hat,
            d_damp_active * Vec3NormBound(&curvature_rate_f, 1e-9),
            &bend_moment_damping);
  Vec3Add(&bend_moment_f, &bend_moment_damping, &bend_moment_f);

  // Calculate magnitude and unit vector of segment
  double Xg_seg_norm = Vec3NormBound(&Xg_seg, 1e-9);
  Vec3Scale(&Xg_seg, 1.0 / Xg_seg_norm, &Xg_seg_hat);

  // Calculate internal reaction shear needed to generate this difference in
  // moments.
  Vec3Sub(&bend_moment_f, &bend_moment_i, &bend_moment_diff);
  Vec3Scale(&bend_moment_diff, 1.0 / Xg_seg_norm, &bend_moment_diff);
  Vec3Cross(&Xg_seg_hat, &bend_moment_diff, F_seg);
  return F_seg;
}

const Vec3 *Tether::CalcAeroForceNode(int32_t i, Vec3 *F_aero) const {
  Vec3 Xg_seg_cent_i, Xg_seg_cent_f;
  Vec3 wind_g;

  environment_.CalcWind(Xg_nodes(i), &wind_g);

  if (i > start_ind()) {
    Vec3LinComb(0.5, &Xg_nodes(i - 1), 0.5, &Xg_nodes(i), &Xg_seg_cent_i);
  } else {
    Xg_seg_cent_i = Xg_start();
  }

  if (i < end_ind()) {
    Vec3LinComb(0.5, &Xg_nodes(i), 0.5, &Xg_nodes(i + 1), &Xg_seg_cent_f);
  } else {
    Xg_seg_cent_f = Xg_end();
  }

  CalcAeroForceSeg(Xg_seg_cent_i, Xg_seg_cent_f, Vg_nodes(i), wind_g, F_aero);

  return F_aero;
}

const Vec3 *Tether::CalcAeroForceSeg(const Vec3 &Xg_seg_cent_i,
                                     const Vec3 &Xg_seg_cent_f,
                                     const Vec3 &Vg_node, const Vec3 &wind_g,
                                     Vec3 *F_seg) const {
  Vec3 Vg_app_node, Xg_seg_cent, Xg_seg_cent_hat, Vg_app_normal;

  Vec3Sub(&Xg_seg_cent_f, &Xg_seg_cent_i, &Xg_seg_cent);
  double Xg_seg_cent_norm = Vec3NormBound(&Xg_seg_cent, 1e-9);
  Vec3Scale(&Xg_seg_cent, 1.0 / Xg_seg_cent_norm, &Xg_seg_cent_hat);

  Vec3Sub(&Vg_node, &wind_g, &Vg_app_node);
  Vec3Sub(&Vg_app_node,
          Vec3Scale(&Xg_seg_cent_hat, Vec3Dot(&Vg_app_node, &Xg_seg_cent_hat),
                    &Xg_seg_cent_hat),
          &Vg_app_normal);
  Vec3Scale(&Vg_app_normal, -0.5 * environment_.air_density() *
                                tether_params_.section_drag_coeff *
                                tether_params_.outer_diameter *
                                Xg_seg_cent_norm * Vec3Norm(&Vg_app_normal),
            F_seg);

  return F_seg;
}
