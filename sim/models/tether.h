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

// The Tether class models the tether as a series of connected lumped
// masses and springs.  The number of lumped masses (i.e. nodes) is
// set in teth_sim_params_.num_nodes, and each node has mass
// m_teth/num_nodes.  The nodes are each (nominally, under no tension)
// separated by L_seg_0_ = L_teth/num_nodes; however the endpoints,
// start and end, are separated from the first and last node by
// L_seg_0_/2.
//
//              node_0      node_1      node_2      node_{N-1}
//
// start  x--/\/--o---/\/\/---o---/\/\/---o--  ...  --o--/\/--x  end
//
//        L_seg_0_/2  L_seg_0_    L_seg_0_            L_seg_0_/2
//
// The position and velocity of the endpoints are determined by
// "constraint" functions that are passed in.  There are three types
// of forces that act on each node: spring tension (from the segment
// before and after the node), aerodynamic, and gravitational.  The
// spring force is determined by the distance between the nodes
// compared to the nominal distance.  The aerodynamic force is modeled
// as if there were a cylinder stretching between two mid segment
// points.

#ifndef SIM_MODELS_TETHER_H_
#define SIM_MODELS_TETHER_H_

#include <stdint.h>

#include <functional>
#include <vector>

#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/system_types.h"
#include "sim/models/environment.h"
#include "sim/models/model.h"
#include "sim/models/rigid_bodies/wing.h"
#include "sim/physics/contactor.h"
#include "sim/sim_types.h"
#include "sim/state.h"

class Tether : public Model {
  friend class TetherTest;

 public:
  typedef std::function<void(const Vec3 &, const Vec3 &, Vec3 *, Vec3 *)>
      CalcBridlePointFunc;

  Tether(const Environment &environment, const TetherParams &tether_params,
         const TetherSimParams &tether_sim_params);
  ~Tether() {}

  void Init(CalcBridlePointFunc calc_bridle_point, const Vec3 &Xg_start__,
            const Vec3 &Vg_start__, double free_length__);
  void Publish() const override;

  void set_free_length(double val) { free_length_.set_val(val); }
  void set_Xg_start(const Vec3 &val) { Xg_start_.set_val(val); }
  void set_Vg_start(const Vec3 &val) { Vg_start_.set_val(val); }
  void set_Xg_end(const Vec3 &val) { Xg_end_.set_val(val); }
  void set_Vg_end(const Vec3 &val) { Vg_end_.set_val(val); }
  void set_start_ind(int val) { start_ind_.set_val(val); }

  void set_ground_surface(const ContactSurface *ground) {
    ground_surface_ = ground;
  }

  double free_length() const { return free_length_.val(); }
  const Vec3 &tether_start_dir_g() const { return tether_start_dir_g_.val(); }

  // The "first node" here refers to the first active node on the
  // tether (or the start position of the tether if there are no
  // active nodes).
  const Vec3 &Xg_first_node() const {
    return (start_ind() > end_ind()) ? Xg_end() : Xg_nodes(start_ind());
  }
  const Vec3 &Vg_first_node() const {
    return (start_ind() > end_ind()) ? Vg_end() : Vg_nodes(start_ind());
  }

  // The "last node" here refers to the last active node on the tether
  // (or the end position of the tether if there are no active nodes).
  const Vec3 &Xg_last_node() const {
    return (start_ind() > end_ind()) ? Xg_start() : Xg_nodes(end_ind());
  }
  const Vec3 &Vg_last_node() const {
    return (start_ind() > end_ind()) ? Vg_start() : Vg_nodes(end_ind());
  }

  const ForceMomentPos &start_force_moment() const {
    return start_force_moment_.val();
  }
  const Vec3 &start_force() const { return start_force_moment_.val().force; }
  const ForceMomentPos &end_force_moment() const {
    return end_force_moment_.val();
  }
  const Vec3 &end_force() const { return end_force_moment_.val().force; }

 protected:
  void AddInternalConnections(ConnectionStore *connections) override;

 private:
  void CalcDerivHelper(double t) override;
  void DiscreteStepHelper(double /*t*/) override {}

  void UpdateStartAndEndIndices();
  void UpdateCurvatures();
  void UpdateForceMoments();
  void UpdateTetherStartDirG();

  // Compute the longitudinal spring, longitudinal damping, aerodynamic,
  // gravity, bending, and total forces on each tether node. Gravity is the only
  // one not stored because it is the only one not part of telemetry.
  void UpdateTetherForces();

  void InitTetherNodes(const Vec3 &Xg_start, const Vec3 &Xg_end,
                       double free_length);

  int32_t GetStartIndex(double tether_len, double *L_start) const;
  int32_t GetEndIndex(double tether_len, double *L_end) const;

  double CalcNodeMass(int32_t i) const;

  void CalcStartForceMomentPos(ForceMomentPos *fmx) const;
  void CalcEndForceMomentPos(ForceMomentPos *fmx) const;

  const Vec3 *CalcLongSpringForceNode(int32_t i, Vec3 *F_long_spring) const;
  const Vec3 *CalcLongSpringForceSeg(const Vec3 &Xg_node_i,
                                     const Vec3 &Xg_node_f, double L_seg_0,
                                     bool allow_artificial_dynamics,
                                     Vec3 *F_seg) const;

  const Vec3 *CalcLongDampingForceNode(int32_t i, Vec3 *F_long_damp) const;
  const Vec3 *CalcLongDampingForceSeg(const Vec3 &Xg_node_i,
                                      const Vec3 &Vg_node_i,
                                      const Vec3 &Xg_node_f,
                                      const Vec3 &Vg_node_f, Vec3 *F_seg) const;

  const Vec3 *CalcBendingSpringDampForceNode(int32_t i,
                                             Vec3 *F_bend_spring_damp) const;
  const Vec3 *CalcBendingSpringDampForceSeg(
      const Vec3 &curvature_i, const Vec3 &curvature_rate_i,
      const Vec3 &curvature_f, const Vec3 &curvature_rate_f, const Vec3 &Xg_seg,
      const double L_seg_0, Vec3 *F_seg) const;

  const Vec3 *CalcAeroForceNode(int32_t i, Vec3 *F_aero) const;
  const Vec3 *CalcAeroForceSeg(const Vec3 &Xg_seg_cent_i,
                               const Vec3 &Xg_seg_cent_f, const Vec3 &Vg_node,
                               const Vec3 &wind_g, Vec3 *F_seg) const;

  void CalcGroundForceNode(int32_t i, Vec3 *F_ground) const;

  const Vec3 &Xg_nodes(int32_t i) const { return Xg_nodes_[i].val(); }
  const Vec3 &Vg_nodes(int32_t i) const { return Vg_nodes_[i].val(); }
  int32_t start_ind() const { return start_ind_.val(); }
  int32_t end_ind() const { return end_ind_.val(); }
  double L_start() const { return L_start_.val(); }
  double L_end() const { return L_end_.val(); }
  double L_first_seg() const {
    return (start_ind() > end_ind()) ? L_start() + L_end() : L_start();
  }
  double L_last_seg() const {
    return (start_ind() > end_ind()) ? L_start() + L_end() : L_end();
  }

  const Vec3 &Xg_start() const { return Xg_start_.val(); }
  const Vec3 &Vg_start() const { return Vg_start_.val(); }
  const Vec3 &Xg_end() const { return Xg_end_.val(); }
  const Vec3 &Vg_end() const { return Vg_end_.val(); }
  const Vec3 &F_aero_nodes(int32_t i) const { return F_aero_nodes_[i].val(); }
  const Vec3 &F_long_spring_nodes(int32_t i) const {
    return F_long_spring_nodes_[i].val();
  }
  const Vec3 &F_long_damp_nodes(int32_t i) const {
    return F_long_damp_nodes_[i].val();
  }
  const Vec3 &F_bend_spring_damp_nodes(int32_t i) const {
    return F_bend_spring_damp_nodes_[i].val();
  }
  const Vec3 &F_ground_nodes(int32_t i) const {
    return F_ground_nodes_[i].val();
  }
  const Vec3 &F_teth_nodes(int32_t i) const { return F_teth_nodes_[i].val(); }
  const Vec3 &curvature_nodes(int32_t i) const {
    return curvature_nodes_[i].val();
  }
  const Vec3 &curvature_rate_nodes(int32_t i) const {
    return curvature_rate_nodes_[i].val();
  }

  // Tether parameters.
  const TetherParams &tether_params_;
  const TetherSimParams &tether_sim_params_;
  const int32_t num_nodes_;
  // Unstretched length [m] of an interior tether segment.
  const double L_seg_0_;
  // Longitudinal damping coefficients [kg/s].
  const double c_damp_staged_;
  const double c_damp_active_;
  // Bending damping coefficient [kg*m/s].
  const double d_damp_coeff_active_;
  const Environment &environment_;

  // Ground contact surface. If this is non-null, then ground contact forces
  // will be applied.
  const ContactSurface *ground_surface_;

  // Input states.

  // Position [m] and velocity [m/s] in ground coordinates of the
  // point where the tether attaches to the levelwind or GSG.
  State<Vec3> Xg_start_, Vg_start_;
  // Position [m] and velocity [m/s] in ground coordinates of the
  // point where the tether attaches to the bridles.
  State<Vec3> Xg_end_, Vg_end_;
  // Length [m] of the currently unspooled tether if it had no tension
  // on it.
  State<double> free_length_;

  // Continuous state.

  // Position [m] of the tether nodes in ground coordinates.
  std::vector<ContinuousState<Vec3>> Xg_nodes_;
  // Velocity [m/s] of the tether nodes in ground coordinates.
  std::vector<ContinuousState<Vec3>> Vg_nodes_;

  // Derived values.

  // Index of the first active node (after the levelwind) and last
  // active node (always the last node now).
  State<int32_t> start_ind_, end_ind_;
  // Length [m] of first segment and last segment.
  State<double> L_start_, L_end_;
  // Departure direction of the tether at the GSG, in ground coordinates.
  State<Vec3> tether_start_dir_g_;
  // Force [N], moment [N-m], position [m] of the tether at Xg_start_
  // and Xg_end_.
  State<ForceMomentPos> start_force_moment_, end_force_moment_;
  // Aerodynamic force [N] on each tether node.
  std::vector<State<Vec3>> F_aero_nodes_;
  // Longitudinal spring force [N] on each tether node.
  std::vector<State<Vec3>> F_long_spring_nodes_;
  // Longitudinal viscous damping force [N] on each tether node.
  std::vector<State<Vec3>> F_long_damp_nodes_;
  // Lateral shear force [N] from bending spring and viscous damping
  // moments on each tether node.
  std::vector<State<Vec3>> F_bend_spring_damp_nodes_;
  // Ground contact force [N] on each tether node.
  std::vector<State<Vec3>> F_ground_nodes_;
  // Sum of forces [N] on each tether node.
  std::vector<State<Vec3>> F_teth_nodes_;
  // Curvature [1/m] of tether at nodes, in ground coordinates.
  std::vector<State<Vec3>> curvature_nodes_;
  // Curvature rate of change [1/m/s] of tether at nodes, in ground coordinates.
  std::vector<State<Vec3>> curvature_rate_nodes_;

  DISALLOW_COPY_AND_ASSIGN(Tether);
};

#endif  // SIM_MODELS_TETHER_H_
