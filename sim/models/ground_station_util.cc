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

#include "sim/models/ground_station_util.h"

#include <glog/logging.h>
#include <math.h>

#include "common/c_math/vec2.h"
#include "control/system_params.h"
#include "sim/physics/contactor.h"

// Calculates whether the contactor has contacted the perch panels.
// The perch panels are modeled as the union of two cylinders with
// specified centers and radii.  The cylinders are pitched about the
// perch y-axis by a specified pitch angle.  The top of the perch
// panels is simply a flat xy-plane in the perch coordinate system.
//
// Args:
//   contactor_pos_p: Position [m] of the contactor in perch
//       coordinates.  This may be within the surface if there is
//       contact.
//   perch_frame: ReferenceFrame of the perch.
//   panel_frame: ReferenceFrame of the perch panel.
//   panel_sim_params: Parameters that describe the geometry of the
//       perch panel.
//   collision_pos_p: Returns the position [m] of the collision on the
//       panel surface in perch coordinates.
//
// Returns:
//   True if the contactor contacts the perch panel.
bool ContactPerchPanel(const Vec3 &contactor_pos_panel,
                       const ReferenceFrame &perch_frame,
                       const ReferenceFrame &panel_frame,
                       const PanelSimParams &panel_sim_params,
                       Vec3 *collision_pos_panel) {
  DCHECK(collision_pos_panel != nullptr);

  const SinglePanelSimParams &port_params = panel_sim_params.port;
  const SinglePanelSimParams &starboard_params = panel_sim_params.starboard;

  // Check that z-extents are properly ordered, and that the top of the port
  // panel is at least as high as the top of the starboard panel.
  DCHECK_LT(port_params.z_extents_p[0], port_params.z_extents_p[1]);
  DCHECK_LT(starboard_params.z_extents_p[0], starboard_params.z_extents_p[1]);
  DCHECK_LE(port_params.z_extents_p[0], port_params.z_extents_p[1]);

  Vec3 contactor_pos_p;
  panel_frame.TransformTo(perch_frame, ReferenceFrame::kPosition,
                          contactor_pos_panel, &contactor_pos_p);

  // Too high for contact; return early.
  //
  // TODO: This logic assumes that the top of the port panel is
  // higher. Revisit for GS02.
  if (contactor_pos_p.z < port_params.z_extents_p[0]) return false;

  // Determine the nearest point on the panels in the panel xy-plane, returning
  // early if there is no collision.
  Vec2 xy_contactor_panel = {contactor_pos_panel.x, contactor_pos_panel.y};
  bool collides_in_xy;
  Vec2 xy_collision_panel;
  if (contactor_pos_p.z < starboard_params.z_extents_p[0]) {
    collides_in_xy =
        ContactCircle(xy_contactor_panel, port_params.radius,
                      port_params.center_panel, &xy_collision_panel);
  } else {
    collides_in_xy =
        ContactTwoCircles(xy_contactor_panel, port_params.radius,
                          port_params.center_panel, starboard_params.radius,
                          starboard_params.center_panel, &xy_collision_panel);
  }
  if (!collides_in_xy) return false;

  // Find the collision with the top of the panels.
  Vec3 top_collision_pos_panel;
  {
    Vec3 top_collision_pos_p = {contactor_pos_p.x, contactor_pos_p.y, 0.0};
    Vec2 port_offset;
    Vec2 starboard_offset;
    Vec2Sub(&xy_collision_panel, &port_params.center_panel, &port_offset);
    Vec2Sub(&xy_collision_panel, &starboard_params.center_panel,
            &starboard_offset);
    if (fabs(Vec2Norm(&port_offset) - port_params.radius) <
        fabs(Vec2Norm(&starboard_offset)) - starboard_params.radius) {
      top_collision_pos_p.z = port_params.z_extents_p[0];
    } else {
      top_collision_pos_p.z = starboard_params.z_extents_p[0];
    }
    panel_frame.TransformFrom(perch_frame, ReferenceFrame::kPosition,
                              top_collision_pos_p, &top_collision_pos_panel);
  }

  Vec3 cylinder_collision_pos_panel = {
      xy_collision_panel.x, xy_collision_panel.y, contactor_pos_panel.z};

  Vec3 tmp;
  const double dist_to_top_collision =
      Vec3Norm(Vec3Sub(&contactor_pos_panel, &top_collision_pos_panel, &tmp));
  const double dist_to_cylinder_collision = Vec3Norm(
      Vec3Sub(&contactor_pos_panel, &cylinder_collision_pos_panel, &tmp));
  *collision_pos_panel = (dist_to_top_collision < dist_to_cylinder_collision)
                             ? top_collision_pos_panel
                             : cylinder_collision_pos_panel;
  return true;
}

RacetrackTetherIntegrator::RacetrackTetherIntegrator(
    const Gs02Params &params, const Gs02SimParams &sim_params)
    : a_(0.0),
      b_(0.0),
      c_(0.0),
      theta_high_(
          GetSystemParams()->ground_station.gs02.drum_angles.racetrack_high),
      tau_max_(
          GetSystemParams()->ground_station.gs02.drum_angles.racetrack_high -
          GetSystemParams()->ground_station.gs02.drum_angles.racetrack_low) {
  const double r0 = params.gsg_pos_drum.z;

  const double dr_dtau =
      (params.drum_radius - params.gsg_pos_drum.z) / tau_max_;
  const double dx_dtau =
      (sim_params.wrap_start_posx_drum - params.gsg_pos_drum.x) / tau_max_;

  a_ = Square(dr_dtau);
  b_ = 2.0 * r0 * dr_dtau;
  c_ = Square(r0) + Square(dx_dtau) + Square(dr_dtau);
}

double RacetrackTetherIntegrator::Antiderivative(double tau) const {
  return ((tau / 2.0 + b_ / (4.0 * a_)) *
              Sqrt(a_ * Square(tau) + b_ * tau + c_) +
          (c_ / (2.0 * Sqrt(a_)) - Square(b_) / (8.0 * pow(a_, 1.5))) *
              asinh((2.0 * a_ * tau + b_) / sqrt(4.0 * a_ * c_ - Square(b_))));
}

double RacetrackTetherIntegrator::WrappedLength(double drum_angle) const {
  double tau = Saturate(theta_high_ - drum_angle, 0.0, tau_max_);
  return Antiderivative(tau) - Antiderivative(0.0);
}
