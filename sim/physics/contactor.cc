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

#include "sim/physics/contactor.h"

#include <glog/logging.h>

#include "common/c_math/force_moment.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "sim/math/util.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"

// TODO: The output force is resolved in the parent frame of the
// contactor. That seems like a scope violation to me.
void Contactor::CalcContactForceMomentPos(ForceMomentPos *fmx) const {
  Vec3 pos_surf;
  frame_.TransformOriginTo(surface_.frame(), ReferenceFrame::kPosition,
                           &pos_surf);

  Vec3 collision_pos_surf;
  if (surface_.CheckCollision(pos_surf, &collision_pos_surf)) {
    // Calculate the velocity of the contactor relative to the surface
    // origin in surface coordinates.
    Vec3 vel_surf;
    frame_.TransformOriginTo(surface_.frame(), ReferenceFrame::kVelocity,
                             &vel_surf);

    // Calculate the velocity of the surface at the collision point
    // relative to the surface origin in surface coordinates.
    ReferenceFrame collision_frame(surface_.frame(), collision_pos_surf);
    Vec3 collision_vel_surf;
    surface_.frame().TransformOriginFrom(
        collision_frame, ReferenceFrame::kVelocity, &collision_vel_surf);

    Vec3 force_surf;
    CalcContactForce(pos_surf, vel_surf, collision_pos_surf, collision_vel_surf,
                     &force_surf);
    surface_.frame().RotateTo(frame_.parent(), force_surf, &fmx->force);
  } else {
    fmx->force = kVec3Zero;
  }

  fmx->moment = kVec3Zero;
  fmx->pos = contactor_params_.pos;
}

void Contactor::CalcContactForce(const Vec3 &contactor_pos,
                                 const Vec3 &contactor_vel,
                                 const Vec3 &collision_pos,
                                 const Vec3 &collision_vel,
                                 Vec3 *contact_force) const {
  Vec3 contact_offset, contact_vel, unit_contact_offset;
  // contact_offset points from surface into object.
  Vec3Sub(&contactor_pos, &collision_pos, &contact_offset);
  Vec3Scale(&contact_offset, 1.0 / Vec3NormBound(&contact_offset, 1e-9),
            &unit_contact_offset);
  Vec3Sub(&contactor_vel, &collision_vel, &contact_vel);

  Vec3 contact_vel_perp, contact_vel_para;
  Vec3Scale(&unit_contact_offset, Vec3Dot(&contact_vel, &unit_contact_offset),
            &contact_vel_perp);
  Vec3Sub(&contact_vel, &contact_vel_perp, &contact_vel_para);

  // Calculate spring force.
  Vec3 spring_force;
  Vec3Scale(&contact_offset, -contactor_params_.spring_const, &spring_force);

  // Only damp during the approach.
  Vec3 damping_force = kVec3Zero;
  if (Vec3Dot(&contact_vel_perp, &contact_offset) > 0.0) {
    Vec3Scale(&contact_vel_perp, -contactor_params_.damping_coeff,
              &damping_force);
  }

  // Apply Coulomb friction above 1 m/s, viscous below.
  Vec3 friction_force;
  if (Vec3Norm(&contact_vel_para) > 1.0) {
    Vec3Normalize(&contact_vel_para, &contact_vel_para);
  }
  Vec3Scale(&contact_vel_para,
            -contactor_params_.friction_coeff * Vec3Norm(&spring_force),
            &friction_force);
  Vec3Add3(&spring_force, &damping_force, &friction_force, contact_force);
}

bool ContactCircle(const Vec2 &pos, double radius, const Vec2 &center,
                   Vec2 *collision_pos) {
  DCHECK_GT(radius, 0.0) << "The radius must be positive.";

  Vec2 offset;
  Vec2Sub(&pos, &center, &offset);
  const double offset_norm = Vec2Norm(&offset);

  constexpr double kMinOffsetFraction = 0.1;
  if (offset_norm < kMinOffsetFraction * radius) {
    LOG(FATAL) << "The contact point is too close to the center of the circle.";
  } else if (offset_norm >= radius) {
    return false;
  }

  Vec2LinComb(1.0, &center, radius / offset_norm, &offset, collision_pos);
  return true;
}

bool ContactTwoCircles(const Vec2 &pos, double radius1, const Vec2 &center1,
                       double radius2, const Vec2 &center2,
                       Vec2 *collision_pos) {
  DCHECK(radius1 > 0.0 && radius2 > 0.0) << "The radii must be positive.";

  // Calculate individual collisions.
  Vec2 collision_pos1, collision_pos2;
  bool collides_with_1 = ContactCircle(pos, radius1, center1, &collision_pos1);
  bool collides_with_2 = ContactCircle(pos, radius2, center2, &collision_pos2);

  // If the contactor collides with both circles, we return the
  // intersection point of the two circles.  Otherwise, we return the
  // collision point of the circle it collided with.
  if (collides_with_1 && collides_with_2) {
    Vec2 circle_offset;
    Vec2Sub(&center2, &center1, &circle_offset);
    double x, y;
    bool circles_intersect =
        IntersectCircles(Vec2Norm(&circle_offset), radius1, radius2, &x, &y);
    DCHECK(circles_intersect) << "Circles do not intersect.";

    // Convert the coordinate system used in the IntersectCircles
    // function, where x lies along the line from the first to second,
    // to this function's coordinate system.  As the IntersectCircles
    // function always returns a positive y, we determine the sign of
    // y here by projecting the contactor position onto the y-axis.
    Vec2 circle_offset_dir;
    Vec2Normalize(&circle_offset, &circle_offset_dir);
    Vec2 circle_offset_perp_dir = {-circle_offset_dir.y, circle_offset_dir.x};
    Vec2 contactor_offset1;
    Vec2Sub(&pos, &center1, &contactor_offset1);
    double sign_y = Sign(Vec2Dot(&contactor_offset1, &circle_offset_perp_dir));

    Vec2LinComb3(1.0, &center1, x, &circle_offset_dir, sign_y * y,
                 &circle_offset_perp_dir, collision_pos);
  } else if (collides_with_1) {
    *collision_pos = collision_pos1;
  } else {
    *collision_pos = collision_pos2;
  }

  return collides_with_1 || collides_with_2;
}

bool ContactBox(const Vec3 &pos, const Vec3 &center, const Vec3 &dimensions,
                Vec3 *collision_pos) {
  DCHECK(dimensions.x > 0.0 && dimensions.y > 0.0 && dimensions.z > 0.0)
      << "The dimensions must be positive.";

  // Calculate the distances from the planes that form the boundary of
  // the box.  The sign of the distance is defined to be positive when
  // the point is on the same side of the plane that is the outside
  // side of the box.
  double distances_from_boundary[] = {pos.x - (center.x + dimensions.x / 2.0),
                                      -pos.x + (center.x - dimensions.x / 2.0),
                                      pos.y - (center.y + dimensions.y / 2.0),
                                      -pos.y + (center.y - dimensions.y / 2.0),
                                      pos.z - (center.z + dimensions.z / 2.0),
                                      -pos.z + (center.z - dimensions.z / 2.0)};
  double max_distance_from_boundary = distances_from_boundary[0];
  int32_t side_index = 0;
  for (int32_t i = 0; i < ARRAYSIZE(distances_from_boundary); ++i) {
    if (distances_from_boundary[i] > 0.0) return false;

    if (distances_from_boundary[i] >= max_distance_from_boundary) {
      max_distance_from_boundary = distances_from_boundary[i];
      side_index = i;
    }
  }

  *collision_pos = pos;
  switch (side_index) {
    case 0:
      collision_pos->x = center.x + dimensions.x / 2.0;
      break;
    case 1:
      collision_pos->x = center.x - dimensions.x / 2.0;
      break;
    case 2:
      collision_pos->y = center.y + dimensions.y / 2.0;
      break;
    case 3:
      collision_pos->y = center.y - dimensions.y / 2.0;
      break;
    case 4:
      collision_pos->z = center.z + dimensions.z / 2.0;
      break;
    case 5:
      collision_pos->z = center.z - dimensions.z / 2.0;
      break;
    default:
      LOG(FATAL) << "Unexpected side_index.";
  }

  return true;
}
