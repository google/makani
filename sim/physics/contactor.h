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

#ifndef SIM_PHYSICS_CONTACTOR_H_
#define SIM_PHYSICS_CONTACTOR_H_

#include <functional>

#include "common/c_math/force_moment.h"
#include "common/c_math/vec3.h"
#include "sim/physics/reference_frame.h"
#include "sim/sim_types.h"

// A ContactSurface represents a surface that can push against a
// Contactor.  This surface has a rigid body frame associated with it,
// so it may be translating or rotating.  The shape of the surface is
// defined by the collision_func_, which returns true if a point is
// within the surface and false otherwise.  Also, the collision
// function is reponsible for returning the closest point on a surface
// to the Contactor point.  Any physical characteristics of the
// Contactor/ContactSurface interaction (e.g. friction coefficient,
// spring constant, etc...) are included in the Contactor.
class ContactSurface {
 public:
  ContactSurface(
      const ReferenceFrame &frame__,
      const std::function<bool(const Vec3 &, Vec3 *)> &collision_func)
      : frame_(frame__), collision_func_(collision_func) {}
  ContactSurface(const ContactSurface &) = default;
  ~ContactSurface() {}

  ContactSurface &operator=(const ContactSurface &) = default;

  bool CheckCollision(const Vec3 &pos, Vec3 *collision) const {
    return collision_func_(pos, collision);
  }

  const ReferenceFrame &frame() const { return frame_; }

  void set_collision_func(const std::function<bool(const Vec3 &, Vec3 *)> &f) {
    collision_func_ = f;
  }

 private:
  // The surface is attached to a rigid body frame that may be
  // translating or rotating.
  const ReferenceFrame &frame_;

  // The collision function returns true if the point is within the
  // surface and false otherwise.  The position of the collision point
  // (i.e. the closest point on the surface to the contactor) is also
  // returned.
  std::function<bool(const Vec3 &, Vec3 *)> collision_func_;
};

// A Contactor represents a point on a rigid body that can interact
// with a ContactSurface.
class Contactor {
 public:
  Contactor(const ContactorParams &contactor_params,
            const ReferenceFrame &parent_frame, const ContactSurface &surface)
      : contactor_params_(contactor_params),
        frame_(ReferenceFrame(parent_frame, contactor_params.pos)),
        surface_(surface) {}
  Contactor(const Contactor &) = default;
  ~Contactor() {}

  Contactor &operator=(const Contactor &) = default;

  // Returns the generalized force from a collision acting on a rigid
  // body in the local coordinate system of the rigid body.  First,
  // the position of the contactor is calculated in the surface's
  // reference frame.  Then, if there is a collision, the contact
  // force is calculated.
  void CalcContactForceMomentPos(ForceMomentPos *fmx) const;

 private:
  // Calculates the force between a contactor and a surface.  This
  // function does not know whether the contactor and the surface are
  // in contact, so it should only be run after contact has been
  // established.
  //
  // Args:
  //   contactor_pos: Position [m] of the contactor in the surface frame.
  //   contactor_vel: Velocity [m/s] of the contactor in the surface frame.
  //   collision_pos: Contactor position [m] projected onto the surface.
  //   collision_vel: Velocity [m/s] of the collision point.
  //   contact_force: Total force [N] resulting from the contact, including
  //       spring, damping, and friction forces.
  void CalcContactForce(const Vec3 &contactor_pos, const Vec3 &contactor_vel,
                        const Vec3 &collision_pos, const Vec3 &collision_vel,
                        Vec3 *contact_force) const;

  // Contactor parameters such as the position in the local frame,
  // spring constant, friction coefficient, and damping coefficient.
  const ContactorParams &contactor_params_;

  // Reference to the rigid body frame, which may be translating or
  // rotating, that this contactor is attached to.
  const ReferenceFrame frame_;

  // Reference to the ContactSurface with which this contactor
  // interacts.
  const ContactSurface &surface_;
};

// Determines whether a point lies within a circle, and calculates the nearest
// point on the circle if there is contact.
//
// Crashes if the contact point is too close to the center of the circle, as the
// contact physics will become invalid. Practically speaking, the contact point
// should always be "pretty close" to the edge of the circle.
//
// Args:
//   pos: Position of the contactor (may be inside circle).
//   radius: Radius of circle.
//   center: Center of circle
//   collision_pos: Collision point on the circle.  This is the closest
//       point to the contactor.
//
// Returns:
//   True, if the point lies within the circle.
bool ContactCircle(const Vec2 &pos, double radius, const Vec2 &center,
                   Vec2 *collision_pos);

// Determines whether a point lies on the surface formed by the union
// of two disks, and calculates the nearest point on the surface's
// boundary if there is contact.
//
// Args:
//   pos: Position of the contactor (may be within disks).
//   radius1: Radius of first circle.
//   center1: Center of first circle
//   radius2: Radius of second circle.
//   center2: Center of second circle.
//   collision_pos: Collision point on surface.  This is the closest
//       point to the contactor on the surface.
//
// Returns:
//   True if the point contacts the surface.
bool ContactTwoCircles(const Vec2 &pos, double first_radius,
                       const Vec2 &first_center, double second_radius,
                       const Vec2 &second_center, Vec2 *collision_pos);

// Determines whether a point lies withing a rectangular box, and
// calculates the nearest point on the surface of the box.
//
// Args:
//   pos: Position of the point in the coordinate system that the box is
//       defined in.
//   center: Center of the box.
//   dimensions: Dimensions of the box.  These are assumed to be aligned with
//       the coordinate system that the box is defined in.
//   collision_pos: Nearest point on the surface of the box if there is a
//       collision.
//
// Returns:
//   True if the point lies withing the box.
bool ContactBox(const Vec3 &pos, const Vec3 &center, const Vec3 &dimensions,
                Vec3 *collision_pos);

#endif  // SIM_PHYSICS_CONTACTOR_H_
