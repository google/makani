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

#ifndef SIM_PHYSICS_REFERENCE_FRAME_H_
#define SIM_PHYSICS_REFERENCE_FRAME_H_

#include <glog/logging.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

// A ReferenceFrame is used to transform vectors (positions,
// velocities, etc.) between coordinate systems that are potentially
// moving, accelerating, and rotating.  Frames may be embedded within
// other frames and all frames eventually trace back to a "root"
// reference frame.
class ReferenceFrame {
  friend class ReferenceFrameTest;

 public:
  enum VectorType {
    kVector,
    kPosition,
    kVelocity,
    kAcceleration,
    kAngularVelocity,
    kAngularAcceleration
  };

  // Constructs the root reference frame.
  ReferenceFrame()
      : parent_(nullptr),
        origin_pos_root_(kVec3Zero),
        origin_vel_root_(kVec3Zero),
        origin_acc_root_(kVec3Zero),
        dcm_root_to_local_(kMat3Identity),
        omega_root_(kVec3Zero),
        domega_root_(kVec3Zero) {}

  // Constructs a general, accelerating reference frame.
  ReferenceFrame(const ReferenceFrame &parent__, const Vec3 &origin_pos_parent,
                 const Vec3 &origin_vel_parent, const Vec3 &origin_acc_parent,
                 const Mat3 &dcm_parent_to_local, const Vec3 &omega_parent,
                 const Vec3 &domega_parent);

  // TODO: It would be more efficient to not run the
  // general, accelerating frame constructor for the special cases.

  // Constructs a rotating reference frame.
  ReferenceFrame(const ReferenceFrame &parent__, const Vec3 &origin_pos_parent,
                 const Vec3 &origin_vel_parent, const Mat3 &dcm_parent_to_local,
                 const Vec3 &omega_parent)
      : ReferenceFrame(parent__, origin_pos_parent, origin_vel_parent,
                       kVec3Zero, dcm_parent_to_local, omega_parent,
                       kVec3Zero) {}

  // Constructs a coordinate transformation.
  ReferenceFrame(const ReferenceFrame &parent__, const Vec3 &origin_pos_parent,
                 const Mat3 &dcm_parent_to_local)
      : ReferenceFrame(parent__, origin_pos_parent, kVec3Zero, kVec3Zero,
                       dcm_parent_to_local, kVec3Zero, kVec3Zero) {}

  // Constructs an offset in the current coordinate system.
  ReferenceFrame(const ReferenceFrame &parent__, const Vec3 &origin_pos_parent)
      : ReferenceFrame(parent__, origin_pos_parent, kVec3Zero, kVec3Zero,
                       kMat3Identity, kVec3Zero, kVec3Zero) {}

  ReferenceFrame(const ReferenceFrame &) = default;
  virtual ~ReferenceFrame() {}

  ReferenceFrame &operator=(const ReferenceFrame &) = default;

  // Returns the current reference frame's parent frame.
  const ReferenceFrame &parent() const { return *DCHECK_NOTNULL(parent_); }

  // Rotates a vector in the local frame to another frame.
  void RotateTo(const ReferenceFrame &frame, const Vec3 &vec_local,
                Vec3 *vec_frame) const;

  // Rotates a vector in another frame to the local frame.
  void RotateFrom(const ReferenceFrame &frame, const Vec3 &vec_frame,
                  Vec3 *vec_local) const;

  // Transforms a position, velocity, etc. vector from the local frame
  // to another frame.  For notational purposes, we define a particle
  // frame, who differs from the local frame by just the position,
  // velocity, etc. vector.  Let f, l, p, r denote the input frame,
  // local frame, particle frame, and root fame.  Let v_lp^l denote
  // the input vector, vec_local, and v_fl^r denote the corresponding
  // vector type that transforms the input frame to the local frame.
  // Here the notation v_ab^c is shorthand for the coordinate
  // derivatives (d/dt)^k x_ab^c for k = 0, 1, 2.  Then this function
  // is performing the transformation:
  //
  //   C_l^f v_lp^l + C_r^f v_fl^r --> C_r^f v_fp^r
  //           :                          \   /
  //        vec_local                    vec_frame
  //
  // For simple vectors, like position, this simplifies to:
  //
  //   r_lp^f + r_fl^f --> r_fp^f
  //
  // For vectors with derivatives, like velocity and acceleration,
  // vec_frame may be interpreted as the velocity (resp. acceleration)
  // from the input frame to the particle resolved in a non-rotating
  // frame aligned with the input frame.
  void TransformTo(const ReferenceFrame &frame, VectorType vector_type,
                   const Vec3 &vec_local, Vec3 *vec_frame) const;

  // Transforms the position, velocity, etc. of the origin from the
  // local frame to another frame.
  void TransformOriginTo(const ReferenceFrame &frame, VectorType vector_type,
                         Vec3 *vec_frame) const;

  // Transforms a position, velocity, etc. vector from another frame
  // to the local frame.  For notational purposes, we define a
  // particle frame, who differs from the input frame by just the
  // position, velocity, etc. vector.  Let f, l, p, r denote the input
  // frame, local frame, particle frame (i.e. where the input vector
  // points to), and root fame.  Let v_fp^f denote the input vector,
  // vec_frame, and v_lf^r denote the corresponding vector type that
  // transforms the local frame to the input frame.  Here the notation
  // v_ab^c is shorthand for the coordinate derivatives (d/dt)^k
  // x_ab^c for k = 0, 1, 2.  Then this function is performing the
  // transformation:
  //
  //   C_f^l v_fp^f + C_r^l v_lf^r --> C_r^l v_lp^r
  //           :                          \   /
  //        vec_frame                    vec_local
  //
  // For simple vectors, like position, this simplifies to:
  //
  //   r_fp^l + r_lf^l --> r_lp^l
  //
  // For vectors with derivatives, like velocity and acceleration,
  // vec_local may be interpreted as the velocity (resp. acceleration)
  // from the local frame to the particle resolved in a non-rotating
  // frame aligned with the local frame.
  void TransformFrom(const ReferenceFrame &frame, VectorType vector_type,
                     const Vec3 &vec_frame, Vec3 *vec_local) const;

  // Transforms the position, velocity, etc. of the origin from
  // another frame to the local frame.
  void TransformOriginFrom(const ReferenceFrame &frame, VectorType vector_type,
                           Vec3 *vec_local) const;

 private:
  // Returns the position, velocity, etc. of the origin of the
  // reference frame expressed in the root frame.
  const Vec3 &GetOriginVector(VectorType vector_type) const;

  // These functions rotate or transform vectors similarly to the
  // other functions; however they always rotate or transform to or
  // from the root frame.
  void RotateToRoot(const Vec3 &vec_local, Vec3 *vec_root) const;
  void RotateFromRoot(const Vec3 &vec_root, Vec3 *vec_local) const;
  void TransformToRoot(VectorType vector_type, const Vec3 &vec_local,
                       Vec3 *vec_root) const;
  void TransformFromRoot(VectorType vector_type, const Vec3 &vec_root,
                         Vec3 *vec_local) const;

  // Calculates the sum of the Coriolis, centrifugal, and Euler
  // fictitious accelerations.
  void CalcFictitiousAcceleration(const Vec3 &pos, const Vec3 &vel,
                                  const Vec3 &omega, const Vec3 &domega,
                                  Vec3 *acc) const;

  // Pointer to the parent reference frame.  If this is nullptr, then
  // this is the root reference frame to which everything else is
  // referenced.
  const ReferenceFrame *parent_;

  // Position [m] of the reference frame origin in the root coordinate
  // system.
  Vec3 origin_pos_root_;

  // Velocity [m/s] of the reference frame origin in the root
  // coordinate system.
  Vec3 origin_vel_root_;

  // Acceleration [m/s^2] of the reference frame origin in the root
  // coordinate system.
  Vec3 origin_acc_root_;

  // Direction cosine matrix that takes vectors in the root coordinate
  // system and converts them to vectors in the local coordinate
  // system.
  Mat3 dcm_root_to_local_;

  // Angular rate [rad/s] of the reference frame in the local
  // coordinate system.
  Vec3 omega_root_;

  // Angular acceleration [rad/s^2] of the reference frame in the
  // local coordinate system.
  Vec3 domega_root_;
};

#endif  // SIM_PHYSICS_REFERENCE_FRAME_H_
