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

#include "sim/physics/reference_frame.h"

#include <glog/logging.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

ReferenceFrame::ReferenceFrame(const ReferenceFrame &parent__,
                               const Vec3 &origin_pos_parent,
                               const Vec3 &origin_vel_parent,
                               const Vec3 &origin_acc_parent,
                               const Mat3 &dcm_parent_to_local,
                               const Vec3 &omega_parent,
                               const Vec3 &domega_parent)
    : parent_(&parent__),
      origin_pos_root_(),
      origin_vel_root_(),
      origin_acc_root_(),
      dcm_root_to_local_(),
      omega_root_(),
      domega_root_() {
  // Calculate DCM from root.
  Mat3Mult(&dcm_parent_to_local, kNoTrans, &parent().dcm_root_to_local_,
           kNoTrans, &dcm_root_to_local_);

  // Calculate the position of the origin in root coordinates.
  parent().TransformToRoot(kPosition, origin_pos_parent, &origin_pos_root_);

  // Calculate the angular rate of the frame relative to the root
  // frame in root coordinates.
  parent().TransformToRoot(kAngularVelocity, omega_parent, &omega_root_);

  // Calculate the velocity of the origin in root coordinates.
  parent().TransformToRoot(kVelocity, origin_vel_parent, &origin_vel_root_);
  Vec3 position_offset_root;
  parent().RotateToRoot(origin_pos_parent, &position_offset_root);
  Vec3 omega_cross_pos;
  Vec3Cross(&parent().omega_root_, &position_offset_root, &omega_cross_pos);
  Vec3Add(&origin_vel_root_, &omega_cross_pos, &origin_vel_root_);

  // Calculate the angular acceleration of the frame relative to the
  // root frame in root coordinates.
  parent().TransformToRoot(kAngularAcceleration, domega_parent, &domega_root_);

  // Calculate the acceleration of the origin in root coordinates.
  Vec3 velocity_offset_root;
  parent().RotateToRoot(origin_vel_parent, &velocity_offset_root);
  Vec3 fictious_acc_root;
  CalcFictitiousAcceleration(position_offset_root, velocity_offset_root,
                             parent().omega_root_, parent().domega_root_,
                             &fictious_acc_root);
  parent().TransformToRoot(kAcceleration, origin_acc_parent, &origin_acc_root_);
  Vec3Add(&origin_acc_root_, &fictious_acc_root, &origin_acc_root_);
}

void ReferenceFrame::RotateTo(const ReferenceFrame &frame,
                              const Vec3 &vec_local, Vec3 *vec_frame) const {
  RotateToRoot(vec_local, vec_frame);
  frame.RotateFromRoot(*vec_frame, vec_frame);
}

void ReferenceFrame::RotateFrom(const ReferenceFrame &frame,
                                const Vec3 &vec_frame, Vec3 *vec_local) const {
  frame.RotateToRoot(vec_frame, vec_local);
  RotateFromRoot(*vec_local, vec_local);
}

void ReferenceFrame::TransformTo(const ReferenceFrame &frame,
                                 VectorType vector_type, const Vec3 &vec_local,
                                 Vec3 *vec_frame) const {
  TransformToRoot(vector_type, vec_local, vec_frame);
  frame.TransformFromRoot(vector_type, *vec_frame, vec_frame);
}

void ReferenceFrame::TransformOriginTo(const ReferenceFrame &frame,
                                       VectorType vector_type,
                                       Vec3 *vec_frame) const {
  TransformTo(frame, vector_type, kVec3Zero, vec_frame);
}

void ReferenceFrame::TransformFrom(const ReferenceFrame &frame,
                                   VectorType vector_type,
                                   const Vec3 &vec_frame,
                                   Vec3 *vec_local) const {
  frame.TransformToRoot(vector_type, vec_frame, vec_local);
  TransformFromRoot(vector_type, *vec_local, vec_local);
}

void ReferenceFrame::TransformOriginFrom(const ReferenceFrame &frame,
                                         VectorType vector_type,
                                         Vec3 *vec_local) const {
  TransformFrom(frame, vector_type, kVec3Zero, vec_local);
}

const Vec3 &ReferenceFrame::GetOriginVector(VectorType vector_type) const {
  switch (vector_type) {
    default:
      DCHECK(false);
    case kVector:
      return kVec3Zero;
    case kPosition:
      return origin_pos_root_;
    case kVelocity:
      return origin_vel_root_;
    case kAcceleration:
      return origin_acc_root_;
    case kAngularVelocity:
      return omega_root_;
    case kAngularAcceleration:
      return domega_root_;
  }
}

void ReferenceFrame::RotateToRoot(const Vec3 &vec_local, Vec3 *vec_root) const {
  Mat3TransVec3Mult(&dcm_root_to_local_, &vec_local, vec_root);
}

void ReferenceFrame::RotateFromRoot(const Vec3 &vec_root,
                                    Vec3 *vec_local) const {
  Mat3Vec3Mult(&dcm_root_to_local_, &vec_root, vec_local);
}

void ReferenceFrame::TransformToRoot(VectorType vector_type,
                                     const Vec3 &vec_local,
                                     Vec3 *vec_root) const {
  InversePoseTransform(&dcm_root_to_local_, &GetOriginVector(vector_type),
                       &vec_local, vec_root);
}

void ReferenceFrame::TransformFromRoot(VectorType vector_type,
                                       const Vec3 &vec_root,
                                       Vec3 *vec_local) const {
  PoseTransform(&dcm_root_to_local_, &GetOriginVector(vector_type), &vec_root,
                vec_local);
}

void ReferenceFrame::CalcFictitiousAcceleration(const Vec3 &pos,
                                                const Vec3 &vel,
                                                const Vec3 &omega,
                                                const Vec3 &domega,
                                                Vec3 *acc) const {
  // Coriolis acceleration: 2 omega x velocity.
  Vec3 acc_coriolis;
  Vec3Cross(&omega, &vel, &acc_coriolis);
  Vec3Scale(&acc_coriolis, 2.0, &acc_coriolis);

  // Centrifugal acceleration: omega x (omega x position).
  Vec3 acc_centrifugal;
  Vec3Cross(&omega, &pos, &acc_centrifugal);
  Vec3Cross(&omega, &acc_centrifugal, &acc_centrifugal);

  // Euler acceleration: domega/dt x position.
  Vec3 acc_euler;
  Vec3Cross(&domega, &pos, &acc_euler);

  Vec3Add3(&acc_coriolis, &acc_centrifugal, &acc_euler, acc);
}
