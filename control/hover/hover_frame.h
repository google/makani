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

// Conversions between ground-station and hover coordinate systems.
//
// The "hover" coordinate system, denoted with an "h", is used to
// describe a close-to-nominal hover orientation, not accounting for
// tether roll or pitch and yaw corrections, in which the hover
// controller operates.  It is defined relative to its parent
// "ground-station" coordinate system, denoted with a "g", with an
// origin at the wing center and axes defined as:
//
//   x: Points straight up, along the negative z axis of the g-frame.
//   y: Points along the cross product of the wing position and the
//      ground z axis, i.e. tangentially along the tether sphere,
//      level with the ground, increasing toward the starboard tip of
//      the wing.
//   z: Points towards the ground-station, level with the ground.

#ifndef CONTROL_HOVER_HOVER_FRAME_H_
#define CONTROL_HOVER_HOVER_FRAME_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// Calculates the direction cosine matrix between the ground and hover
// frames.
void CalcDcmGToH(const Vec3 *hover_pos_g, const Vec3 *hover_origin_g,
                 Mat3 *dcm_g2h);

// Transforms a point in the ground frame to the hover frame.
void TransformGToH(const Vec3 *point_g, const Vec3 *hover_pos_g,
                   const Vec3 *hover_origin_g, Vec3 *point_h);

// Rotates a vector from the ground frame to the hover frame.
void RotateGToH(const Vec3 *vector_g, const Vec3 *hover_pos_g,
                const Vec3 *hover_origin_g, Vec3 *vector_h);

// Transforms a point in the hover frame to the ground frame.
void TransformHToG(const Vec3 *point_h, const Vec3 *hover_pos_g,
                   const Vec3 *hover_origin_g, Vec3 *point_g);

// Rotates a vector from the hover frame to the ground frame.
void RotateHToG(const Vec3 *vector_h, const Vec3 *hover_pos_g,
                const Vec3 *hover_origin_g, Vec3 *vector_g);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_HOVER_HOVER_FRAME_H_
