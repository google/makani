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

#include "control/hover/hover_frame.h"

#include <math.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

void CalcDcmGToH(const Vec3 *hover_pos_g, const Vec3 *hover_origin_g,
                 Mat3 *dcm_g2h) {
  Vec3 origin_to_pos_g;
  Vec3Sub(hover_pos_g, hover_origin_g, &origin_to_pos_g);

  AngleToDcm(atan2(-origin_to_pos_g.y, -origin_to_pos_g.x), PI / 2.0, 0.0,
             kRotationOrderZyx, dcm_g2h);
}

void TransformGToH(const Vec3 *point_g, const Vec3 *hover_pos_g,
                   const Vec3 *hover_origin_g, Vec3 *point_h) {
  Mat3 dcm_g2h;
  CalcDcmGToH(hover_pos_g, hover_origin_g, &dcm_g2h);
  PoseTransform(&dcm_g2h, hover_pos_g, point_g, point_h);
}

void RotateGToH(const Vec3 *vector_g, const Vec3 *hover_pos_g,
                const Vec3 *hover_origin_g, Vec3 *vector_h) {
  Mat3 dcm_g2h;
  CalcDcmGToH(hover_pos_g, hover_origin_g, &dcm_g2h);
  Mat3Vec3Mult(&dcm_g2h, vector_g, vector_h);
}

void TransformHToG(const Vec3 *point_h, const Vec3 *hover_pos_g,
                   const Vec3 *hover_origin_g, Vec3 *point_g) {
  Mat3 dcm_g2h;
  CalcDcmGToH(hover_pos_g, hover_origin_g, &dcm_g2h);
  InversePoseTransform(&dcm_g2h, hover_pos_g, point_h, point_g);
}

void RotateHToG(const Vec3 *vector_h, const Vec3 *hover_pos_g,
                const Vec3 *hover_origin_g, Vec3 *vector_g) {
  Mat3 dcm_g2h;
  CalcDcmGToH(hover_pos_g, hover_origin_g, &dcm_g2h);
  Mat3TransVec3Mult(&dcm_g2h, vector_h, vector_g);
}
