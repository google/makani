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

#include "control/crosswind/crosswind_frame.h"

#include <math.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"

void CalcDcmGToCw(const Vec3 *path_center_g, Mat3 *dcm_g2cw) {
  Vec3 ex = *path_center_g, ey, ez;
  Vec3Scale(&ex, -1.0 / Vec3NormBound(&ex, DBL_TOL), &ex);
  Vec3Cross(&kVec3Z, &ex, &ey);
  Vec3Scale(&ey, 1.0 / Vec3NormBound(&ey, DBL_TOL), &ey);
  Vec3Cross(&ex, &ey, &ez);

  dcm_g2cw->d[0][0] = ex.x;
  dcm_g2cw->d[0][1] = ex.y;
  dcm_g2cw->d[0][2] = ex.z;
  dcm_g2cw->d[1][0] = ey.x;
  dcm_g2cw->d[1][1] = ey.y;
  dcm_g2cw->d[1][2] = ey.z;
  dcm_g2cw->d[2][0] = ez.x;
  dcm_g2cw->d[2][1] = ez.y;
  dcm_g2cw->d[2][2] = ez.z;
}

void CalcDcmCwToT(double loop_angle, Mat3 *dcm_cw2t) {
  AngleToDcm(0.0, PI / 2.0, loop_angle, kRotationOrderXyz, dcm_cw2t);
}

void CalcDcmGToT(const Vec3 *path_center_g, double loop_angle, Mat3 *dcm_g2t) {
  Mat3 dcm_g2cw, dcm_cw2t;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  CalcDcmCwToT(loop_angle, &dcm_cw2t);
  Mat3Mat3Mult(&dcm_cw2t, &dcm_g2cw, dcm_g2t);
}

void TransformGToCw(const Vec3 *point_g, const Vec3 *path_center_g,
                    Vec3 *point_cw) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  PoseTransform(&dcm_g2cw, path_center_g, point_g, point_cw);
}

void RotateGToCw(const Vec3 *vector_g, const Vec3 *path_center_g,
                 Vec3 *vector_cw) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Mat3Vec3Mult(&dcm_g2cw, vector_g, vector_cw);
}

void TransformCwToG(const Vec3 *point_cw, const Vec3 *path_center_g,
                    Vec3 *point_g) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  InversePoseTransform(&dcm_g2cw, path_center_g, point_cw, point_g);
}

void RotateCwToG(const Vec3 *vector_cw, const Vec3 *path_center_g,
                 Vec3 *vector_g) {
  Mat3 dcm_g2cw;
  CalcDcmGToCw(path_center_g, &dcm_g2cw);
  Mat3TransVec3Mult(&dcm_g2cw, vector_cw, vector_g);
}

void CalcDcmFToG(const Vec3 *apparent_wind_g, const Vec3 *anchor2kite_g,
                 Mat3 *dcm_f2g) {
  // The Fly Frame is defined as: x axis pointing into the apparent wind,
  // y axis tangent to the sphere, z axis in a right handed sense.
  Vec3 if_g, jf_g, kf_g;
  Vec3 is_g;

  // X axis is "forward" into the oncoming apparent wind.
  Vec3Normalize(apparent_wind_g, &if_g);
  Vec3Scale(&if_g, -1.0, &if_g);

  // A unit vector normal to the sphere
  Vec3Normalize(anchor2kite_g, &is_g);

  // The Y axis is in the direction "X cross Sphere Normal"
  Vec3 ifcrossis_g;
  Vec3Cross(&if_g, &is_g, &ifcrossis_g);
  Vec3Normalize(&ifcrossis_g, &jf_g);

  // Z axis is defined in a right handed sense
  Vec3Cross(&if_g, &jf_g, &kf_g);
  Vec3Normalize(&kf_g, &kf_g);

  // dcm_f2g contains frame f's unit vectors written using g
  // frame components.
  dcm_f2g->d[0][0] = if_g.x;
  dcm_f2g->d[1][0] = if_g.y;
  dcm_f2g->d[2][0] = if_g.z;
  dcm_f2g->d[0][1] = jf_g.x;
  dcm_f2g->d[1][1] = jf_g.y;
  dcm_f2g->d[2][1] = jf_g.z;
  dcm_f2g->d[0][2] = kf_g.x;
  dcm_f2g->d[1][2] = kf_g.y;
  dcm_f2g->d[2][2] = kf_g.z;
}

void CalcDcmFToB(double phi_a_cmd, double beta_cmd, double alpha_cmd,
                 Mat3 *dcm_f2b) {
  AngleToDcm(phi_a_cmd, -beta_cmd, alpha_cmd, kRotationOrderXzy, dcm_f2b);
}
