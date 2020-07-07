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

#include "control/perch_frame.h"

#include <math.h>

#include "common/c_math/coord_trans.h"
#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "control/system_types.h"

// Conversions between Ground-station and Perch.

const Mat3 *CalcDcmGToP(double perch_azi, Mat3 *dcm_g2p) {
  // TODO(b/136033164): Update to account for vessel attitude.
  return AngleToDcm(perch_azi, 0.0, 0.0, kRotationOrderZyx, dcm_g2p);
}

const Vec3 *GToP(const Vec3 *Xg, double perch_azi, Vec3 *Xp) {
  // TODO(b/136033164): Update to account for vessel attitude.
  Mat3 dcm_g2p;
  CalcDcmGToP(perch_azi, &dcm_g2p);
  return PoseTransform(&dcm_g2p, &kVec3Zero, Xg, Xp);
}

const Vec3 *RotGToP(const Vec3 *Vg, double perch_azi, Vec3 *Vp) {
  // TODO(b/136033164): Update to account for vessel attitude.
  Mat3 dcm_g2p;
  CalcDcmGToP(perch_azi, &dcm_g2p);
  return Mat3Vec3Mult(&dcm_g2p, Vg, Vp);
}

const Vec3 *PToG(const Vec3 *Xp, double perch_azi, Vec3 *Xg) {
  // TODO(b/136033164): Update to account for vessel attitude.
  Mat3 dcm_g2p;
  CalcDcmGToP(perch_azi, &dcm_g2p);
  return InversePoseTransform(&dcm_g2p, &kVec3Zero, Xp, Xg);
}

const Vec3 *RotPToG(const Vec3 *Vp, double perch_azi, Vec3 *Vg) {
  // TODO(b/136033164): Update to account for vessel attitude.
  Mat3 dcm_g2p;
  CalcDcmGToP(perch_azi, &dcm_g2p);
  return Mat3TransVec3Mult(&dcm_g2p, Vp, Vg);
}

// Conversions between Perch and Levelwind.

// The levelwind frame, denoted lw, is centered at the intersection of
// the axis-of-rotation of the levelwind and a line tangent to the
// levelwind that passes through the rotation axis of the perch.
// y_lw points along the levelwind's axis of rotation (positive elevation is up)
// z_lw points away from the kite
//
// See wiki for official definitions:
// https://wiki.makanipower.com/
// index.php/Coordinate_systems_and_other_conventions
const Mat3 *CalcDcmPToLw(double levelwind_ele, Mat3 *dcm_p2lw) {
  const LevelwindParams *params = &GetSystemParams()->levelwind;
  return AngleToDcm(params->azimuth_offset, levelwind_ele - PI / 2.0, 0.0,
                    kRotationOrderZyx, dcm_p2lw);
}

// Returns the location, in perch coordinates, of the origin of the
// levelwind coordinate system.
const Vec3 *CalcXpLevelwindOrigin(double drum_angle, Vec3 *Xp_lw_frame_origin) {
  const SystemParams *params = GetSystemParams();
  *Xp_lw_frame_origin = params->perch.levelwind_origin_p_0;
  Xp_lw_frame_origin->z +=
      params->levelwind.drum_angle_to_vertical_travel * drum_angle;
  return Xp_lw_frame_origin;
}

const Vec3 *PToLw(const Vec3 *Xp, double levelwind_ele, double drum_angle,
                  Vec3 *X_lw) {
  Mat3 dcm_p2lw;
  CalcDcmPToLw(levelwind_ele, &dcm_p2lw);
  Vec3 Xp_lw_frame_origin;
  CalcXpLevelwindOrigin(drum_angle, &Xp_lw_frame_origin);
  return PoseTransform(&dcm_p2lw, &Xp_lw_frame_origin, Xp, X_lw);
}

const Vec3 *RotPToLw(const Vec3 *Xp, double levelwind_ele, Vec3 *X_lw) {
  Mat3 dcm_p2lw;
  CalcDcmPToLw(levelwind_ele, &dcm_p2lw);
  return Mat3Vec3Mult(&dcm_p2lw, Xp, X_lw);
}

const Vec3 *LwToP(const Vec3 *X_lw, double levelwind_ele, double drum_angle,
                  Vec3 *Xp) {
  Mat3 dcm_p2lw;
  CalcDcmPToLw(levelwind_ele, &dcm_p2lw);
  Vec3 Xp_lw_frame_origin;
  CalcXpLevelwindOrigin(drum_angle, &Xp_lw_frame_origin);
  return InversePoseTransform(&dcm_p2lw, &Xp_lw_frame_origin, X_lw, Xp);
}

const Vec3 *RotLwToP(const Vec3 *X_lw, double levelwind_ele, Vec3 *Xp) {
  Mat3 dcm_p2lw;
  CalcDcmPToLw(levelwind_ele, &dcm_p2lw);
  return Mat3TransVec3Mult(&dcm_p2lw, X_lw, Xp);
}

// Conversions between Perch and Winch drum.

const Mat3 *CalcDcmPToWd(double drum_angle, Mat3 *dcm_p2wd) {
  return AngleToDcm(drum_angle, 0.0, 0.0, kRotationOrderZyx, dcm_p2wd);
}

// Return the origin of the winch drum frame, with respect to the
// platform frame (GSv2) or perch frame (GSv1 and TopHat).
static const Vec3 *GetDrumOriginP(void) {
  return GetSystemParams()->gs_model == kGroundStationModelGSv2
             ? &GetSystemParams()->ground_station.gs02.drum_origin_p
             : &GetSystemParams()->perch.winch_drum_origin_p;
}

const Vec3 *PToWd(const Vec3 *Xp, double drum_angle, Vec3 *X_wd) {
  Mat3 dcm_p2wd;
  CalcDcmPToWd(drum_angle, &dcm_p2wd);
  return PoseTransform(&dcm_p2wd, GetDrumOriginP(), Xp, X_wd);
}

const Vec3 *RotPToWd(const Vec3 *Vp, double drum_angle, Vec3 *V_wd) {
  Mat3 dcm_p2wd;
  CalcDcmPToWd(drum_angle, &dcm_p2wd);
  return Mat3Vec3Mult(&dcm_p2wd, Vp, V_wd);
}

const Vec3 *WdToP(const Vec3 *X_wd, double drum_angle, Vec3 *Xp) {
  Mat3 dcm_p2wd;
  CalcDcmPToWd(drum_angle, &dcm_p2wd);
  return InversePoseTransform(&dcm_p2wd, GetDrumOriginP(), X_wd, Xp);
}

const Vec3 *RotWdToP(const Vec3 *V_wd, double drum_angle, Vec3 *Vp) {
  Mat3 dcm_p2wd;
  CalcDcmPToWd(drum_angle, &dcm_p2wd);
  return Mat3TransVec3Mult(&dcm_p2wd, V_wd, Vp);
}
