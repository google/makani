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

// Conversions between ground-station and crosswind coordinate systems.
//
// The "crosswind" coordinate system, denoted with an "cw", is used to
// describe the plane the wing flies circles in during crosswind
// flight.  The plane may be moved around to accommodate shifting wind
// directions or various levels of power generation and tension
// control.  It is defined relative to its parent "ground-station"
// coordinate system, denoted with a "g", as:
//
//   x: Normal to the crosswind flight plane and points towards the
//      ground-station.
//   y: Horizontal in crosswind flight plane and increases in a
//      counter-clockwise direction about the zg axis.
//   z: Positive downward in crosswind flight plane, forming a
//      right-handed orthogonal coordinate system.

#ifndef CONTROL_CROSSWIND_CROSSWIND_FRAME_H_
#define CONTROL_CROSSWIND_CROSSWIND_FRAME_H_

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"

#ifdef __cplusplus
extern "C" {
#endif

// Calculates the direction cosine matrix between the ground and
// crosswind frames.
void CalcDcmGToCw(const Vec3 *path_center_g, Mat3 *dcm_g2cw);

// Calculates the direction cosine matrix between the crosswind
// frame (cw) and the frame (t) tangent to the crosswind circle with
// the x-axis pointing in the direction of flight.
void CalcDcmCwToT(double loop_angle, Mat3 *dcm_cw2t);

void CalcDcmGToT(const Vec3 *path_center_g, double loop_angle, Mat3 *dcm_g2t);

// Transforms a point in the ground frame to the crosswind frame.
void TransformGToCw(const Vec3 *point_g, const Vec3 *path_center_g,
                    Vec3 *point_cw);

// Rotates a vector from the ground frame to the crosswind frame.
void RotateGToCw(const Vec3 *vector_g, const Vec3 *path_center_g,
                 Vec3 *vector_cw);

// Transforms a point in the crosswind frame to the ground frame.
void TransformCwToG(const Vec3 *point_cw, const Vec3 *path_center_g,
                    Vec3 *point_g);

// Rotates a vector from the crosswind frame to the ground frame.
void RotateCwToG(const Vec3 *vector_cw, const Vec3 *path_center_g,
                 Vec3 *vector_g);

// Construct the DCM relating the ground frame and the fly frame.
//
// Input arguments:
//
// apparent_wind_g      Apparent wind at the kite written in frame g
// anchor2kite_g        Vector pointing from tether anchor to kite, frame g
//
// Output arguments:
//
// dcm_f2g              Transformation from f frame components to g frame
void CalcDcmFToG(const Vec3 *apparent_wind_g, const Vec3 *anchor2kite_g,
                 Mat3 *dcm_f2g);

// Construct the DCM relating the fly frame and body frame.
//
// Input arguments:
//
// phi_a_cmd          "Airspeed roll angle" command.
// beta_cmd           Sideslip command.
// alpha_cmd          Angle of attack command.
//
// Output arguments:
//
// dcm_f2b            Transformation from f frame components to b frame
void CalcDcmFToB(double phi_a_cmd, double beta_cmd, double alpha_cmd,
                 Mat3 *dcm_f2b);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_CROSSWIND_CROSSWIND_FRAME_H_
