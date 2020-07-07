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

// An idealized tether would follow the shape of a catenary curve.  A
// catenary has a number of properties that are useful to know:
//
// - The horizontal tension is equal everywhere.  At the vertex of the
//   catenary (its lowest point), there is no vertical tension, so the
//   total tension there equals the horizontal tension.  This
//   intrinsic tension is often denoted T_0 in the engineering
//   literature.
//
// - The vertical tension at a point in the catenary is equal to the
//   weight of the cable between that point and the vertex.
//
// - The tension is always tangential to the shape of the curve.  The
//   catenary cable transfers no moment.
//
// The functional form of a catenary curve is:
//
//   h(r) = (1/2a) cosh [ (r + constant) * (2a) ] + constant
//
// where
//
//   a = (1/2) (mu / tension_x)
//
//   tension_x: intrinsic tension (at the vertex) [N].
//   mu: linear weight density of the cable [N/m].
//
// Here we approximate the catenary as a parabola:
//
//   h(r) = a*r^2 + b*r
//
// where we have assumed a coordinate system where the parabola passes
// through the origin, which is taken to be the ground site attachment
// point.  Thus r is the distance from the GS attachment point and h
// is the altitude above the GS attachment point.  The parameter b is
// set to fulfill a second boundary condition.

#ifndef CONTROL_TETHER_UTIL_H_
#define CONTROL_TETHER_UTIL_H_

#include "common/c_math/vec3.h"

typedef struct { double a, b; } TetherParabola;

#ifdef __cplusplus
extern "C" {
#endif

// Functions for finding cable shape.

// Calculates a tether parabola based on horizontal tension and a
// point it passes through.  The horizontal tension and a point (r, h)
// define a parabola through the origin:
//
//   a = mu / (2 * tension_x)
//   b = (h - a * r^2) / r
//
// where mu is the linear weight density.
void TensionAndPointToParabola(double tension_x, double r, double h, double mu,
                               TetherParabola *tp);

// Calculates a tether parabola based on horizontal tension and an
// angle at a distance r from the origin.  The horizontal tension,
// angle, and position, r, define a parabola through the origin:
//
//   a = mu / (2 * tension_x)
//   b = tan(angle) - 2 * a * r
//
// where mu is the linear weight density.
void TensionAndAngleToParabola(double tension_x, double r, double angle,
                               double mu, TetherParabola *tp);

// Functions for finding info on cable points.

// Returns the parabola height at a distance r:
//
//   h = a r^2 + b r
//
double ParabolaHeight(const TetherParabola *tp, double r);

// Returns the parabola angle at a distance r:
//
//   dh/dr = 2 a r + b
//   angle = atan(2 a r + b)
//
double ParabolaAngle(const TetherParabola *tp, double r);

// Finds the x-coordinate of the parabola minimum (vertex):
//
//   r_min = -b / (2 * a)
//
double ParabolaVertex(const TetherParabola *tp);

// Finds the minimum height of the parabola.
double ParabolaMinimum(const TetherParabola *tp);

// Finds the horizontal component of tension in the catenary.
double ParabolaHorizontalTension(const TetherParabola *tp, double mu);

// Finds the vertical component of tension at a particular point in
// the catenary.
double ParabolaVerticalTension(const TetherParabola *tp, double mu, double r);

// Returns the total tension at a point in the catenary.
double ParabolaTension(const TetherParabola *tp, double mu, double r);

// By modeling the tether as a parabola, this converts a horizontal
// tension command to an equivalent elevation angle assuming we are
// controlling the tether to leave the levelwind at a target angle.
double ConvertTensionToElevation(double horizontal_tension_cmd,
                                 const Vec3 *wing_pos_g,
                                 double target_tether_elevation);

// By modeling the tether as a parabola, this converts a horizontal
// tension command and radial distance to an altitude target that
// controls the tether to leave the levelwind at a target angle.
double ConvertTensionToHeight(double horizontal_tension_cmd,
                              double wing_radial_distance,
                              double target_tether_elevation);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CONTROL_TETHER_UTIL_H_
