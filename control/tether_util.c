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

#include "control/tether_util.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

#include "common/c_math/util.h"
#include "control/system_params.h"

// TODO: Remove arbitrary 1e-6.
void TensionAndPointToParabola(double tension_x, double r, double h, double mu,
                               TetherParabola *tp) {
  assert(tension_x > 0.0);
  assert(mu > 0.0);
  assert(tp != NULL);

  if (-1e-6 < r && r < 1e-6) {
    // The problem is ill conditioned if the radius is near zero.
    assert(false);
    tp->a = 0.0;
    tp->b = 0.0;
  } else {
    tp->a = mu / (2.0 * fmax(tension_x, 1e-6));
    tp->b = (h - tp->a * r * r) / r;
  }
}

void TensionAndAngleToParabola(double tension_x, double r, double angle,
                               double mu, TetherParabola *tp) {
  assert(tension_x > 0.0);
  assert(mu > 0.0);
  assert(tp != NULL);

  tp->a = mu / (2.0 * fmax(tension_x, 1e-6));
  double dh_dr = Saturate(tan(angle), -1e9, 1e9);
  tp->b = dh_dr - 2.0 * tp->a * r;
}

double ParabolaHeight(const TetherParabola *tp, double r) {
  assert(tp != NULL && tp->a > 0.0);
  return tp->a * r * r + tp->b * r;
}

double ParabolaAngle(const TetherParabola *tp, double r) {
  assert(tp != NULL && tp->a > 0.0);
  return atan(2.0 * tp->a * r + tp->b);
}

double ParabolaVertex(const TetherParabola *tp) {
  assert(tp != NULL && tp->a > 0.0);
  return -tp->b / (2.0 * fmax(tp->a, 1e-6));
}

double ParabolaMinimum(const TetherParabola *tp) {
  assert(tp != NULL && tp->a > 0.0);
  return ParabolaHeight(tp, ParabolaVertex(tp));
}

double ParabolaHorizontalTension(const TetherParabola *tp, double mu) {
  assert(tp != NULL && tp->a > 0.0 && mu > 0.0);
  return mu / (2.0 * fmax(tp->a, 1e-6));
}

double ParabolaVerticalTension(const TetherParabola *tp, double mu, double r) {
  assert(tp != NULL && tp->a > 0.0 && mu > 0.0);
  return ParabolaHorizontalTension(tp, mu) * (2.0 * tp->a * r + tp->b);
}

double ParabolaTension(const TetherParabola *tp, double mu, double r) {
  assert(tp != NULL && tp->a > 0.0 && mu > 0.0);
  return hypot(ParabolaHorizontalTension(tp, mu),
               ParabolaVerticalTension(tp, mu, r));
}

// By modeling the tether as a parabola, this converts a horizontal
// tension command and radial distance to an altitude target that
// controls the tether to leave the levelwind at a target angle.
double ConvertTensionToHeight(double horizontal_tension_cmd,
                              double wing_radial_distance,
                              double target_tether_elevation) {
  assert(horizontal_tension_cmd >= 0.0);

  // Find the tether parabola parameters that fit the feed-forward
  // tension and target levelwind angle.
  TetherParabola tether_parabola;
  TensionAndAngleToParabola(
      horizontal_tension_cmd, 0.0, target_tether_elevation,
      g_sys.phys->g * g_sys.tether->linear_density, &tether_parabola);

  return ParabolaHeight(&tether_parabola, wing_radial_distance);
}

// By modeling the tether as a parabola, this converts a horizontal
// tension command to an equivalent elevation angle assuming we are
// controlling the tether to leave the levelwind at a target angle.
double ConvertTensionToElevation(double horizontal_tension_cmd,
                                 const Vec3 *wing_pos_g,
                                 double target_tether_elevation) {
  assert(wing_pos_g);
  double xy_norm = Vec3XyNorm(wing_pos_g);
  double height = ConvertTensionToHeight(horizontal_tension_cmd, xy_norm,
                                         target_tether_elevation);
  return atan2(height, xy_norm);
}
