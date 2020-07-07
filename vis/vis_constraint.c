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

#include "vis/vis_constraint.h"

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "vis/vis_util.h"

void DrawConstraint() {
  Vec3 proboscis_g = sim_telem.wing.proboscis_pos_g;
  Vec3 anchor_g = GetSimParams()->constraint_sim.anchor_pos_g;

  // Draw the constraint anchor point.
  DrawPoint(&anchor_g);

  // Draw the line between the anchor point and the proboscis.  Color
  // the line green when there is low tension on it and red otherwise.
  double tension_fraction =
      Saturate(sim_telem.wing.constraint_tension / 1000.0, 0.0, 1.0);
  glColor4d(tension_fraction, 1.0 - tension_fraction, 0.0, 0.5);
  glBegin(GL_LINE_STRIP);
  glVertex3d(anchor_g.x, anchor_g.y, anchor_g.z);
  glVertex3d(proboscis_g.x, proboscis_g.y, proboscis_g.z);
  glEnd();
}
