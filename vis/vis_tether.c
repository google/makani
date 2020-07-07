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

#include "vis/vis_tether.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/mat3.h"
#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"

void DrawTether() {
  const TetherTelemetry *t = &sim_telem.tether;
  const WingTelemetry *w = &sim_telem.wing;
  int32_t num_vis_nodes = t->end_ind - t->start_ind + 1;
  assert(num_vis_nodes <= MAX_TETHER_NODES);

  Vec3 Xg_teth[MAX_TETHER_NODES + 2];
  Xg_teth[0] = t->Xg_start;
  memcpy(&Xg_teth[1], &t->Xg_nodes[t->start_ind],
         (size_t)num_vis_nodes * sizeof(Vec3));
  Xg_teth[num_vis_nodes + 1] = t->Xg_end;

  glPushMatrix();

  // Draw tether segments.
  glEnableClientState(GL_VERTEX_ARRAY);
  glLineWidth(2.0f);
  glColor4f(0.8f, 0.8f, 0.8f, 1.0f);
  glVertexPointer(3, GL_DOUBLE, 0, Xg_teth);
  glDrawArrays(GL_LINE_STRIP, 0, num_vis_nodes + 2);

  // Draw bridle lines.
  if (!t->released) {
    Mat3 dcm_g2b;
    Vec3 Xg_knot = {Xg_teth[num_vis_nodes + 1].x, Xg_teth[num_vis_nodes + 1].y,
                    Xg_teth[num_vis_nodes + 1].z};
    Vec3 Xg_bridle;
    QuatToDcm(&w->q, &dcm_g2b);

    for (int32_t i = 0; i < kNumBridles; ++i) {
      Mat3TransVec3Mult(&dcm_g2b, &g_sys.wing->bridle_pos[i], &Xg_bridle);
      glBegin(GL_LINE_STRIP);
      glVertex3d(Xg_knot.x, Xg_knot.y, Xg_knot.z);
      glVertex3d(w->Xg.x + Xg_bridle.x, w->Xg.y + Xg_bridle.y,
                 w->Xg.z + Xg_bridle.z);
      glEnd();
    }
  }

  // Draw tether nodes.
  glPointSize(4);
  glColor4f(0.8f, 0.8f, 0.8f, 1.0f);
  glDrawArrays(GL_POINTS, 0, num_vis_nodes + 2);
  glDisableClientState(GL_VERTEX_ARRAY);
  glPopMatrix();
}
