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

#include "vis/vis_wing.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>

#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"
#include "vis/m600_model.h"
#include "vis/oktoberkite_model.h"
#include "vis/vis_util.h"

extern bool g_lighting_enabled;
static GLuint winglist;

void InitWing(void) {
  // Check our assumptions about data types.
  assert(sizeof(modelM600Verts[0]) == sizeof(double));
  assert(sizeof(modelM600Verts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelM600Verts) / sizeof(modelM600Verts[0]) ==
         modelM600NumVerts * 3);

  assert(sizeof(modelOktoberkiteVerts[0]) == sizeof(double));
  assert(sizeof(modelOktoberkiteVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelOktoberkiteVerts) / sizeof(modelOktoberkiteVerts[0]) ==
         modelOktoberkiteNumVerts * 3);

  int num_vertices;
  Vec3 *vertices;

  // Set input data to arrays and draw them.
  if (GetSystemParams()->wing_model == kWingModelYm600) {
    num_vertices = modelM600NumVerts;
    vertices = (Vec3 *)modelM600Verts;
  } else if (GetSystemParams()->wing_model == kWingModelOktoberKite) {
    num_vertices = modelOktoberkiteNumVerts;
    vertices = (Vec3 *)modelOktoberkiteVerts;
  } else {
    assert(!"Unsupport wing serial.");
    num_vertices = 0;
    vertices = NULL;
  }

  // Calculate the normals and compile a "display list" used to draw the wing
  // later.
  InitModel(num_vertices, vertices, &winglist);
}

void DrawWing(const Vec3 *Xg, const Vec3 *eulers) {
  const SimParams *sim_params = GetSimParams();
  Mat3 dcm;
  GLfloat m[16];

  AngleToDcm(eulers->z, eulers->y, eulers->x, kRotationOrderZyx, &dcm);
  DcmToGlMatrixf(&dcm, m);

  glPushMatrix();

  glTranslated(Xg->x, Xg->y, Xg->z);
  glMultMatrixf(m);

  // Now in body coordinates.
  DrawCoordSystem(1.0);

  // Draw the contactors if we are close to the perch.
  if (Vec3Norm(Xg) < 0.1 * GetSystemParams()->tether.length) {
    for (int32_t i = 0; i < kNumPerchContactors; ++i) {
      DrawPoint(&sim_params->contact_sim.perch_contactors[i].pos);
    }
  }

  // Draw the proboscis if under constraints.
  if (sim_params->sim_opt & kSimOptConstraintSystem) {
    DrawPoint(&GetSystemParams()->wing.proboscis_pos);
  }

  // Draw the wing itself.
  glColor4d(1.0, 1.0, 1.0, 0.8);

  if (g_lighting_enabled) glEnable(GL_LIGHTING);
  glCallList(winglist);
  if (g_lighting_enabled) glDisable(GL_LIGHTING);
  glPopMatrix();
}
