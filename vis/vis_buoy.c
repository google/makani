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

#include "vis/vis_buoy.h"
#include <math.h>
#include "common/c_math/util.h"
#include "control/system_types.h"
#include "sim/sim_params.h"
#include "vis/spar_buoy.h"
#include "vis/vis_util.h"

extern bool g_lighting_enabled;
static GLuint spar_buoy_list;

void InitBuoy(void) {
  InitModel(modelSpar_buoyNumVerts, (Vec3 *)modelSpar_buoyVerts,
            &spar_buoy_list);
}

void DrawBuoy(const Vec3 *Xg, const Mat3 *dcm_g2v, bool alternate_color,
              const BuoyParams *buoy_params) {
  GLfloat m[16];

  // Calculate Xg_center_of_mass.
  const Vec3 *Xv_center_of_mass = &buoy_params->center_of_mass_pos;
  Vec3 Xg_center_of_mass;
  InversePoseTransform(dcm_g2v, Xg, Xv_center_of_mass, &Xg_center_of_mass);

  // Obtain buoy geometry.
  const Vec3 v_origin2top_v = {0., 0., buoy_params->top_deck_pos_z_v};

  Vec3 spar_top2cm_v;
  Vec3Sub(Xv_center_of_mass, &v_origin2top_v, &spar_top2cm_v);

  const double tower_height = buoy_params->tower_height;

  // Draw the spar.
  glPushMatrix();

  // Translate center of mass to its position.
  glTranslated(Xg_center_of_mass.x, Xg_center_of_mass.y, Xg_center_of_mass.z);

  // Rotate buoy about the center of mass.
  DcmToGlMatrixf(dcm_g2v, m);
  glMultMatrixf(m);

  // Translate the center of mass to the origin.
  glTranslated(-spar_top2cm_v.x, -spar_top2cm_v.y,
               -spar_top2cm_v.z - tower_height);

  if (alternate_color) {
    glColor4d(0.67, 0.85, 0.075, 0.4);  // Lime green.
  } else {
    glColor4d(0.85, 0.67, 0.075, 0.8);  // Dark orange.
  }

  if (g_lighting_enabled) glEnable(GL_LIGHTING);
  glCallList(spar_buoy_list);
  if (g_lighting_enabled) glDisable(GL_LIGHTING);
  glRotated(180.0, 1.0, 0.0, 0.0);

  glPopMatrix();
}

void TranslateGroundStationToBuoy(const Vec3 *Xg, const Mat3 *dcm_g2v) {
  GLfloat m[16];
  glPushMatrix();

  // Translate ground station to origin of the vessel frame.
  glTranslated(Xg->x, Xg->y, Xg->z);
  DcmToGlMatrixf(dcm_g2v, m);
  glMultMatrixf(m);
}
