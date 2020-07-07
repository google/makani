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

#include "vis/vis_perch.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <math.h>
#include <stdint.h>

#include "common/c_math/geometry.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/perch_frame.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "sim/sim_types.h"
#include "vis/vis_util.h"

// Draws the entire perch system, including winch drum, perch panels,
// GSG location, and levelwind location.
void DrawPerch() {
  // Draw the winch drum in ground coordinates because the simulator
  // telemetry returns coordinates in the ground frame.
  DrawWinchDrum(GetSystemParams(), GetSimParams());

  glPushMatrix();

  glRotated(sim_telem.perch.theta_p * 180.0 / PI, 0.0, 0.0, 1.0);
  DrawPerchPanel(&GetSimParams()->perch_sim.panel);

  glPopMatrix();
}

// Draw the tower.
void DrawTower() {
  const double top_radius = 1.6;
  const double bottom_radius = 2.0;
  const double height = GetSystemParams()->ground_frame.ground_z;

  GLUquadric *cyl = gluNewQuadric();
  glColor4d(0.012, 0.356, 0.560, 0.9);
  gluCylinder(cyl, top_radius, bottom_radius, height, 32, 32);
  gluDeleteQuadric(cyl);
}

static void DrawSinglePanel(const PanelSimParams *panel_params,
                            const SinglePanelSimParams *single_panel,
                            const double bookseam_plane[4]) {
  // We begin in the perch/platform frame.
  double clip_z_top[4] = {0.0, 0.0, 1.0, -single_panel->z_extents_p[0]};
  glClipPlane(GL_CLIP_PLANE0, clip_z_top);
  double clip_z_bot[4] = {0.0, 0.0, -1.0, single_panel->z_extents_p[1]};
  glClipPlane(GL_CLIP_PLANE1, clip_z_bot);

  // Transform into the panel frame.
  glPushMatrix();
  const Vec3 *o = &panel_params->origin_pos_p;
  glTranslated(o->x, o->y, o->z);
  double yaw, pitch, roll;
  DcmToAngle(&panel_params->dcm_p2panel, kRotationOrderZyx, &yaw, &pitch,
             &roll);
  glRotated(RadToDeg(yaw), 0.0, 0.0, 1.0);
  glRotated(RadToDeg(pitch), 0.0, 1.0, 0.0);
  glRotated(RadToDeg(roll), 1.0, 0.0, 0.0);

  double clip_x[4] = {1.0, 0.0, 0.0, -single_panel->center_panel.x};
  glClipPlane(GL_CLIP_PLANE2, clip_x);
  double clip_y_low[4] = {0.0, 1.0, 0.0, -panel_params->y_extents_panel[0]};
  glClipPlane(GL_CLIP_PLANE3, clip_y_low);
  double clip_y_high[4] = {0.0, -1.0, 0.0, panel_params->y_extents_panel[1]};
  glClipPlane(GL_CLIP_PLANE4, clip_y_high);

  glClipPlane(GL_CLIP_PLANE5, bookseam_plane);

  glEnable(GL_CLIP_PLANE0);
  glEnable(GL_CLIP_PLANE1);
  glEnable(GL_CLIP_PLANE2);
  glEnable(GL_CLIP_PLANE3);
  glEnable(GL_CLIP_PLANE4);
  glEnable(GL_CLIP_PLANE5);

  Vec3 center = {single_panel->center_panel.x, single_panel->center_panel.y,
                 0.0};
  glColor4d(0.5, 0.5, 0.5, 0.4);
  GLUquadricObj *quad = gluNewQuadric();
  DrawCylinder(quad, &center, single_panel->radius, 10.0, kVisXyPlane);
  gluDeleteQuadric(quad);

  // Disable the clipping planes for everything else.
  glDisable(GL_CLIP_PLANE0);
  glDisable(GL_CLIP_PLANE1);
  glDisable(GL_CLIP_PLANE2);
  glDisable(GL_CLIP_PLANE3);
  glDisable(GL_CLIP_PLANE4);
  glDisable(GL_CLIP_PLANE5);
  glPopMatrix();
}

// Calculates a clipping plane, in panel coordinates, to use for the bookseam --
// the line of contact between the two panels.  The normal vector is oriented
// port-to-starboard, so by default the starboard side will be clipped.
static void CalcBookseamPlane(const PanelSimParams *panel_params,
                              double bookseam_plane[4]) {
  Vec3 normal;
  Vec3 c1 = {panel_params->port.center_panel.x,
             panel_params->port.center_panel.y, 0.0};
  Vec3 c2 = {panel_params->starboard.center_panel.x,
             panel_params->starboard.center_panel.y, 0.0};
  Vec3Sub(&c1, &c2, &normal);

  Vec3 point;
  Vec3LinComb(0.5, &c1, 0.5, &c2, &point);

  bookseam_plane[0] = normal.x;
  bookseam_plane[1] = normal.y;
  bookseam_plane[2] = normal.z;
  bookseam_plane[3] = -Vec3Dot(&normal, &point);
}

// Draw the perch panel, which consists of the union of two
// cylindrical sections each with a specified center and radius and
// are rotated about the perch y-axis by a small angle.  The top of
// these panels is clipped at a set z height.
void DrawPerchPanel(const PanelSimParams *panel_params) {
  DrawCoordSystem(1.0);

  double bookseam_plane[4];
  CalcBookseamPlane(panel_params, bookseam_plane);

  DrawSinglePanel(panel_params, &panel_params->port, bookseam_plane);

  for (int32_t i = 0; i < 4; ++i) {
    bookseam_plane[i] = -bookseam_plane[i];
  }
  DrawSinglePanel(panel_params, &panel_params->starboard, bookseam_plane);
}

// Draw the winch drum.
void DrawWinchDrum(const SystemParams *system_params,
                   const SimParams *sim_params) {
  glColor4d(0.5, 0.5, 0.5, 0.4);
  double cyl_height =
      -system_params->levelwind.drum_angle_to_vertical_travel *
      (system_params->tether.length / system_params->winch.r_drum);

  Vec3 winch_drum_origin_g;
  PToG(&system_params->perch.winch_drum_origin_p, sim_telem.perch.theta_p,
       &winch_drum_origin_g);
  winch_drum_origin_g.z = system_params->perch.gsg_pos_wd.z;

  GLUquadricObj *quad = gluNewQuadric();
  DrawCylinder(quad, &winch_drum_origin_g, system_params->winch.r_drum,
               cyl_height, kVisXyPlane);
  gluDeleteQuadric(quad);

  // Draw GSG point.
  glPointSize(9);
  glColor4d(1.0, 0.0, 0.0, 0.7);
  glBegin(GL_POINTS);
  glVertex3d(sim_telem.perch.gsg_pos_g.x, sim_telem.perch.gsg_pos_g.y,
             sim_telem.perch.gsg_pos_g.z);
  glEnd();

  // Draw levelwind position.
  Vec3 levelwind_hub_p = {sim_params->perch_sim.levelwind_hub_p.x,
                          sim_params->perch_sim.levelwind_hub_p.y,
                          sim_telem.perch.levelwind_pos_g.z};
  Vec3 levelwind_hub_g;
  PToG(&levelwind_hub_p, sim_telem.perch.theta_p, &levelwind_hub_g);
  glColor4d(0.7, 0.7, 0.7, 1.0);
  DrawCircle(&levelwind_hub_g, sim_params->perch_sim.levelwind_radius, &kVec3Z);

  // Draw anchor point.
  glPointSize(9);
  glColor4d(0.7, 0.7, 0.7, 1.0);
  glBegin(GL_POINTS);
  glVertex3d(sim_telem.perch.levelwind_pos_g.x,
             sim_telem.perch.levelwind_pos_g.y,
             sim_telem.perch.levelwind_pos_g.z);
  glEnd();
}
