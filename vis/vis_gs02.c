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

#include "vis/vis_gs02.h"

#include <math.h>

#include "control/system_params.h"
#include "sim/sim_params.h"
#include "vis/gs02_detwist.h"
#include "vis/gs02_drum.h"
#include "vis/gs02_levelwind_arm.h"
#include "vis/gs02_levelwind_cassette.h"
#include "vis/gs02_platform.h"
#include "vis/gs02_termination.h"
#include "vis/gs02_yoke.h"
#include "vis/vis_util.h"

extern bool g_lighting_enabled;
static GLuint gs02_detwist_list;
static GLuint gs02_drum_list;
static GLuint gs02_levelwind_arm_list;
static GLuint gs02_levelwind_cassette_list;
static GLuint gs02_platform_list;
static GLuint gs02_termination_list;
static GLuint gs02_yoke_list;

void InitGs02(void) {
  // Check our assumptions about data types.
  assert(sizeof(modelGs02_detwistVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_detwistVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_detwistVerts) / sizeof(modelGs02_detwistVerts[0]) ==
         modelGs02_detwistNumVerts * 3);

  assert(sizeof(modelGs02_drumVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_drumVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_drumVerts) / sizeof(modelGs02_drumVerts[0]) ==
         modelGs02_drumNumVerts * 3);

  assert(sizeof(modelGs02_levelwind_armVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_levelwind_armVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_levelwind_armVerts) /
             sizeof(modelGs02_levelwind_armVerts[0]) ==
         modelGs02_levelwind_armNumVerts * 3);

  assert(sizeof(modelGs02_levelwind_cassetteVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_levelwind_cassetteVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_levelwind_cassetteVerts) /
             sizeof(modelGs02_levelwind_cassetteVerts[0]) ==
         modelGs02_levelwind_cassetteNumVerts * 3);

  assert(sizeof(modelGs02_platformVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_platformVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_platformVerts) / sizeof(modelGs02_platformVerts[0]) ==
         modelGs02_platformNumVerts * 3);

  assert(sizeof(modelGs02_terminationVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_terminationVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_terminationVerts) /
             sizeof(modelGs02_terminationVerts[0]) ==
         modelGs02_terminationNumVerts * 3);

  assert(sizeof(modelGs02_yokeVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_yokeVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_yokeVerts) / sizeof(modelGs02_yokeVerts[0]) ==
         modelGs02_yokeNumVerts * 3);

  InitModel(modelGs02_detwistNumVerts, (Vec3 *)modelGs02_detwistVerts,
            &gs02_detwist_list);
  InitModel(modelGs02_drumNumVerts, (Vec3 *)modelGs02_drumVerts,
            &gs02_drum_list);
  InitModel(modelGs02_levelwind_armNumVerts,
            (Vec3 *)modelGs02_levelwind_armVerts, &gs02_levelwind_arm_list);
  InitModel(modelGs02_levelwind_cassetteNumVerts,
            (Vec3 *)modelGs02_levelwind_cassetteVerts,
            &gs02_levelwind_cassette_list);
  InitModel(modelGs02_platformNumVerts, (Vec3 *)modelGs02_platformVerts,
            &gs02_platform_list);
  InitModel(modelGs02_terminationNumVerts, (Vec3 *)modelGs02_terminationVerts,
            &gs02_termination_list);
  InitModel(modelGs02_yokeNumVerts, (Vec3 *)modelGs02_yokeVerts,
            &gs02_yoke_list);
}

void DrawGs02(double platform_azi, double drum_angle, double levelwind_ele,
              double detwist_angle, double gsg_yoke, double gsg_termination) {
  const GroundFrameParams ground_frame_params = GetSystemParams()->ground_frame;
  const GroundStationParams gs_params = GetSystemParams()->ground_station;
  const Gs02Params gs02_params = gs_params.gs02;
  const BuoySimParams buoy_sim_params = GetSimParams()->buoy_sim;

  glPushMatrix();

  if (!(GetSystemParams()->offshore)) {
    // Go back to NED and apply vessel heading offset.
    glRotated(-RadToDeg(ground_frame_params.heading), 0.0, 0.0, 1.0);
    glRotated(RadToDeg(buoy_sim_params.mooring_lines.yaw_equilibrium_heading),
              0.0, 0.0, 1.0);
  }

  // Apply GS02 platform azimuth.
  glRotated(RadToDeg(platform_azi), 0.0, 0.0, 1.0);

  {
    glPushMatrix();

    // Draw the GS rotating frame and perch panel.
    glColor4d(1.0, 1.0, 1.0, 0.3);
    if (g_lighting_enabled) glEnable(GL_LIGHTING);
    glCallList(gs02_platform_list);
    if (g_lighting_enabled) glDisable(GL_LIGHTING);

    // Compute the levelwind shuttle position offset in meters.
    const double *cam_drum_angle_rad = gs02_params.caming_table_drum_angle_rad;
    double l_offset_x_m = 0.001 * gs02_params.caming_table_min_offset_mm;
    if (drum_angle <= cam_drum_angle_rad[0]) {
      // Follow low pitch grooves caming
      const double *coeff = gs02_params.caming_table_low_pitch_mm;
      l_offset_x_m = 0.001 * PolyVal(coeff, drum_angle, 2);
    } else if (drum_angle <= cam_drum_angle_rad[1]) {
      // Follow high pitch grooves caming
      const double *coeff = gs02_params.caming_table_high_pitch_mm;
      l_offset_x_m = 0.001 * PolyVal(coeff, drum_angle, 2);
    }

    // Translate to the levelwind frame
    const Vec3 l = gs02_params.levelwind_origin_p;
    glTranslated(l.x + l_offset_x_m, l.y, l.z);

    // Rotate to the levelwind frame
    const double *l2s = gs02_params.levelwind_ele_to_shoulder;
    double levelwind_shoulder = PolyVal(l2s, levelwind_ele, 2);
    glRotated(RadToDeg(levelwind_shoulder), 1.0, 0.0, 0.0);

    // Draw the levelwind frame
    glColor4d(1.0, 1.0, 1.0, 0.8);
    if (g_lighting_enabled) glEnable(GL_LIGHTING);
    glCallList(gs02_levelwind_arm_list);
    if (g_lighting_enabled) glDisable(GL_LIGHTING);

    // Translate to the levelwind cassette
    const Vec3 c = gs02_params.cassette_origin_l;
    glTranslated(c.x, c.y, c.z);

    // Rotate to the levelwind cassette
    const double *l2c = gs02_params.levelwind_ele_to_wrist;
    double levelwind_wrist = PolyVal(l2c, levelwind_ele, 2);
    glRotated(RadToDeg(levelwind_wrist), 1.0, 0.0, 0.0);

    // Draw the levelwind cassette
    glColor4d(1.0, 1.0, 1.0, 0.8);
    if (g_lighting_enabled) glEnable(GL_LIGHTING);
    glCallList(gs02_levelwind_cassette_list);
    if (g_lighting_enabled) glDisable(GL_LIGHTING);

    // Translate and rotate back to the drum frame.
    glRotated(RadToDeg(-levelwind_wrist), 1.0, 0.0, 0.0);
    glTranslated(-c.x, -c.y, -c.z);
    glRotated(RadToDeg(-levelwind_shoulder), 1.0, 0.0, 0.0);
    glTranslated(-l.x - l_offset_x_m, -l.y, -l.z);
    const Vec3 d = gs02_params.drum_origin_p;
    glTranslated(d.x, d.y, d.z);
    glRotated(RadToDeg(drum_angle), 1.0, 0.0, 0.0);

    // Draw the drum.
    glColor4d(1.0, 1.0, 1.0, 0.3);
    if (g_lighting_enabled) glEnable(GL_LIGHTING);
    glCallList(gs02_drum_list);
    if (g_lighting_enabled) glDisable(GL_LIGHTING);

    // Translate to the gsg positions
    const Vec3 gsg = gs02_params.gsg_pos_drum;
    glTranslated(gsg.x, gsg.y, gsg.z);

    // Rotate to the detwist angle
    const double detwist_elevation = gs02_params.detwist_elevation;
    glRotated(RadToDeg(-detwist_elevation), 0.0, 1.0, 0.0);
    glRotated(RadToDeg(detwist_angle), 1.0, 0.0, 0.0);

    // Draw the detwist
    glColor4d(0.8, 0.8, 0.8, 0.8);
    if (g_lighting_enabled) glEnable(GL_LIGHTING);
    glCallList(gs02_detwist_list);
    if (g_lighting_enabled) glDisable(GL_LIGHTING);

    // Rotate to the yoke
    if (gsg_yoke != 0.0) {
      glRotated(RadToDeg(gsg_yoke), 0.0, 0.0, 1.0);
    } else {
      // Use default value if sim telemetry is not updated. We know this happens
      // in reel mode in the sim.
      glRotated(RadToDeg(gs02_params.gsg_yoke_angle_in_reel_rad), 0.0, 0.0,
                1.0);
    }

    // Draw the yoke
    glColor4d(0.609, 0.391, 0.047, 0.8);
    if (g_lighting_enabled) glEnable(GL_LIGHTING);
    glCallList(gs02_yoke_list);
    if (g_lighting_enabled) glDisable(GL_LIGHTING);

    // Rotate to the termination
    if (gsg_termination != 0.0) {
      glRotated(RadToDeg(-gsg_termination), 0.0, 1.0, 0.0);
    } else {
      // Use default value if sim telemetry is not updated. We know this happens
      // in reel mode in the sim.
      glRotated(RadToDeg(-gs02_params.gsg_termination_angle_in_reel_rad), 0.0,
                1.0, 0.0);
    }

    // Draw the termination
    glColor4d(0.8, 0.8, 0.8, 0.8);
    if (g_lighting_enabled) glEnable(GL_LIGHTING);
    glCallList(gs02_termination_list);
    if (g_lighting_enabled) glDisable(GL_LIGHTING);

    glPopMatrix();
  }
}
