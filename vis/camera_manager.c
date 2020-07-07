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

#include "vis/camera_manager.h"

#include <math.h>
#include <stdbool.h>

#include <GL/gl.h>
#include <GL/glu.h>

#include "common/c_math/util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "vis/vis_main.h"

// If true, then the camera focus is always on the wing.
static bool g_follow_wing = false;

// If true, then the camera eyepoint is on the wing and moves with the wing.
static bool g_fly_with_wing = false;

// Fpv eyepoint location and rotation with respect to wing datum and body axes.
// Source:  Fpv mount design documentation.
static const Vec3 kFpvOffset = {.x = 1.060, .y = 0.0, .z = 0.425};

static const Vec3 kFpvRotation = {
    .x = 0.0 * PI / 180.0, .y = -8.0 * PI / 180.0, .z = 0.0 * PI / 180.0};

static const float kFpvFieldOfViewY = 45.0;

CameraPose camera_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Some views are defined below:
// - view_cross: Zoomed out and is useful for watching crosswind flights.
// - view_loop: Centered near the pearch so that the loop looks like a circle.

CameraPose view_cross = {.phi = 180.0 * PI / 180.0,
                         .theta = 5.0 * PI / 180.0,
                         .rho = 900.0,
                         .focus_x = 0.0,
                         .focus_y = -10.0,
                         .focus_z = 0.0,
                         .field_of_view_y = 45.0};
CameraPose view_loop = {.phi = 195.0 * PI / 180.0,
                        .theta = -35.0 * PI / 180.0,
                        .rho = 13.0,
                        .focus_x = -1.0,
                        .focus_y = -9.0,
                        .focus_z = -4.0,
                        .field_of_view_y = 45.0};

CameraPose view_x = {.phi = 0.0 * PI / 180.0,
                     .theta = 0.0 * PI / 180.0,
                     .rho = 500.0,
                     .focus_x = 0.0,
                     .focus_y = 0.0,
                     .focus_z = 0.0,
                     .field_of_view_y = 45.0};

CameraPose view_ch_y = {.phi = 270.0 * PI / 180.0,
                        .theta = 0.0 * PI / 180.0,
                        .rho = 1.0,
                        .focus_x = -436.0,
                        .focus_y = 50.0,
                        .focus_z = -3.0,
                        .field_of_view_y = 45.0};

CameraPose view_y = {.phi = 90.0 * PI / 180.0,
                     .theta = 0.0 * PI / 180.0,
                     .rho = 500.0,
                     .focus_x = 0.0,
                     .focus_y = 0.0,
                     .focus_z = 0.0,
                     .field_of_view_y = 45.0};

CameraPose view_z = {.phi = 0.0 * PI / 180.0,
                     .theta = 90.0 * PI / 180.0,
                     .rho = 500.0,
                     .focus_x = 0.0,
                     .focus_y = 0.0,
                     .focus_z = 0.0,
                     .field_of_view_y = 45.0};

CameraPose view_gs = {.phi = 0.0,
                      .theta = 15.0 * PI / 180.0,
                      .rho = 40.0,
                      .focus_x = 0.0,
                      .focus_y = 0.0,
                      .focus_z = 0.0,
                      .field_of_view_y = 45.0};

// This sets the camera view angle and focus to the parameters specified in
// new_cam.
void SetCamera(const CameraPose *new_cam) {
  SetFollowWing(false);
  SetFlyWithWing(false);
  camera_pose = *new_cam;
}

// Sets view_perch: Zoomed in to the perch and is useful for watching the wing
// take off.
void SetPerchView(void) {
  double phi = 0.0;
  double theta = 0.0;
  double rho = 1.0;
  double focus_x = 0.0;
  double focus_y = 0.0;
  double focus_z = 0.0;
  float field_of_view_y = 45.0;
  switch (GetSystemParams()->test_site) {
    case kTestSiteChinaLake:
      phi = 190.0 * PI / 180.0;
      theta = -10.0 * PI / 180.0;
      rho = 50.0;
      focus_x = -434.0;
      focus_y = 0.0;
      focus_z = 0.0;
      field_of_view_y = 45.0;
      break;
    case kTestSiteAlameda:
    case kTestSiteParkerRanch:
      phi = 160.0 * PI / 180.0;
      theta = -10.0 * PI / 180.0;
      rho = 40.0;
      focus_x = 0.0;
      focus_y = 0.0;
      focus_z = -1.0;
      field_of_view_y = 45.0;
      break;
    case kTestSiteNorway:
    default:
      phi = (GetSimParams()->phys_sim.wind_direction -
             GetSystemParams()->ground_frame.heading - PI - 15.0 * PI / 180.0);
      theta = -10.0 * PI / 180.0;
      rho = 50.0;
      focus_x = 0.0;
      focus_y = 0.0;
      focus_z = -1.0;
      field_of_view_y = 45.0;
      break;
    case kTestSiteForceSigned:
    case kNumTestSites:
      assert(false);
  }
  CameraPose view_perch = {.phi = phi,
                           .theta = theta,
                           .rho = rho,
                           .focus_x = focus_x,
                           .focus_y = focus_y,
                           .focus_z = focus_z,
                           .field_of_view_y = field_of_view_y};
  SetCamera(&view_perch);
}

// Mimics view from command center at test site from an occupant's POV
// (pivoting about reference eyepoint) with narrow field of view for
// launch/land.
void SetCommandCenterNarrowView(void) {
  double phi = 0.0;
  double theta = 0.0;
  double rho = 1.0;
  double focus_x = 0.0;
  double focus_y = 0.0;
  double focus_z = 0.0;
  float field_of_view_y = 45.0;
  double msl_pos_z_g = 0.0;
  double buoy_azi_g = 0.0;
  double barge_azi_g = 0.0;
  double barge_azi_min = 0.0;
  double wind_direction_g = 0.0;
  double r_barge_to_buoy = 1.0;
  double ground_frame_heading = GetSystemParams()->ground_frame.heading;
  switch (GetSystemParams()->test_site) {
    case kTestSiteChinaLake:
      // Perch-relative measurements (505m W, 202m S) on 28 July 2016.
      phi = 158.0 * PI / 180.0;
      theta = 0.0 * PI / 180.0;
      rho = 1.0;
      focus_x = 69.0;
      focus_y = -202.0;
      focus_z = 0.0;
      field_of_view_y = 45.0;
      break;
    case kTestSiteNorway:
      msl_pos_z_g = GetSimParams()->buoy_sim.msl_pos_z_g;
      buoy_azi_g =
          Wrap(GetSimParams()->buoy_sim.mooring_lines.yaw_equilibrium_heading -
                   ground_frame_heading,
               0.0, 2.0 * PI);
      wind_direction_g =
          Wrap(GetSimParams()->phys_sim.wind_direction - ground_frame_heading,
               0.0, 2.0 * PI);
      // Approximate distance from the command center on the barge to the buoy.
      r_barge_to_buoy = 300.0;
      barge_azi_min = Wrap(buoy_azi_g - PI, 0.0, 2.0 * PI);
      barge_azi_g = SaturateWrapped(wind_direction_g, barge_azi_min, buoy_azi_g,
                                    0.0, 2.0 * PI);
      phi = Wrap((barge_azi_g - PI), 0.0, 2.0 * PI);
      theta = 0.0 * PI / 180.0;
      rho = 1.0;
      focus_x = r_barge_to_buoy * cos(barge_azi_g);
      focus_y = r_barge_to_buoy * sin(barge_azi_g);
      // The command center viewpoint is approximately 6 m above the sea.
      focus_z = msl_pos_z_g - 6.0;
      field_of_view_y = 45.0;
      break;
    case kTestSiteAlameda:
    case kTestSiteParkerRanch:
      // The command center is 330 m NE (45 deg) from the tower.
      phi = 225.0 * PI / 180.0;
      theta = 0.0 * PI / 180.0;
      rho = 0.5;
      focus_x = 233.0;
      focus_y = 233.0;
      focus_z = 5.5;
      field_of_view_y = 45.0;
      break;
    default:
    case kTestSiteForceSigned:
    case kNumTestSites:
      assert(false);
  }
  CameraPose view_cc_narrow_fov = {.phi = phi,
                                   .theta = theta,
                                   .rho = rho,
                                   .focus_x = focus_x,
                                   .focus_y = focus_y,
                                   .focus_z = focus_z,
                                   .field_of_view_y = field_of_view_y};

  SetCamera(&view_cc_narrow_fov);
}

// Mimics view from command center at the test site in the
// system parameters with a wide field of view for crosswind.
void SetCommandCenterWideView(void) {
  double phi = 0.0;
  double theta = 0.0;
  double rho = 1.0;
  double focus_x = 0.0;
  double focus_y = 0.0;
  double focus_z = 0.0;
  float field_of_view_y = 45.0;
  double msl_pos_z_g = 0.0;
  double buoy_azi_g = 0.0;
  double barge_azi_g = 0.0;
  double barge_azi_min = 0.0;
  double wind_direction_g = 0.0;
  double r_barge_to_buoy = 1.0;
  double ground_frame_heading = GetSystemParams()->ground_frame.heading;
  switch (GetSystemParams()->test_site) {
    case kTestSiteChinaLake:
      // Perch-relative measurements (505m W, 202m S) on 28 July 2016.
      phi = 148.0 * PI / 180.0;
      theta = -30.0 * PI / 180.0;
      rho = 1.0;
      focus_x = 69.0;
      focus_y = -202.0;
      focus_z = 0.0;
      field_of_view_y = 70.0;
      break;
    case kTestSiteNorway:
      msl_pos_z_g = GetSimParams()->buoy_sim.msl_pos_z_g;
      buoy_azi_g =
          Wrap(GetSimParams()->buoy_sim.mooring_lines.yaw_equilibrium_heading -
                   ground_frame_heading,
               0.0, 2.0 * PI);
      wind_direction_g =
          Wrap(GetSimParams()->phys_sim.wind_direction - ground_frame_heading,
               0.0, 2.0 * PI);
      // Approximate distance from the command center on the barge to the buoy.
      r_barge_to_buoy = 300.0;
      barge_azi_min = Wrap(buoy_azi_g - PI, 0.0, 2.0 * PI);
      barge_azi_g = SaturateWrapped(wind_direction_g, barge_azi_min, buoy_azi_g,
                                    0.0, 2.0 * PI);
      // Currently does not focus on crosswind loops if barge azi is saturated.
      // Must manually rotate.
      phi = Wrap(barge_azi_g - PI, 0.0, 2.0 * PI);
      theta = -30.0 * PI / 180.0;
      rho = 1.0;
      focus_x = r_barge_to_buoy * cos(barge_azi_g);
      focus_y = r_barge_to_buoy * sin(barge_azi_g);
      // The command center viewpoint is approximately 6 m above the sea.
      focus_z = msl_pos_z_g - 6.0;
      field_of_view_y = 70.0;
      break;
    case kTestSiteAlameda:
    case kTestSiteParkerRanch:
      // The command center is 330 m NE (45 deg) from the tower.
      phi = 225.0 * PI / 180.0;
      theta = -30.0 * PI / 180.0;
      rho = 0.5;
      focus_x = 233.0;
      focus_y = 233.0;
      focus_z = 5.5;
      field_of_view_y = 70.0;
      break;
    default:
    case kTestSiteForceSigned:
    case kNumTestSites:
      assert(false);
  }
  CameraPose view_cc_wide_fov = {.phi = phi,
                                 .theta = theta,
                                 .rho = rho,
                                 .focus_x = focus_x,
                                 .focus_y = focus_y,
                                 .focus_z = focus_z,
                                 .field_of_view_y = field_of_view_y};
  SetCamera(&view_cc_wide_fov);
}

// The tether view is a profile view of the tether and crosswind loop.
// This depends on wind_direction in the sim parameters, so defining this as a
// function.
void SetTetherView(void) {
  const double wind_direction_g = GetSimParams()->phys_sim.wind_direction -
                                  GetSystemParams()->ground_frame.heading;
  CameraPose view_tether = {.phi = wind_direction_g + PI / 2.0,
                            .theta = -20.0 * PI / 180.0,
                            .rho = 500.0,
                            .focus_x = -250.0,
                            .focus_y = 0.0,
                            .focus_z = -170.0,
                            .field_of_view_y = 45.0};
  SetCamera(&view_tether);
}

void SetFollowWing(bool follow_wing) { g_follow_wing = follow_wing; }

void SetFlyWithWing(bool fly_with_wing) { g_fly_with_wing = fly_with_wing; }

void UpdateCamera(const Vec3 *Xg, const Vec3 *eulers) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();  // Reset the projection matrix.

  gluPerspective(camera_pose.field_of_view_y,
                 (GLfloat)window_width / (GLfloat)window_height, 0.1f,
                 100000.0f);  // Calculate aspect ratio of the window.

  if (g_follow_wing) {
    camera_pose.focus_x = Xg->x;
    camera_pose.focus_y = Xg->y;
    camera_pose.focus_z = Xg->z;
  }
  if (g_fly_with_wing) {
    // Reset FOV to fpv camera value.
    camera_pose.field_of_view_y = kFpvFieldOfViewY;

    // Offset to correct axes reference frame.
    glRotated(90.0, 0.0, 1.0, 0.0);

    // Rotate the FPV camera mount.
    glRotated(-kFpvRotation.x * 180.0 / PI, 1.0, 0.0, 0.0);  // Roll
    glRotated(-kFpvRotation.y * 180.0 / PI, 0.0, 0.0, 1.0);  // Pitch
    glRotated(kFpvRotation.z * 180.0 / PI, 0.0, 1.0, 0.0);   // Yaw

    // Translate to FPV location on wing.
    glTranslated(-kFpvOffset.x, kFpvOffset.z, -kFpvOffset.y);

    // Fly with the wing.
    glRotated(-eulers->x * 180.0 / PI, 1.0, 0.0, 0.0);
    glRotated(-eulers->y * 180.0 / PI, 0.0, 0.0, 1.0);
    glRotated(eulers->z * 180.0 / PI, 0.0, 1.0, 0.0);
    glTranslated(-Xg->x, Xg->z, -Xg->y);
  } else {
    double camera_x =
        -cos(camera_pose.phi) * cos(camera_pose.theta) * camera_pose.rho +
        camera_pose.focus_x;
    double camera_y =
        -sin(camera_pose.phi) * cos(camera_pose.theta) * camera_pose.rho +
        camera_pose.focus_y;
    double camera_z =
        -sin(camera_pose.theta) * camera_pose.rho + camera_pose.focus_z;

    gluLookAt(camera_x, -camera_z, camera_y, camera_pose.focus_x,
              -camera_pose.focus_z, camera_pose.focus_y, 0, 1, 0);
  }

  glMatrixMode(GL_MODELVIEW);
}
