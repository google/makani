// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "vis/vis_main.h"

#include <math.h>

#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <list>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/tether_message.h"
#include "avionics/linux/aio.h"
#include "avionics/linux/clock.h"
#include "avionics/network/aio_node.h"
#include "avionics/network/message_type.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/cvt_control_telemetry.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/monitor/monitor_params.h"
#include "lib/json_load/load_params.h"
#include "sim/cvt_sim_telemetry.h"
#include "sim/sim_params.h"
#include "sim/sim_telemetry.h"
#include "vis/camera_manager.h"
#include "vis/gs02_tower.h"
#include "vis/keyboard_mouse.h"
#include "vis/vis_buoy.h"
#include "vis/vis_constraint.h"
#include "vis/vis_gs02.h"
#include "vis/vis_perch.h"
#include "vis/vis_sea.h"
#include "vis/vis_tether.h"
#include "vis/vis_util.h"
#include "vis/vis_wing.h"

DEFINE_bool(show_cues, true,
            "Show visual cues such as the flight path and flight mode.");
DEFINE_bool(show_breadcrumbs, true, "Show a trace of flight positions.");
DEFINE_bool(show_landing_zone, true, "Show landing zone.");
DEFINE_bool(use_control_telemetry, false,
            "Use control telemetry instead of simulator telemetry.");
DEFINE_bool(use_control_debug, false,
            "Use control debug instead of simulator telemetry.");
DEFINE_bool(use_tether_down, false,
            "Use TetherDownMessage instead of simulator telemetry.");
DEFINE_bool(show_phantom_buoy, false,
            "Display a phantom buoy based on its estimated state.");
DEFINE_bool(lighting, false, "Enable OpenGL lighting (3D shading).");

int window;
int window_width = 640;
int window_height = 480;
extern bool g_lighting_enabled;
static GLuint gs02_tower_list;

typedef enum {
  kDataSourceSimulatorTelemetry,
  kDataSourceControlTelemetry,
  kDataSourceControlDebug,
  kDataSourceTetherDown
} DataSource;

DataSource g_data_source;
TetherDownMergeState g_tether_down_merge_state;
const int g_trace_len = 100;
std::list<Vec3> g_fly_trace;
bool g_lighting_enabled;

void DrawPoints(const std::list<Vec3> &points) {
  glPointSize(2.0f);
  glColor4d(0.0, 0.5, 1.0, 0.7);
  glBegin(GL_POINTS);
  for (auto i : points) {
    const Vec3 &point = i;
    glVertex3d(point.x, point.y, point.z);
  }
  glEnd();
}

// A general OpenGL initialization function.  Sets all of the initial
// parameters.We call this right after our OpenGL window is created.
void InitGL() {
  // Clear background color to black.
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

  // Enables clearing of the depth buffer.
  glClearDepth(1.0);
  glDepthFunc(GL_LESS);     // Type of depth test to do.
  glEnable(GL_DEPTH_TEST);  // Enables depth testing.
  glShadeModel(GL_SMOOTH);  // Enables smooth color shading.

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  glPointSize(3.0);
  glEnable(GL_POINT_SMOOTH);
  glEnable(GL_LINE_SMOOTH);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);  // Required for lighting.

  {
    GLfloat gl_ones[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, gl_ones);
  }

  // Light positions are given by 4-dimensional homogeneous
  // coordinates; a final coordinate of zero indicates a light at
  // infinity.
  {
    glEnable(GL_LIGHT0);
    GLfloat position[] = {0.0, 0.0, -1.0, 0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, position);
  }

  {
    glEnable(GL_LIGHT1);
    GLfloat position[] = {0.0, 0.0, 1.0, 0.0};
    glLightfv(GL_LIGHT1, GL_POSITION, position);

    GLfloat ones[] = {1.0, 1.0, 1.0, 1.0};
    glLightfv(GL_LIGHT1, GL_DIFFUSE, ones);
    glLightfv(GL_LIGHT1, GL_SPECULAR, ones);
  }

  if (GetSystemParams()->gs_model == kGroundStationModelGSv2) {
    SetCamera(&view_gs);
  } else {
    SetPerchView();
  }

  GLenum err;
  while ((err = glGetError()) != GL_NO_ERROR) {
    std::cout << "GL error " << err << std::endl;
  }
}

static void ReshapeWindow(const int width, const int height) {
  window_width = width;
  window_height = height;
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);
}

void InitGlut(int argc, char **argv) {
  glutInit(&argc, argv);

  // Select type of Display mode:
  //   Double buffer
  //   RGBA color
  //   Alpha components supported
  //   Depth buffered for automatic clipping
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);

  // Get a window_width x window_height window.
  glutInitWindowSize(window_width, window_height);

  // Window starts at the upper left corner of the screen.
  glutInitWindowPosition(0, 0);

  // Open a window.
  window = glutCreateWindow("Makani Visualizer v2.0");

  // Register the function to do all our OpenGL drawing.
  glutDisplayFunc(&DrawGLScene);

  // Go full screen.  This is as soon as possible.
  //  glutFullScreen();

  // Even if there are no events, redraw our gl scene.
  // note: should probably take the frame rate function and put it here,
  // instead of in DrawGlScene
  glutIdleFunc(&DrawGLScene);

  // Register the function called when our window is resized.
  glutReshapeFunc(&ReshapeWindow);

  // Register the function called when the keyboard/mouse is pressed.
  glutMouseFunc(&MouseClicked);
  glutKeyboardFunc(&KeyPressed);
  glutSpecialFunc(&ArrowKeyPressed);
  glutMotionFunc(&MouseMoved);
}

static void DrawGrid(void) {
  glPushMatrix();

  const float spacing_major = 100.0f;
  const float spacing_minor = 10.0f;
  const float radius = 2000.0f;
  const float ground_z =
      static_cast<float>(GetSystemParams()->ground_frame.ground_z);

  DrawCoordSystem(1.0);

  // Draw major grid lines.
  glLineWidth(1.5f);
  glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
  for (float x = -radius; x <= radius; x += spacing_major) {
    // Lines which run east/west.
    glBegin(GL_LINE_STRIP);
    glVertex3f(x, -radius, ground_z);
    glVertex3f(x, radius, ground_z);
    glEnd();
    // Lines which run north/south.
    glBegin(GL_LINE_STRIP);
    glVertex3f(-radius, x, ground_z);
    glVertex3f(radius, x, ground_z);
    glEnd();
  }

  // Draw minor grid lines.
  glLineWidth(1.5f);
  glColor4f(0.5f, 0.5f, 0.5f, 0.1f);
  for (float x = -radius; x <= radius; x += spacing_minor) {
    // Lines which run east/west.
    glBegin(GL_LINE_STRIP);
    glVertex3f(x, -radius, ground_z);
    glVertex3f(x, radius, ground_z);
    glEnd();
    // Lines which run north/south.
    glBegin(GL_LINE_STRIP);
    glVertex3f(-radius, x, ground_z);
    glVertex3f(radius, x, ground_z);
    glEnd();
  }

  glPopMatrix();
}

static void DrawCrosswindPath(FlightMode flight_mode,
                              const CrosswindTelemetry *cwt) {
  if (AnyCrosswindFlightMode(flight_mode)) {
    glLineWidth(10.0f);
    glColor4d(1.0, 1.0, 1.0, 0.1);

    DrawCircle(&cwt->path_center_g, cwt->path_radius_target,
               &cwt->path_center_g);
  }
}

static void DrawText(const char *text) {
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  glColor4d(1.0, 1.0, 1.0, 0.5);
  glRasterPos2d(-1.0, 0.95);
  glutBitmapString(GLUT_BITMAP_8_BY_13, (const unsigned char *)text);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

static void DrawTransInPath(FlightMode flight_mode,
                            const TransInTelemetry *trans_in) {
  if (flight_mode == kFlightModeTransIn) {
    glLineWidth(10.0f);
    glColor4d(1.0, 1.0, 1.0, 0.1);

    Vec3 normal = kVec3Zero;
    normal.x = -sin(trans_in->ti_origin_azimuth);
    normal.y = cos(trans_in->ti_origin_azimuth);
    DrawCircle(&kVec3Zero, GetSystemParams()->tether.length +
                               GetSystemParams()->wing.bridle_rad,
               &normal);
  }
}

static void DrawLandingZone(void) {
  glPushMatrix();

  // Draw the landing zone slightly above ground, so it isn't obscured by grid
  // lines.
  glTranslated(0.0, 0.0, GetSystemParams()->ground_frame.ground_z - 0.1);
  glColor4d(0.5, 0.5, 0.5, 1.0);

  const MonLandingZoneParams *params = &GetMonitorParams()->landing_zone;
  if (GetSystemParams()->test_site == kTestSiteChinaLake) {
    glBegin(GL_POLYGON);
    for (int32_t i = 0; i < ARRAYSIZE(params->primary_vertices); ++i) {
      glVertex2d(params->primary_vertices[i].x, params->primary_vertices[i].y);
    }
    glEnd();

    glBegin(GL_POLYGON);
    for (int32_t i = 0; i < ARRAYSIZE(params->secondary_vertices); ++i) {
      glVertex2d(params->secondary_vertices[i].x,
                 params->secondary_vertices[i].y);
    }
    glEnd();
  } else if (GetSystemParams()->test_site == kTestSiteParkerRanch) {
    glBegin(GL_POLYGON);
    for (int32_t i = 0; i < ARRAYSIZE(params->pr_pad_vertices); ++i) {
      glVertex2d(params->pr_pad_vertices[i].x, params->pr_pad_vertices[i].y);
    }
    glEnd();
  } else if (GetSystemParams()->test_site == kTestSiteNorway) {
    glRotated(-RadToDeg(GetSystemParams()->ground_frame.heading), 0.0, 0.0,
              1.0);
    // Draw North line
    glBegin(GL_POLYGON);
    glVertex2d(0.0, 0.0);
    glVertex2d(100.0, 10.0);
    glVertex2d(350.0, 0.0);
    glVertex2d(100.0, -10.0);
    glEnd();
    // Draw barge allowable zone.
    const double buoy_azi =
        (GetSimParams()->buoy_sim.mooring_lines.yaw_equilibrium_heading);
    const double r_max = 310.0;
    const double arc_n = 12;
    double azi_angle = 0.0;
    glBegin(GL_POLYGON);
    for (int32_t i = 0; i <= arc_n; ++i) {
      azi_angle = buoy_azi - i * PI / arc_n;
      glVertex2d(r_max * cos(azi_angle), r_max * sin(azi_angle));
    }
    glEnd();
  }
  glPopMatrix();
}

static void HandleTetherDownMessage(const TetherDownMessage *tether_down,
                                    double *time, FlightMode *flight_mode,
                                    Vec3 *Xg, Vec3 *eulers) {
  *time = tether_down->control_telemetry.flight_mode_time * 0.10;
  *flight_mode =
      static_cast<FlightMode>(tether_down->control_telemetry.flight_mode);
  Xg->x = tether_down->control_telemetry.pos_g[0];
  Xg->y = tether_down->control_telemetry.pos_g[1];
  Xg->z = tether_down->control_telemetry.pos_g[2];
  eulers->x = tether_down->control_telemetry.roll;
  eulers->y = tether_down->control_telemetry.pitch;
  eulers->z = tether_down->control_telemetry.yaw;
}

static void HandleControlTelemetry(const ControlTelemetry *control_telemetry,
                                   double *time, FlightMode *flight_mode,
                                   Vec3 *Xg, Vec3 *eulers,
                                   double *platform_azi) {
  *time = control_telemetry->time;
  *flight_mode = static_cast<FlightMode>(control_telemetry->flight_mode);
  *Xg = control_telemetry->state_est.Xg;
  DcmToAngle(&control_telemetry->state_est.dcm_g2b, kRotationOrderZyx,
             &eulers->z, &eulers->y, &eulers->x);
  *platform_azi = control_telemetry->state_est.perch_azi.angle;
}

static void HandleSimTelemetry(const SimTelemetry *sim_telemetry,
                               const ControlTelemetry *control_telemetry,
                               double *time, FlightMode *flight_mode, Vec3 *Xg,
                               Vec3 *eulers, double *platform_azi) {
  *time = sim_telemetry->time;
  *flight_mode = static_cast<FlightMode>(control_telemetry->flight_mode);
  *Xg = sim_telemetry->wing.Xg;
  *eulers = sim_telemetry->wing.eulers;
  *platform_azi = sim_telemetry->gs02.azimuth;
}

static void UpdateControlTelemetry() {
  for (int32_t i = 0; static_cast<ControllerLabel>(i) < kNumControllers; ++i) {
    AioNode controller_node =
        ControllerLabelToControllerAioNode(static_cast<ControllerLabel>(i));
    if (CvtGetControlTelemetry(controller_node, GetControlTelemetry(), nullptr,
                               nullptr)) {
      return;
    }
  }
}

static void UpdateControlDebug() {
  for (int32_t i = 0; static_cast<ControllerLabel>(i) < kNumControllers; ++i) {
    AioNode controller_node =
        ControllerLabelToControllerAioNode(static_cast<ControllerLabel>(i));
    if (CvtGetControlDebugMessage(controller_node, GetControlTelemetry(),
                                  nullptr, nullptr)) {
      return;
    }
  }
}

static void ParseTelemetryData(double *time, FlightMode *flight_mode, Vec3 *Xg,
                               Vec3 *eulers, double *platform_azi) {
  switch (g_data_source) {
    case kDataSourceTetherDown:
      HandleTetherDownMessage(
          TetherDownMergeCvtPeek(&g_tether_down_merge_state), time, flight_mode,
          Xg, eulers);
      // TODO: Retrieve the ground station state from TetherUpMessage.
      *platform_azi = NAN;
      break;
    case kDataSourceControlTelemetry:
      UpdateControlTelemetry();
      HandleControlTelemetry(GetControlTelemetry(), time, flight_mode, Xg,
                             eulers, platform_azi);
      break;
    case kDataSourceControlDebug:
      UpdateControlDebug();
      HandleControlTelemetry(GetControlTelemetry(), time, flight_mode, Xg,
                             eulers, platform_azi);
      break;
    default:
      assert(false);
    case kDataSourceSimulatorTelemetry:
      UpdateControlTelemetry();
      CvtGetSimTelemetry(kAioNodeSimulator, &sim_telem, nullptr, nullptr);
      HandleSimTelemetry(&sim_telem, GetControlTelemetry(), time, flight_mode,
                         Xg, eulers, platform_azi);
      break;
  }
}

static void DrawGroundStation(const Vec3 *vessel_pos_g, const Mat3 *dcm_g2v,
                              double platform_azi, double drum_angle,
                              double levelwind_ele, double detwist_angle,
                              double gsg_yoke, double gsg_termination) {
  TranslateGroundStationToBuoy(vessel_pos_g, dcm_g2v);

  if (!GetSystemParams()->offshore) {
    if (GetSystemParams()->gs_model == kGroundStationModelGSv2) {
      glPushMatrix();

      // Draw the Parker Ranch tower.
      glRotated(-90.0, 0.0, 0.0, 1.0);
      glColor4d(1.0, 1.0, 1.0, 0.4);
      if (g_lighting_enabled) glEnable(GL_LIGHTING);
      glCallList(gs02_tower_list);
      if (g_lighting_enabled) glDisable(GL_LIGHTING);
      glRotated(90.0, 0.0, 0.0, 1.0);

      glPopMatrix();
    } else {
      DrawTower();
    }
  }

  // TODO: Use validity flags on each required quantity to decide
  // what to draw, instead of trying to index on every combination of ground
  // station type and telemetry source.
  if (g_data_source == kDataSourceSimulatorTelemetry &&
      (GetSystemParams()->gs_model == kGroundStationModelGSv1 ||
       GetSystemParams()->gs_model == kGroundStationModelTopHat)) {
    DrawPerch();
  } else if ((g_data_source == kDataSourceSimulatorTelemetry ||
              g_data_source == kDataSourceControlTelemetry) &&
             GetSystemParams()->gs_model == kGroundStationModelGSv2) {
    DrawGs02(platform_azi, drum_angle, levelwind_ele, detwist_angle, gsg_yoke,
             gsg_termination);
  }

  // Draw the simulated constraint system.
  if ((g_data_source == kDataSourceSimulatorTelemetry) &&
      (GetSimParams()->sim_opt & kSimOptConstraintSystem)) {
    DrawConstraint();
  }

  // End of translation to the base of tower/GS.
  glPopMatrix();
}

// The main drawing function.
void DrawGLScene() {
  // Render GL stuff.
  // Clear screen and depth buffer.
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
  glPushMatrix();

  // Put the visualizer in NED.
  glRotatef(90.0f, 1.0f, 0.0f, 0.0f);

  double time;
  FlightMode flight_mode;
  Vec3 Xg, eulers;
  double platform_azi;

  {
    // Receive incoming messages for 10ms.
    int64_t timeout_us = 10000;
    int64_t end_ts = ClockGetUs() + timeout_us;
    for (int64_t cvt_timeout = timeout_us; cvt_timeout >= 0L;
         cvt_timeout = end_ts - ClockGetUs()) {
      AioRecvToCvt(cvt_timeout, NULL, NULL);
    }
  }
  ParseTelemetryData(&time, &flight_mode, &Xg, &eulers, &platform_azi);

  UpdateCamera(&Xg, &eulers);

  // Draw static elements of the scene.
  if (FLAGS_show_landing_zone) {
    DrawLandingZone();
  }

  if (time > 0.0) {
    // Draw the buoy.
    if (GetSystemParams()->offshore) {
      // Draw the simulated buoy.
      if (g_data_source == kDataSourceSimulatorTelemetry) {
        DrawBuoy(&sim_telem.buoy.Xg, &sim_telem.buoy.dcm_g2v, false,
                 &GetSystemParams()->buoy);
      }

      // Draw the estimated buoy.
      if (g_data_source == kDataSourceControlTelemetry ||
          ((g_data_source == kDataSourceSimulatorTelemetry) &&
           FLAGS_show_phantom_buoy)) {
        const StateEstimate *state_est = &GetControlTelemetry()->state_est;
        DrawBuoy(&state_est->vessel.pos_g, &state_est->vessel.dcm_g2v,
                 g_data_source == kDataSourceSimulatorTelemetry,
                 &GetSystemParams()->buoy);
      }
    }

    // Draw the sea or ground.
    if ((GetSystemParams()->offshore) &&
        g_data_source == kDataSourceSimulatorTelemetry) {
      DrawSea(reinterpret_cast<double *>(&sim_telem.sea.wave_transl_coord),
              reinterpret_cast<double *>(&sim_telem.sea.wave_elev_g));
    } else {
      DrawGrid();
    }

    // Draw the controller's plans.
    if (FLAGS_show_cues && (g_data_source == kDataSourceSimulatorTelemetry ||
                            g_data_source == kDataSourceControlTelemetry)) {
      DrawCrosswindPath(flight_mode, &GetControlTelemetry()->crosswind);
      DrawTransInPath(flight_mode, &GetControlTelemetry()->trans_in);
    }

    // Draw the tether.
    if (g_data_source == kDataSourceSimulatorTelemetry) {
      DrawTether();
    }

    // Draw the ground station.
    if (g_data_source == kDataSourceSimulatorTelemetry) {
      DrawGroundStation(&sim_telem.buoy.Xg, &sim_telem.buoy.dcm_g2v,
                        sim_telem.gs02.azimuth, sim_telem.gs02.drum_angle,
                        sim_telem.gsg.levelwind_ele, sim_telem.gsg.twist,
                        sim_telem.gsg.gsg_yoke, sim_telem.gsg.gsg_termination);
    }

    if (g_data_source == kDataSourceControlTelemetry ||
        ((g_data_source == kDataSourceSimulatorTelemetry) &&
         FLAGS_show_phantom_buoy)) {
      const StateEstimate *state_est = &GetControlTelemetry()->state_est;
      const ControlInput *control_input = &GetControlTelemetry()->control_input;
      const EstimatorTelemetry *estimator = &GetControlTelemetry()->estimator;
      const GroundStationParams &gs_params = GetSystemParams()->ground_station;
      // TODO(snolet): Need to use fault flags to select between the A and B
      // sensor branch.
      DrawGroundStation(&state_est->vessel.pos_g, &state_est->vessel.dcm_g2v,
                        state_est->perch_azi.angle,
                        state_est->winch.position / gs_params.gs02.drum_radius,
                        control_input->perch.levelwind_ele[kPlatformSensorsB],
                        estimator->ground_station.detwist_angle,
                        control_input->gsg[kDrumSensorsB].azi,
                        control_input->gsg[kDrumSensorsB].ele);
    }

    // Draw the wing.
    if (Vec3Norm(&Xg) > 0.0) {
      DrawWing(&Xg, &eulers);

      if (FLAGS_show_breadcrumbs) {
        bool add_new_point = true;
        if (g_fly_trace.size()) {
          Vec3 diff;
          double distance = Vec3Norm(
              Vec3Sub(&Xg, (const Vec3 *)&(*g_fly_trace.crbegin()), &diff));
          if (distance < 10.0) {
            add_new_point = false;
          }
        }
        if (add_new_point) {
          g_fly_trace.push_back(Xg);
        }
        while (g_fly_trace.size() > g_trace_len) {
          g_fly_trace.pop_front();
        }
        DrawPoints(g_fly_trace);
      }
    }
  }

  // Done drawing in NED.
  glPopMatrix();

  if (FLAGS_show_cues) {
    char status_str[300];

    if (g_data_source == kDataSourceControlTelemetry ||
        g_data_source == kDataSourceControlDebug ||
        g_data_source == kDataSourceSimulatorTelemetry) {
      const ControlTelemetry &ct = *GetControlTelemetry();
      const double ground_z = GetSystemParams()->ground_frame.ground_z;

      double wind_dir = ct.state_est.wind_g.dir_f;

      if (GetSystemParams()->gs_model == kGroundStationModelTopHat &&
          GetSystemParams()->gs_model == kGroundStationModelGSv1) {
        // When using GSv1 and the TopHat, it was conventional to
        // report the wind direction with regards to how well aligned
        // it was with the "nominal downwind direction."
        wind_dir = Wrap(wind_dir, -PI, PI);
      } else {
        // The estimator reports the direction the wind is blowing
        // towards.  However, in aviation and meteorology, it's
        // conventional to report the direction the wind is blowing
        // TO.  Now that the g-frame is aligned with NED, we also
        // adopt that convention.
        wind_dir = Wrap(wind_dir + PI, 0.0, 2.0 * PI);
      }

      snprintf(status_str, sizeof(status_str),
               "         Time [s]: % 6.2f s\n"
               "      Flight Mode: %s\n"
               " Loop Angle [deg]: % 5.2f\n"
               "       Beta [deg]: % 5.2f\n"
               "      Alpha [deg]: % 5.2f\n"
               "   Wind Dir [deg]: % 4.1f\n"
               "     |Wind| [m/s]: % 5.2f\n"
               "|App. Wind| [m/s]: % 5.2f\n"
               " AGL Altitude [m]: % 5.2f\n"
               "     Tension [kN]: % 5.2f\n",
               time, FlightModeToString(flight_mode),
               RadToDeg(ct.crosswind.loop_angle),
               RadToDeg(ct.state_est.apparent_wind.sph_f.beta),
               RadToDeg(ct.state_est.apparent_wind.sph_f.alpha),
               RadToDeg(wind_dir), ct.state_est.wind_g.speed_f,
               ct.state_est.apparent_wind.sph_f.v,
               -ct.state_est.Xg.z - ground_z,
               ct.state_est.tether_force_b.sph.tension * 0.001);
    } else {
      snprintf(status_str, sizeof(status_str),
               "       time: % 6.2f s\n"
               "flight_mode: %s\n",
               time, FlightModeToString(flight_mode));
    }
    DrawText(status_str);
  }

  // Swap the buffers to display, since double buffering is used.
  glutSwapBuffers();
}

int main(int argc, char **argv) {
  google::SetUsageMessage("");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_use_control_telemetry && FLAGS_use_tether_down &&
      FLAGS_use_control_debug) {
    std::cerr << "At most one of --use_control_telemetry, --use_control_debug "
              << ", and --use_tether_down may be specified." << std::endl;
    return 1;
  }

  if (FLAGS_use_tether_down) {
    g_data_source = kDataSourceTetherDown;
  } else if (FLAGS_use_control_telemetry) {
    g_data_source = kDataSourceControlTelemetry;
  } else if (FLAGS_use_control_debug) {
    g_data_source = kDataSourceControlDebug;
  } else {
    g_data_source = kDataSourceSimulatorTelemetry;
  }

  g_lighting_enabled = FLAGS_lighting;

  json_load::LoadControlParams(GetControlParamsUnsafe());
  json_load::LoadSimParams(GetSimParamsUnsafe());
  json_load::LoadSystemParams(GetSystemParamsUnsafe());

  MessageType subscribe_types[] = {
      kMessageTypeSimTelemetry, kMessageTypeControlTelemetry,
      kMessageTypeControlDebug, kMessageTypeTetherDown};

  TetherDownMergeStateInit(&g_tether_down_merge_state);

  AioSetup(kAioNodeVisualizer, GetSystemParams()->comms.aio_port,
           subscribe_types, ARRAYSIZE(subscribe_types));

  InitGlut(argc, argv);
  InitGL();
  InitWing();
  InitGs02();
  InitBuoy();

  // Check our assumptions about data types for the PR tower model.
  assert(sizeof(modelGs02_towerVerts[0]) == sizeof(double));
  assert(sizeof(modelGs02_towerVerts[0]) * 3 == sizeof(Vec3));
  assert(sizeof(modelGs02_towerVerts) / sizeof(modelGs02_towerVerts[0]) ==
         modelGs02_towerNumVerts * 3);

  // Initialize the PR tower model.
  InitModel(modelGs02_towerNumVerts,
            reinterpret_cast<Vec3 *>(modelGs02_towerVerts), &gs02_tower_list);

  glutMainLoop();

  AioClose();
  return 0;
}
