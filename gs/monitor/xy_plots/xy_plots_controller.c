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

#include "gs/monitor/xy_plots/xy_plots_controller.h"

#include <assert.h>
#include <gtk/gtk.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "common/c_math/util.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_path.h"
#include "control/crosswind/crosswind_types.h"
#include "control/hover/hover_path.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_params.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/xy_plot.h"

static void DrawCrosswindVectorField(XYPlot *xyp) {
  static bool first_call = true;
  // NOTE: This does not account for possible variations in the crosswind path
  // radius
  double path_radius =
      GetControlParams()->crosswind.playbook.entries[0].path_radius_target;

  double width = 1.5 * 2.0 * path_radius;
  CrosswindPathType path_type = kCrosswindPathNormal;
  static double x_tab[16];
  static double y_tab[ARRAYSIZE(x_tab)];
  static Vec3 crosswind_best_vel_cw[ARRAYSIZE(x_tab)][ARRAYSIZE(x_tab)];
  if (first_call) {
    // Calculate vector fields
    double incr = width / (ARRAYSIZE(x_tab) - 1);
    int32_t i = 0;
    for (double x0 = -width / 2.0; x0 <= width / 2.0; x0 += incr) {
      int32_t j = 0;
      x_tab[i] = x0;
      for (double y0 = -width / 2.0; y0 <= width / 2.0; y0 += incr) {
        assert(i < ARRAYSIZE(x_tab) && j < ARRAYSIZE(x_tab));
        Vec3 target_pos_cw = {0.0, -x0, -y0};
        y_tab[j] = y0;
        // NOTE: The path radius used here is the nominal value and may differ
        // from the value in use by the controller.
        CrosswindPathCalcBestHeading(
            &target_pos_cw, path_radius, path_type, &kVec3Zero,
            &GetControlParams()->crosswind.path, &crosswind_best_vel_cw[i][j]);
        Vec3Normalize(&crosswind_best_vel_cw[i][j],
                      &crosswind_best_vel_cw[i][j]);
        j++;
      }
      i++;
    }
  }

  GtkPlotLine pl = {.line_style = GTK_PLOT_LINE_SOLID,
                    .cap_style = 0,
                    .join_style = 0,
                    .line_width = 1.0,
                    .color = {0, 0xbbbb, 0xbbbb, 0xbbbb}};

  // Draw vector field
  double vec_len = width / 40.0;
  for (int32_t i = 0; i < ARRAYSIZE(x_tab); ++i) {
    for (int32_t j = 0; j < ARRAYSIZE(x_tab); ++j) {
      xy_plot_draw_line(xyp, pl, x_tab[i], y_tab[j],
                        x_tab[i] - vec_len * crosswind_best_vel_cw[i][j].y,
                        y_tab[j] - vec_len * crosswind_best_vel_cw[i][j].z);
    }
  }

  // Draw circle
  const double dth = 0.2;
  for (double th = 0.0; th < 2.0 * PI; th += dth) {
    xy_plot_draw_line(xyp, pl, path_radius * cos(th), path_radius * sin(th),
                      path_radius * cos(th + dth), path_radius * sin(th + dth));
  }

  first_call = false;
}

void UpdateCrosswindCircle(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawCrosswindVectorField;
    xy_plot_set_num_lines(xyp, 2);

    // NOTE: This does not account for possible variations in the crosswind path
    // radius
    double path_radius =
        GetControlParams()->crosswind.playbook.entries[0].path_radius_target;
    double width = 1.6 * 2.0 * path_radius;
    xy_plot_set_xyrange(xyp, -width / 2.0, width / 2.0, -width / 2.0,
                        width / 2.0);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "-z_cw [m]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "-y_cw [m]");
  }
  if (!CheckControllerTelemetry()) return;
  xy_plot_add_points(xyp, 0, -ct->crosswind.current_pos_cw.x,
                     -ct->crosswind.current_pos_cw.y);
  xy_plot_add_points(xyp, 1, -ct->crosswind.target_pos_cw.x,
                     -ct->crosswind.target_pos_cw.y);
  if (ct->flight_mode != ct_z1->flight_mode) {
    xy_plot_refresh_slow(xyp);
  }
}

static void DrawWinchSpeedSetpoint(XYPlot *xyp) {
  GtkPlotLine pl = {.line_style = GTK_PLOT_LINE_SOLID,
                    .cap_style = 0,
                    .join_style = 0,
                    .line_width = 1.0,
                    .color = {0, 0xbbbb, 0xbbbb, 0xbbbb}};

  const double tether_length = GetSystemParams()->tether.length;
  const HoverWinchParams *winch_params = &GetControlParams()->hover.winch;

  double winch_pos_z1 = 0.0, winch_vel_in_z1 = 0.0, winch_vel_out_z1 = 0.0;
  for (double winch_pos = -tether_length; winch_pos < 0.0; winch_pos += 2.0) {
    double winch_vel_out =
        Interp1WarpY(winch_params->winch_position_pay_out_table,
                     winch_params->winch_speed_pay_out_table,
                     HOVER_WINCH_PAY_OUT_TABLE_LENGTH, winch_pos,
                     kInterpOptionSaturate, &Square, &Sqrt);
    double winch_vel_in =
        -Interp1WarpY(winch_params->winch_position_reel_in_table,
                      winch_params->winch_speed_reel_in_table,
                      HOVER_WINCH_REEL_IN_TABLE_LENGTH, winch_pos,
                      kInterpOptionSaturate, &Square, &Sqrt);

    if (winch_pos > -tether_length) {
      xy_plot_draw_line(xyp, pl, winch_pos_z1, winch_vel_out_z1, winch_pos,
                        winch_vel_out);
      xy_plot_draw_line(xyp, pl, winch_pos_z1, winch_vel_in_z1, winch_pos,
                        winch_vel_in);
    }
    winch_pos_z1 = winch_pos;
    winch_vel_in_z1 = winch_vel_in;
    winch_vel_out_z1 = winch_vel_out;
  }
}

void UpdateWinchSpeedPos(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawWinchSpeedSetpoint;
    xy_plot_set_num_lines(xyp, 1);
    xy_plot_set_xyrange(xyp, -460.0, 0.0, -5.0, 5.0);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "Winch Vel [m/s]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "Winch Pos [m]");
  }
  xy_plot_add_points(xyp, 0, ct->control_input.perch.winch_pos,
                     ct->control_output.winch_vel_cmd);
}

static void DrawHoverElevationLimits(XYPlot *xyp) {
  GtkPlotLine pl = {.line_style = GTK_PLOT_LINE_SOLID,
                    .cap_style = 0,
                    .join_style = 0,
                    .line_width = 1.0,
                    .color = {0, 0xbbbb, 0xbbbb, 0xbbbb}};

  const double tether_length = GetSystemParams()->tether.length;
  const HoverPathParams *params = &GetControlParams()->hover.path;

  double Xg_norm_last = 0.0;
  double elevation_min_last, elevation_max_last;
  HoverPathCalcElevationLimits(0.0, params, &elevation_min_last,
                               &elevation_max_last);

  for (double Xg_norm = 0.0; Xg_norm < tether_length; Xg_norm += 2.0) {
    double elevation_min, elevation_max;
    HoverPathCalcElevationLimits(Xg_norm, params, &elevation_min,
                                 &elevation_max);

    xy_plot_draw_line(xyp, pl, Xg_norm_last, elevation_min_last, Xg_norm,
                      elevation_min);
    xy_plot_draw_line(xyp, pl, Xg_norm_last, elevation_max_last, Xg_norm,
                      elevation_max);

    Xg_norm_last = Xg_norm;
    elevation_min_last = elevation_min;
    elevation_max_last = elevation_max;
  }
}

void UpdateHoverElevationCommand(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawHoverElevationLimits;
    xy_plot_set_num_lines(xyp, 2);
    xy_plot_set_xyrange(xyp, 0.0, GetSystemParams()->tether.length, -0.1, 0.7);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "Elevation [rad]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "|Xg| [m]");
  }
  xy_plot_add_points(xyp, 0, Vec3Norm(&ct->state_est.Xg),
                     ct->hover.elevation_cmd);
  xy_plot_add_points(xyp, 1, Vec3Norm(&ct->state_est.Xg),
                     atan2(-ct->state_est.Xg.z, Vec3XyNorm(&ct->state_est.Xg)));
}

// TODO: Convert this plot from cartesian to polar.
static void DrawTransOutThresh(XYPlot *xyp) {
  GtkPlotLine pl = {.line_style = GTK_PLOT_LINE_SOLID,
                    .cap_style = 0,
                    .join_style = 0,
                    .line_width = 1.0,
                    .color = {0, 0xbbbb, 0xbbbb, 0xbbbb}};

  const CrosswindModeParams *params = &GetControlParams()->crosswind.mode;

  double loop_angle_z1 = 0.0, Vg_norm_thresh_z1 = 0.0;
  for (double loop_angle = 0.0; loop_angle < 2.0 * PI; loop_angle += 0.01) {
    double Vg_norm_thresh = Interp1(
        params->loop_angle_table, params->max_trans_out_speed_table,
        ARRAYSIZE(params->loop_angle_table), loop_angle, kInterpOptionSaturate);
    xy_plot_draw_line(xyp, pl, loop_angle_z1, Vg_norm_thresh_z1, loop_angle,
                      Vg_norm_thresh);
    loop_angle_z1 = loop_angle;
    Vg_norm_thresh_z1 = Vg_norm_thresh;
  }
}

void UpdateTransOutThresh(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawTransOutThresh;
    xy_plot_set_num_lines(xyp, 1);
    xy_plot_set_xyrange(xyp, 0.0, 2.0 * PI, 0.0, 40.0);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "loop_angle [rad]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "|Vg| [m/s]");
  }
  xy_plot_add_points(xyp, 0, ct->crosswind.loop_angle,
                     Vec3Norm(&ct->state_est.Vg));
}

static void DrawLevelwind(XYPlot *xyp) {
  GtkPlotLine pl = {.line_style = GTK_PLOT_LINE_SOLID,
                    .cap_style = 0,
                    .join_style = 0,
                    .line_width = 1.0,
                    .color = {0, 0xbbbb, 0xbbbb, 0xbbbb}};

  const HoverPathParams *params = &GetControlParams()->hover.path;
  double perched_wing_pos_norm = Vec3Norm(&params->perched_wing_pos_p);
  double alt_off =
      -(params->perched_wing_pos_p.z + params->ascend_offset_g_z +
        perched_wing_pos_norm * sin(params->launch_perch_elevation_min));
  double tol = 0.25;  // [m]
  double r_max = 10.0;
  double alt_max = r_max * tan(params->launch_perch_elevation_max);

  xy_plot_draw_line(xyp, pl, 0, alt_off, r_max, alt_max);
  xy_plot_draw_line(xyp, pl, 0, alt_off - tol, r_max, alt_max - tol);
  xy_plot_draw_line(xyp, pl, 0, alt_off + tol, r_max, alt_max + tol);

  const SystemParams *system_params = GetSystemParams();
  // 0 is an approximation
  Vec2 cen = {Vec3XyNorm(&system_params->perch.levelwind_origin_p_0), 0.0};
  double r = system_params->wing.bridle_rad +
             system_params->wing.bridle_pos[0].z +
             system_params->levelwind.pivot_axis_to_bridle_point;

  // Draw circle
  const double dth = 0.2;
  double x1, y1, x2, y2;
  for (double th = 0.0; th < 2.0 * PI; th += dth) {
    x1 = r * cos(th) + cen.x;
    y1 = r * sin(th) + cen.y;
    x2 = r * cos(th + dth) + cen.x;
    y2 = r * sin(th + dth) + cen.y;
    xy_plot_draw_line(xyp, pl, x1, y1, x2, y2);
  }
}

void UpdatePerchPosition(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawLevelwind;
    xy_plot_set_num_lines(xyp, 2);
    xy_plot_set_xyrange(xyp, 0.0, 10.0, -1.0, 4.0);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "Alt [m]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "Radius [m}");
  }

  if (ct->estimator.current_gps_receiver >= 0 &&
      ct->estimator.current_gps_receiver < kNumWingGpsReceivers) {
    int32_t receiver = ct->estimator.current_gps_receiver;
    double r_gps = Vec3XyNorm(&ct->estimator.gps[receiver].Xg);
    double r_glas = Vec3XyNorm(&ct->estimator.glas.Xg);
    xy_plot_add_points(xyp, 0, r_gps, -ct->estimator.gps[receiver].Xg.z);
    xy_plot_add_points(xyp, 1, r_glas, -ct->estimator.glas.Xg.z);
  }
}

static void DrawPowerCurve(XYPlot *xyp) {
  GtkPlotLine pl = {.line_style = GTK_PLOT_LINE_SOLID,
                    .cap_style = 0,
                    .join_style = 0,
                    .line_width = 1.0,
                    .color = {0, 0xbbbb, 0xbbbb, 0xbbbb}};

  // Target power curve.
  double wind[] = {4.0, 4.5, 5.0, 5.5,  6.0,  6.5,  7.0,  7.5, 8.0,
                   8.5, 9.0, 9.5, 10.0, 10.5, 11.0, 11.5, 12.0};
  double power[] = {-25.0, 2.0,   32.0,  69.0,  113.0, 165.0,
                    225.0, 295.0, 373.0, 453.0, 526.0, 591.0,
                    648.0, 686.0, 705.0, 706.0, 707.0};

  for (int32_t i = 0; i < ARRAYSIZE(wind) - 1; ++i) {
    xy_plot_draw_line(xyp, pl, wind[i], power[i], wind[i + 1], power[i + 1]);
  }
}

void UpdatePowerCurve(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawPowerCurve;
    xy_plot_set_num_lines(xyp, 2);
    xy_plot_set_xyrange(xyp, 3.0, 12.0, -8.0, 22.0);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "Power [kW]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "Wind [m/s]");
  }

  if (fd->seq % 250U == 0U) {
    xy_plot_add_points(xyp, 0, fd->wind_speed_f_1min,
                       fd->power_gen_f_1min / 1000.0);
    xy_plot_add_points(xyp, 1, fd->wind_speed_f_5min,
                       fd->power_gen_f_5min / 1000.0);
  }
}

void UpdatePowerPerLoop(XYPlot *xyp, int32_t init) {
  static int32_t loop_number_z1;
  if (init) {
    xy_plot_set_num_lines(xyp, 1);
    xy_plot_set_xyrange(xyp, 0.0, 100.0, -5.0, 20.0);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "Power [kW]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "Loop [#]");
    loop_number_z1 = 0;
  }

  if (loop_number_z1 != fd->loop_number) {
    xy_plot_add_points(xyp, 0, (double)(fd->loop_number % 100),
                       fd->power_per_loop / 1000.0);

    if (fd->loop_number % 500 == 0) xy_plot_refresh_slow(xyp);
  }

  loop_number_z1 = fd->loop_number;
}

void UpdateRotorPitchYaw(XYPlot *xyp, int32_t init) {
  if (init) {
    xy_plot_set_num_lines(xyp, 2);
    xy_plot_set_xyrange(xyp, -10e3, 10e3, -10e3, 10e3);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "Yaw moment [N-m]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "Pitch moment [N-m]");
  }
  xy_plot_add_points(xyp, 0, ct->thrust_moment.moment.z,
                     ct->thrust_moment.moment.y);
  xy_plot_add_points(xyp, 1, ct->hover.int_moment.z, ct->hover.int_moment.y);
}

// Draw the boundaries of the envelope of allowed flight positions
// while flying under constraints.
static void DrawConstraintXyWindow(XYPlot *xyp) {
  GtkPlotLine blue_pl = {.line_style = GTK_PLOT_LINE_SOLID,
                         .cap_style = 0,
                         .join_style = 0,
                         .line_width = 1.0,
                         .color = {0, 0x8888, 0x8888, 0xffff}};
  GtkPlotLine green_pl = {.line_style = GTK_PLOT_LINE_SOLID,
                          .cap_style = 0,
                          .join_style = 0,
                          .line_width = 1.0,
                          .color = {0, 0x8888, 0xffff, 0x8888}};

  const ConstraintWindowParams *params =
      &GetMonitorParams()->control.constraint_window;

  // Draw horizontal cross-sections.
  for (int32_t i = 0; i < params->num_vertices_xy_cross - 1; ++i) {
    xy_plot_draw_line(xyp, blue_pl, params->xs_low[i], params->ys_low[i],
                      params->xs_low[i + 1], params->ys_low[i + 1]);
    xy_plot_draw_line(xyp, blue_pl, params->xs_mid[i], params->ys_mid[i],
                      params->xs_mid[i + 1], params->ys_mid[i + 1]);
    xy_plot_draw_line(xyp, blue_pl, params->xs_high[i], params->ys_high[i],
                      params->xs_high[i + 1], params->ys_high[i + 1]);
  }

  // Draw vertical cross-section.
  for (int32_t i = 0; i < params->num_vertices_xz_cross - 1; ++i) {
    xy_plot_draw_line(xyp, green_pl, params->xs_vertical[i],
                      params->zs_vertical[i], params->xs_vertical[i + 1],
                      params->zs_vertical[i + 1]);
  }

  // Reverse the y-axis
  gtk_plot_reflect_y(xyp->plot, TRUE);
}

void UpdateConstraintWindow(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawConstraintXyWindow;
    xy_plot_set_num_lines(xyp, 2);

    const ConstraintWindowParams *params =
        &GetMonitorParams()->control.constraint_window;

    xy_plot_set_xyrange(xyp, params->plot_xlim[0], params->plot_xlim[1],
                        params->plot_ylim[0], params->plot_ylim[1]);
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "x [m]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left,
                            "y (blue), vertical (green) [m]");
  }
  xy_plot_add_points(xyp, 0, ct->state_est.Xg.x, ct->state_est.Xg.y);
  xy_plot_add_points(xyp, 1, ct->state_est.Xg.x, ct->state_est.Xg.z);
}

static void DrawManualPositionWindow(XYPlot *xyp) {
  // Draw a green circle at the origin.
  GtkPlotLine green_pl = {.line_style = GTK_PLOT_LINE_SOLID,
                          .cap_style = 0,
                          .join_style = 0,
                          .line_width = 5.0,
                          .color = {0, 0x0000, 0x8888, 0x0000}};
  double radius = 20.0;
  int32_t num_edges = 100;
  double d_theta = 2.0 * PI / (double)num_edges;

  for (double theta = 0.0; theta < 2.0 * PI - d_theta / 2.0; theta += d_theta) {
    xy_plot_draw_line(xyp, green_pl, radius * sin(theta), radius * cos(theta),
                      radius * sin(theta + d_theta),
                      radius * cos(theta + d_theta));
  }

  // Invert the plot's axes to correspond to a top-down view.
  gtk_plot_reflect_x(xyp->plot, TRUE);
  gtk_plot_reflect_y(xyp->plot, TRUE);
}

// Plots the position of the wing in the manual monitor.
//
// The ground frame -x direction points up on the plot, and its +y direction
// points right. This is to mimic a view from the command center, with up
// oriented downwind.
void UpdateManualPosition(XYPlot *xyp, int32_t init) {
  if (init) {
    xyp->draw_background = &DrawManualPositionWindow;
    xy_plot_set_num_lines(xyp, 1);
    xy_plot_set_xyrange(xyp, -1000.0, 1000.0, -2000.0, 100.0);

    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "y [m]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "x [m]");
  }

  xy_plot_add_points(xyp, 0, ct->state_est.Xg.y, ct->state_est.Xg.x);
}

void UpdateAeroAngles(XYPlot *xyp, int32_t init) {
  static const double kAlphaMin = -4.0;
  static const double kAlphaMax = 6.0;
  static const double kBetaMin = -15.0;
  static const double kBetaMax = 15.0;

  if (init) {
    xy_plot_set_xyrange(xyp, kBetaMin, kBetaMax, kAlphaMin, kAlphaMax);
    xy_plot_set_num_lines(xyp, 1);

    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "beta [deg]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "alpha [deg]");
  }

  double alpha_deg =
      Saturate(ct->state_est.apparent_wind.sph_f.alpha * 180.0 / PI, kAlphaMin,
               kAlphaMax);
  double beta_deg = Saturate(
      ct->state_est.apparent_wind.sph_f.beta * 180.0 / PI, kBetaMin, kBetaMax);
  xy_plot_add_points(xyp, 0, beta_deg, alpha_deg);
}

// Draw glideslope lines approximating glide ratio at WingSave-trimmed
// conditions.  Two lines correspond to flight path angle from threshold and
// overrun of landing area from wing's POV.
static void DrawGlideslope(XYPlot *xyp) {
  static const double kGsRange = -2000.0;
  GtkPlotLine pl = {.line_style = GTK_PLOT_LINE_SOLID,
                    .cap_style = 0,
                    .join_style = 0,
                    .line_width = 2.0,
                    .color = {0, 0xbbbb, 0xbbbb, 0xbbbb}};
  const MonLandingZoneParams *params = &GetMonitorParams()->landing_zone;
  xy_plot_draw_line(xyp, pl, params->overrun, 0.0, kGsRange,
                    (kGsRange - params->overrun) * tan(params->glideslope));
  xy_plot_draw_line(xyp, pl, params->threshold, 0.0, kGsRange,
                    (kGsRange - params->threshold) * tan(params->glideslope));
}

void UpdateGlideslope(XYPlot *xyp, int32_t init) {
  static const double kRangeMin = -2000.0;
  static const double kRangeMax = 100.0;
  static const double kElevationMin = 0.0;
  static const double kElevationMax = 500.0;
  if (init) {
    xyp->draw_background = &DrawGlideslope;
    xy_plot_set_num_lines(xyp, 1);
    xy_plot_set_xyrange(xyp, kRangeMin, kRangeMax, kElevationMin,
                        kElevationMax);

    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->left, "Elevation [m]");
    gtk_plot_axis_set_title(GTK_PLOT(xyp->plot)->bottom, "Range [m]");
  }
  double range_m = Saturate(ct->state_est.Xg.x, kRangeMin, kRangeMax);
  double elevation_m =
      Saturate(-ct->state_est.Xg.z, kElevationMin, kElevationMax);
  xy_plot_add_points(xyp, 0, range_m, elevation_m);
}
