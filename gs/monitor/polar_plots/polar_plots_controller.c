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

#include "gs/monitor/polar_plots/polar_plots_controller.h"

#include <gtk/gtk.h>
#include <stdint.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/polar_plot.h"

static void DrawPolarTransOutThresh(PolarPlot *pp) {
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
    polar_plot_draw_line(pp, pl, loop_angle_z1 * 180.0 / PI, Vg_norm_thresh_z1,
                         loop_angle * 180.0 / PI, Vg_norm_thresh);
    loop_angle_z1 = loop_angle;
    Vg_norm_thresh_z1 = Vg_norm_thresh;
  }
}

void UpdatePolarTransOutThresh(PolarPlot *pp, int32_t init) {
  if (init) {
    pp->draw_background = &DrawPolarTransOutThresh;
    polar_plot_set_num_lines(pp, 1);
    gtk_plot_polar_rotate(GTK_PLOT_POLAR(pp->plot), 180.0);
    gtk_plot_axis_set_title(GTK_PLOT(pp->plot)->left, "|Vg| [m/s]");
  }
  polar_plot_add_points(pp, 0, ct->crosswind.loop_angle * 180.0 / PI,
                        Vec3Norm(&ct->state_est.Vg));
}
