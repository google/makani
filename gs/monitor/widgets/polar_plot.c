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

#include "gs/monitor/widgets/polar_plot.h"

#include <assert.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <math.h>
#include <stdlib.h>

#include "gs/monitor/widgets/widget_colors.h"

static void polar_plot_class_init(
    PolarPlotClass *class __attribute__((unused))) {}

static void polar_plot_init(PolarPlot *pp) {
  for (int32_t i = 0; i < POLAR_PLOT_MAX_LINES; ++i) {
    pp->r_data[i] = malloc(POLAR_PLOT_MAX_POINTS * sizeof(*(pp->r_data[i])));
    pp->th_data[i] = malloc(POLAR_PLOT_MAX_POINTS * sizeof(*(pp->th_data[i])));
  }
  pp->draw_background = NULL;

  // Create plot
  pp->plot = GTK_PLOT(gtk_plot_polar_new_with_size(NULL, 1, 1));

  gtk_plot_grids_set_visible(pp->plot, TRUE, FALSE, TRUE, FALSE);
  gtk_plot_major_vgrid_set_attributes(pp->plot, GTK_PLOT_LINE_SOLID, 0.1f,
                                      &BLACK);
  gtk_plot_major_hgrid_set_attributes(pp->plot, GTK_PLOT_LINE_SOLID, 0.1f,
                                      &BLACK);

  double th_min = 0.0, th_max = 360.0;
  double r_min = 0.0, r_max = 40.0;
  gtk_plot_set_range(pp->plot, th_min, th_max, r_min, r_max);
  gtk_plot_set_ticks(pp->plot, GTK_PLOT_AXIS_X, (th_max - th_min) / 8.0, 4);
  gtk_plot_set_ticks(pp->plot, GTK_PLOT_AXIS_Y, (r_max - r_min) / 4.0, 4);

  gtk_plot_axis_show_title(GTK_PLOT(pp->plot)->left);
  gtk_plot_axis_move_title(GTK_PLOT(pp->plot)->left, 90.0, -0.1, 0.5);

  gtk_plot_hide_legends(pp->plot);
  gtk_plot_clip_data(pp->plot, FALSE);
  gtk_widget_show(GTK_WIDGET(pp->plot));  // Shouldn't be necessary, but is

  pp->num_lines = POLAR_PLOT_MAX_LINES;
  // Create plot_data
  for (int32_t i = 0; i < POLAR_PLOT_MAX_LINES; ++i) {
    pp->plot_data[i] = GTK_PLOT_DATA(gtk_plot_data_new());
    gtk_plot_add_data(pp->plot, pp->plot_data[i]);
    gtk_plot_data_set_numpoints(pp->plot_data[i], 0);
    gtk_plot_data_set_line_attributes(pp->plot_data[i], GTK_PLOT_LINE_NONE, 0,
                                      0, 1.5, COLORS[i]);
    gtk_plot_data_set_symbol(pp->plot_data[i], GTK_PLOT_SYMBOL_CIRCLE,
                             GTK_PLOT_SYMBOL_FILLED, 1, 1, COLORS[i],
                             COLORS[i]);

    gtk_plot_data_set_x(pp->plot_data[i], pp->r_data[i]);
    gtk_plot_data_set_y(pp->plot_data[i], pp->th_data[i]);

    pp->num_undrawn_points[i] = 0;

    // These widget_shows shouldn't be necessary but are
    gtk_widget_show(GTK_WIDGET(pp->plot_data[i]));
  }

  // Add and pack everything
  pp->canvas = GTK_PLOT_CANVAS(gtk_plot_canvas_new(400, 400, 1.0));
  GtkPlotCanvasChild *child = gtk_plot_canvas_plot_new(pp->plot);
  gtk_plot_canvas_put_child(pp->canvas, child, 0.15, 0.05, 0.90, 0.87);
  gtk_box_pack_start(GTK_BOX(&pp->box), GTK_WIDGET(pp->canvas), FALSE, FALSE,
                     2);
}

GType polar_plot_get_type(void) {
  static GType pp_type = 0;
  if (!pp_type) {
    const GTypeInfo pp_info = {
        sizeof(PolarPlotClass),
        NULL,  // base_init
        NULL,  // base_finalize
        (GClassInitFunc)polar_plot_class_init,
        NULL,  // class_finalize
        NULL,  // class_data
        sizeof(PolarPlot),
        0,  // n_preallocs
        (GInstanceInitFunc)polar_plot_init,
        NULL  // value_table
    };
    pp_type = g_type_register_static(GTK_TYPE_HBOX, "PolarPlot", &pp_info, 0);
  }
  return pp_type;
}

GtkWidget *polar_plot_new(int32_t num_lines) {
  PolarPlot *pp = g_object_new(polar_plot_get_type(), NULL);
  polar_plot_set_num_lines(pp, num_lines);
  return GTK_WIDGET(pp);
}

void polar_plot_set_num_lines(PolarPlot *pp, int32_t num_lines) {
  pp->num_lines = num_lines;
}

void polar_plot_add_points(PolarPlot *pp, int32_t line_num, double th,
                           double r) {
  int32_t num_points = gtk_plot_data_get_numpoints(pp->plot_data[line_num]);
  if (num_points >= 2000) num_points = 0;

  pp->r_data[line_num][num_points] = r;
  pp->th_data[line_num][num_points] = th;
  gtk_plot_data_set_numpoints(pp->plot_data[line_num], num_points + 1);

  pp->num_undrawn_points[line_num]++;

  if (num_points == 1000) polar_plot_refresh_slow(pp);
}

void polar_plot_draw_line(PolarPlot *pp, GtkPlotLine pl, double th0, double r0,
                          double th1, double r1) {
  double px0, py0, px1, py1;
  gtk_plot_get_pixel(pp->plot, r0, th0, &px0, &py0);
  gtk_plot_get_pixel(pp->plot, r1, th1, &px1, &py1);
  gtk_plot_draw_line(pp->plot, pl, px0, py0, px1, py1);
}

void polar_plot_refresh(PolarPlot *pp) {
  GtkPlotPC *old = pp->plot->pc;
  pp->plot->pc = pp->canvas->pc;
  GList *dataset = pp->plot->data_sets;
  int32_t i = 0;
  while (dataset) {
    if (GTK_IS_PLOT_DATA(dataset->data))
      gtk_plot_data_draw_points(dataset->data, pp->num_undrawn_points[i]);
    dataset = dataset->next;
    pp->num_undrawn_points[i] = 0;
    i++;
  }
  pp->plot->pc = old;

  gtk_plot_canvas_refresh(pp->canvas);
}

void polar_plot_refresh_slow(PolarPlot *pp) {
  GdkColor c = gtk_widget_get_style(GTK_WIDGET(pp))->bg[GTK_STATE_NORMAL];
  gtk_plot_canvas_set_background(pp->canvas, &c);

  if (pp->draw_background) {
    GtkPlotPC *old = pp->plot->pc;
    pp->plot->pc = pp->canvas->pc;
    // I wanted just the circle to be white, but couldn't get this to work.
    // pp->plot->background = c;
    // gtk_plot_pc_draw_circle(pp->plot->pc, TRUE, 210.0, 184.0, 296.0);
    pp->draw_background(pp);
    pp->plot->pc = old;
  }
  gtk_plot_canvas_refresh(pp->canvas);
  for (int32_t i = 0; i < POLAR_PLOT_MAX_LINES; ++i)
    pp->num_undrawn_points[i] = 0;
}
