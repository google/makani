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

#include "gs/monitor/widgets/xy_plot.h"

#include <assert.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <math.h>
#include <stdlib.h>

#include "gs/monitor/widgets/widget_colors.h"

static void xy_plot_class_init(XYPlotClass *class __attribute__((unused))) {}

static void xy_plot_init(XYPlot *xyp) {
  for (int32_t i = 0; i < XY_PLOT_MAX_LINES; ++i) {
    xyp->x_data[i] = malloc(XY_PLOT_MAX_POINTS * sizeof(*(xyp->x_data[i])));
    xyp->y_data[i] = malloc(XY_PLOT_MAX_POINTS * sizeof(*(xyp->y_data[i])));
  }
  xyp->draw_background = NULL;

  // Create plot
  xyp->plot = GTK_PLOT(gtk_plot_new_with_size(NULL, 1, 1));

  gtk_plot_x0_set_visible(xyp->plot, TRUE);
  gtk_plot_y0_set_visible(xyp->plot, TRUE);
  gtk_plot_grids_set_visible(xyp->plot, TRUE, FALSE, TRUE, FALSE);
  gtk_plot_major_vgrid_set_attributes(xyp->plot, GTK_PLOT_LINE_SOLID, 0.1f,
                                      &BLACK);
  gtk_plot_major_hgrid_set_attributes(xyp->plot, GTK_PLOT_LINE_SOLID, 0.1f,
                                      &BLACK);
  double x_min = -100, x_max = 100;
  double y_min = -100, y_max = 100;
  gtk_plot_set_range(xyp->plot, x_min, x_max, y_min, y_max);
  gtk_plot_set_ticks(xyp->plot, GTK_PLOT_AXIS_X, (x_max - x_min) / 5, 4);
  gtk_plot_set_ticks(xyp->plot, GTK_PLOT_AXIS_Y, (y_max - y_min) / 5, 4);

  // Hide axis titles and labels
  gtk_plot_axis_hide_title(GTK_PLOT(xyp->plot)->right);
  gtk_plot_axis_hide_title(GTK_PLOT(xyp->plot)->top);
  gtk_plot_axis_show_title(GTK_PLOT(xyp->plot)->left);
  gtk_plot_axis_show_title(GTK_PLOT(xyp->plot)->bottom);
  gtk_plot_axis_show_labels(GTK_PLOT(xyp->plot)->right, GTK_PLOT_LABEL_NONE);
  gtk_plot_axis_show_labels(GTK_PLOT(xyp->plot)->top, GTK_PLOT_LABEL_NONE);

  gtk_plot_hide_legends(xyp->plot);
  gtk_plot_clip_data(xyp->plot, TRUE);
  gtk_widget_show(GTK_WIDGET(xyp->plot));  // Shouldn't be necessary, but is

  xyp->num_lines = XY_PLOT_MAX_LINES;
  // Create plot_data
  for (int32_t i = 0; i < XY_PLOT_MAX_LINES; ++i) {
    xyp->plot_data[i] = GTK_PLOT_DATA(gtk_plot_data_new());
    gtk_plot_add_data(xyp->plot, xyp->plot_data[i]);
    gtk_plot_data_set_numpoints(xyp->plot_data[i], 0);
    gtk_plot_data_set_line_attributes(xyp->plot_data[i], GTK_PLOT_LINE_NONE, 0,
                                      0, 1.5, COLORS[i]);
    gtk_plot_data_set_symbol(xyp->plot_data[i], GTK_PLOT_SYMBOL_CIRCLE,
                             GTK_PLOT_SYMBOL_FILLED, 1, 1, COLORS[i],
                             COLORS[i]);

    gtk_plot_data_set_x(xyp->plot_data[i], xyp->x_data[i]);
    gtk_plot_data_set_y(xyp->plot_data[i], xyp->y_data[i]);

    xyp->num_undrawn_points[i] = 0;

    // These widget_shows shouldn't be necessary but are
    gtk_widget_show(GTK_WIDGET(xyp->plot_data[i]));
  }

  // Add and pack everything
  xyp->canvas = GTK_PLOT_CANVAS(gtk_plot_canvas_new(400, 400, 1.0));
  GtkPlotCanvasChild *child = gtk_plot_canvas_plot_new(xyp->plot);
  gtk_plot_canvas_put_child(xyp->canvas, child, 0.15, 0.05, 0.95, 0.87);
  gtk_box_pack_start(GTK_BOX(&xyp->box), GTK_WIDGET(xyp->canvas), FALSE, FALSE,
                     2);
}

GType xy_plot_get_type(void) {
  static GType xyp_type = 0;
  if (!xyp_type) {
    const GTypeInfo xyp_info = {
        sizeof(XYPlotClass),
        NULL,  // base_init
        NULL,  // base_finalize
        (GClassInitFunc)xy_plot_class_init,
        NULL,  // class_finalize
        NULL,  // class_data
        sizeof(XYPlot),
        0,  // n_preallocs
        (GInstanceInitFunc)xy_plot_init,
        NULL  // value_table
    };
    xyp_type = g_type_register_static(GTK_TYPE_HBOX, "XYPlot", &xyp_info, 0);
  }
  return xyp_type;
}

GtkWidget *xy_plot_new(int32_t num_lines) {
  XYPlot *xyp = g_object_new(xy_plot_get_type(), NULL);
  xy_plot_set_num_lines(xyp, num_lines);
  return GTK_WIDGET(xyp);
}

void xy_plot_set_num_lines(XYPlot *xyp, int32_t num_lines) {
  xyp->num_lines = num_lines;
}

void xy_plot_add_points(XYPlot *xyp, int32_t line_num, double x, double y) {
  int32_t num_points = gtk_plot_data_get_numpoints(xyp->plot_data[line_num]);
  if (num_points >= 500) num_points = 0;

  xyp->x_data[line_num][num_points] = x;
  xyp->y_data[line_num][num_points] = y;
  gtk_plot_data_set_numpoints(xyp->plot_data[line_num], num_points + 1);

  xyp->num_undrawn_points[line_num]++;

  if (num_points == 250) xy_plot_refresh_slow(xyp);
}

void xy_plot_draw_line(XYPlot *xyp, GtkPlotLine pl, double x0, double y0,
                       double x1, double y1) {
  double px0, py0, px1, py1;
  gtk_plot_get_pixel(xyp->plot, x0, y0, &px0, &py0);
  gtk_plot_get_pixel(xyp->plot, x1, y1, &px1, &py1);
  gtk_plot_draw_line(xyp->plot, pl, px0, py0, px1, py1);
}

void xy_plot_set_xyrange(XYPlot *xyp, double x_min, double x_max, double y_min,
                         double y_max) {
  gtk_plot_set_xrange(xyp->plot, x_min, x_max);
  double x_major_step = 0.2 * (x_max - x_min);
  assert(x_major_step > 0.0);
  gtk_plot_set_ticks(xyp->plot, GTK_PLOT_AXIS_X, x_major_step, 4);

  gtk_plot_set_yrange(xyp->plot, y_min, y_max);
  double y_major_step = 0.2 * (y_max - y_min);
  assert(y_major_step > 0.0);
  gtk_plot_set_ticks(xyp->plot, GTK_PLOT_AXIS_Y, y_major_step, 4);

  xy_plot_refresh_slow(xyp);
}

void xy_plot_refresh(XYPlot *xyp) {
  GtkPlotPC *old = xyp->plot->pc;
  xyp->plot->pc = xyp->canvas->pc;
  GList *dataset = xyp->plot->data_sets;
  int32_t i = 0;
  while (dataset) {
    if (GTK_IS_PLOT_DATA(dataset->data))
      gtk_plot_data_draw_points(dataset->data, xyp->num_undrawn_points[i]);
    dataset = dataset->next;
    xyp->num_undrawn_points[i] = 0;
    i++;
  }
  xyp->plot->pc = old;

  gtk_plot_canvas_refresh(xyp->canvas);
}

void xy_plot_refresh_slow(XYPlot *xyp) {
  GdkColor c = gtk_widget_get_style(GTK_WIDGET(xyp))->bg[GTK_STATE_NORMAL];
  gtk_plot_canvas_set_background(xyp->canvas, &c);
  if (xyp->draw_background) {
    GtkPlotPC *old = xyp->plot->pc;
    xyp->plot->pc = xyp->canvas->pc;
    xyp->draw_background(xyp);
    xyp->plot->pc = old;
  }
  gtk_plot_canvas_refresh(xyp->canvas);
  for (int32_t i = 0; i < XY_PLOT_MAX_LINES; ++i)
    xyp->num_undrawn_points[i] = 0;
}
