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

#include "gs/monitor/widgets/chart.h"

#include <assert.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <math.h>
#include <stdlib.h>

#include "gs/monitor/widgets/widget_colors.h"

static void chart_class_init(ChartClass *class __attribute__((unused))) {}

static void chart_init(Chart *ch) {
  ch->x_data = malloc(CHART_MAX_POINTS * sizeof(double));
  for (int32_t i = 0; i < CHART_MAX_LINES; ++i)
    ch->y_data[i] = malloc(CHART_MAX_POINTS * sizeof(double));

  double y_min = -6, y_max = 6;
  // Create plot
  ch->plot = GTK_PLOT(gtk_plot_new_with_size(NULL, 1, 1));

  gtk_plot_y0_set_visible(ch->plot, TRUE);
  gtk_plot_grids_set_visible(ch->plot, TRUE, FALSE, TRUE, FALSE);
  gtk_plot_major_vgrid_set_attributes(ch->plot, GTK_PLOT_LINE_SOLID, 0.1f,
                                      &BLACK);
  gtk_plot_major_hgrid_set_attributes(ch->plot, GTK_PLOT_LINE_SOLID, 0.1f,
                                      &BLACK);

  gtk_plot_set_range(ch->plot, 0, 10, y_min, y_max);
  gtk_plot_set_ticks(ch->plot, GTK_PLOT_AXIS_X, 1, 4);
  gtk_plot_set_ticks(ch->plot, GTK_PLOT_AXIS_Y, (y_max - y_min) / 5, 4);

  // Hide axis titles and labels
  gtk_plot_axis_hide_title(gtk_plot_get_axis(ch->plot, GTK_PLOT_AXIS_LEFT));
  gtk_plot_axis_hide_title(gtk_plot_get_axis(ch->plot, GTK_PLOT_AXIS_BOTTOM));
  gtk_plot_axis_show_labels(gtk_plot_get_axis(ch->plot, GTK_PLOT_AXIS_BOTTOM),
                            GTK_PLOT_LABEL_NONE);
  gtk_plot_axis_show_labels(gtk_plot_get_axis(ch->plot, GTK_PLOT_AXIS_TOP),
                            GTK_PLOT_LABEL_NONE);

  gtk_plot_axis_set_labels_style(
      gtk_plot_get_axis(ch->plot, GTK_PLOT_AXIS_LEFT), GTK_PLOT_LABEL_FLOAT, 0);
  gtk_plot_hide_legends(ch->plot);
  gtk_plot_clip_data(ch->plot, TRUE);
  gtk_widget_show(GTK_WIDGET(ch->plot));  // Shouldn't be necessary, but is

  ch->num_lines = CHART_MAX_LINES;
  // Create plot_data
  for (int32_t i = 0; i < CHART_MAX_LINES; ++i) {
    ch->plot_data[i] = GTK_PLOT_DATA(gtk_plot_data_new());
    gtk_plot_add_data(ch->plot, ch->plot_data[i]);
    gtk_plot_data_set_numpoints(ch->plot_data[i], 0);
    gtk_plot_data_set_line_attributes(ch->plot_data[i], GTK_PLOT_LINE_SOLID, 0,
                                      0, 1.5, COLORS[i]);
    gtk_plot_data_set_x(ch->plot_data[i], ch->x_data);
    gtk_plot_data_set_y(ch->plot_data[i], ch->y_data[i]);

    // These widget_shows shouldn't be necessary but are
    gtk_widget_show(GTK_WIDGET(ch->plot_data[i]));
  }

  ch->num_undrawn_points = 0;

  // Add and pack everything
  ch->canvas = GTK_PLOT_CANVAS(gtk_plot_canvas_new(490, 100, 1.0));
  GtkPlotCanvasChild *child = gtk_plot_canvas_plot_new(ch->plot);
  gtk_plot_canvas_put_child(ch->canvas, child, 0.10, 0.04, 1, 0.92);
  gtk_box_pack_start(GTK_BOX(&ch->box), GTK_WIDGET(ch->canvas), FALSE, FALSE,
                     2);
}

GType chart_get_type(void) {
  static GType ch_type = 0;
  if (!ch_type) {
    const GTypeInfo ch_info = {
        sizeof(ChartClass),
        NULL,  // base_init
        NULL,  // base_finalize
        (GClassInitFunc)chart_class_init,
        NULL,  // class_finalize
        NULL,  // class_data
        sizeof(Chart),
        0,  // n_preallocs
        (GInstanceInitFunc)chart_init,
        NULL  // value_table
    };
    ch_type = g_type_register_static(GTK_TYPE_HBOX, "Chart", &ch_info, 0);
  }
  return ch_type;
}

GtkWidget *chart_new(int32_t num_lines) {
  Chart *ch = g_object_new(chart_get_type(), NULL);
  chart_set_num_lines(ch, num_lines);
  return GTK_WIDGET(ch);
}

void chart_set_num_lines(Chart *ch, int32_t num_lines) {
  ch->num_lines = num_lines;
}

void chart_set_style(Chart *ch, ChartStyle style) {
  switch (style) {
    default:
    case CHART_STYLE_NORMAL:
      gtk_plot_canvas_set_size(ch->canvas, 490, 100);
      break;

    case CHART_STYLE_DOUBLE:
      gtk_plot_canvas_set_size(ch->canvas, 490, 200);
      break;
  }
}

// Adds a single new set of points for a specific time.
void chart_add_points(Chart *ch, double x, const double points[]) {
  int32_t num_points = gtk_plot_data_get_numpoints(ch->plot_data[0]);
  if (num_points == 0 || num_points >= CHART_MAX_POINTS ||
      ch->x_data[num_points - 1] > fmod(x, 10.0)) {
    num_points = 0;
  }

  for (int32_t i = 0; i < ch->num_lines; ++i) {
    ch->x_data[num_points] = fmod(x, 10.0);
    ch->y_data[i][num_points] = points[i];
    gtk_plot_data_set_numpoints(ch->plot_data[i], num_points + 1);
  }

  ch->num_undrawn_points++;

  if (num_points == 0) chart_refresh_slow(ch);
}

void chart_set_yrange(Chart *ch, double y_min, double y_max) {
  gtk_plot_set_yrange(ch->plot, y_min, y_max);
  double major_step = (y_max - y_min) / 4.0;
  assert(major_step > 0.0);
  gtk_plot_set_ticks(ch->plot, GTK_PLOT_AXIS_Y, major_step, 4);
  gtk_plot_axis_set_labels_style(
      gtk_plot_get_axis(ch->plot, GTK_PLOT_AXIS_LEFT), GTK_PLOT_LABEL_FLOAT,
      (int32_t)fmax(-log10(y_max - y_min) + 3.0, 0.0));
  chart_refresh_slow(ch);
}

// The normal (i.e. quick) refresh only adds the undrawn points to the
// plot.  We only refresh if there is more than one undrawn point so
// that we can continue drawing a line from the previous point.
void chart_refresh(Chart *ch) {
  if (ch->num_undrawn_points > 1) {
    GtkPlotPC *old = ch->plot->pc;
    ch->plot->pc = ch->canvas->pc;
    GList *dataset = ch->plot->data_sets;
    while (dataset) {
      if (GTK_IS_PLOT_DATA(dataset->data))
        gtk_plot_data_draw_points(dataset->data, ch->num_undrawn_points - 1);
      dataset = dataset->next;
    }
    ch->plot->pc = old;

    gtk_plot_canvas_refresh(ch->canvas);
    ch->num_undrawn_points = 1;
  }
}

// This slow refresh redraws everything (axes labels, grid, etc...).
// It is called only when the plots roll over the time window.
void chart_refresh_slow(Chart *ch) {
  GdkColor c = gtk_widget_get_style(GTK_WIDGET(ch))->bg[GTK_STATE_NORMAL];
  gtk_plot_canvas_set_background(ch->canvas, &c);
  gtk_plot_canvas_refresh(ch->canvas);
  ch->num_undrawn_points = 0;
}
