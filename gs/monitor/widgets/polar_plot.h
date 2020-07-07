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

#ifndef GS_MONITOR_WIDGETS_POLAR_PLOT_H_
#define GS_MONITOR_WIDGETS_POLAR_PLOT_H_

#include <glib-object.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <stdint.h>

G_BEGIN_DECLS

#define POLAR_PLOT_TYPE (polar_plot_get_type())
#define POLAR_PLOT(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), POLAR_PLOT_TYPE, PolarPlot))
#define POLAR_PLOT_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), POLAR_PLOT_TYPE, PolarPlotClass))
#define IS_POLAR_PLOT(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), POLAR_PLOT_TYPE))
#define IS_POLAR_PLOT_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass), POLAR_PLOT_TYPE))

#define POLAR_PLOT_MAX_LINES 3
#define POLAR_PLOT_MAX_POINTS 5000

typedef struct _PolarPlot PolarPlot;
typedef struct _PolarPlotClass PolarPlotClass;

struct _PolarPlot {
  GtkHBox box;
  GtkPlotCanvas *canvas;
  GtkPlot *plot;
  GtkPlotData *plot_data[POLAR_PLOT_MAX_LINES];
  int32_t num_lines;
  double *r_data[POLAR_PLOT_MAX_LINES];
  double *th_data[POLAR_PLOT_MAX_LINES];
  int32_t num_undrawn_points[POLAR_PLOT_MAX_LINES];
  void (*draw_background)(PolarPlot *pp);
};

struct _PolarPlotClass {
  GtkHBoxClass parent_class;
  void (*polar_plot)(PolarPlot *polar_plot);
};

GType polar_plot_get_type(void);
GtkWidget *polar_plot_new(int num_lines);
void polar_plot_set_num_lines(PolarPlot *pp, int32_t num_lines);
void polar_plot_add_points(PolarPlot *pp, int32_t line_num, double th,
                           double r);
void polar_plot_draw_line(PolarPlot *pp, GtkPlotLine pl, double th0, double r0,
                          double th1, double r1);
void polar_plot_refresh(PolarPlot *pp);
void polar_plot_refresh_slow(PolarPlot *pp);

G_END_DECLS

#endif  // GS_MONITOR_WIDGETS_POLAR_PLOT_H_
