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

#ifndef GS_MONITOR_WIDGETS_XY_PLOT_H_
#define GS_MONITOR_WIDGETS_XY_PLOT_H_

#include <glib-object.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <stdint.h>

G_BEGIN_DECLS

#define XY_PLOT_TYPE (xy_plot_get_type())
#define XY_PLOT(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), XY_PLOT_TYPE, XYPlot))
#define XY_PLOT_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), XY_PLOT_TYPE, XYPlotClass))
#define IS_XY_PLOT(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), XY_PLOT_TYPE))
#define IS_XY_PLOT_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), XY_PLOT_TYPE))

#define XY_PLOT_MAX_LINES 3
#define XY_PLOT_MAX_POINTS 5000

typedef struct _XYPlot XYPlot;
typedef struct _XYPlotClass XYPlotClass;

struct _XYPlot {
  GtkHBox box;
  GtkPlotCanvas *canvas;
  GtkPlot *plot;
  GtkPlotData *plot_data[XY_PLOT_MAX_LINES];
  int32_t num_lines;
  double *x_data[XY_PLOT_MAX_LINES];
  double *y_data[XY_PLOT_MAX_LINES];
  int32_t num_undrawn_points[XY_PLOT_MAX_LINES];
  void (*draw_background)(XYPlot *xyp);
};

struct _XYPlotClass {
  GtkHBoxClass parent_class;
  void (*xy_plot)(XYPlot *xy_plot);
};

GType xy_plot_get_type(void);
GtkWidget *xy_plot_new(int num_lines);
void xy_plot_set_num_lines(XYPlot *ch, int32_t num_lines);
void xy_plot_add_points(XYPlot *ch, int32_t line_num, double x, double y);
void xy_plot_draw_line(XYPlot *xyp, GtkPlotLine pl, double x0, double y0,
                       double x1, double y1);
void xy_plot_set_xyrange(XYPlot *xyp, double x_min, double x_max, double y_min,
                         double y_max);
void xy_plot_refresh(XYPlot *ch);
void xy_plot_refresh_slow(XYPlot *ch);

G_END_DECLS

#endif  // GS_MONITOR_WIDGETS_XY_PLOT_H_
