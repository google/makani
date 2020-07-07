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

#ifndef GS_MONITOR_WIDGETS_CHART_H_
#define GS_MONITOR_WIDGETS_CHART_H_

#include <glib-object.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <stdint.h>

G_BEGIN_DECLS

#define CHART_TYPE (chart_get_type())
#define CHART(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), CHART_TYPE, Chart))
#define CHART_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), CHART_TYPE, ChartClass))
#define IS_CHART(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), CHART_TYPE))
#define IS_CHART_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), CHART_TYPE))

#define CHART_MAX_LINES 8
#define CHART_MAX_POINTS 5000

typedef enum { CHART_STYLE_NORMAL, CHART_STYLE_DOUBLE } ChartStyle;

typedef struct _Chart Chart;
typedef struct _ChartClass ChartClass;

struct _Chart {
  GtkHBox box;
  GtkPlotCanvas *canvas;
  GtkPlot *plot;
  GtkPlotData *plot_data[CHART_MAX_LINES];
  int32_t num_lines;
  double *x_data;
  double *y_data[CHART_MAX_LINES];
  int32_t num_undrawn_points;
};

struct _ChartClass {
  GtkHBoxClass parent_class;
  void (*chart)(Chart *chart);
};

GType chart_get_type(void);
GtkWidget *chart_new(int num_lines);
void chart_set_num_lines(Chart *ch, int32_t num_lines);
void chart_set_style(Chart *ch, ChartStyle style);
void chart_add_points(Chart *ch, double x, const double points[]);
void chart_set_yrange(Chart *ch, double y_min, double y_max);
void chart_refresh(Chart *ch);
void chart_refresh_slow(Chart *ch);

G_END_DECLS

#endif  // GS_MONITOR_WIDGETS_CHART_H_
