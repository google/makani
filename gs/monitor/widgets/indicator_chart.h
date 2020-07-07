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

#ifndef GS_MONITOR_WIDGETS_INDICATOR_CHART_H_
#define GS_MONITOR_WIDGETS_INDICATOR_CHART_H_

#include <glib-object.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>

#include "gs/monitor/widgets/chart.h"
#include "gs/monitor/widgets/indicator.h"

G_BEGIN_DECLS

#define INDICATOR_CHART_TYPE (indicator_chart_get_type())
#define INDICATOR_CHART(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), INDICATOR_CHART_TYPE, IndicatorChart))
#define INDICATOR_CHART_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), INDICATOR_CHART_TYPE, IndicatorChartClass))
#define IS_INDICATOR_CHART(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), INDICATOR_CHART_TYPE))
#define IS_INDICATOR_CHART_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass), INDICATOR_CHART_TYPE))

typedef struct _IndicatorChart IndicatorChart;
typedef struct _IndicatorChartClass IndicatorChartClass;

struct _IndicatorChart {
  GtkHBox box;
  Indicator *indicator;
  Chart *chart;
};

struct _IndicatorChartClass {
  GtkHBoxClass parent_class;
  void (*indicator_chart)(IndicatorChart *indicator_chart);
};

GType indicator_chart_get_type(void);
GtkWidget *indicator_chart_new(const char *label_str, int num_lines);

G_END_DECLS

#endif  // GS_MONITOR_WIDGETS_INDICATOR_CHART_H_
