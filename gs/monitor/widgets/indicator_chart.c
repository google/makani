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

#include "gs/monitor/widgets/indicator_chart.h"

#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <math.h>

#include "gs/monitor/widgets/indicator.h"

static void indicator_chart_class_init(
    IndicatorChartClass *class __attribute__((unused))) {}

static void indicator_chart_init(IndicatorChart *ich) {
  ich->indicator =
      INDICATOR(indicator_new_with_style("", INDICATOR_STYLE_COMPACT));
  ich->chart = CHART(chart_new(6));

  // Add and pack everything
  gtk_box_pack_start(GTK_BOX(ich), GTK_WIDGET(ich->indicator), FALSE, FALSE, 0);
  gtk_box_pack_end(GTK_BOX(ich), GTK_WIDGET(ich->chart), FALSE, FALSE, 0);
}

GType indicator_chart_get_type(void) {
  static GType ich_type = 0;
  if (!ich_type) {
    const GTypeInfo ich_info = {
        sizeof(IndicatorChartClass),
        NULL,  // base_init
        NULL,  // base_finalize
        (GClassInitFunc)indicator_chart_class_init,
        NULL,  // class_finalize
        NULL,  // class_data
        sizeof(IndicatorChart),
        0,  // n_preallocs
        (GInstanceInitFunc)indicator_chart_init,
        NULL  // value_table
    };
    ich_type =
        g_type_register_static(GTK_TYPE_HBOX, "IndicatorChart", &ich_info, 0);
  }
  return ich_type;
}

GtkWidget *indicator_chart_new(const char *label_str, int num_lines) {
  IndicatorChart *ich = g_object_new(indicator_chart_get_type(), NULL);
  indicator_set_label(ich->indicator, label_str);
  indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
  chart_set_num_lines(ich->chart, num_lines);
  return GTK_WIDGET(ich);
}
