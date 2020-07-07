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

#include "gs/monitor/widgets/indicator.h"

#include <assert.h>
#include <string.h>

static void indicator_class_init(
    IndicatorClass *class __attribute__((unused))) {}

static void indicator_init(Indicator *ind) {
  // Create vbox that may be used for the compact indicator
  ind->vbox = GTK_VBOX(gtk_vbox_new(FALSE, 0));

  // Create label (and label_color_box)
  ind->label_color_box = GTK_EVENT_BOX(gtk_event_box_new());
  ind->label = GTK_LABEL(gtk_label_new("label"));
  gtk_widget_modify_font(GTK_WIDGET(ind->label),
                         pango_font_description_from_string("Bold"));
  gtk_misc_set_alignment(GTK_MISC(ind->label), 0.5f, 0.5f);
  gtk_widget_set_size_request(GTK_WIDGET(ind->label), 135, 25);

  // Create value (and value_color_box)
  ind->value_color_box = GTK_EVENT_BOX(gtk_event_box_new());
  ind->value = GTK_LABEL(gtk_label_new(""));
  strncpy(ind->last_value_str, "", sizeof(ind->last_value_str));
  gtk_widget_modify_font(GTK_WIDGET(ind->value),
                         pango_font_description_from_string("Mono 10.0"));
  gtk_misc_set_alignment(GTK_MISC(ind->value), 0.0f, 0.5f);
  gtk_misc_set_padding(GTK_MISC(ind->value), 10.0f, 0.0f);
  GdkColor color;
  gdk_color_parse("#e8e8e8", &color);
  gtk_widget_modify_bg(GTK_WIDGET(ind->value_color_box), GTK_STATE_NORMAL,
                       &color);

  // Add and pack everything but the value.
  // The value gets packed by indicator_set_style.
  gtk_container_add(GTK_CONTAINER(ind->label_color_box),
                    GTK_WIDGET(ind->label));
  gtk_box_pack_start(GTK_BOX(ind->vbox), GTK_WIDGET(ind->label_color_box),
                     FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(ind), GTK_WIDGET(ind->vbox), FALSE, FALSE, 0);
  gtk_container_add(GTK_CONTAINER(ind->value_color_box),
                    GTK_WIDGET(ind->value));

  // Initialize in good state because first call to set_state goes to
  // NONE, which must be different to take effect
  ind->state = INDICATOR_STATE_GOOD;
}

GType indicator_get_type(void) {
  static GType ind_type = 0;
  if (!ind_type) {
    const GTypeInfo ind_info = {
        sizeof(IndicatorClass),
        NULL,  // base_init
        NULL,  // base_finalize
        (GClassInitFunc)indicator_class_init,
        NULL,  // class_finalize
        NULL,  // class_data
        sizeof(Indicator),
        0,  // n_preallocs
        (GInstanceInitFunc)indicator_init,
        NULL  // value_table
    };
    ind_type = g_type_register_static(GTK_TYPE_HBOX, "Indicator", &ind_info, 0);
  }
  return ind_type;
}

GtkWidget *indicator_new(const char *label_str) {
  Indicator *ind = g_object_new(indicator_get_type(), NULL);
  indicator_set_style(ind, INDICATOR_STYLE_NORMAL);
  indicator_set_label(ind, label_str);
  indicator_set_state(ind, INDICATOR_STATE_NONE);
  return GTK_WIDGET(ind);
}

GtkWidget *indicator_new_with_style(const char *label_str,
                                    IndicatorStyle style) {
  Indicator *ind = g_object_new(indicator_get_type(), NULL);
  indicator_set_style(ind, style);
  indicator_set_label(ind, label_str);
  indicator_set_state(ind, INDICATOR_STATE_NONE);
  return GTK_WIDGET(ind);
}

void indicator_set_style(Indicator *ind, IndicatorStyle style) {
  switch (style) {
    default:
    case INDICATOR_STYLE_NORMAL:
      gtk_box_pack_start(GTK_BOX(ind), GTK_WIDGET(ind->value_color_box), FALSE,
                         FALSE, 0);
      gtk_label_set_width_chars(ind->value, 38);
      gtk_label_set_max_width_chars(ind->value, 38);
      break;

    case INDICATOR_STYLE_COMPACT:
      gtk_box_pack_start(GTK_BOX(ind->vbox), GTK_WIDGET(ind->value_color_box),
                         FALSE, FALSE, 0);
      gtk_widget_set_size_request(GTK_WIDGET(ind->value_color_box), 135, 75);
      break;
  }
}

void indicator_set_label(Indicator *ind, const char *label_str) {
  g_return_if_fail(ind != NULL);
  g_return_if_fail(IS_INDICATOR(ind));
  gtk_label_set_text(ind->label, label_str);
}

const char *indicator_get_label(Indicator *ind) {
  assert(ind != NULL);
  assert(IS_INDICATOR(ind));
  return gtk_label_get_text(ind->label);
}

void indicator_set_value(Indicator *ind, const char *value_str) {
  g_return_if_fail(ind != NULL);
  g_return_if_fail(IS_INDICATOR(ind));

  if (strnlen(value_str, INDICATOR_MAX_VALUE_LEN) >= INDICATOR_MAX_VALUE_LEN) {
    value_str = "LABEL TOO LONG.";
  }
  if (strncmp(ind->last_value_str, value_str, sizeof(ind->last_value_str))) {
    gtk_label_set_text(ind->value, value_str);
    strncpy(ind->last_value_str, value_str, sizeof(ind->last_value_str));
  }
}

void indicator_set_state(Indicator *ind, IndicatorState state) {
  g_return_if_fail(ind != NULL);
  g_return_if_fail(IS_INDICATOR(ind));

  if (state != ind->state) {
    ind->state = state;
    GdkColor color;
    // NULL returns text color to default
    gtk_widget_modify_fg(GTK_WIDGET(ind->value), GTK_STATE_NORMAL, NULL);
    switch (state) {
      case INDICATOR_STATE_EMPTY:
        color = gtk_widget_get_style(GTK_WIDGET(ind))->bg[GTK_STATE_NORMAL];
        gtk_widget_modify_bg(GTK_WIDGET(ind->value_color_box), GTK_STATE_NORMAL,
                             &color);
        break;
      default:
      case INDICATOR_STATE_NONE:
        gdk_color_parse("dark gray", &color);
        gtk_widget_modify_fg(GTK_WIDGET(ind->value), GTK_STATE_NORMAL, &color);
        gdk_color_parse("gray", &color);
        break;
      case INDICATOR_STATE_GOOD:
        // Green
        gdk_color_parse("#5ACB5A", &color);
        break;
      case INDICATOR_STATE_WARNING:
        // Yellow
        gdk_color_parse("#FFEF08", &color);
        break;
      case INDICATOR_STATE_ERROR:
        // Red
        gdk_color_parse("#FF3333", &color);
        break;
      case INDICATOR_STATE_ERROR_SPECIAL:
        // Magenta
        gdk_color_parse("#FF11FF", &color);
    }
    gtk_widget_modify_bg(GTK_WIDGET(ind->label_color_box), GTK_STATE_NORMAL,
                         &color);
  }
}
