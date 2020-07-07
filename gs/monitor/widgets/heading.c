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

#include "gs/monitor/widgets/heading.h"

static void heading_class_init(HeadingClass *class __attribute__((unused))) {}

static void heading_init(Heading *hdg) {
  hdg->label = GTK_LABEL(gtk_label_new("label"));
  gtk_widget_modify_font(GTK_WIDGET(hdg->label),
                         pango_font_description_from_string("Bold"));
  gtk_box_pack_start(GTK_BOX(&hdg->box), GTK_WIDGET(hdg->label), TRUE, TRUE, 0);
}

GType heading_get_type(void) {
  static GType hdg_type = 0;
  if (!hdg_type) {
    const GTypeInfo hdg_info = {
        sizeof(HeadingClass),
        NULL,  // base_init
        NULL,  // base_finalize
        (GClassInitFunc)heading_class_init,
        NULL,  // class_finalize
        NULL,  // class_data
        sizeof(Heading),
        0,  // n_preallocs
        (GInstanceInitFunc)heading_init,
        NULL  // value_table
    };
    hdg_type = g_type_register_static(GTK_TYPE_HBOX, "Heading", &hdg_info, 0);
  }
  return hdg_type;
}

GtkWidget *heading_new(const char *label_str) {
  Heading *hdg = g_object_new(heading_get_type(), NULL);
  heading_set_label(hdg, label_str);
  return GTK_WIDGET(hdg);
}

void heading_set_label(Heading *hdg, const char *label_str) {
  g_return_if_fail(hdg != NULL);
  g_return_if_fail(IS_HEADING(hdg));
  gtk_label_set_text(hdg->label, label_str);
}

const char *heading_get_label(const Heading *hdg) {
  return gtk_label_get_text(hdg->label);
}
