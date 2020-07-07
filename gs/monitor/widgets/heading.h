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

#ifndef GS_MONITOR_WIDGETS_HEADING_H_
#define GS_MONITOR_WIDGETS_HEADING_H_

#include <glib-object.h>
#include <glib.h>

#include <gtk/gtkhbox.h>
#include <gtk/gtklabel.h>

G_BEGIN_DECLS

#define HEADING_TYPE (heading_get_type())
#define HEADING(obj) (G_TYPE_CHECK_INSTANCE_CAST((obj), HEADING_TYPE, Heading))
#define HEADING_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass), HEADING_TYPE, HeadingClass))
#define IS_HEADING(obj) (G_TYPE_CHECK_INSTANCE_TYPE((obj), HEADING_TYPE))
#define IS_HEADING_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass), HEADING_TYPE))

typedef struct {
  GtkHBox box;
  GtkLabel *label;
} Heading;

typedef struct {
  GtkHBoxClass parent_class;
  void (*heading)(Heading *heading);
} HeadingClass;

GType heading_get_type(void);
GtkWidget *heading_new(const char *label_str);

void heading_set_label(Heading *hdg, const char *label_str);
const char *heading_get_label(const Heading *hdg);

G_END_DECLS

#endif  // GS_MONITOR_WIDGETS_HEADING_H_
