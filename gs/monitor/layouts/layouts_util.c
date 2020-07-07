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

#include "gs/monitor/layouts/layouts_util.h"

#include <gtk/gtk.h>

// Offset the monitors so they display on the large screen monitors
// in the command trailer, without being cut-off at the edges.
void OffsetGtkBox(GtkWidget *fixed, GtkWidget *box) {
  gtk_fixed_put(GTK_FIXED(fixed), box, 20, 20);
}
