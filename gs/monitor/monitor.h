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

#ifndef GS_MONITOR_MONITOR_H_
#define GS_MONITOR_MONITOR_H_

#include <gtk/gtk.h>

#include "control/control_telemetry.h"
#include "gs/aio_snapshot/aio_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*UpdateFuncType)(GtkWidget *, int32_t);

typedef struct {
  GtkWidget *parent;
  GType widget_type;
  UpdateFuncType update_func;
} MonitorWidgetConfig;

#define MONITOR_MAX_WIDGETS 67
typedef struct {
  MonitorWidgetConfig mwconfig[MONITOR_MAX_WIDGETS];
  GtkWidget *widgets[MONITOR_MAX_WIDGETS];
} MonitorConfig;

void ConfigMonitor(int argc, char *argv[], GtkWidget *window,
                   MonitorConfig *mconfig_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_MONITOR_H_
