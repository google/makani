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

#ifndef GS_MONITOR_LAYOUTS_LAYOUTS_M600_H_
#define GS_MONITOR_LAYOUTS_LAYOUTS_M600_H_

#include <gtk/gtk.h>

#include "gs/monitor/monitor.h"

#ifdef __cplusplus
extern "C" {
#endif

void ConfigArbitrationMonitor(GtkWidget *window, MonitorConfig *mconfig_out);
void ConfigFlightMonitor(GtkWidget *window, MonitorConfig *mconfig_out);
void ConfigGroundStationMonitor(GtkWidget *window, MonitorConfig *mconfig_out);
void ConfigMotorsMonitor(GtkWidget *window, MonitorConfig *mconfig_out);
void ConfigNetworkMonitor(GtkWidget *window, MonitorConfig *mconfig_out);
void ConfigServosMonitor(GtkWidget *window, MonitorConfig *mconfig_out);
void ConfigPilotMonitor(GtkWidget *window, MonitorConfig *mconfig_out);
void ConfigStatusMonitor(GtkWidget *window, MonitorConfig *mconfig);
void ConfigWingSensorsMonitor(GtkWidget *window, MonitorConfig *mconfig_out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_LAYOUTS_LAYOUTS_M600_H_
