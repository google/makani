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

#include <assert.h>
#include <gtk/gtk.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "gs/monitor/layouts/layouts_controller.h"
#include "gs/monitor/layouts/layouts_m600.h"
#include "gs/monitor/monitor.h"

void ConfigMonitor(int argc, char *argv[], GtkWidget *window,
                   MonitorConfig *mconfig_out) {
  char title[40];
  snprintf(title, sizeof(title), "monitor (%s)", argv[1]);
  gtk_window_set_title((GtkWindow *)window, title);

  if (argc <= 1 || !strcmp(argv[1], "flight")) {
    ConfigFlightMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "arbitration")) {
    ConfigArbitrationMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "crosswind")) {
    ConfigCrosswindMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "estimator")) {
    ConfigEstimatorMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "fault_detection")) {
    ConfigFaultDetectionMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "gs")) {
    ConfigGroundStationMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "hover")) {
    ConfigHoverMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "manual")) {
    ConfigManualMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "motors")) {
    ConfigMotorsMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "network")) {
    ConfigNetworkMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "pilot")) {
    ConfigPilotMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "servos")) {
    ConfigServosMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "status")) {
    ConfigStatusMonitor(window, mconfig_out);
  } else if (!strcmp(argv[1], "sensors")) {
    ConfigWingSensorsMonitor(window, mconfig_out);
  } else {
    assert(!(bool)"Invalid layout name.");
  }
}
