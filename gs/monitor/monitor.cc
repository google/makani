// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gs/monitor/monitor.h"

#include <assert.h>
#include <gflags/gflags.h>
#include <gtk/gtk.h>
#include <gtkextra/gtkextra.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/pack_control_telemetry.h"
#include "control/system_params.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "gs/aio_snapshot/pack_aio_telemetry.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_params.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/heading.h"
#include "gs/monitor/widgets/indicator.h"
#include "gs/monitor/widgets/indicator_chart.h"
#include "gs/monitor/widgets/polar_plot.h"
#include "gs/monitor/widgets/xy_plot.h"
#include "lib/json_load/load_params.h"
#include "lib/udpio/udpio.h"

udp_config aio_telem_1_cfg;
udp_config aio_telem_2_cfg;
udp_config aio_telem_3_cfg;

static const int32_t kTimerHandlerPeriodMs = 25;

// TimerHandler is called every kTimerHandlerPeriodMs milliseconds
// (currently at 40 Hz).  It then reads in up to kMaxNumMessages
// messages sent from aio_snapshot (generally 2 or 3 as the snapshot
// tool transmits at 100 Hz) and passes each in turn to RunFilter.
static gboolean TimerHandler(MonitorConfig *mconfig) {
  const int32_t kMaxNumMessages = 30;
  MonitorWidgetConfig *mwconfig = mconfig->mwconfig;

  int32_t num_messages = 0;
  bool aio_telem_1_updated = true;
  bool aio_telem_2_updated = true;
  bool aio_telem_3_updated = true;
  AioTelemetry1 aio_telem_1;
  AioTelemetry2 aio_telem_2;
  AioTelemetry3 aio_telem_3;
  while ((aio_telem_1_updated || aio_telem_2_updated || aio_telem_3_updated) &&
         num_messages < kMaxNumMessages) {
    UDP_RECV_PACKED(&aio_telem_1_cfg, UnpackAioTelemetry1, &aio_telem_1,
                    &aio_telem_1_updated);
    UDP_RECV_PACKED(&aio_telem_2_cfg, UnpackAioTelemetry2, &aio_telem_2,
                    &aio_telem_2_updated);
    UDP_RECV_PACKED(&aio_telem_3_cfg, UnpackAioTelemetry3, &aio_telem_3,
                    &aio_telem_3_updated);

    if (aio_telem_1_updated || aio_telem_2_updated || aio_telem_3_updated) {
      num_messages++;
    }
    RunFilter(&aio_telem_1, aio_telem_1_updated, &aio_telem_2,
              aio_telem_2_updated, &aio_telem_3, aio_telem_3_updated);

    for (int32_t i = 0; i < MONITOR_MAX_WIDGETS && mwconfig[i].parent; ++i) {
      UpdateFuncType UpdateFunc = *mwconfig[i].update_func;
      UpdateFunc(mconfig->widgets[i], FALSE);
    }
  }

  uint8_t buf[UDP_BUFLEN];
  if (num_messages == kMaxNumMessages) {
    while (udp_recv(&aio_telem_1_cfg, buf) > 0) {
    }
    while (udp_recv(&aio_telem_2_cfg, buf) > 0) {
    }
    while (udp_recv(&aio_telem_3_cfg, buf) > 0) {
    }
    printf("\nDropping data (packets arriving too fast to process)\n");
  }

  // Update charts.
  for (int32_t i = 0; i < MONITOR_MAX_WIDGETS && mwconfig[i].parent; ++i) {
    if (mwconfig[i].widget_type == INDICATOR_CHART_TYPE) {
      chart_refresh(INDICATOR_CHART(mconfig->widgets[i])->chart);
    } else if (mwconfig[i].widget_type == XY_PLOT_TYPE) {
      xy_plot_refresh(XY_PLOT(mconfig->widgets[i]));
    } else if (mwconfig[i].widget_type == POLAR_PLOT_TYPE) {
      polar_plot_refresh(POLAR_PLOT(mconfig->widgets[i]));
    }
  }

  return TRUE;
}

int main(int argc, char *argv[]) {
  google::SetUsageMessage("");
  google::ParseCommandLineFlags(&argc, &argv, true);

  json_load::LoadControlParams(GetControlParamsUnsafe());
  json_load::LoadSystemParams(GetSystemParamsUnsafe());
  json_load::LoadMonitorParams(GetMonitorParamsUnsafe());

  udp_setup_listener(&aio_telem_1_cfg,
                     GetSystemParams()->comms.udpio.aio_telemetry_1_remote_port,
                     1, 1);
  udp_setup_listener(&aio_telem_2_cfg,
                     GetSystemParams()->comms.udpio.aio_telemetry_2_remote_port,
                     1, 1);
  udp_setup_listener(&aio_telem_3_cfg,
                     GetSystemParams()->comms.udpio.aio_telemetry_3_remote_port,
                     1, 1);

  gtk_init(&argc, &argv);

  GtkWidget *window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

  MonitorConfig mconfig;
  memset(&mconfig, 0, sizeof(mconfig));
  ConfigMonitor(argc, argv, window, &mconfig);
  MonitorWidgetConfig *mwconfig = mconfig.mwconfig;

  for (int32_t i = 0; i < MONITOR_MAX_WIDGETS && mwconfig[i].parent; ++i) {
    // Create widgets.
    if (mconfig.widgets[i] != NULL) {
      // Allow for initialized widgets.
    } else if (mwconfig[i].widget_type == HEADING_TYPE) {
      mconfig.widgets[i] = heading_new("");
    } else if (mwconfig[i].widget_type == INDICATOR_TYPE) {
      mconfig.widgets[i] = indicator_new("");
    } else if (mwconfig[i].widget_type == INDICATOR_CHART_TYPE) {
      mconfig.widgets[i] = indicator_chart_new("", 3);
    } else if (mwconfig[i].widget_type == XY_PLOT_TYPE) {
      mconfig.widgets[i] = xy_plot_new(3);
    } else if (mwconfig[i].widget_type == POLAR_PLOT_TYPE) {
      mconfig.widgets[i] = polar_plot_new(3);
    }
    gtk_box_pack_start(GTK_BOX(mwconfig[i].parent), mconfig.widgets[i], FALSE,
                       FALSE, 3);

    // Initialize update functions.
    UpdateFuncType UpdateFunc = *mwconfig[i].update_func;
    UpdateFunc(mconfig.widgets[i], TRUE);
  }

  // Makes the "X" button work.
  g_signal_connect(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit),
                   NULL);

  // We set this to default idle because it is more important that the
  // indicators change colors, etc. than we get recent data.
  g_timeout_add_full(G_PRIORITY_DEFAULT_IDLE, (uint32_t)kTimerHandlerPeriodMs,
                     (GSourceFunc)TimerHandler, &mconfig, NULL);

  gtk_widget_show_all(window);

  // Refresh charts after show_all so they get the correct background
  // color.
  for (int32_t i = 0; i < MONITOR_MAX_WIDGETS && mwconfig[i].parent; ++i) {
    if (mwconfig[i].widget_type == INDICATOR_CHART_TYPE) {
      chart_refresh_slow(INDICATOR_CHART(mconfig.widgets[i])->chart);
    } else if (mwconfig[i].widget_type == XY_PLOT_TYPE) {
      xy_plot_refresh_slow(XY_PLOT(mconfig.widgets[i]));
    } else if (mwconfig[i].widget_type == POLAR_PLOT_TYPE) {
      polar_plot_refresh_slow(POLAR_PLOT(mconfig.widgets[i]));
    }
  }

  gtk_main();

  return 0;
}
