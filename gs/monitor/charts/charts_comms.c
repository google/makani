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

#include "gs/monitor/charts/charts_comms.h"

#include <stdint.h>

#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator_chart.h"
#include "system/labels.h"

void UpdateWingCsPacketsDropped(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Pkt Drop [#]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, 0.0, 20.0);
  }

  if (!CheckAioComms() ||
      (!CheckCsComms(kCoreSwitchA) && !CheckCsComms(kCoreSwitchB))) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  const CoreSwitchStats cs_a_port_status =
      aio_2->core_switch_slow_statuses[kCoreSwitchA].switch_stats;
  const CoreSwitchStats cs_b_port_status =
      aio_2->core_switch_slow_statuses[kCoreSwitchB].switch_stats;

  double global_overflow_count[2] = {0.0, 0.0};
  for (int32_t i = 0; i < NUM_CORE_SWITCH_PORTS; ++i) {
    global_overflow_count[0] += cs_a_port_status.stats[i].tx_dropped_packets;
    global_overflow_count[1] += cs_b_port_status.stats[i].tx_dropped_packets;
  }

  MON_PRINTF(ich->indicator, "CS A: %0.0f\nCS B: %0.0f",
             global_overflow_count[0], global_overflow_count[1]);
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  chart_add_points(ich->chart, fd->t, global_overflow_count);
}
