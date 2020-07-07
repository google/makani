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

#include "gs/monitor/charts/charts_sensors.h"

#include <stdint.h>

#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator_chart.h"
#include "system/labels.h"

void UpdateLoadcellsPort(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Loadcell Port [N]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, -400.0, 400.0);
  }

  if (!CheckLoadcellComms(kLoadcellNodePortA)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  double loadcell_strain[2] = {
      (double)aio_1->loadcell_statuses[kLoadcellNodePortA]
          .loadcell_data.strain[0]
          .value,
      (double)aio_1->loadcell_statuses[kLoadcellNodePortA]
          .loadcell_data.strain[1]
          .value};

  MON_PRINTF(ich->indicator, "X: %0.0f\nYZ: %0.0f", loadcell_strain[0],
             loadcell_strain[1]);
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  chart_add_points(ich->chart, fd->t, loadcell_strain);
}

void UpdateLoadcellsStarboard(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Loadcell Star [N]");
    chart_set_num_lines(ich->chart, 2);
    chart_set_yrange(ich->chart, -400.0, 400.0);
  }

  if (!CheckLoadcellComms(kLoadcellNodeStarboardA)) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  double loadcell_strain[2] = {
      (double)aio_1->loadcell_statuses[kLoadcellNodeStarboardA]
          .loadcell_data.strain[0]
          .value,
      (double)aio_1->loadcell_statuses[kLoadcellNodeStarboardA]
          .loadcell_data.strain[1]
          .value};

  MON_PRINTF(ich->indicator, "X: %0.0f\nYZ: %0.0f", loadcell_strain[0],
             loadcell_strain[1]);
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  chart_add_points(ich->chart, fd->t, loadcell_strain);
}
