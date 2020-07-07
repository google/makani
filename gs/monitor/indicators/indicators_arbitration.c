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

#include "gs/monitor/indicators/indicators_arbitration.h"

#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"
#include "system/labels.h"

void UpdateArbitrationMonitor(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Monitor");

  if (!CheckAioComms() || !CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  MON_PRINTF(ind, "%s", ControllerLabelToString(fd->leader));

  if (fd->leader == kControllerA) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  }
}

void UpdateArbitrationControllers(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Controllers");

  if (!CheckAioComms() || !CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool warning = false;
  const char *no_label = "-";
  const char *leaders[kNumControllers];
  for (int32_t i = 0; i < kNumControllers; ++i) {
    if (CheckControllerComms(i)) {
      ControllerLabel leader = aio_1->control_telemetries[i].sync.leader;
      warning = warning || (fd->leader != leader);
      leaders[i] = ControllerLabelToString(leader);
    } else {
      leaders[i] = no_label;
    }
  }

  MON_PRINTF(ind, "A: %s    B: %s    C: %s", leaders[0], leaders[1],
             leaders[2]);

  if (warning) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}
