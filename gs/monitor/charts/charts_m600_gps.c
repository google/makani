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

#include "gs/monitor/charts/charts_m600_gps.h"

#include <stdbool.h>
#include <stdint.h>

#include "common/c_math/vec3.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator_chart.h"
#include "system/labels.h"
#include "system/labels_util.h"

void UpdateGsGpsToWingDist(IndicatorChart *ich, int32_t init) {
  const WingGpsReceiverLabel wing_gps_label = kWingGpsReceiverCrosswind;

  if (init) {
    indicator_set_label(ich->indicator, "GS GPS to Wing [m]");
    chart_set_num_lines(ich->chart, 1);
    chart_set_yrange(ich->chart, 0.0, 500.0);
  }
  if (!CheckAioComms() || !CheckWingGpsSolutionComms(wing_gps_label) ||
      !CheckGsGpsSolutionComms()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  Vec3 wing_pos_ecef = kVec3Zero;
  GpsReceiverType gps_type =
      WingGpsReceiverLabelToGpsReceiverType(wing_gps_label);

  if (gps_type == kGpsReceiverTypeNovAtel) {
    const NovAtelLogBestXyz *best_xyz =
        &aio_1->wing_gps_novatel_solutions[wing_gps_label].best_xyz;
    wing_pos_ecef.x = best_xyz->pos_x;
    wing_pos_ecef.y = best_xyz->pos_y;
    wing_pos_ecef.z = best_xyz->pos_z;
  } else if (gps_type == kGpsReceiverTypeSeptentrio) {
    const SeptentrioBlockPvtCartesian *pvt =
        &aio_1->wing_gps_septentrio_solutions[wing_gps_label].pvt_cartesian;
    wing_pos_ecef.x = pvt->x;
    wing_pos_ecef.y = pvt->y;
    wing_pos_ecef.z = pvt->z;
  } else {
    assert(!(bool)"Invalid GPS type.");
  }

  Vec3 gs_gps_pos_ecef = {aio_1->gs_gps_solution.best_xyz.pos_x,
                          aio_1->gs_gps_solution.best_xyz.pos_y,
                          aio_1->gs_gps_solution.best_xyz.pos_z};
  Vec3 diff;
  Vec3Sub(&wing_pos_ecef, &gs_gps_pos_ecef, &diff);

  double dist = Vec3Norm(&diff);
  MON_PRINTF(ich->indicator, "%g", dist);
  chart_add_points(ich->chart, fd->t, &dist);
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
}
