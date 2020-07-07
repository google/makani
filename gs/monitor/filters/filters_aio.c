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

#include "gs/monitor/filters/filters_aio.h"

#include <stdbool.h>
#include <stdint.h>

#include "common/macros.h"
#include "control/control_types.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "gs/monitor/monitor_util.h"  // For ShouldInitialize.
#include "system/labels.h"

static void UpdateStaleCounter(bool updated, int32_t *counter) {
  if (updated) {
    *counter = 0;
  } else if (*counter < INT32_MAX) {
    (*counter)++;
  }
}

// These macros are only used in FilterAioTelemetry to reduce
// repetition.
#define RESET_STALE_COUNT(field) aio_stale_counts_z1.field = INT32_MAX;

#define RESET_STALE_COUNT_ARRAY(field)                                 \
  for (int32_t i = 0; i < ARRAYSIZE(aio_stale_counts_z1.field); ++i) { \
    aio_stale_counts_z1.field[i] = INT32_MAX;                          \
  }

#define UPDATE_STALE_COUNT(telem, telem_updated, field)       \
  UpdateStaleCounter(telem_updated && telem->field##_updated, \
                     &aio_stale_counts_z1.field);             \
  aio_stale_counts->field = aio_stale_counts_z1.field;

#define UPDATE_STALE_COUNT_ARRAY(telem, telem_updated, field)          \
  for (int32_t i = 0; i < ARRAYSIZE(aio_stale_counts_z1.field); ++i) { \
    UpdateStaleCounter(telem_updated && telem->field##_updated[i],     \
                       &aio_stale_counts_z1.field[i]);                 \
    aio_stale_counts->field[i] = aio_stale_counts_z1.field[i];         \
  }

COMPILE_ASSERT(ARRAYSIZE(((const FilterAioStaleCounts *)0)->tether_down) ==
                   ARRAYSIZE(((const AioTelemetry1 *)0)->tether_down),
               TetherDown_array_lengths_do_not_match);

void FilterAioTelemetry(
    const AioTelemetry1 *aio_telem_1, bool aio_telem_1_updated,
    const AioTelemetry2 *aio_telem_2, bool aio_telem_2_updated,
    const AioTelemetry3 *aio_telem_3, bool aio_telem_3_updated,
    int32_t *aio_telem_stale_count, FilterAioStaleCounts *aio_stale_counts) {
  static int32_t aio_telem_stale_count_z1;
  static FilterAioStaleCounts aio_stale_counts_z1;
  if (ShouldInitialize()) {
    aio_telem_stale_count_z1 = INT32_MAX;

    RESET_STALE_COUNT_ARRAY(control_telemetries);
    RESET_STALE_COUNT_ARRAY(control_slow_telemetries);
    RESET_STALE_COUNT_ARRAY(controller_q7_slow_statuses);
    RESET_STALE_COUNT_ARRAY(core_switch_slow_statuses);
    RESET_STALE_COUNT_ARRAY(core_switch_statuses);
    RESET_STALE_COUNT_ARRAY(drum_sensors);
    RESET_STALE_COUNT_ARRAY(flight_computer_sensors);
    RESET_STALE_COUNT(ground_station_weather);
    RESET_STALE_COUNT(gs_gps_compass);
    RESET_STALE_COUNT(gs_gps_observations);
    RESET_STALE_COUNT(gs_gps_solution);
    RESET_STALE_COUNT(joystick);
    RESET_STALE_COUNT(joystick_monitor);
    RESET_STALE_COUNT_ARRAY(loadcell_statuses);
    RESET_STALE_COUNT_ARRAY(motor_statuses);
    RESET_STALE_COUNT_ARRAY(platform_sensors);
    RESET_STALE_COUNT_ARRAY(recorder_q7_slow_statuses);
    RESET_STALE_COUNT_ARRAY(recorder_statuses);
    RESET_STALE_COUNT_ARRAY(self_test);
    RESET_STALE_COUNT_ARRAY(servo_statuses);
    RESET_STALE_COUNT_ARRAY(slow_statuses);
    RESET_STALE_COUNT_ARRAY(tether_down);
    RESET_STALE_COUNT(winch_plc);
    RESET_STALE_COUNT_ARRAY(wing_gps_novatel_observations);
    RESET_STALE_COUNT_ARRAY(wing_gps_novatel_solutions);
    RESET_STALE_COUNT_ARRAY(wing_gps_septentrio_observations);
    RESET_STALE_COUNT_ARRAY(wing_gps_septentrio_solutions);
  }

  UpdateStaleCounter(
      aio_telem_1_updated || aio_telem_2_updated || aio_telem_3_updated,
      &aio_telem_stale_count_z1);
  *aio_telem_stale_count = aio_telem_stale_count_z1;

  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           control_telemetries);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           control_slow_telemetries);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_2, aio_telem_1_updated,
                           controller_q7_slow_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_2, aio_telem_1_updated,
                           core_switch_slow_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           core_switch_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated, drum_sensors);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           flight_computer_sensors);
  UPDATE_STALE_COUNT(aio_telem_1, aio_telem_1_updated, ground_station_weather);
  UPDATE_STALE_COUNT(aio_telem_1, aio_telem_1_updated, gs_gps_compass);
  UPDATE_STALE_COUNT(aio_telem_1, aio_telem_1_updated, gs_gps_observations);
  UPDATE_STALE_COUNT(aio_telem_1, aio_telem_1_updated, gs_gps_solution);
  UPDATE_STALE_COUNT(aio_telem_1, aio_telem_1_updated, joystick);
  UPDATE_STALE_COUNT(aio_telem_1, aio_telem_1_updated, joystick_monitor);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated, loadcell_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated, motor_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated, platform_sensors);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_2, aio_telem_1_updated,
                           recorder_q7_slow_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated, recorder_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated, servo_statuses);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated, tether_down);
  UPDATE_STALE_COUNT(aio_telem_1, aio_telem_1_updated, winch_plc);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           wing_gps_novatel_observations);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           wing_gps_novatel_solutions);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           wing_gps_septentrio_observations);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_1, aio_telem_1_updated,
                           wing_gps_septentrio_solutions);

  UPDATE_STALE_COUNT_ARRAY(aio_telem_3, aio_telem_3_updated, self_test);
  UPDATE_STALE_COUNT_ARRAY(aio_telem_2, aio_telem_2_updated, slow_statuses);
}

#undef RESET_STALE_COUNT
#undef RESET_STALE_COUNT_ARRAY
#undef UPDATE_STALE_COUNT
#undef UPDATE_STALE_COUNT_ARRAY
