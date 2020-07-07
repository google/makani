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

#ifndef GS_MONITOR_FILTERS_FILTERS_AIO_H_
#define GS_MONITOR_FILTERS_FILTERS_AIO_H_

#include <stdbool.h>
#include <stdint.h>

#include "gs/aio_snapshot/aio_telemetry.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t control_telemetries[kNumControllers];
  int32_t control_slow_telemetries[kNumControllers];
  int32_t controller_q7_slow_statuses[kNumControllers];
  int32_t core_switch_slow_statuses[kNumCoreSwitches];
  int32_t core_switch_statuses[kNumCoreSwitches];
  int32_t drum_sensors[kNumDrums];
  int32_t flight_computer_sensors[kNumFlightComputers];
  int32_t ground_station_weather;
  int32_t gs_gps_compass;
  int32_t gs_gps_observations;
  int32_t gs_gps_solution;
  int32_t joystick;
  int32_t joystick_monitor;
  int32_t loadcell_statuses[kNumLoadcellNodes];
  int32_t motor_statuses[kNumMotors];
  int32_t platform_sensors[kNumPlatforms];
  int32_t recorder_q7_slow_statuses[kNumRecorderQ7s];
  int32_t recorder_statuses[kNumRecorderTms570s];
  int32_t self_test[kNumAioNodes];
  int32_t servo_statuses[kNumServos];
  int32_t slow_statuses[kNumAioNodes];
  int32_t tether_down[2];
  int32_t winch_plc;
  int32_t wing_gps_novatel_observations[kNumWingGpsReceivers];
  int32_t wing_gps_novatel_solutions[kNumWingGpsReceivers];
  int32_t wing_gps_septentrio_observations[kNumWingGpsReceivers];
  int32_t wing_gps_septentrio_solutions[kNumWingGpsReceivers];
} FilterAioStaleCounts;

void FilterAioTelemetry(
    const AioTelemetry1 *aio_telem_1, bool aio_telem_1_updated,
    const AioTelemetry2 *aio_telem_2, bool aio_telem_2_updated,
    const AioTelemetry3 *aio_telem_3, bool aio_telem_3_updated,
    int32_t *aio_telem_stale_count, FilterAioStaleCounts *aio_stale_counts);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_FILTERS_FILTERS_AIO_H_
