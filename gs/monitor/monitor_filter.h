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

#ifndef GS_MONITOR_MONITOR_FILTER_H_
#define GS_MONITOR_MONITOR_FILTER_H_

#include <stdbool.h>
#include <stdint.h>

#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/system_types.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "gs/monitor/filters/filters_aio.h"
#include "gs/monitor/monitor_util.h"
#include "system/labels.h"

#ifdef __cplusplus
extern "C" {
#endif

// Buffer for up to 1 minute averages at 250Hz.
#define MON_1MIN_SAMPLE_BUFFER 15000

// Authochecks structs
typedef struct {
  double magnitude;
  double mean[3], diff[3];
  double stdev[3], skewness[3], kurtosis[3];
} ACIMUStats;

typedef struct {
  double mean[kNumLoadcellSensors], stdev[kNumLoadcellSensors];
} ACLoadcellStats;

typedef struct {
  double static_mean, static_stdev;
  double diff_mean[3], diff_stdev[3];
} ACPitotStats;

typedef struct {
  double gps_sigmas_mean[3];
  ACIMUStats acc, acc_lowg, mag, gyro;
  ACLoadcellStats loadcell;
  ACPitotStats pitot;
} ACData;

typedef struct {
  bool enabled, latched_fault;
  bool stacking_level_fault[NUM_STACKING_LEVELS];
} FmmFilterData;

// Filter data struct
typedef struct {
  uint64_t seq;
  double t;
  double invalid_rate;
  int64_t time_in_autonomous_sec, time_in_crosswind_sec;
  ControllerLabel leader;
  int32_t aio_telem_stale_count;
  FilterAioStaleCounts aio_stale_counts;
  int32_t bbox_stale_count;
  FmmFilterData fmm;
  bool gps_dropout;
  double power_gen, power_gen_f_1min, power_gen_f_5min;
  double wind_speed, wind_speed_f_1min, wind_speed_f_5min, wind_speed_std;
  double servo_temps_f[kNumServos];
  double servo_loads_f[kNumServos];
  FaultMask latched_motor_faults[kNumMotors];
  ACData autochecks;
  double power_per_loop;
  double current_gps_receiver_time;
  int32_t loop_number;
} FilterData;

// Nice short abbreviations for everything
extern const ControlSlowTelemetry *cst;
extern const ControlTelemetry *ct;
extern const ControlTelemetry *ct_z1;
extern const ControllerCommandMessage *cc;
extern const AioTelemetry1 *aio_1;
extern const AioTelemetry2 *aio_2;
extern const AioTelemetry3 *aio_3;
extern const FilterData *fd;

void RunFilter(const AioTelemetry1 *aio_telem_1_in, bool aio_telem_1_updated,
               const AioTelemetry2 *aio_telem_2_in, bool aio_telem_2_updated,
               const AioTelemetry3 *aio_telem_3_in, bool aio_telem_3_updated);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_MONITOR_FILTER_H_
