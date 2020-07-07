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

#include "gs/monitor/filters/filters_controller.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "control/common.h"
#include "control/control_types.h"
#include "control/estimator/estimator_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/monitor/monitor_util.h"  // For ShouldInitialize.
#include "system/labels.h"

void FilterDataPerLoop(double loop_angle, double power_gen,
                       FlightMode flight_mode, int32_t *loop_number,
                       double *power_per_loop) {
  static int32_t loop_count;
  static uint32_t per_loop_count;
  static double loop_angle_z1, cumean_loop_power, mean_loop_power_z1;
  if (ShouldInitialize()) {
    loop_count = 0;
    per_loop_count = 0;
    loop_angle_z1 = loop_angle;
    cumean_loop_power = power_gen;
    mean_loop_power_z1 = power_gen;
  }

  // Count crosswind loops.
  double dloop_angle = loop_angle - loop_angle_z1;
  if ((fabs(dloop_angle) > PI / 2.0) && AnyCrosswindFlightMode(flight_mode)) {
    loop_count = loop_count + Sign(dloop_angle);

    // Note that resetting per loop counter resets cummulative mean power
    // calculation.
    per_loop_count = 0;

    // Record last loop's mean power.
    mean_loop_power_z1 = cumean_loop_power;
  }

  // Find cummulative mean power for each loop.
  // TODO: Add a recursive cumulative mean function to filter.c.
  cumean_loop_power =
      (per_loop_count * cumean_loop_power + power_gen) / (per_loop_count + 1.0);
  per_loop_count = per_loop_count + 1;

  loop_angle_z1 = loop_angle;

  *loop_number = loop_count;
  *power_per_loop = mean_loop_power_z1;
}

bool FilterGpsDropout(const FaultMask *gps_pos_fault,
                      const FaultMask *gps_vel_fault,
                      const FaultDetectionGpsParams *params,
                      double filter_time) {
  static double dropout_begin_time;
  if (ShouldInitialize()) {
    dropout_begin_time = filter_time;
  }

  // If there are no GPS faults, reset the dropout timer
  if (!HasAnyFault(gps_pos_fault) && !HasAnyFault(gps_vel_fault)) {
    dropout_begin_time = filter_time;
  }

  double threshold = (double)params->no_update_counts_limit[0] * (*g_sys.ts);
  return (filter_time - dropout_begin_time) > threshold;
}

void FilterLatchedMotorFaults(const FaultMask motor_faults[],
                              FaultMask latched_motor_faults[]) {
  static FaultMask latched_motor_faults_z1[kNumMotors];
  static int32_t counters[kNumMotors];
  if (ShouldInitialize()) {
    for (int32_t i = 0; i < kNumMotors; ++i) {
      latched_motor_faults[i] = motor_faults[i];
      counters[i] = 0;
    }
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    bool faulted = HasAnyFault(&motor_faults[i]);
    bool latched = LatchOn((int32_t)faulted, 1.0, *g_sys.ts, &counters[i]);
    if (faulted || !latched) latched_motor_faults_z1[i] = motor_faults[i];
    latched_motor_faults[i] = latched_motor_faults_z1[i];
  }
}

double FilterTime(FlightMode flight_mode, int64_t *time_in_autonomous_sec,
                  int64_t *time_in_crosswind_sec) {
  static struct timeval tv0;
  static FlightMode flight_mode_z1;
  if (ShouldInitialize()) {
    gettimeofday(&tv0, NULL);
    flight_mode_z1 = flight_mode;
  }

  struct timeval tv;
  gettimeofday(&tv, NULL);

  // Elapsed time since perch-ascend
  static struct timeval tv0_autonomous = {0, 0};
  if (AnyCrosswindFlightMode(flight_mode) ||
      AnyAutoHoverFlightMode(flight_mode)) {
    if (flight_mode_z1 == kFlightModePerched ||
        (tv0_autonomous.tv_sec == 0 && tv0_autonomous.tv_usec == 0)) {
      gettimeofday(&tv0_autonomous, NULL);
    }
    *time_in_autonomous_sec = tv.tv_sec - tv0_autonomous.tv_sec;
  } else {
    *time_in_autonomous_sec = 0;
  }

  // Ellapsed time in crosswind
  static struct timeval tv0_crosswind = {0, 0};
  if (AnyCrosswindFlightMode(flight_mode)) {
    if (!AnyCrosswindFlightMode(flight_mode_z1) ||
        (tv0_crosswind.tv_sec == 0 && tv0_crosswind.tv_usec == 0)) {
      gettimeofday(&tv0_crosswind, NULL);
    }
    *time_in_crosswind_sec = tv.tv_sec - tv0_crosswind.tv_sec;
  } else {
    *time_in_crosswind_sec = 0;
  }

  // Update persistent state.
  flight_mode_z1 = flight_mode;

  return (double)(tv.tv_sec - tv0.tv_sec) +
         1e-6 * (double)(tv.tv_usec - tv0.tv_usec);
}

double FilterCurrentGpsReceiverTime(int32_t current_gps_receiver) {
  static int32_t last_gps_receiver;
  static double time;
  if (ShouldInitialize()) {
    last_gps_receiver = current_gps_receiver;
    time = INFINITY;
  }
  if (last_gps_receiver != current_gps_receiver) {
    time = 0.0;
  } else {
    time += *g_sys.ts;
  }

  last_gps_receiver = current_gps_receiver;

  return time;
}
