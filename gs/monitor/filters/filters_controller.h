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

#ifndef GS_MONITOR_FILTERS_FILTERS_CONTROLLER_H_
#define GS_MONITOR_FILTERS_FILTERS_CONTROLLER_H_

#include <stdint.h>

#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void FilterDataPerLoop(double loop_angle, double power_gen,
                       FlightMode flight_mode, int32_t *loop_number,
                       double *power_per_loop);
bool FilterGpsDropout(const FaultMask *gps_pos_fault,
                      const FaultMask *gps_vel_fault,
                      const FaultDetectionGpsParams *params,
                      double filter_time);
void FilterLatchedMotorFaults(const FaultMask motor_faults[],
                              FaultMask latched_motor_faults[]);
double FilterLoopTime(int64_t idle_usec);
double FilterTime(FlightMode flight_mode, int64_t *time_in_autonomous_sec,
                  int64_t *time_in_crosswind_sec);

double FilterCurrentGpsReceiverTime(int32_t current_gps_receiver);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // GS_MONITOR_FILTERS_FILTERS_CONTROLLER_H_
