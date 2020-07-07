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

#include "gs/monitor/monitor_filter.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/aio_snapshot/aio_telemetry.h"
#include "gs/monitor/filters/filters_aio.h"
#include "gs/monitor/filters/filters_controller.h"
#include "gs/monitor/filters/filters_power.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/monitor_util.h"  // For ShouldInitialize().
#include "system/labels.h"

static FilterData filter_data;
static ControlSlowTelemetry control_slow_telem;
static ControlTelemetry control_telem;
static ControlTelemetry control_telem_z1;
static ControllerCommandMessage controller_command;
static AioTelemetry1 aio_telem_1;
static AioTelemetry2 aio_telem_2;
static AioTelemetry3 aio_telem_3;

// Nice short abbreviations for everything
const FilterData *fd = &filter_data;
const AioTelemetry1 *aio_1 = &aio_telem_1;
const AioTelemetry2 *aio_2 = &aio_telem_2;
const AioTelemetry3 *aio_3 = &aio_telem_3;
const ControlSlowTelemetry *cst = &control_slow_telem;
const ControlTelemetry *ct = &control_telem;
const ControlTelemetry *ct_z1 = &control_telem_z1;
const ControllerCommandMessage *cc = &controller_command;

void RunFilter(const AioTelemetry1 *aio_telem_1_in, bool aio_telem_1_updated,
               const AioTelemetry2 *aio_telem_2_in, bool aio_telem_2_updated,
               const AioTelemetry3 *aio_telem_3_in, bool aio_telem_3_updated) {
  assert(MON_1MIN_SAMPLE_BUFFER >= (uint32_t)ceil(60.0 / (*g_sys.ts)));
  static uint64_t seq;
  if (ShouldInitialize()) {
    seq = 0U;
  }

  // Update communication information.
  if (aio_telem_1_updated) aio_telem_1 = *aio_telem_1_in;
  if (aio_telem_2_updated) aio_telem_2 = *aio_telem_2_in;
  if (aio_telem_3_updated) aio_telem_3 = *aio_telem_3_in;
  FilterAioTelemetry(aio_1, aio_telem_1_updated, aio_2, aio_telem_2_updated,
                     aio_3, aio_telem_3_updated,
                     &filter_data.aio_telem_stale_count,
                     &filter_data.aio_stale_counts);

  const FaultDetectionParams *fd_params = &GetControlParams()->fault_detection;

  // Select the first non-stale controller as the "leader".
  // TODO: Add library functionality to ensure this logic stays
  // synchronized with the controller's leader selection.
  for (int32_t i = 0; i < kNumControllers; ++i) {
    if (filter_data.aio_stale_counts.control_telemetries[i] <
        fd_params->control.no_update_counts_limit) {
      filter_data.leader = (ControllerLabel)i;
      break;
    }
  }
  assert(0 <= filter_data.leader && filter_data.leader < kNumControllers);

  if (aio_telem_1_updated &&
      aio_telem_1.control_telemetries_updated[filter_data.leader]) {
    control_telem_z1 = control_telem;
    control_telem = aio_telem_1.control_telemetries[filter_data.leader];
  }

  if (aio_telem_1_updated &&
      aio_telem_1.control_slow_telemetries_updated[filter_data.leader]) {
    control_slow_telem =
        aio_telem_1.control_slow_telemetries[filter_data.leader];
  }

  if (aio_telem_1_updated &&
      aio_telem_1.controller_commands_updated[filter_data.leader]) {
    controller_command = aio_telem_1.controller_commands[filter_data.leader];
  }

  filter_data.seq = seq++;
  filter_data.t =
      FilterTime(control_telem.flight_mode, &filter_data.time_in_autonomous_sec,
                 &filter_data.time_in_crosswind_sec);

  filter_data.current_gps_receiver_time = FilterCurrentGpsReceiverTime(
      control_telem.estimator.current_gps_receiver);

  // TODO(b/27500432): Handle Septentrio here.
  filter_data.gps_dropout =
      FilterGpsDropout(&control_telem.faults[kSubsysWingGpsCrosswindPos],
                       &control_telem.faults[kSubsysWingGpsCrosswindVel],
                       &fd_params->wing_gps, filter_data.t);
  FilterLatchedMotorFaults(&control_telem.faults[SUBSYS_MOTORS],
                           filter_data.latched_motor_faults);
  FilterDataPerLoop(control_telem.crosswind.loop_angle, filter_data.power_gen,
                    control_telem.flight_mode, &filter_data.loop_number,
                    &filter_data.power_per_loop);
  FilterFmmProxy(aio_1->motor_statuses, &filter_data.fmm);
  CompleteInitialization();
}
