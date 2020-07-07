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

#include "gs/monitor/filters/filters_power.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "common/c_math/filter.h"
#include "common/c_math/util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"  // For ShouldInitialize.
#include "system/labels.h"

// TODO: This filter should no longer be needed once we have telemetry
// from the Fault Mitigation Module. We can then add voltage thresholds as
// parameters.
//
// From gregleygh by email on 2015-02-24:
//
// The FMM is enabled once both Level 2 and Level 3 rise above 200V.
// The FMM remains enabled until both Level 2 and Level 3 fall below 200V.
// A fault occurs if the voltage on any level exceeds 875V,
// or drops below 100V.
// The FMM will latch and indicate a fault for that level.
void FilterFmmProxy(const MotorStatusMessage motor_status[kNumMotors],
                    FmmFilterData *fmm) {
  static int32_t latch_counter;
  static bool fmm_enabled_z1;
  if (ShouldInitialize()) {
    latch_counter = 0;
    fmm_enabled_z1 = false;
  }

  double stacking_level[NUM_STACKING_LEVELS][2] = {
      {motor_status[kMotorPbi].bus_voltage,
       motor_status[kMotorSti].bus_voltage},
      {motor_status[kMotorSbi].bus_voltage,
       motor_status[kMotorPti].bus_voltage},
      {motor_status[kMotorPbo].bus_voltage,
       motor_status[kMotorSto].bus_voltage},
      {motor_status[kMotorSbo].bus_voltage,
       motor_status[kMotorPto].bus_voltage}};

  const double kFmmFaultLowVoltage = 100.0;
  const double kFmmFaultHighVoltage = 875.0;
  bool any_stacking_level_fault = false;
  for (int32_t i = 0; i < NUM_STACKING_LEVELS; ++i) {
    if (fmm_enabled_z1 &&
        ((MinArray(stacking_level[i], 2, NULL) < kFmmFaultLowVoltage) ||
         (MaxArray(stacking_level[i], 2, NULL) > kFmmFaultHighVoltage))) {
      fmm->stacking_level_fault[i] = true;
      any_stacking_level_fault = true;
    } else {
      fmm->stacking_level_fault[i] = false;
    }
  }

  const double kFmmEnableVoltage = 200.0;
  fmm->enabled = (MinArray(stacking_level[1], 2, NULL) > kFmmEnableVoltage) &&
                 (MinArray(stacking_level[2], 2, NULL) > kFmmEnableVoltage);
  fmm_enabled_z1 = fmm->enabled;

  // Latch the proxy fault state for 5 minutes to allow an operator to notice.
  // We don't latch indefinitely because the monitor proxy for the FMM won't
  // reset with the hardware.
  const double kLatchHold = 300.0;  // [s]
  fmm->latched_fault =
      LatchOn(any_stacking_level_fault, kLatchHold, *g_sys.ts, &latch_counter);
}
