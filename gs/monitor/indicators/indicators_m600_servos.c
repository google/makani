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

#include "gs/monitor/indicators/indicators_m600_servos.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/servo_types.h"
#include "avionics/firmware/monitors/servo_types.h"
#include "common/macros.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"
#include "system/labels.h"

static void DisplayServoSideArmed(
    Indicator *ind, const ServoLabel servo_labels[NUM_SIDE_SERVOS]) {
  if (!CheckServoComms(servo_labels[0]) && !CheckServoComms(servo_labels[1]) &&
      !CheckServoComms(servo_labels[2])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool servo_armed[NUM_SIDE_SERVOS];
  bool all_armed = true;
  for (int32_t i = 0; i < NUM_SIDE_SERVOS; ++i) {
    servo_armed[i] =
        aio_1->servo_statuses[servo_labels[i]].flags.status & kServoStatusArmed;
    all_armed &= servo_armed[i];
  }

  MON_PRINTF(ind, "%s:%d %s:%d %s:%d", ServoLabelToString(servo_labels[0]),
             servo_armed[0], ServoLabelToString(servo_labels[1]),
             servo_armed[1], ServoLabelToString(servo_labels[2]),
             servo_armed[2]);

  if (!all_armed) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateServosArmedPort(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Port Armed");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const ServoLabel servo_labels[NUM_SIDE_SERVOS] = {kServoA1, kServoA2,
                                                    kServoA4};
  DisplayServoSideArmed(ind, servo_labels);
}

void UpdateServosArmedStarboard(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Star Armed");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const ServoLabel servo_labels[NUM_SIDE_SERVOS] = {kServoA5, kServoA7,
                                                    kServoA8};
  DisplayServoSideArmed(ind, servo_labels);
}

void UpdateServosArmedTail(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Tail Armed");

  const ServoLabel servo_labels[] = {kServoE1, kServoE2, kServoR1, kServoR2};
  if (!CheckAioComms() ||
      (!CheckServoComms(servo_labels[0]) && !CheckServoComms(servo_labels[1]) &&
       !CheckServoComms(servo_labels[2]) &&
       !CheckServoComms(servo_labels[3]))) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool servo_armed[ARRAYSIZE(servo_labels)];
  bool all_armed = true;
  for (int32_t i = 0; i < ARRAYSIZE(servo_labels); ++i) {
    servo_armed[i] =
        aio_1->servo_statuses[servo_labels[i]].flags.status & kServoStatusArmed;

    all_armed &= servo_armed[i];
  }

  MON_PRINTF(ind, "%s:%d %s:%d %s:%d %s:%d",
             ServoLabelToString(servo_labels[0]), servo_armed[0],
             ServoLabelToString(servo_labels[1]), servo_armed[1],
             ServoLabelToString(servo_labels[2]), servo_armed[2],
             ServoLabelToString(servo_labels[3]), servo_armed[3]);

  if (!all_armed) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateLvBusTail(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Tail Bus [V]");
  const ServoLabel servo_labels[] = {kServoE1, kServoE2, kServoR1, kServoR2};
  if (!CheckServoComms(servo_labels[0]) && !CheckServoComms(servo_labels[1]) &&
      !CheckServoComms(servo_labels[2]) && !CheckServoComms(servo_labels[3])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  double voltage_A[ARRAYSIZE(servo_labels)];
  double voltage_B[ARRAYSIZE(servo_labels)];
  bool warning = false;
  for (int32_t i = 0; i < ARRAYSIZE(servo_labels); ++i) {
    const ServoMonitorData *servo_mon_data =
        &aio_1->servo_statuses[servo_labels[i]].servo_mon;

    voltage_A[i] = (double)servo_mon_data->analog_data[kServoAnalogVoltageLvA];
    voltage_B[i] = (double)servo_mon_data->analog_data[kServoAnalogVoltageLvB];

    warning = warning ||
              CheckWarning(&servo_mon_data->flags, kServoMonitorWarningLvA) ||
              CheckWarning(&servo_mon_data->flags, kServoMonitorWarningLvB);
  }

  MON_PRINTF(ind,
             "A %s:%5.1f %s:%5.1f %s:%5.1f %s:%5.1f\n"
             "B %s:%5.1f %s:%5.1f %s:%5.1f %s:%5.1f",
             ServoLabelToString(servo_labels[0]), voltage_A[0],
             ServoLabelToString(servo_labels[1]), voltage_A[1],
             ServoLabelToString(servo_labels[2]), voltage_A[2],
             ServoLabelToString(servo_labels[3]), voltage_A[3],
             ServoLabelToString(servo_labels[0]), voltage_B[0],
             ServoLabelToString(servo_labels[1]), voltage_B[1],
             ServoLabelToString(servo_labels[2]), voltage_B[2],
             ServoLabelToString(servo_labels[3]), voltage_B[3]);

  if (warning) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}
