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

#include "gs/monitor/charts/charts_m600_motors.h"

#include <float.h>
#include <math.h>
#include <stdint.h>

#include "avionics/network/aio_labels.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator_chart.h"
#include "system/labels.h"

const MotorLabel kMotorOrder[] = {kMotorSbo, kMotorSbi, kMotorPbi, kMotorPbo,
                                  kMotorPto, kMotorPti, kMotorSti, kMotorSto};

static void ChartMotorRowSpeeds(IndicatorChart *ich,
                                const MotorLabel motor_labels[NUM_ROW_MOTORS]) {
  if (!CheckAioComms()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }
  if (!CheckMotorComms(motor_labels[0]) && !CheckMotorComms(motor_labels[1]) &&
      !CheckMotorComms(motor_labels[2]) && !CheckMotorComms(motor_labels[3])) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  double omega[2 * NUM_ROW_MOTORS] = {
      (double)aio_1->motor_statuses[motor_labels[0]].omega,
      (double)aio_1->motor_statuses[motor_labels[1]].omega,
      (double)aio_1->motor_statuses[motor_labels[2]].omega,
      (double)aio_1->motor_statuses[motor_labels[3]].omega,
      (double)aio_1->motor_statuses[motor_labels[0]].omega_upper_limit,
      (double)aio_1->motor_statuses[motor_labels[1]].omega_upper_limit,
      (double)aio_1->motor_statuses[motor_labels[2]].omega_upper_limit,
      (double)aio_1->motor_statuses[motor_labels[3]].omega_upper_limit};

  MON_PRINTF(ich->indicator, "%s:% 0.0f\n%s:% 0.0f\n%s:% 0.0f\n%s:% 0.0f",
             MotorLabelToString(motor_labels[0]), omega[0],
             MotorLabelToString(motor_labels[1]), omega[1],
             MotorLabelToString(motor_labels[2]), omega[2],
             MotorLabelToString(motor_labels[3]), omega[3]);
  chart_add_points(ich->chart, fd->t, omega);

  if (fabs(MinArray(omega, NUM_ROW_MOTORS, NULL)) < 1.0 &&
      fabs(MaxArray(omega, NUM_ROW_MOTORS, NULL)) < 1.0) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorSpeedsBottom(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Speed [rad/s]");
    chart_set_num_lines(ich->chart, 2 * NUM_ROW_MOTORS);
    chart_set_yrange(ich->chart, -180.0, 20.0);
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPbo, kMotorPbi,
                                                   kMotorSbi, kMotorSbo};
  ChartMotorRowSpeeds(ich, motor_labels);
}

void UpdateMotorSpeedsTop(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Speed [rad/s]");
    chart_set_num_lines(ich->chart, 2 * NUM_ROW_MOTORS);
    chart_set_yrange(ich->chart, -20.0, 180.0);
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPto, kMotorPti,
                                                   kMotorSti, kMotorSto};
  ChartMotorRowSpeeds(ich, motor_labels);
}

void UpdateStackBusPower(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Power (bus) [kW]");
    chart_set_num_lines(ich->chart, 1);
    chart_set_yrange(ich->chart, -1000.0, 1000.0);
  }

  if (!CheckAioComms()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  int32_t num_part_levels = 0;
  int32_t num_full_levels = 0;
  double v_stack = 0.0;
  double i_stack = 0.0;
  for (int32_t i = 0; i < kNumMotors / 2; ++i) {
    int32_t i_a = kMotorOrder[i];
    int32_t i_b = kMotorOrder[i + kNumMotors / 2];
    bool updated_a = CheckMotorComms(i_a);
    bool updated_b = CheckMotorComms(i_b);

    if (updated_a && updated_b) {
      v_stack += 0.5 * ((double)aio_1->motor_statuses[i_a].bus_voltage +
                        (double)aio_1->motor_statuses[i_b].bus_voltage);
      i_stack += (double)aio_1->motor_statuses[i_a].bus_current +
                 (double)aio_1->motor_statuses[i_b].bus_current;
      num_part_levels++;
      num_full_levels++;
    } else if (updated_a) {
      v_stack += (double)aio_1->motor_statuses[i_a].bus_voltage;
      num_part_levels++;
    } else if (updated_b) {
      v_stack += (double)aio_1->motor_statuses[i_b].bus_voltage;
      num_part_levels++;
    }
  }

  if (num_full_levels > 0) {
    i_stack = i_stack / (double)num_full_levels;
    v_stack = v_stack * (double)(kNumMotors / 2) / (double)num_part_levels;
    double p_stack_kw = i_stack * v_stack * 0.001;
    MON_PRINTF(ich->indicator,
               "Power: % 3.0f kW\nV:     % 3.0f V\nI:     % 3.0f A\n",
               p_stack_kw, v_stack, i_stack);
    chart_add_points(ich->chart, fd->t, &p_stack_kw);

    if (num_full_levels == kNumMotors / 2) {
      indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
    } else {
      indicator_set_state(ich->indicator, INDICATOR_STATE_WARNING);
    }
  } else {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    MON_PRINTF(ich->indicator,
               "Power: % 3.0f kW\nV:     % 3.0f V\nI:     % 3.0f A\n", 0.0, 0.0,
               0.0);
  }
}
