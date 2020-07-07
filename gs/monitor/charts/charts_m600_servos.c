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

#include "gs/monitor/charts/charts_m600_servos.h"

#include <stdint.h>

#include "common/c_math/util.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator_chart.h"
#include "system/labels.h"

static void ChartServoSidePositions(
    IndicatorChart *ich, const ServoLabel servo_labels[NUM_SIDE_SERVOS]) {
  if (!CheckServoComms(servo_labels[0]) && !CheckServoComms(servo_labels[1]) &&
      !CheckServoComms(servo_labels[2])) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  double servo_angle[2 * NUM_SIDE_SERVOS] = {
      (180.0 / PI) * aio_1->servo_statuses[servo_labels[0]].angle_estimate,
      (180.0 / PI) * aio_1->servo_statuses[servo_labels[1]].angle_estimate,
      (180.0 / PI) * aio_1->servo_statuses[servo_labels[2]].angle_estimate,
      (180.0 / PI) * (double)cc->servo_angle[servo_labels[0]],
      (180.0 / PI) * (double)cc->servo_angle[servo_labels[1]],
      (180.0 / PI) * (double)cc->servo_angle[servo_labels[2]]};

  MON_PRINTF(ich->indicator, "%s:% 0.0f\n%s:% 0.0f\n%s:% 0.0f",
             ServoLabelToString(servo_labels[0]), servo_angle[0],
             ServoLabelToString(servo_labels[1]), servo_angle[1],
             ServoLabelToString(servo_labels[2]), servo_angle[2]);
  chart_add_points(ich->chart, fd->t, servo_angle);
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
}

void UpdateServoPortPosChart(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Port Ail Pos [deg]");
    chart_set_num_lines(ich->chart, 2 * NUM_SIDE_SERVOS);
    chart_set_yrange(ich->chart, -120.0, 120.0);
  }

  if (!CheckAioComms()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  const ServoLabel servo_labels[NUM_SIDE_SERVOS] = {kServoA1, kServoA2,
                                                    kServoA4};

  ChartServoSidePositions(ich, servo_labels);
}

void UpdateServoStarPosChart(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Star Ail Pos [deg]");
    chart_set_num_lines(ich->chart, 2 * NUM_SIDE_SERVOS);
    chart_set_yrange(ich->chart, -120.0, 120.0);
  }

  if (!CheckAioComms()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  const ServoLabel servo_labels[NUM_SIDE_SERVOS] = {kServoA5, kServoA7,
                                                    kServoA8};

  ChartServoSidePositions(ich, servo_labels);
}

static void ChartServoTailPositions(
    IndicatorChart *ich, const ServoLabel servo_labels[NUM_TAIL_SERVOS]) {
  if (!CheckServoComms(servo_labels[0]) && !CheckServoComms(servo_labels[1])) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  double servo_angle[2 * NUM_TAIL_SERVOS] = {
      (180.0 / PI) * aio_1->servo_statuses[servo_labels[0]].angle_estimate,
      (180.0 / PI) * aio_1->servo_statuses[servo_labels[1]].angle_estimate,
      (180.0 / PI) * (double)cc->servo_angle[servo_labels[0]],
      (180.0 / PI) * (double)cc->servo_angle[servo_labels[1]]};

  MON_PRINTF(ich->indicator, "%s:% 0.0f\n%s:% 0.0f",
             ServoLabelToString(servo_labels[0]), servo_angle[0],
             ServoLabelToString(servo_labels[1]), servo_angle[1]);
  chart_add_points(ich->chart, fd->t, servo_angle);
  indicator_set_state(ich->indicator, INDICATOR_STATE_GOOD);
}

void UpdateServoElePosChart(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Ele Pos [deg]");
    chart_set_num_lines(ich->chart, 2 * NUM_TAIL_SERVOS);
    chart_set_yrange(ich->chart, -120.0, 120.0);
  }

  if (!CheckAioComms()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  const ServoLabel servo_labels[NUM_TAIL_SERVOS] = {kServoE1, kServoE2};

  ChartServoTailPositions(ich, servo_labels);
}

void UpdateServoRudPosChart(IndicatorChart *ich, int32_t init) {
  if (init) {
    indicator_set_label(ich->indicator, "Rud Pos [deg]");
    chart_set_num_lines(ich->chart, 2 * NUM_TAIL_SERVOS);
    chart_set_yrange(ich->chart, -120.0, 120.0);
  }

  if (!CheckAioComms()) {
    indicator_set_state(ich->indicator, INDICATOR_STATE_NONE);
    return;
  }

  const ServoLabel servo_labels[NUM_TAIL_SERVOS] = {kServoR1, kServoR2};

  ChartServoTailPositions(ich, servo_labels);
}
