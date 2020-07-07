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

#include "gs/monitor/indicators/indicators_m600_motors.h"

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/motor_thermal_types.h"
#include "avionics/motor/firmware/flags.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "gs/monitor/monitor.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_params.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"
#include "system/labels.h"

static const char *ConvFmmToStr(bool fmm) { return fmm ? "On " : "Off"; }

static const char *ConvFmmFaultToStr(bool fault) { return fault ? "F" : "-"; }

void UpdateFmmProxy(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "FMM Proxy");
  }

  if (!CheckAioComms() || !CheckAnyMotorComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if (!fd->fmm.enabled || fd->fmm.latched_fault) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "%s L1: %s L2: %s L3: %s L4: %s",
             ConvFmmToStr(fd->fmm.enabled),
             ConvFmmFaultToStr(fd->fmm.stacking_level_fault[0]),
             ConvFmmFaultToStr(fd->fmm.stacking_level_fault[1]),
             ConvFmmFaultToStr(fd->fmm.stacking_level_fault[2]),
             ConvFmmFaultToStr(fd->fmm.stacking_level_fault[3]));
}

static void DisplayMotorRowArmed(
    Indicator *ind, const MotorLabel motor_labels[NUM_ROW_MOTORS]) {
  if (!CheckMotorComms(motor_labels[0]) && !CheckMotorComms(motor_labels[1]) &&
      !CheckMotorComms(motor_labels[2]) && !CheckMotorComms(motor_labels[3])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool motor_armed[NUM_ROW_MOTORS];
  bool all_armed = true;
  for (int32_t i = 0; i < NUM_ROW_MOTORS; ++i) {
    motor_armed[i] = aio_1->motor_statuses[motor_labels[i]].motor_status &
                     (kMotorStatusArmed | kMotorStatusRunning);
    all_armed &= motor_armed[i];
  }

  MON_PRINTF(ind, "%s:%d %s:%d %s:%d %s:%d",
             MotorLabelToString(motor_labels[0]), motor_armed[0],
             MotorLabelToString(motor_labels[1]), motor_armed[1],
             MotorLabelToString(motor_labels[2]), motor_armed[2],
             MotorLabelToString(motor_labels[3]), motor_armed[3]);

  if (!all_armed) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorsArmedBottom(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Armed");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPbo, kMotorPbi,
                                                   kMotorSbi, kMotorSbo};
  DisplayMotorRowArmed(ind, motor_labels);
}

void UpdateMotorsArmedTop(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Armed");

  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPto, kMotorPti,
                                                   kMotorSti, kMotorSto};
  DisplayMotorRowArmed(ind, motor_labels);
}

void UpdateMotorErrors(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Motor Errors");

  if (!CheckAioComms() || !CheckAnyMotorComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  static char error_list[kNumMotors][32];
  int32_t num_errors = 0;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (aio_1->motor_statuses[(MotorLabel)i].motor_error) {
      snprintf(error_list[num_errors], sizeof(error_list[num_errors]), "%s",
               MotorLabelToString((MotorLabel)i));
      num_errors++;
    }
  }
  assert(num_errors <= kNumMotors);

  // The display cycles through the error list at 1Hz, so if the list is
  // changing rapidly, we might miss an error. That is OK as the purpose of this
  // indicator is to alert a human operator that there is a faulty motor, which
  // will be achieved if the indicator is cycling/jittering or if some errors
  // persist. Look at the logs for a reliable record of motor errors.
  if (num_errors > 0) {
    indicator_set_value(ind, error_list[((int32_t)fd->t) % num_errors]);
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_value(ind, "None");
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorWarnings(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Motor Warnings");

  if (!CheckAioComms() || !CheckAnyMotorComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  static char warning_list[kNumMotors][32];
  int32_t num_warnings = 0;
  for (int32_t i = 0; i < kNumMotors; ++i) {
    if (aio_1->motor_statuses[(MotorLabel)i].motor_warning) {
      snprintf(warning_list[num_warnings], sizeof(warning_list[num_warnings]),
               "%s", MotorLabelToString((MotorLabel)i));
      num_warnings++;
    }
  }
  assert(num_warnings <= kNumMotors);

  if (num_warnings > 0) {
    indicator_set_value(ind, warning_list[((int32_t)fd->t) % num_warnings]);
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_value(ind, "None");
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

// Find and display the maximum thermistor temperature in each
// thermistor of a row of motors. Reference: https://goo.gl/AxSenh.
// TODO: All motor temperature limits should be in a config
// parameters file.
static void DisplayMotorControllerRowTemps(Indicator *ind,
                                           const MotorLabel motor_labels[]) {
  if (!CheckMotorComms(motor_labels[0]) && !CheckMotorComms(motor_labels[1]) &&
      !CheckMotorComms(motor_labels[2]) && !CheckMotorComms(motor_labels[3])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool issue_error = false;
  bool issue_warning = false;
  double controller_temp[NUM_ROW_MOTORS];
  for (int32_t i = 0; i < NUM_ROW_MOTORS; ++i) {
    const float *temps = aio_1->motor_statuses[motor_labels[i]].temps;
    double controller_therm[] = {(double)temps[kMotorThermalChannelHeatPlate1],
                                 (double)temps[kMotorThermalChannelHeatPlate2]};
    controller_temp[i] =
        MaxArray(controller_therm, ARRAYSIZE(controller_therm), NULL);
    if (controller_temp[i] > 80.0) {
      issue_error = true;
    } else if (controller_temp[i] > 75.0) {
      issue_warning = true;
    }
  }
  MON_PRINTF(ind, "%s:%0.0f %s:%0.0f %s:%0.0f %s:%0.0f",
             MotorLabelToString(motor_labels[0]), controller_temp[0],
             MotorLabelToString(motor_labels[1]), controller_temp[1],
             MotorLabelToString(motor_labels[2]), controller_temp[2],
             MotorLabelToString(motor_labels[3]), controller_temp[3]);
  if (issue_error) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (issue_warning) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorControllerTempsBottom(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Drive temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPbo, kMotorPbi,
                                                   kMotorSbi, kMotorSbo};
  DisplayMotorControllerRowTemps(ind, motor_labels);
}

void UpdateMotorControllerTempsTop(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Drive temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPto, kMotorPti,
                                                   kMotorSti, kMotorSto};
  DisplayMotorControllerRowTemps(ind, motor_labels);
}

static void DisplayMotorBoardRowTemps(Indicator *ind,
                                      const MotorLabel motor_labels[]) {
  if (!CheckMotorComms(motor_labels[0]) && !CheckMotorComms(motor_labels[1]) &&
      !CheckMotorComms(motor_labels[2]) && !CheckMotorComms(motor_labels[3])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool issue_error = false;
  bool issue_warning = false;
  double board_temps[NUM_ROW_MOTORS];
  for (int32_t i = 0; i < NUM_ROW_MOTORS; ++i) {
    const float *temps = aio_1->motor_statuses[motor_labels[i]].temps;
    board_temps[i] = temps[kMotorThermalChannelBoard];
    if (board_temps[i] >= 80.0) {
      issue_error = true;
    } else if (board_temps[i] >= 75.0) {
      issue_warning = true;
    }
  }
  MON_PRINTF(ind, "%s:%0.0f %s:%0.0f %s:%0.0f %s:%0.0f",
             MotorLabelToString(motor_labels[0]), board_temps[0],
             MotorLabelToString(motor_labels[1]), board_temps[1],
             MotorLabelToString(motor_labels[2]), board_temps[2],
             MotorLabelToString(motor_labels[3]), board_temps[3]);
  if (issue_error) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (issue_warning) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorBoardTempsBottom(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Board temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPbo, kMotorPbi,
                                                   kMotorSbi, kMotorSbo};
  DisplayMotorBoardRowTemps(ind, motor_labels);
}

void UpdateMotorBoardTempsTop(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Board temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPto, kMotorPti,
                                                   kMotorSti, kMotorSto};
  DisplayMotorBoardRowTemps(ind, motor_labels);
}

// Gin3 motor controller capacitor temperature limits set for hover
// flight.
// Reference: https://docs.google.com/document/d/1zTc3MilYnQU2vVccUihkX-
//            dp5eFZQf2dw46jYRM8i6s/edit#heading=h.7n10hmu1b6r3.
static void DisplayMotorCapRowTemps(Indicator *ind,
                                    const MotorLabel motor_labels[]) {
  if (!CheckMotorComms(motor_labels[0]) && !CheckMotorComms(motor_labels[1]) &&
      !CheckMotorComms(motor_labels[2]) && !CheckMotorComms(motor_labels[3])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool issue_error = false;
  bool issue_warning = false;
  double cap_temps[NUM_ROW_MOTORS];
  for (int32_t i = 0; i < NUM_ROW_MOTORS; ++i) {
    const float *temps = aio_1->motor_statuses[motor_labels[i]].temps;
    cap_temps[i] = temps[kMotorThermalChannelCapacitor];
    if (cap_temps[i] >= 90.0) {
      issue_error = true;
    } else if (cap_temps[i] >= 85.0) {
      issue_warning = true;
    }
  }
  MON_PRINTF(ind, "%s:%0.0f %s:%0.0f %s:%0.0f %s:%0.0f",
             MotorLabelToString(motor_labels[0]), cap_temps[0],
             MotorLabelToString(motor_labels[1]), cap_temps[1],
             MotorLabelToString(motor_labels[2]), cap_temps[2],
             MotorLabelToString(motor_labels[3]), cap_temps[3]);
  if (issue_error) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (issue_warning) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorCapTempsBottom(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Cap temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPbo, kMotorPbi,
                                                   kMotorSbi, kMotorSbo};
  DisplayMotorCapRowTemps(ind, motor_labels);
}

void UpdateMotorCapTempsTop(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Cap temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPto, kMotorPti,
                                                   kMotorSti, kMotorSto};
  DisplayMotorCapRowTemps(ind, motor_labels);
}

// Motor temperature limits set for hover flight with YASA motors. See thermal
// derating requirements at the link below:
// https://drive.google.com/file/d/0BxXAZAboMu_QT1VYT1pGNTQ1eU0/
// NOTE: These monitors are therefore deprecated for Protean runs on the
// iron bird.
static void DisplayMotorRowTemps(Indicator *ind,
                                 const MotorLabel motor_labels[]) {
  if (!CheckMotorComms(motor_labels[0]) && !CheckMotorComms(motor_labels[1]) &&
      !CheckMotorComms(motor_labels[2]) && !CheckMotorComms(motor_labels[3])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool issue_error = false;
  bool issue_warning = false;
  double stator_core_therm[NUM_ROW_MOTORS];
  double winding_therm[NUM_ROW_MOTORS];
  const MonThermalLimitsParams *limits = &GetMonitorParams()->thermal;
  const double kMaxValidTemperature = 1200.0;
  for (int32_t i = 0; i < NUM_ROW_MOTORS; ++i) {
    const float *temps = aio_1->motor_statuses[motor_labels[i]].temps;
    stator_core_therm[i] = (double)temps[kMotorThermalChannelStatorCore];
    winding_therm[i] = (double)temps[kMotorThermalChannelStatorCoil];
    if ((limits->motor_stator_core.very_high < stator_core_therm[i] &&
         stator_core_therm[i] < kMaxValidTemperature) ||
        (limits->motor_stator_winding.very_high < winding_therm[i] &&
         winding_therm[i] < kMaxValidTemperature)) {
      issue_error = true;
    } else if ((limits->motor_stator_core.high < stator_core_therm[i] &&
                stator_core_therm[i] < kMaxValidTemperature) ||
               (limits->motor_stator_winding.high < winding_therm[i] &&
                winding_therm[i] < kMaxValidTemperature)) {
      issue_warning = true;
    }
  }

  MON_PRINTF(ind,
             "c %s:%0.0f %s:%0.0f %s:%0.0f %s:%0.0f\n"
             "w %s:%0.0f %s:%0.0f %s:%0.0f %s:%0.0f",
             MotorLabelToString(motor_labels[0]), stator_core_therm[0],
             MotorLabelToString(motor_labels[1]), stator_core_therm[1],
             MotorLabelToString(motor_labels[2]), stator_core_therm[2],
             MotorLabelToString(motor_labels[3]), stator_core_therm[3],
             MotorLabelToString(motor_labels[0]), winding_therm[0],
             MotorLabelToString(motor_labels[1]), winding_therm[1],
             MotorLabelToString(motor_labels[2]), winding_therm[2],
             MotorLabelToString(motor_labels[3]), winding_therm[3]);

  if (issue_error) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (issue_warning) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorTempsBottom(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Stator temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPbo, kMotorPbi,
                                                   kMotorSbi, kMotorSbo};
  DisplayMotorRowTemps(ind, motor_labels);
}

void UpdateMotorTempsTop(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Stator temps [C]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPto, kMotorPti,
                                                   kMotorSti, kMotorSto};
  DisplayMotorRowTemps(ind, motor_labels);
}

static void DisplayMotorRowVoltages(
    Indicator *ind, const MotorLabel motor_labels[NUM_ROW_MOTORS]) {
  if (!CheckMotorComms(motor_labels[0]) && !CheckMotorComms(motor_labels[1]) &&
      !CheckMotorComms(motor_labels[2]) && !CheckMotorComms(motor_labels[3])) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool issue_error = false;
  bool issue_warning = false;
  for (int32_t i = 0; i < NUM_ROW_MOTORS; ++i) {
    double bus_voltage = aio_1->motor_statuses[motor_labels[i]].bus_voltage;
    if (bus_voltage >= 940.0) {
      issue_error = true;
    } else if (bus_voltage <= 820.0 || 900.0 <= bus_voltage) {
      issue_warning = true;
    }
  }

  MON_PRINTF(ind, "%s:%0.0f %s:%0.0f %s:%0.0f %s:%0.0f",
             MotorLabelToString(motor_labels[0]),
             aio_1->motor_statuses[motor_labels[0]].bus_voltage,
             MotorLabelToString(motor_labels[1]),
             aio_1->motor_statuses[motor_labels[1]].bus_voltage,
             MotorLabelToString(motor_labels[2]),
             aio_1->motor_statuses[motor_labels[2]].bus_voltage,
             MotorLabelToString(motor_labels[3]),
             aio_1->motor_statuses[motor_labels[3]].bus_voltage);

  if (issue_error) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (issue_warning) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateMotorVoltagesBottom(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Voltages [V]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPbo, kMotorPbi,
                                                   kMotorSbi, kMotorSbo};
  DisplayMotorRowVoltages(ind, motor_labels);
}

void UpdateMotorVoltagesTop(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Voltages [V]");
  }
  if (!CheckAioComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MotorLabel motor_labels[NUM_ROW_MOTORS] = {kMotorPto, kMotorPti,
                                                   kMotorSti, kMotorSto};
  DisplayMotorRowVoltages(ind, motor_labels);
}
