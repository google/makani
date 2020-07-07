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

#include "gs/monitor/indicators/indicators_controller.h"

#include <assert.h>
#include <math.h>  // For fabs.
#include <stdint.h>

#include "avionics/common/winch_messages.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/c_math/voting.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/control_params.h"
#include "control/control_telemetry.h"
#include "control/control_types.h"
#include "control/crosswind/crosswind_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/hover/hover_types.h"
#include "control/perch_frame.h"
#include "control/sensor_util.h"
#include "control/system_params.h"
#include "control/system_types.h"
#include "gs/monitor/common.h"
#include "gs/monitor/indicators/indicators_util.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_params.h"
#include "gs/monitor/monitor_types.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"
#include "system/labels.h"
#include "system/labels_util.h"

void UpdateAccelStart(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Accel. start");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  double y_offset = ct->state_est.Xg.y - ct->hover.raw_wing_pos_g_cmd.y;

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if (fabs(y_offset) > params->far_accel_start &&
      ct->flight_mode == kFlightModePilotHover)
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  else
    indicator_set_state(ind, INDICATOR_STATE_GOOD);

  if (y_offset > 0.0) {
    MON_PRINTF(ind, "%0.0f m %s", y_offset, "Sprinter right");
  } else {
    MON_PRINTF(ind, "%0.0f m %s", -y_offset, "Sprinter left");
  }
}

void UpdateApparentWind(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Apparent Wind");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if (ct->state_est.apparent_wind.solution_type ==
          kApparentWindSolutionTypePitot ||
      ct->state_est.apparent_wind.solution_type ==
          kApparentWindSolutionTypeComplementary) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }

  MON_PRINTF(ind,
             "Alpha:% 6.1f deg  Velocity:% 3.0f m/s\n"
             "Beta: % 6.1f deg",
             ct->state_est.apparent_wind.sph_f.alpha * 180.0 / PI,
             ct->state_est.apparent_wind.sph_f.v,
             ct->state_est.apparent_wind.sph_f.beta * 180.0 / PI);
}

void UpdateAutoglide(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Autoglide");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
  MON_PRINTF(ind, "Status: %s\nRoll error: % 6.1f deg\nPitch error:% 6.1f deg",
             ct->manual.auto_glide_active ? "Active  " : "Inactive",
             ct->manual.roll_error * 180.0 / PI,
             ct->manual.pitch_error * 180.0 / PI);
}

void UpdateControlTime(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Time [s]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
  MON_PRINTF(ind, "Total: %6.1f   Mode: %6.1f", ct->time, ct->flight_mode_time);
}

void UpdateControllerTiming(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Loop Time:");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  int64_t controller_usec = ct->finish_usec - ct->start_usec;

  MON_PRINTF(ind, "Total: %3.0f%% (max: %3.0f%%) Cont.: %3.0f%%",
             100.0 * (double)ct->loop_usec * 1e-6 / *g_sys.ts,
             100.0 * (double)ct->max_loop_usec * 1e-6 / *g_sys.ts,
             100.0 * (double)controller_usec * 1e-6 / *g_sys.ts);

  if ((double)ct->max_loop_usec * 1e-6 > 0.65 * *g_sys.ts) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateEstimatorAccBDiff(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Acc_b Diff. [m/s^2]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  Vec3 acc_b;
  Median3Vec3(&ct->estimator.acc_b_estimates[kWingImuA],
              &ct->estimator.acc_b_estimates[kWingImuB],
              &ct->estimator.acc_b_estimates[kWingImuC], &acc_b);
  double acc_b_diff[kNumWingImus];
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    Vec3 diff;
    Vec3Sub(&acc_b, &ct->estimator.acc_b_estimates[i], &diff);
    acc_b_diff[i] = Vec3Norm(&diff);
  }
  MON_PRINTF(ind, "A: %0.2f  B: %0.2f  C: %0.2f", acc_b_diff[kWingImuA],
             acc_b_diff[kWingImuB], acc_b_diff[kWingImuC]);
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

void UpdateEstimatorCurrentGpsReceiver(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Current GPS");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const MonEstimatorParams *params = &GetMonitorParams()->est;

  if (fd->current_gps_receiver_time < params->min_current_gps_receiver_time) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  const char *name;
  switch (ct->estimator.current_gps_receiver) {
    case kWingGpsReceiverCrosswind:
      name = "Crosswind";
      break;
    case kWingGpsReceiverHover:
      name = "Hover";
      break;
    default:
      name = "<Unknown>";
      indicator_set_state(ind, INDICATOR_STATE_WARNING);
      break;
  }

  MON_PRINTF(ind, "%s (for %3.0f seconds)", name,
             fd->current_gps_receiver_time);
}

// The GPS Diff indicator shows the difference in position estimate
// between each individual GPS receiver, referenced to the body
// origin, compared to the estimator's current position estimate. Note
// that errors in attitude estimation will also show up here, as the
// attitude is required to reference individual sensor measurements to
// the body origin.
void UpdateEstimatorGpsDiff(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Gps Diff.");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  Vec3 dx[kNumWingGpsReceivers];
  double dx_mag[kNumWingGpsReceivers];
  double dx_max = 0.0;
  for (int i = 0; i < kNumWingGpsReceivers; ++i) {
    const EstimatorPositionGpsEstimate gps = ct->estimator.gps[i];
    Vec3Sub(&gps.Xg, &ct->state_est.Xg, &dx[i]);
    dx_mag[i] = gps.wing_pos_valid ? Vec3Norm(&dx[i]) : NAN;
    dx_max = dx_mag[i] > dx_max ? dx_mag[i] : dx_max;
  }

  assert(kNumWingGpsReceivers == 4);
  MON_PRINTF(ind, "%6.2f %6.2f %6.2f %6.2f m", dx_mag[0], dx_mag[1], dx_mag[2],
             dx_mag[3]);

  assert(!isnan(dx_max));
  if (dx_max > 1.0) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateEstimatorGsPosEcef(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "GS Pos Ecef [m]");
  }

  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const SystemParams *system_params = GetSystemParams();
  Vec3 gs_pos_ecef;
  Mat3 dcm_ecef2g;  // Not used.
  GsGpsPosEcefToGsPosEcef(&ct->control_input.gs_gps.pos,
                          &system_params->gs_gps.primary_antenna_p.pos,
                          system_params->ground_frame.heading, &gs_pos_ecef,
                          &dcm_ecef2g);
  Vec3 diff;
  Vec3Sub(&ct->estimator.ground_station.pose.pos_ecef, &gs_pos_ecef, &diff);

  MON_PRINTF(ind, "dx: %0.2f  dy: %0.2f  dz: %0.2f", diff.x, diff.y, diff.z);

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if (Vec3Norm(&diff) > params->max_gs_pos_ecef_error) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateEstimatorGsgBias(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "GSG Bias [deg]");
  }

  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if ((ct->estimator.gsg_bias.azi > params->gsg_bias_azi.high) ||
      (ct->estimator.gsg_bias.azi < params->gsg_bias_azi.low) ||
      (ct->estimator.gsg_bias.ele > params->gsg_bias_ele.high) ||
      (ct->estimator.gsg_bias.ele < params->gsg_bias_ele.low)) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "azi: %3.2g   ele: %3.2g",
             (180.0 / PI) * ct->estimator.gsg_bias.azi,
             (180.0 / PI) * ct->estimator.gsg_bias.ele);
}

void UpdateEstimatorGyroBias(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Gyro Biases [rad/s]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  double gyro_bias_max[kNumWingImus];
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    double gyro_bias[] = {fabs(ct->estimator.gyro_biases[i].x),
                          fabs(ct->estimator.gyro_biases[i].y),
                          fabs(ct->estimator.gyro_biases[i].z)};
    gyro_bias_max[i] = MaxArray(gyro_bias, ARRAYSIZE(gyro_bias), NULL);
  }

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  double max_gyro_bias =
      MaxArray(gyro_bias_max, ARRAYSIZE(gyro_bias_max), NULL);
  if (max_gyro_bias > params->gyro_bias.very_high) {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  } else if (max_gyro_bias > params->gyro_bias.high) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "A: %0.5f  B: %0.5f  C: %0.5f", gyro_bias_max[kWingImuA],
             gyro_bias_max[kWingImuB], gyro_bias_max[kWingImuC]);
}

void UpdateEstimatorGyroDiff(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "PQR Diff. [rad/s]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  double pqr_diff[kNumWingImus];
  for (int32_t i = 0; i < kNumWingImus; ++i) {
    Vec3 diff;
    Vec3Sub(&ct->control_input.imus[i].gyro, &ct->estimator.gyro_biases[i],
            &diff);
    Vec3Sub(&ct->state_est.pqr, &diff, &diff);
    pqr_diff[i] = Vec3Norm(&diff);
  }
  MON_PRINTF(ind, "A: %0.4f  B: %0.4f  C: %0.4f", pqr_diff[kWingImuA],
             pqr_diff[kWingImuB], pqr_diff[kWingImuC]);
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

void UpdateEstimatorMagnetometerDiff(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Mag. Diff. [G]");
  }

  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const MonEstimatorParams *params = &GetMonitorParams()->est;

  assert(ARRAYSIZE(ct->control_input.imus) == 3);
  double mag_diff[3];
  for (int32_t i = 0; i < 3; ++i) {
    Vec3 diff;
    Vec3Sub(&ct->control_input.imus[i].mag,
            &ct->control_input.imus[(i + 1) % 3].mag, &diff);
    mag_diff[i] = Vec3Norm(&diff);
  }

  if (MaxArray(mag_diff, ARRAYSIZE(mag_diff), NULL) > params->max_mag_diff) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "A-B: %0.3f  B-C: %0.3f  C-A: %0.3f", mag_diff[0],
             mag_diff[1], mag_diff[2]);
}

// TODO: Investigate why this monitor triggers false positives.
void UpdateEstimatorWinchDisagreement(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Winch Pos. Diff. [m]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  // Compute the distance from the perched wing position to the wing.
  Vec3 tmp;
  double distance_est = Vec3Norm(
      Vec3Sub(GToP(&ct->state_est.Xg, ct->state_est.perch_azi.angle, &tmp),
              &GetControlParams()->hover.path.perched_wing_pos_p, &tmp));

  // This estimate position does not include catenary effects.
  double winch_distance_est = GetSystemParams()->tether.length +
                              PI * GetSystemParams()->winch.r_drum +
                              ct->state_est.winch.position;

  // Default to no warning.
  indicator_set_state(ind, INDICATOR_STATE_GOOD);

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if (ct->state_est.perch_azi.valid && ct->state_est.winch.valid) {
    // On the perch.
    if (ct->flight_mode == kFlightModePerched &&
        (fabs(winch_distance_est) >
         params->winch_position_max_perched_disagreement)) {
      indicator_set_state(ind, INDICATOR_STATE_WARNING);
    }

    // Before the perch rotation.
    if (ct->state_est.winch.position < -PI * GetSystemParams()->winch.r_drum &&
        (fabs(distance_est - winch_distance_est) >
         params->winch_distance_disagreement_threshold)) {
      indicator_set_state(ind, INDICATOR_STATE_WARNING);
    }
  }

  MON_PRINTF(ind, "Difference: %3.1g  Winch: %3.1g",
             distance_est - winch_distance_est, winch_distance_est);
}

// Helper struct for StringAppend.
//   data: String to which to append.
//   size: Size of data in bytes.
//   pos: Position at which the next append should occur.
typedef struct {
  char *data;
  const int32_t size;
  int32_t pos;
} StringAppendHelper;

// Appends src to dest. Upon return, dest->pos is updated.
static void StringAppend(const char *src, StringAppendHelper *dest) {
  assert(src != NULL);
  assert(dest != NULL);
  assert(dest->data != NULL);
  if (dest->size < 0 || dest->pos < 0 || dest->pos >= dest->size) {
    assert(false);
    return;
  }

  // bytes_available includes room for a terminating "\0". bytes_attempted does
  // not include the terminating "\0".
  int32_t bytes_available = dest->size - dest->pos;
  int32_t bytes_attempted =
      snprintf(dest->data + dest->pos, (size_t)bytes_available, "%s", src);
  assert(bytes_attempted >= 0);
  if (bytes_attempted < 0) return;

  dest->pos = bytes_attempted < bytes_available ? dest->pos + bytes_attempted
                                                : dest->size - 1;
}

// Displays all active faults for non-disabled subsystems, in the format
//     <subsystem_1>: <fault_11> <fault_12> ...
//     <subsystem_2>: <fault_21> <fault_22> ...
//     ...
// Excessively long lines are truncated, both based on the width of the
// indicator and local line length limits. (Practically speaking, there's room
// for 3 fault names, and if there are more than that, you have bigger problems
// than this monitor can identify.) Additionally, the number of rows is
// truncated if there are too many to display.
void UpdateFdAllActive(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Faults");
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const int32_t kMaxSubsystemsDisplayed = 10;
  const int32_t kLineSize = 40;
  int32_t num_subsystems_faulted = 0;

  char display[(kMaxSubsystemsDisplayed + 1) * kLineSize];
  StringAppendHelper display_helper = {
      .data = display, .size = (int32_t)sizeof(display), .pos = 0};

  for (int32_t i_subsys = 0; i_subsys < kNumSubsystems; ++i_subsys) {
    const FaultMask *mask = &ct->faults[i_subsys];
    if (!HasAnyFault(mask) || HasFault(kFaultTypeDisabled, mask)) continue;

    ++num_subsystems_faulted;
    if (num_subsystems_faulted > kMaxSubsystemsDisplayed) {
      StringAppend("\n<too many to display>", &display_helper);
      break;
    }

    char line[kLineSize];
    StringAppendHelper line_helper = {
        .data = line, .size = (int32_t)sizeof(line), .pos = 0};

    StringAppend(SubsystemLabelToString((SubsystemLabel)i_subsys),
                 &line_helper);
    StringAppend(":", &line_helper);

    for (int32_t i_type = 0; i_type < kNumFaultTypes; ++i_type) {
      if (HasFault(i_type, mask)) {
        StringAppend(" ", &line_helper);
        StringAppend(FaultTypeToString((FaultType)i_type), &line_helper);
      }
    }

    if (num_subsystems_faulted > 1) {
      StringAppend("\n", &display_helper);
    }
    StringAppend(line, &display_helper);
  }

  if (num_subsystems_faulted == 0) {
    indicator_set_value(ind, "none");
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_value(ind, display);
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }
}

// Displays all disabled subsystems.
void UpdateFdDisabled(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Disabled");
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  char display[512];
  StringAppendHelper display_helper = {
      .data = display, .size = (int32_t)sizeof(display), .pos = 0};
  bool any_disabled = false;

  for (int32_t i_subsys = 0; i_subsys < kNumSubsystems; ++i_subsys) {
    if (!HasFault(kFaultTypeDisabled, &ct->faults[i_subsys])) continue;

    if (any_disabled) {
      StringAppend("\n", &display_helper);
    }
    StringAppend(SubsystemLabelToString((SubsystemLabel)i_subsys),
                 &display_helper);
    any_disabled = true;
  }

  indicator_set_value(ind, any_disabled ? display : "none");
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

// Fault Detection Indicators
// This function implements special logic for handling sensors which are
// disabled in the controller (i.e. those flagged with kFaultTypeDisabled).
static void UpdateFromFaultDispInfo(const char *label,
                                    const FaultDispInfo fdisp[], int32_t len,
                                    Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, label);

  if (!CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  int32_t num_disabled_faults = 0;
  int32_t num_other_faults = 0;
  for (int32_t i = 0; i < len; ++i) {
    // This logic is written so that a kFaultTypeDisabled fault will
    // hide other faults indicated by the controller.
    if (HasFault(kFaultTypeDisabled, &fdisp[i].fault)) {
      num_disabled_faults++;
    } else if (HasAnyFault(&fdisp[i].fault)) {
      num_other_faults++;
    }
  }

  IndicatorState state;
  // If all sensors are disabled, ignore.
  if (num_disabled_faults == len) {
    state = INDICATOR_STATE_NONE;
  } else if (num_other_faults > 0) {
    state = INDICATOR_STATE_WARNING;
  } else {
    state = INDICATOR_STATE_GOOD;
  }
  indicator_set_state(ind, state);

  if (num_disabled_faults + num_other_faults == 0) {
    indicator_set_value(ind, "None");
  } else {
    indicator_set_value(ind, GetFaultDispStr(fdisp, len));
  }
}

// Helper function for updating indicators representing a single fault.
static void UpdateFdSimple(const char *label, SubsystemLabel subsystem,
                           Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[] = {{"", ct->faults[subsystem]}};
  UpdateFromFaultDispInfo(label, fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdGps(Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[] = {{"Pos CW: ", ct->faults[kSubsysWingGpsCrosswindPos]},
                           {"Vel CW: ", ct->faults[kSubsysWingGpsCrosswindVel]},
                           {"Pos H: ", ct->faults[kSubsysWingGpsHoverPos]},
                           {"Vel H: ", ct->faults[kSubsysWingGpsHoverVel]}};
  UpdateFromFaultDispInfo("FD GPS", fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdGsCompass(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD GS Compass", kSubsysGsCompass, ind, init);
}

void UpdateFdGsGps(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD GS GPS", kSubsysGsGpsPos, ind, init);
}

static void UpdateFdGsg(const char *label, const FaultMask *gsg_faults,
                        Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[] = {{"Azi: ", gsg_faults[kFaultDetectionGsgSignalAzi]},
                           {"Ele: ", gsg_faults[kFaultDetectionGsgSignalEle]}};
  UpdateFromFaultDispInfo(label, fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdGsgA(Indicator *ind, int32_t init) {
  UpdateFdGsg("FD GSG A", &ct->faults[SUBSYS_GSG_A], ind, init);
}

void UpdateFdGsgB(Indicator *ind, int32_t init) {
  UpdateFdGsg("FD GSG B", &ct->faults[SUBSYS_GSG_B], ind, init);
}

static void UpdateFdImu(const char *label, const FaultMask imu_faults[],
                        Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[] = {{"Acc: ", imu_faults[kFaultDetectionImuSignalAcc]},
                           {"Gyro: ", imu_faults[kFaultDetectionImuSignalGyro]},
                           {"Mag: ", imu_faults[kFaultDetectionImuSignalMag]}};
  UpdateFromFaultDispInfo(label, fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdImuA(Indicator *ind, int32_t init) {
  UpdateFdImu("FD IMU A", &ct->faults[SUBSYS_IMU_A], ind, init);
}

void UpdateFdImuB(Indicator *ind, int32_t init) {
  UpdateFdImu("FD IMU B", &ct->faults[SUBSYS_IMU_B], ind, init);
}

void UpdateFdImuC(Indicator *ind, int32_t init) {
  UpdateFdImu("FD IMU C", &ct->faults[SUBSYS_IMU_C], ind, init);
}

void UpdateFdJoystick(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Joystick", kSubsysJoystick, ind, init);
}

void UpdateFdLevelwindEleA(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Levelwind Ele. A", kSubsysLevelwindEleA, ind, init);
}

void UpdateFdLevelwindEleB(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Levelwind Ele. B", kSubsysLevelwindEleB, ind, init);
}

void UpdateFdLoadcells(Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[kNumLoadcellSensors];
  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) {
    snprintf(fdisp[i].prefix, sizeof(fdisp[i].prefix), "%s: ",
             LoadcellSensorLabelToString((LoadcellSensorLabel)i));
    fdisp[i].fault = ct->faults[SUBSYS_LOADCELLS + i];
  }
  UpdateFromFaultDispInfo("FD Loadcells", fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdMotors(Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[kNumMotors];
  for (int32_t i = 0; i < kNumMotors; ++i) {
    snprintf(fdisp[i].prefix, sizeof(fdisp[i].prefix), "%s: ",
             MotorLabelToString((MotorLabel)i));
    fdisp[i].fault = fd->latched_motor_faults[i];
  }
  UpdateFromFaultDispInfo("FD Motors", fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdPerchAziA(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Perch Azi. A", kSubsysPerchAziA, ind, init);
}

void UpdateFdPerchAziB(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Perch Azi. B", kSubsysPerchAziB, ind, init);
}

static void UpdateFdPitot(const char *label, const FaultMask faults[],
                          Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[] = {
      {"Alt: ", faults[kFaultDetectionPitotSignalStatic]},
      {"Pitch: ", faults[kFaultDetectionPitotSignalAlpha]},
      {"Yaw: ", faults[kFaultDetectionPitotSignalBeta]},
      {"Speed: ", faults[kFaultDetectionPitotSignalDynamic]}};
  UpdateFromFaultDispInfo(label, fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdPitotHighSpeed(Indicator *ind, int32_t init) {
  UpdateFdPitot("FD Pitot HS", &ct->faults[SUBSYS_PITOT_SENSOR_HIGH_SPEED], ind,
                init);
}

void UpdateFdPitotLowSpeed(Indicator *ind, int32_t init) {
  UpdateFdPitot("FD Pitot LS", &ct->faults[SUBSYS_PITOT_SENSOR_LOW_SPEED], ind,
                init);
}

void UpdateFdProximitySensor(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Prox. Sensor", kSubsysProximitySensor, ind, init);
}

void UpdateFdServos(Indicator *ind, int32_t init) {
  FaultDispInfo fdisp[] = {{"A1: ", ct->faults[kSubsysServoA1]},
                           {"A2: ", ct->faults[kSubsysServoA2]},
                           {"A4: ", ct->faults[kSubsysServoA4]},
                           {"A5: ", ct->faults[kSubsysServoA5]},
                           {"A7: ", ct->faults[kSubsysServoA7]},
                           {"A8: ", ct->faults[kSubsysServoA8]},
                           {"E1: ", ct->faults[kSubsysServoE1]},
                           {"E2: ", ct->faults[kSubsysServoE2]},
                           {"R1: ", ct->faults[kSubsysServoR1]},
                           {"R2: ", ct->faults[kSubsysServoR2]}};
  UpdateFromFaultDispInfo("FD Servos", fdisp, ARRAYSIZE(fdisp), ind, init);
}

void UpdateFdWinch(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Winch", kSubsysWinch, ind, init);
}

void UpdateFdWindSensor(Indicator *ind, int32_t init) {
  UpdateFdSimple("FD Wind Sensor", kSubsysWindSensor, ind, init);
}

void UpdateFlightMode(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Flight mode");
  }

  if (!CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if (!CheckControllerRunning()) {
    InitializationState init_state = (InitializationState)ct->init_state;
    MON_PRINTF(ind, "Init: %s", InitializationStateToString(init_state));
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else if (ct->flight_mode > kFlightModeOffTether) {
    MON_PRINTF(ind, "%s: %d", FlightModeToString(ct->flight_mode),
               ct->flight_mode);
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    MON_PRINTF(ind, "%s", FlightModeToString(ct->flight_mode));
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateFlightModeGates(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Gates");
  }

  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  int32_t num_gate_strs = 0;
  const char *gate_strs[32] = {NULL};
  FlightMode gated_mode = GetNextFlightMode(ct->flight_mode);
  int32_t num_gates = GetNumFlightModeGates(gated_mode);
  for (int32_t i = 0; i < num_gates; ++i) {
    if (ct->flight_mode_gates[gated_mode] & (1 << i)) {
      gate_strs[num_gate_strs] = GateToString(gated_mode, i);
      num_gate_strs++;
    }

    if (num_gate_strs >= ARRAYSIZE(gate_strs)) break;
  }

  if (num_gate_strs > 0) {
    int32_t i = ((int32_t)ct->time) % num_gate_strs;
    MON_PRINTF(ind, "%s", gate_strs[i]);
  } else {
    MON_PRINTF(ind, "%s", "");
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

void UpdateFlightPlan(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Flight plan");
  }

  if (!CheckControllerSlowTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  // For all modes flight, we expect the kFlightPlanTurnKey flight plan.
  if (cst->flight_plan == kFlightPlanTurnKey ||
      cst->flight_plan == kFlightPlanHighHover) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_ERROR);
  }

  MON_PRINTF(ind, "%s", FlightPlanToString(cst->flight_plan));
}

void UpdateFlightTimer(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Flight time");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);

  char auto_hms[16], cw_hms[16];
  ConvTimeSecToStr(fd->time_in_autonomous_sec, sizeof(auto_hms), auto_hms);
  ConvTimeSecToStr(fd->time_in_crosswind_sec, sizeof(cw_hms), cw_hms);
  // Time since perch ascend and time in CW.
  MON_PRINTF(ind, "Auto: %s | CW: %s", auto_hms, cw_hms);
}

void UpdateHeightAgl(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Height AGL");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
  double altitude =
      -(ct->state_est.Xg.z - GetSystemParams()->ground_frame.ground_z);
  MON_PRINTF(ind,
             "Height:  % 7.2f m\n"
             "Descent: % 7.2f m/s",
             altitude, ct->state_est.Vg.z);
}

void UpdateHoverAngleCommand(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Angle command");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);

  MON_PRINTF(ind,
             "R:% 7.2f rad    int:% 7.2f rad\n"
             "P:% 7.2f rad    int:% 7.2f rad\n"
             "Y:% 7.2f rad    int:% 7.2f rad",
             ct->hover.angles_cmd.x, 0.0, ct->hover.angles_cmd.y,
             ct->hover.int_pitch, ct->hover.angles_cmd.z,
             ct->hover.int_angles.z);
}

void UpdateHoverGainRampScale(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Gain ramp scale");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  if (ct->hover.gain_ramp_scale < 1.0) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ind, "%0.2f", ct->hover.gain_ramp_scale);
}

void UpdateHoverThrustMoment(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Thrust-moment");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  bool thrust_saturated =
      fabs(ct->thrust_moment.thrust - ct->thrust_moment_avail.thrust) > 10.0;
  bool roll_saturated = fabs(ct->thrust_moment.moment.x -
                             ct->thrust_moment_avail.moment.x) > 10.0;
  bool pitch_saturated = fabs(ct->thrust_moment.moment.y -
                              ct->thrust_moment_avail.moment.y) > 10.0;
  bool yaw_saturated = fabs(ct->thrust_moment.moment.z -
                            ct->thrust_moment_avail.moment.z) > 10.0;

  // Don't warn on roll saturation since we don't control roll with
  // the motors.
  if (pitch_saturated || yaw_saturated || thrust_saturated) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  char saturated_str[80] = "none";
  if (thrust_saturated || roll_saturated || pitch_saturated || yaw_saturated) {
    snprintf(saturated_str, sizeof(saturated_str), "%s%s%s%s",
             thrust_saturated ? "thrust " : "", roll_saturated ? "roll " : "",
             pitch_saturated ? "pitch " : "", yaw_saturated ? "yaw " : "");
  }
  MON_PRINTF(
      ind,
      "T:% 7.2f kN     int:% 7.2f kN\n"
      "R:% 7.2f kN-m   int:% 7.2f kN-m\n"
      "P:% 7.2f kN-m   int:% 7.2f kN-m\n"
      "Y:% 7.2f kN-m   int:% 7.2f kN-m\n"
      "Saturated: %s",
      ct->thrust_moment_avail.thrust / 1e3, ct->hover.int_thrust / 1e3,
      ct->thrust_moment_avail.moment.x / 1e3, ct->hover.int_moment.x / 1e3,
      ct->thrust_moment_avail.moment.y / 1e3, ct->hover.int_moment.y / 1e3,
      ct->thrust_moment_avail.moment.z / 1e3, ct->hover.int_moment.z / 1e3,
      saturated_str);
}

void UpdateJoystick(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Joystick");
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const JoystickEstimate *joystick = &ct->state_est.joystick;
  indicator_set_state(
      ind, joystick->valid ? INDICATOR_STATE_GOOD : INDICATOR_STATE_WARNING);
  MON_PRINTF(ind, "T:% 0.2f   R:% 0.2f P:% 0.2f Y:% 0.2f",
             joystick->data.throttle, joystick->data.roll, joystick->data.pitch,
             joystick->data.yaw);
}

void UpdateManualFlaps(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Manual Flaps [deg]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  indicator_set_state(ind, INDICATOR_STATE_GOOD);

  MON_PRINTF(ind,
             "Port:% 3.0f  Cen: % 3.0f  Star:% 3.0f\n"
             "Ele: % 3.0f  Rud: % 3.0f ",
             ct->control_output.flaps[kFlapA1] * 180.0 / PI,
             ct->control_output.flaps[kFlapA4] * 180.0 / PI,
             ct->control_output.flaps[kFlapA8] * 180.0 / PI,
             ct->control_output.flaps[kFlapEle] * 180.0 / PI,
             ct->control_output.flaps[kFlapRud] * 180.0 / PI);
}

void UpdateManualState(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Manual State");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
  MON_PRINTF(ind, "Release: %s",
             ct->manual.release_latched ? "true " : "false");
}

void UpdatePayout(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Payout [m]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  const MonEstimatorParams *params = &GetMonitorParams()->est;
  if (!ct->state_est.winch.valid) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else if ((ct->flight_mode == kFlightModePerched ||
              ct->flight_mode == kFlightModeHoverAscend) &&
             (fabs(ct->state_est.winch.payout) > params->far_payout_at_perch)) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "%3.2f", ct->state_est.winch.payout);
}

void UpdatePerchAzi(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Perch Azi. [deg]");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  double perch_azi_deg = ct->state_est.perch_azi.angle * 180.0 / PI;
  const HoverParams *hover_params = &GetControlParams()->hover;
  if (!ct->state_est.perch_azi.valid) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
    MON_PRINTF(ind, "%3.2f (Invalid)", perch_azi_deg);
    return;
  }

  double wind_misalignment =
      fabs(Wrap(ct->state_est.perch_azi.angle + PI - ct->state_est.wind_g.dir_f,
                -PI, PI));
  if (ct->flight_mode == kFlightModePerched && ct->state_est.wind_g.valid &&
      wind_misalignment >=
          hover_params->mode.max_perch_wind_misalignment_for_ascend) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "%3.2f (Wind Misalignment: %3.2f)", perch_azi_deg,
             wind_misalignment * 180.0 / PI);
}

void UpdateProximityFlagValid(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Proximity flag valid");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if ((cst->flight_plan == kFlightPlanHighHover ||
       cst->flight_plan == kFlightPlanLaunchPerch ||
       cst->flight_plan == kFlightPlanTurnKey) &&
      !ct->state_est.winch.proximity_valid) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ind, "%d", ct->state_est.winch.proximity_valid);
}

void UpdateTensionComponents(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Tension");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if (ct->state_est.tether_force_b.valid) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }

  // Low-pass-filter the tether force vector.
  static Vec3 tether_force_b_f = {0.0, 0.0, 0.0};
  LpfVec3(&ct->state_est.tether_force_b.vector, 0.1, *g_sys.ts,
          &tether_force_b_f);

  // Rotate tether_force_b into ground coordinates.
  Vec3 tether_force_g;
  Mat3TransVec3Mult(&ct->state_est.dcm_g2b, &tether_force_b_f, &tether_force_g);

  const double tether_force_horiz = hypot(tether_force_g.x, tether_force_g.y);
  const double tether_force_vert = tether_force_g.z;
  const double tether_force_angle =
      atan2(tether_force_vert, tether_force_horiz);

  MON_PRINTF(ind,
             "Horiz:% 6.1f kN\n"
             "Vert: % 6.1f kN\n"
             "Angle:  % 4.0f deg",
             tether_force_horiz / 1000.0, tether_force_vert / 1000.0,
             tether_force_angle * 180.0 / PI);
}

void UpdateTensionValid(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Tension valid");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if (!ct->state_est.tether_force_b.valid) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
  MON_PRINTF(ind, "%d", ct->state_est.tether_force_b.valid);
}

void UpdateThrottle(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Throttle");
  }
  if (!CheckControllerTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
  MON_PRINTF(ind, "%0.2f", ct->control_input.joystick.throttle);
}

void UpdateVersion(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Version");
  }

  if (!CheckControllerSlowTelemetry()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  // TODO: Remove the wing serial and test site warnings once
  // we begin flying other wings at other places.
  if (cst->hitl_config.sim_level == kSimulatorHitlLevelNone &&
      (cst->wing_serial == kWingSerial01 ||
       cst->wing_serial == kWingSerial04Hover ||
       cst->wing_serial == kWingSerial04Crosswind ||
       cst->wing_serial == kWingSerial06Hover ||
       cst->wing_serial == kWingSerial06Crosswind ||
       cst->wing_serial == kWingSerial07Hover ||
       cst->wing_serial == kWingSerial07Crosswind) &&
      (cst->test_site == kTestSiteParkerRanch ||
       cst->test_site == kTestSiteNorway)) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }

  MON_PRINTF(ind, "%s %s\nSN%s %s %s %s", cst->build_info.date,
             cst->build_info.time, WingSerialToString(cst->wing_serial),
             GroundStationModelToString(cst->gs_model),
             TestSiteToString(cst->test_site),
             GetHitlDispStr((SimulatorHitlLevel)cst->hitl_config.sim_level,
                            cst->hitl_config.use_software_joystick));
}

void UpdateWinchPos(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Winch pos | vel");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
  MON_PRINTF(ind, "%4.2f m | %4.2f m/s", ct->control_input.perch.winch_pos,
             ct->control_output.winch_vel_cmd);
}

void UpdateWind(Indicator *ind, int32_t init) {
  if (init) {
    indicator_set_label(ind, "Wind");
  }
  if (!CheckControllerRunning()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if (ct->state_est.wind_g.valid) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }

  MON_PRINTF(ind,
             "x:% 5.1f y:% 5.1f z:% 5.1f\n"
             "%4.2f m/s @ %d deg %s",
             ct->state_est.wind_g.vector_f.x, ct->state_est.wind_g.vector_f.y,
             ct->state_est.wind_g.vector_f.z, ct->state_est.wind_g.speed_f,
             ((int32_t)(ct->state_est.wind_g.dir_f * 180.0 / PI) + 360) % 360,
             ct->state_est.wind_g.valid ? "" : "[invalid!]");
}
