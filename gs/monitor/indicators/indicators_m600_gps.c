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

#include "gs/monitor/indicators/indicators_m600_gps.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/gps_receiver.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/septentrio_types.h"
#include "common/c_math/coord_trans.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "gs/monitor/monitor_filter.h"
#include "gs/monitor/monitor_params.h"
#include "gs/monitor/monitor_util.h"
#include "gs/monitor/widgets/indicator.h"
#include "system/labels.h"
#include "system/labels_util.h"

void UpdateGsGpsCompassHeading(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GS Heading");

  if (!CheckAioComms() || !CheckGsGpsCompassComms() ||
      ((double)aio_1->gs_gps_compass.heading_latency * 1e-6 > 0.1)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  if ((aio_1->gs_gps_compass.heading.pos_type == kNovAtelSolutionTypeNone) ||
      (aio_1->gs_gps_compass.heading.pos_sol_status !=
       kNovAtelSolutionStatusSolComputed)) {
    // TODO: Once a compass is installed, change this to a warning.
    MON_PRINTF(ind, "%s", "No solution.");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  MON_PRINTF(ind, "%0.0f", aio_1->gs_gps_compass.heading.heading);
  indicator_set_state(ind, INDICATOR_STATE_GOOD);
}

static void UpdateNovAtelCn0(const NovAtelSolutionMessage *sol,
                             Indicator *ind) {
  MON_PRINTF(ind, "L1 C/N0 Avg: %.1f Max: %.1f Tracked: %d", sol->avg_cn0,
             sol->max_cn0, sol->best_xyz.num_tracked);

  if (sol->avg_cn0 >= 40.0 && sol->best_xyz.num_tracked >= 5) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }
}

static bool CheckNovAtelTypeAllowed(const NovAtelSolutionType *allowed_types,
                                    int32_t num_allowed_types,
                                    NovAtelSolutionType type) {
  for (int32_t i = 0; i < num_allowed_types; ++i) {
    if (type == allowed_types[i]) return true;
  }

  return false;
}

static void UpdateNovAtelPosVelType(
    const NovAtelSolutionMessage *solution,
    const NovAtelSolutionType *allowed_pos_types, int32_t num_allowed_pos_types,
    const NovAtelSolutionType *allowed_vel_types, int32_t num_allowed_vel_types,
    Indicator *ind) {
  NovAtelSolutionStatus pos_sol_status =
      (NovAtelSolutionStatus)solution->best_xyz.pos_sol_status;
  NovAtelSolutionStatus vel_sol_status =
      (NovAtelSolutionStatus)solution->best_xyz.vel_sol_status;
  NovAtelSolutionType pos_type =
      (NovAtelSolutionType)solution->best_xyz.pos_type;
  NovAtelSolutionType vel_type =
      (NovAtelSolutionType)solution->best_xyz.vel_type;

  if (CheckNovAtelTypeAllowed(allowed_pos_types, num_allowed_pos_types,
                              pos_type) &&
      CheckNovAtelTypeAllowed(allowed_vel_types, num_allowed_vel_types,
                              vel_type)) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }

  const char *pos_string = NovAtelSolutionTypeToString(pos_type);
  if (pos_sol_status != kNovAtelSolutionStatusSolComputed) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
    pos_string = NovAtelSolutionStatusToString(pos_sol_status);
  }

  const char *vel_string = NovAtelSolutionTypeToString(vel_type);
  if (vel_sol_status != kNovAtelSolutionStatusSolComputed) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
    vel_string = NovAtelSolutionStatusToString(vel_sol_status);
  }

  MON_PRINTF(ind, "%s / %s", pos_string, vel_string);
}

static void UpdateNovAtelSigmas(const NovAtelSolutionMessage *solution,
                                double max_gps_pos_sigma,
                                double max_gps_vel_sigma, Indicator *ind) {
  const NovAtelLogBestXyz *best_xyz = &solution->best_xyz;

  double pos_sigma = Sqrt(best_xyz->pos_x_sigma * best_xyz->pos_x_sigma +
                          best_xyz->pos_y_sigma * best_xyz->pos_y_sigma +
                          best_xyz->pos_z_sigma * best_xyz->pos_z_sigma);
  double vel_sigma = Sqrt(best_xyz->vel_x_sigma * best_xyz->vel_x_sigma +
                          best_xyz->vel_y_sigma * best_xyz->vel_y_sigma +
                          best_xyz->vel_z_sigma * best_xyz->vel_z_sigma);

  if (pos_sigma > max_gps_pos_sigma || vel_sigma > max_gps_vel_sigma ||
      ((pos_sigma == 0.0 || vel_sigma == 0.0) &&
       ((NovAtelSolutionType)best_xyz->pos_type !=
        kNovAtelSolutionTypeFixedPos))) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "Pos: %2.2f [m] / Vel: %2.2f [m/s]", pos_sigma, vel_sigma);
}

void UpdateGsGpsCn0(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GS C/N0");

  if (!CheckAioComms() || !CheckGsGpsSolutionComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  UpdateNovAtelCn0(&aio_1->gs_gps_solution, ind);
}

void UpdateGsGpsPos(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GS GPS Pos");

  if (!CheckAioComms() || !CheckGsGpsSolutionComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const NovAtelSolutionMessage *gs_gps_solution = &aio_1->gs_gps_solution;
  const Vec3 gs_gps_pos_ecef = {.x = gs_gps_solution->best_xyz.pos_x,
                                .y = gs_gps_solution->best_xyz.pos_y,
                                .z = gs_gps_solution->best_xyz.pos_z};
  Vec3 gs_gps_pos_llh;
  EcefToLlh(&gs_gps_pos_ecef, &gs_gps_pos_llh);

  if ((gs_gps_solution->best_xyz.pos_type == kNovAtelSolutionTypeNone) ||
      (gs_gps_solution->best_xyz.pos_sol_status !=
       kNovAtelSolutionStatusSolComputed)) {
    MON_PRINTF(ind, "%s", "No solution.");
    indicator_set_state(ind, INDICATOR_STATE_NONE);
  } else {
    MON_PRINTF(ind, "%11.7f°%s %11.7f°%s % 6.1f m",  //
               fabs(gs_gps_pos_llh.x), gs_gps_pos_llh.x > 0.0 ? "N" : "S",
               fabs(gs_gps_pos_llh.y), gs_gps_pos_llh.y > 0.0 ? "E" : "W",
               gs_gps_pos_llh.z);
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }
}

void UpdateGsGpsPosVelType(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GS Pos. / Vel. Type");

  if (!CheckAioComms() || !CheckGsGpsSolutionComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const NovAtelSolutionType allowed_pos_types[] = {
      kNovAtelSolutionTypeFixedPos};

  const NovAtelSolutionType allowed_vel_types[] = {
      kNovAtelSolutionTypeDopplerVelocity};

  UpdateNovAtelPosVelType(&aio_1->gs_gps_solution, allowed_pos_types,
                          ARRAYSIZE(allowed_pos_types), allowed_vel_types,
                          ARRAYSIZE(allowed_vel_types), ind);
}

void UpdateGsGpsSigmas(Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GS GPS Sigmas");

  if (!CheckAioComms() || !CheckGsGpsSolutionComms()) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  UpdateNovAtelSigmas(&aio_1->gs_gps_solution, 0.01, 0.01, ind);
}

static void UpdateWingGpsNovAtelCn0(WingGpsReceiverLabel label, Indicator *ind,
                                    int32_t init) {
  if (init) indicator_set_label(ind, "C/N0");

  if (!CheckAioComms() || !CheckWingGpsSolutionComms(label)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  UpdateNovAtelCn0(&aio_1->wing_gps_novatel_solutions[label], ind);
}

static void UpdateWingGpsNovAtelPosVelType(WingGpsReceiverLabel label,
                                           Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Pos. / Vel. Type");

  if (!CheckAioComms() || !CheckWingGpsSolutionComms(label)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const NovAtelSolutionType allowed_pos_types[] = {
      kNovAtelSolutionTypeL1Float, kNovAtelSolutionTypeL1Int,
      kNovAtelSolutionTypeNarrowInt};

  const NovAtelSolutionType allowed_vel_types[] = {
      kNovAtelSolutionTypeL1Float, kNovAtelSolutionTypeL1Int,
      kNovAtelSolutionTypeNarrowInt};

  UpdateNovAtelPosVelType(&aio_1->wing_gps_novatel_solutions[label],
                          allowed_pos_types, ARRAYSIZE(allowed_pos_types),
                          allowed_vel_types, ARRAYSIZE(allowed_vel_types), ind);
}

static void UpdateWingGpsNovAtelSigmas(WingGpsReceiverLabel label,
                                       Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GPS Sigmas");

  if (!CheckAioComms() || !CheckWingGpsSolutionComms(label)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const MonEstimatorParams *params = &GetMonitorParams()->est;
  UpdateNovAtelSigmas(&aio_1->wing_gps_novatel_solutions[label],
                      params->max_gps_pos_sigma, params->max_gps_vel_sigma,
                      ind);
}

static void UpdateWingGpsSeptentrioCn0(WingGpsReceiverLabel label,
                                       Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "C/N0");

  if (!CheckAioComms() || !CheckWingGpsSolutionComms(label)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const SeptentrioSolutionMessage *sol =
      &aio_1->wing_gps_septentrio_solutions[label];
  MON_PRINTF(ind, "L1 C/N0 Avg: %.1f Max: %.1f Tracked: %d", sol->avg_cn0,
             sol->max_cn0, sol->pvt_cartesian.nr_sv);

  if (sol->avg_cn0 >= 40.0 && sol->pvt_cartesian.nr_sv >= 5) {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  }
}

static void UpdateWingGpsSeptentrioMode(WingGpsReceiverLabel label,
                                        Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "Mode");

  if (!CheckAioComms() || !CheckWingGpsSolutionComms(label)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const SeptentrioBlockPvtCartesian *pvt_cartesian =
      &aio_1->wing_gps_septentrio_solutions[label].pvt_cartesian;
  SeptentrioPvtMode mode =
      pvt_cartesian->mode & kSeptentrioPvtModeBitSolutionMask;

  if (pvt_cartesian->error != 0) {
    MON_PRINTF(ind, "%s", SeptentrioPvtErrorToString(pvt_cartesian->error));
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    MON_PRINTF(ind, "%s", SeptentrioPvtModeToString(mode));
    if (mode != kSeptentrioPvtModeRtkFixed &&
        mode != kSeptentrioPvtModeRtkFloat) {
      indicator_set_state(ind, INDICATOR_STATE_WARNING);
    } else {
      indicator_set_state(ind, INDICATOR_STATE_GOOD);
    }
  }
}

static void UpdateWingGpsSeptentrioSigmas(WingGpsReceiverLabel label,
                                          Indicator *ind, int32_t init) {
  if (init) indicator_set_label(ind, "GPS Sigmas");

  if (!CheckAioComms() || !CheckWingGpsSolutionComms(label)) {
    indicator_set_state(ind, INDICATOR_STATE_NONE);
    return;
  }

  const SeptentrioBlockPosCovCartesian *pos_cov_cartesian =
      &aio_1->wing_gps_septentrio_solutions[label].pos_cov_cartesian;
  const SeptentrioBlockVelCovCartesian *vel_cov_cartesian =
      &aio_1->wing_gps_septentrio_solutions[label].vel_cov_cartesian;

  double pos_sigma =
      Sqrt(pos_cov_cartesian->cov_xx + pos_cov_cartesian->cov_yy +
           pos_cov_cartesian->cov_zz);
  double vel_sigma =
      Sqrt(vel_cov_cartesian->cov_xx + vel_cov_cartesian->cov_yy +
           vel_cov_cartesian->cov_zz);

  const MonEstimatorParams *params = &GetMonitorParams()->est;

  if (pos_sigma > params->max_gps_pos_sigma ||
      vel_sigma > params->max_gps_vel_sigma || pos_sigma == 0.0 ||
      vel_sigma == 0.0) {
    indicator_set_state(ind, INDICATOR_STATE_WARNING);
  } else {
    indicator_set_state(ind, INDICATOR_STATE_GOOD);
  }

  MON_PRINTF(ind, "Pos: %2.2f [m] / Vel: %2.2f [m/s]", pos_sigma, vel_sigma);
}

static void DispatchWingGpsUpdate(
    void (*novatel_func)(WingGpsReceiverLabel, Indicator *, int32_t),
    void (*septentrio_func)(WingGpsReceiverLabel, Indicator *, int32_t),
    WingGpsReceiverLabel gps_label, Indicator *ind, int32_t init) {
  GpsReceiverType gps_type = WingGpsReceiverLabelToGpsReceiverType(gps_label);
  if (gps_type == kGpsReceiverTypeNovAtel) {
    novatel_func(gps_label, ind, init);
  } else if (gps_type == kGpsReceiverTypeSeptentrio) {
    septentrio_func(gps_label, ind, init);
  } else {
    assert(!(bool)"Invalid GPS type.");
  }
}

void UpdateWingGpsACn0(Indicator *ind, int32_t init) {
  DispatchWingGpsUpdate(UpdateWingGpsNovAtelCn0, UpdateWingGpsSeptentrioCn0,
                        kWingGpsReceiverCrosswind, ind, init);
}

void UpdateWingGpsBCn0(Indicator *ind, int32_t init) {
  DispatchWingGpsUpdate(UpdateWingGpsNovAtelCn0, UpdateWingGpsSeptentrioCn0,
                        kWingGpsReceiverHover, ind, init);
}

void UpdateWingGpsASolutionType(Indicator *ind, int32_t init) {
  DispatchWingGpsUpdate(UpdateWingGpsNovAtelPosVelType,
                        UpdateWingGpsSeptentrioMode, kWingGpsReceiverCrosswind,
                        ind, init);
}

void UpdateWingGpsBSolutionType(Indicator *ind, int32_t init) {
  DispatchWingGpsUpdate(UpdateWingGpsNovAtelPosVelType,
                        UpdateWingGpsSeptentrioMode, kWingGpsReceiverHover, ind,
                        init);
}

void UpdateWingGpsASigmas(Indicator *ind, int32_t init) {
  DispatchWingGpsUpdate(UpdateWingGpsNovAtelSigmas,
                        UpdateWingGpsSeptentrioSigmas,
                        kWingGpsReceiverCrosswind, ind, init);
}

void UpdateWingGpsBSigmas(Indicator *ind, int32_t init) {
  DispatchWingGpsUpdate(UpdateWingGpsNovAtelSigmas,
                        UpdateWingGpsSeptentrioSigmas, kWingGpsReceiverHover,
                        ind, init);
}
