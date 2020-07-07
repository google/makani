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

#include "control/avionics/avionics_conversion.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/novatel_types.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/winch_messages.h"
#include "common/c_math/mat3.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/avionics/avionics_interface_types.h"
#include "control/control_types.h"
#include "control/system_types.h"
#include "system/labels.h"

void ConvertControllerSync(const ControllerSyncMessage *sync_message,
                           ControlSyncData *sync_data) {
  assert(sync_message != NULL && sync_data != NULL);
  sync_data->sequence = sync_message->sequence;
  sync_data->flight_mode = (FlightMode)sync_message->flight_mode;
}

void ConvertImuAccGyro(const FlightComputerImuMessage *imu_in,
                       const ImuParams *params, ImuData *imu_out) {
  assert(imu_in != NULL && params != NULL && imu_out != NULL);

  // TODO: Incorporate coning/sculling data.
  Vec3 imu_in_acc = {ApplyCal((double)imu_in->raw.acc[0], &params->acc_cal[0]),
                     ApplyCal((double)imu_in->raw.acc[1], &params->acc_cal[1]),
                     ApplyCal((double)imu_in->raw.acc[2], &params->acc_cal[2])};
  Vec3 imu_in_gyro = {
      ApplyCal((double)imu_in->raw.gyro[0], &params->gyro_cal[0]),
      ApplyCal((double)imu_in->raw.gyro[1], &params->gyro_cal[1]),
      ApplyCal((double)imu_in->raw.gyro[2], &params->gyro_cal[2])};

  Mat3TransVec3Mult(&params->dcm_parent2m, &imu_in_acc, &imu_out->acc);
  Mat3TransVec3Mult(&params->dcm_parent2m, &imu_in_gyro, &imu_out->gyro);
}

void ConvertImuMag(const FlightComputerSensorMessage *sensor_in,
                   const ImuParams *params, ImuData *imu_out) {
  assert(sensor_in != NULL && params != NULL && imu_out != NULL);

  Vec3 imu_in_mag = {
      ApplyCal((double)sensor_in->aux.mag[0], &params->mag_cal[0]),
      ApplyCal((double)sensor_in->aux.mag[1], &params->mag_cal[1]),
      ApplyCal((double)sensor_in->aux.mag[2], &params->mag_cal[2])};

  Mat3TransVec3Mult(&params->dcm_parent2m, &imu_in_mag, &imu_out->mag);
}

void ConvertJoystick(const TetherJoystick *joystick_in,
                     bool tether_release_armed, const JoystickParams *params,
                     JoystickData *joystick_out) {
  assert(joystick_in != NULL && params != NULL && joystick_out != NULL);

  joystick_out->throttle =
      ApplyCal(joystick_in->throttle, &params->cal.throttle);
  joystick_out->roll = ApplyCal(joystick_in->roll, &params->cal.roll);
  joystick_out->pitch = ApplyCal(joystick_in->pitch, &params->cal.pitch);
  joystick_out->yaw = ApplyCal(joystick_in->yaw, &params->cal.yaw);

  if (joystick_in->tri_switch == kJoystickSwitchPositionDown ||
      joystick_in->tri_switch == kJoystickSwitchPositionMiddle ||
      joystick_in->tri_switch == kJoystickSwitchPositionUp) {
    joystick_out->switch_position = joystick_in->tri_switch;
  } else {
    joystick_out->switch_position = kJoystickSwitchPositionUp;
  }

  // The momentary switch controls both tether release and autoglide. Separate
  // outputs are necessary for these two uses because tether release is gated on
  // armedness of the release mechanism, whereas autoglide is not.
  joystick_out->release =
      tether_release_armed &&
      joystick_in->momentary_switch == kJoystickSwitchPositionUp;
  joystick_out->engage_auto_glide =
      joystick_in->momentary_switch == kJoystickSwitchPositionUp;
}

void ConvertLoadcells(const LoadcellMessage loadcell_messages[],
                      const bool loadcell_nodes_faulted[],
                      const LoadcellParams params[], double loadcells_out[],
                      bool *tether_release_armed, bool *tether_released) {
  assert(loadcell_messages != NULL && params != NULL && loadcells_out != NULL);

  for (int32_t bridle = 0; bridle < kNumBridles; ++bridle) {
    for (int32_t channel = 0; channel < NUM_LOADCELL_CHANNELS; ++channel) {
      LoadcellSensorLabel loadcell =
          BridleAndChannelToLoadcellSensorLabel((BridleLabel)bridle, channel);

      float strain = GetStrain(
          loadcell_messages, &params[bridle].channels[channel].strain_location);
      loadcells_out[loadcell] =
          ApplyCal(strain, &params[bridle].channels[channel].cal);
    }
  }

  // Determine whether the tether release is armed on both sides.
  bool port_release_armed = false;
  LoadcellNodeLabel port_nodes[] = {kLoadcellNodePortA, kLoadcellNodePortB};
  for (int32_t i = 0; i < ARRAYSIZE(port_nodes); ++i) {
    LoadcellNodeLabel node = port_nodes[i];
    port_release_armed |= !loadcell_nodes_faulted[node] &&
                          loadcell_messages[node].tether_release_fully_armed;
  }
  bool starboard_release_armed = false;
  LoadcellNodeLabel starboard_nodes[] = {kLoadcellNodeStarboardA,
                                         kLoadcellNodeStarboardB};
  for (int32_t i = 0; i < ARRAYSIZE(starboard_nodes); ++i) {
    LoadcellNodeLabel node = starboard_nodes[i];
    starboard_release_armed |=
        !loadcell_nodes_faulted[node] &&
        loadcell_messages[node].tether_release_fully_armed;
  }
  *tether_release_armed = (port_release_armed && starboard_release_armed);

  // Determine whether the tether has been released. We ignore faults here as
  // only no-update faults are currently detected, and if a node ever signalled
  // release, we expect it to continue doing so.
  *tether_released = false;
  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    *tether_released |= (loadcell_messages[i].tether_released &&
                         loadcell_messages[i].tether_released_safety_code ==
                             TETHER_RELEASE_SAFETY_CODE);
  }
}

static GpsSolutionType NovAtelSolutionTypeToGpsSolutionType(
    NovAtelSolutionType type, bool is_velocity) {
  switch (type) {
    case kNovAtelSolutionTypeNone:
      return kGpsSolutionTypeNone;

    case kNovAtelSolutionTypeFixedPos:
      return kGpsSolutionTypeFixedPosition;

    case kNovAtelSolutionTypeFixedHeight:
      return kGpsSolutionTypeFixedHeight;

    case kNovAtelSolutionTypeDopplerVelocity:
      if (is_velocity) {
        return kGpsSolutionTypeStandAlone;
      } else {
        return kGpsSolutionTypeUnsupported;
      }

    case kNovAtelSolutionTypeSingle:
      return kGpsSolutionTypeStandAlone;

    case kNovAtelSolutionTypePsrdiff:
      return kGpsSolutionTypeDifferential;
      break;

    case kNovAtelSolutionTypeL1Float:
      return kGpsSolutionTypeRtkFloat;

    case kNovAtelSolutionTypeIonofreeFloat:
      return kGpsSolutionTypeRtkIonoFreeFloat;

    case kNovAtelSolutionTypeNarrowFloat:
      return kGpsSolutionTypeRtkNarrowFloat;

    case kNovAtelSolutionTypeL1Int:
      return kGpsSolutionTypeRtkInt;

    case kNovAtelSolutionTypeWideInt:
      return kGpsSolutionTypeRtkWideInt;

    case kNovAtelSolutionTypeNarrowInt:
      return kGpsSolutionTypeRtkNarrowInt;

    default:
    case kNovAtelSolutionTypeWaas:
    case kNovAtelSolutionTypePropagated:
    case kNovAtelSolutionTypeOmnistar:
    case kNovAtelSolutionTypeCdgps:
    case kNovAtelSolutionTypeRtkDirectIns:
    case kNovAtelSolutionTypeOmnistarHp:
    case kNovAtelSolutionTypeOmnistarXp:
      return kGpsSolutionTypeUnsupported;
  }
}

void ConvertNovAtelBestXyzToGps(const NovAtelLogBestXyz *best_xyz,
                                GpsData *gps_data) {
  assert(best_xyz != NULL && gps_data != NULL);

  gps_data->new_data = true;
  gps_data->time_of_week_ms = (int32_t)best_xyz->timestamp.tow;

  gps_data->pos.x = best_xyz->pos_x;
  gps_data->pos.y = best_xyz->pos_y;
  gps_data->pos.z = best_xyz->pos_z;
  gps_data->pos_sigma.x = (double)best_xyz->pos_x_sigma;
  gps_data->pos_sigma.y = (double)best_xyz->pos_y_sigma;
  gps_data->pos_sigma.z = (double)best_xyz->pos_z_sigma;

  gps_data->vel.x = best_xyz->vel_x;
  gps_data->vel.y = best_xyz->vel_y;
  gps_data->vel.z = best_xyz->vel_z;
  gps_data->vel_sigma.x = (double)best_xyz->vel_x_sigma;
  gps_data->vel_sigma.y = (double)best_xyz->vel_y_sigma;
  gps_data->vel_sigma.z = (double)best_xyz->vel_z_sigma;

  gps_data->pos_sol_type = NovAtelSolutionTypeToGpsSolutionType(
      (NovAtelSolutionType)best_xyz->pos_type, false);
  gps_data->vel_sol_type = NovAtelSolutionTypeToGpsSolutionType(
      (NovAtelSolutionType)best_xyz->vel_type, true);
}

static GpsSolutionType SeptentrioPvtModeToGpsSolutionType(
    SeptentrioPvtMode mode) {
  if (mode & kSeptentrioPvtModeBit2dMode) {
    return kGpsSolutionTypeFixedHeight;
  }
  switch (mode & kSeptentrioPvtModeBitSolutionMask) {
    case kSeptentrioPvtModeNoSolution:
      return kGpsSolutionTypeNone;

    case kSeptentrioPvtModeStandAlone:
      return kGpsSolutionTypeStandAlone;

    case kSeptentrioPvtModeDifferential:
      return kGpsSolutionTypeDifferential;

    case kSeptentrioPvtModeFixedLocation:
      return kGpsSolutionTypeFixedPosition;

    case kSeptentrioPvtModeRtkFixed:
      return kGpsSolutionTypeRtkInt;

    case kSeptentrioPvtModeRtkFloat:
      return kGpsSolutionTypeRtkFloat;

    default:
    case kSeptentrioPvtModeSbasAided:
    case kSeptentrioPvtModeMovingBaseRtkFixed:
    case kSeptentrioPvtModeMovingBaseRtkFloat:
    case kSeptentrioPvtModePrecisePointPositioning:
      return kGpsSolutionTypeUnsupported;
  }
}

void ConvertSeptentrioSolutionToGps(const SeptentrioSolutionMessage *solution,
                                    GpsData *gps_data) {
  assert(solution != NULL && gps_data != NULL);

  gps_data->new_data = true;
  gps_data->time_of_week_ms = (int32_t)solution->pvt_cartesian.timestamp.tow;

  gps_data->pos.x = solution->pvt_cartesian.x;
  gps_data->pos.y = solution->pvt_cartesian.y;
  gps_data->pos.z = solution->pvt_cartesian.z;

  gps_data->pos_sigma.x = sqrt(solution->pos_cov_cartesian.cov_xx);
  gps_data->pos_sigma.y = sqrt(solution->pos_cov_cartesian.cov_yy);
  gps_data->pos_sigma.z = sqrt(solution->pos_cov_cartesian.cov_zz);

  gps_data->vel.x = solution->pvt_cartesian.v_x;
  gps_data->vel.y = solution->pvt_cartesian.v_y;
  gps_data->vel.z = solution->pvt_cartesian.v_z;

  gps_data->vel_sigma.x = sqrt(solution->vel_cov_cartesian.cov_xx);
  gps_data->vel_sigma.y = sqrt(solution->vel_cov_cartesian.cov_yy);
  gps_data->vel_sigma.z = sqrt(solution->vel_cov_cartesian.cov_zz);

  gps_data->pos_sol_type = SeptentrioPvtModeToGpsSolutionType(
      (SeptentrioPvtMode)solution->pvt_cartesian.mode);
  gps_data->vel_sol_type = SeptentrioPvtModeToGpsSolutionType(
      (SeptentrioPvtMode)solution->pvt_cartesian.mode);
}

void ConvertPitot(const PitotSensor *pitot_in, const PitotSensorParams *params,
                  PitotData *pitot_out) {
  assert(pitot_in != NULL && params != NULL && pitot_out != NULL);

  pitot_out->stat_press = ApplyCal(pitot_in->altitude, &params->stat_cal);
  pitot_out->diff.alpha_press = ApplyCal(pitot_in->pitch, &params->alpha_cal);
  pitot_out->diff.beta_press = ApplyCal(pitot_in->yaw, &params->beta_cal);
  pitot_out->diff.dyn_press = ApplyCal(pitot_in->speed, &params->dyn_cal);
}

void ConvertNovAtelCompass(const NovAtelCompassMessage *novatel_compass,
                           GpsCompassData *compass) {
  assert(novatel_compass != NULL && compass != NULL);
  compass->new_data = true;
  compass->heading = novatel_compass->heading.heading;
  compass->heading_sigma = novatel_compass->heading.heading_sigma;
  compass->pitch = novatel_compass->heading.pitch;
  compass->pitch_sigma = novatel_compass->heading.pitch_sigma;
  compass->angle_sol_type = NovAtelSolutionTypeToGpsSolutionType(
      (NovAtelSolutionType)novatel_compass->heading.pos_type, false);

  compass->heading_rate = novatel_compass->heading_rate.heading_rate;
  compass->pitch_rate = novatel_compass->heading_rate.pitch_rate;
  compass->rate_sol_type = NovAtelSolutionTypeToGpsSolutionType(
      (NovAtelSolutionType)novatel_compass->heading_rate.pos_type, true);
}
