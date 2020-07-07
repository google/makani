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

#include "control/avionics/avionics_faults.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/encoder_types.h"
#include "avionics/common/faults.h"
#include "avionics/common/tether_message.h"
#include "avionics/motor/firmware/flags.h"
#include "common/macros.h"
#include "control/common.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/system_params.h"

static void CountWithReset(bool reset, int32_t *count) {
  assert(count != NULL && *count >= 0);
  if (reset) {
    *count = 0;
  } else if (*count < INT32_MAX) {
    (*count)++;
  }
}

static void CheckNoUpdateWithReset(bool reset, int32_t no_update_counts_limit,
                                   int32_t *num_no_updates, FaultMask *fault) {
  assert(num_no_updates != NULL && fault != NULL);
  assert(no_update_counts_limit >= 0 && *num_no_updates >= 0);
  CountWithReset(reset, num_no_updates);
  SetFault(kFaultTypeNoUpdate, *num_no_updates > no_update_counts_limit, fault);
}

// Update an array of three no-update counters corresponding to the
// components of a Vec3 signal and set a fault if any counter exceeds
// a given limit.
static void CheckAnyNoUpdateVec3(bool possibly_updated, const Vec3 *v,
                                 const Vec3 *v_z1, bool check_for_value_change,
                                 int32_t no_update_counts_limit,
                                 int32_t num_no_updates[], FaultMask *fault) {
  assert(v != NULL && v_z1 != NULL);
  assert(no_update_counts_limit >= 0);
  assert(num_no_updates != NULL && fault != NULL);

  CountWithReset(
      possibly_updated && (!check_for_value_change || (v->x != v_z1->x)),
      &num_no_updates[0]);
  CountWithReset(
      possibly_updated && (!check_for_value_change || (v->y != v_z1->y)),
      &num_no_updates[1]);
  CountWithReset(
      possibly_updated && (!check_for_value_change || (v->z != v_z1->z)),
      &num_no_updates[2]);

  SetFault(kFaultTypeNoUpdate,
           MaxArrayInt32(num_no_updates, 3, NULL) > no_update_counts_limit,
           fault);
}

static void CheckImuSignal(FaultDetectionImuSignalType signal, const Vec3 *v,
                           const Vec3 *v_z1, bool cvt_updated,
                           const FaultDetectionImuParams *params,
                           AvionicsFaultsImuState *state, FaultMask faults[]) {
  if (!state->initialized[signal] && cvt_updated) {
    for (int32_t j = 0; j < 3; ++j) {
      state->num_no_updates[signal][j] = 0;
    }
    state->initialized[signal] = true;
  } else {
    // If not initialized, we can only hit this case with !cvt_updated, in which
    // case each signal will be counted as a no-update.
    CheckAnyNoUpdateVec3(cvt_updated, v, v_z1, true,
                         params->no_update_counts_limits[signal],
                         state->num_no_updates[signal], &faults[signal]);
  }
}

static void ImuFaultsStateInit(AvionicsFaultsImuState *imu) {
  for (int32_t i = 0; i < kNumFaultDetectionImuSignals; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      imu->num_no_updates[i][j] = INT32_MAX;
    }
    imu->initialized[i] = false;
  }
  imu->acc_z1 = kVec3Zero;
  imu->gyro_z1 = kVec3Zero;
  imu->mag_z1 = kVec3Zero;
}

static void GpsFaultsStateInit(AvionicsFaultsGpsState *gps) {
  for (int32_t i = 0; i < kNumFaultDetectionGpsSignals; ++i) {
    for (int32_t j = 0; j < 3; ++j) {
      gps->num_no_updates[i][j] = INT32_MAX;
    }
  }
  gps->pos_z1 = kVec3Zero;
  gps->vel_z1 = kVec3Zero;
  gps->initialized = false;
}

void AvionicsFaultsInit(AvionicsFaultsState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));

  for (int32_t i = 0; i < kNumControllers; ++i) {
    state->controller_sync[i].num_no_updates = INT32_MAX;
    state->controller_sync[i].sequence_z1 = 0U;
    state->controller_sync[i].initialized = false;
  }

  state->ground_station.num_no_updates = INT32_MAX;
  state->gs_compass.num_no_updates = INT32_MAX;
  state->gs_gps.num_no_updates = INT32_MAX;

  for (int32_t i = 0; i < kNumDrums; ++i) {
    for (int32_t j = 0; j < kNumFaultDetectionGsgSignals; ++j) {
      state->gsg[i].num_no_updates[j] = INT32_MAX;
    }
  }

  for (int32_t i = 0; i < kNumWingImus; ++i) {
    ImuFaultsStateInit(&state->imus[i]);
  }

  state->joystick.num_no_updates = INT32_MAX;

  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    state->levelwind_ele[i].num_no_updates = INT32_MAX;
    state->perch_azi[i].num_no_updates = INT32_MAX;
  }

  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    state->loadcells.num_no_updates[i] = INT32_MAX;
  }

  for (int32_t i = 0; i < kNumMotors; ++i) {
    state->motors[i].num_no_updates = INT32_MAX;
  }

  for (int32_t i = 0; i < kNumPitotSensors; ++i) {
    state->pitots[i].num_no_updates = INT32_MAX;
  }

  state->proximity_sensor.num_no_updates = INT32_MAX;
  state->winch_sensor.num_no_updates = INT32_MAX;
  state->wind_sensor.num_no_updates = INT32_MAX;

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    GpsFaultsStateInit(&state->wing_gps[i]);
  }
}

void GroundvionicsFaultsInit(GroundvionicsFaultsState *state) {
  assert(state != NULL);
  memset(state, 0, sizeof(*state));

  state->gs_compass.num_no_updates = INT32_MAX;

  ImuFaultsStateInit(&state->imu);
  GpsFaultsStateInit(&state->gs_gps);
}

void AvionicsFaultsCheckControllerSync(
    const ControlSyncData *sync, bool cvt_updated,
    const FaultDetectionControllerParams *params,
    AvionicsFaultsControllerSyncState *state, FaultMask *fault) {
  if (!state->initialized && cvt_updated) {
    state->num_no_updates = 0;
    state->initialized = true;
  } else {
    CheckNoUpdateWithReset(
        cvt_updated && IsValidFlightMode(sync->flight_mode) &&
            (sync->sequence != state->sequence_z1),
        params->no_update_counts_limit, &state->num_no_updates, fault);
  }
  state->sequence_z1 = sync->sequence;
}

void AvionicsFaultsCheckGroundEstimate(
    const GroundEstimateMessage *ground_estimate, bool updated,
    const FaultDetectionGroundEstimatorParams *params,
    AvionicsFaultsGroundEstimatorState *state, FaultMask *fault) {
  SetFault(kFaultTypeThrownError, !ground_estimate->position_valid,
           &fault[kFaultDetectionGroundStationEstimatorSignalPosition]);
  SetFault(kFaultTypeThrownError, !ground_estimate->attitude_valid,
           &fault[kFaultDetectionGroundStationEstimatorSignalAttitude]);

  // TODO: Add a separate param for attitude no_update_counts_limit.
  CheckNoUpdateWithReset(
      updated, params->no_update_counts_limit, &state->position_num_no_updates,
      &fault[kFaultDetectionGroundStationEstimatorSignalPosition]);
  CheckNoUpdateWithReset(
      updated, params->no_update_counts_limit, &state->attitude_num_no_updates,
      &fault[kFaultDetectionGroundStationEstimatorSignalAttitude]);
}

void AvionicsFaultsCheckGroundStation(
    const TetherGroundStation *ground_station, bool updated,
    const FaultDetectionGroundStationParams *params,
    AvionicsFaultsGroundStationState *state, FaultMask *fault) {
  CheckNoUpdateWithReset(updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);
  // Check error only so warnings are ignored.
  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(ground_station->no_update_count) ||
               (ground_station->flags & kTetherGroundStationFlagError),
           fault);
}

void AvionicsFaultsCheckDetwist(const TetherGroundStation *ground_station,
                                FaultMask *fault) {
  // Check error only so warnings are ignored.
  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(ground_station->no_update_count) ||
               (ground_station->flags & kTetherGroundStationFlagDetwistError),
           fault);
}

void AvionicsFaultsCheckDrum(const TetherGroundStation *ground_station,
                             FaultMask *fault) {
  // Check error only so warnings are ignored.
  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(ground_station->no_update_count) ||
               (ground_station->flags & kTetherGroundStationFlagDrumError),
           fault);
}

void AvionicsFaultsCheckGsCompass(const TetherGsGpsCompass *compass,
                                  bool updated,
                                  const FaultDetectionGsCompassParams *params,
                                  AvionicsFaultsGsCompassState *state,
                                  FaultMask *fault) {
  CheckNoUpdateWithReset(updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(compass->no_update_count) ||
               (compass->flags & kTetherGsGpsCompassFlagFault),
           fault);
}

void AvionicsFaultsCheckGsGps(const TetherGsGpsPosition *position,
                              const TetherGpsStatus *status, bool updated,
                              const FaultDetectionGsGpsParams *params,
                              AvionicsFaultsGsGpsState *state,
                              FaultMask *fault) {
  CheckNoUpdateWithReset(updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(position->no_update_count) ||
               !TetherIsNoUpdateCountValid(status->no_update_count) ||
               (position->flags & kTetherGsGpsPositionFlagFault) ||
               (status->status == kTetherGpsSolutionStatusNone),
           fault);
  SetFault(kFaultTypeOutOfRange, status->pos_sigma > params->pos_sigma_max,
           fault);
}

void AvionicsFaultsCheckGsg(const TetherDrum *status, bool status_updated,
                            const GsgData *gsg,
                            const FaultDetectionGsgParams *params,
                            AvionicsFaultsGsgState *state, FaultMask faults[]) {
  double gsg_arr[] = {gsg->azi, gsg->ele};
  assert(ARRAYSIZE(gsg_arr) == kNumFaultDetectionGsgSignals);

  if (status_updated) {
    SetFault(kFaultTypeThrownError,
             !TetherIsNoUpdateCountValid(status->no_update_count) ||
                 (status->flags & kTetherDrumFlagGsgAxis1Fault),
             &faults[kFaultDetectionGsgSignalAzi]);

    SetFault(kFaultTypeThrownError,
             !TetherIsNoUpdateCountValid(status->no_update_count) ||
                 (status->flags & kTetherDrumFlagGsgAxis2Fault),
             &faults[kFaultDetectionGsgSignalEle]);
  }

  for (int32_t i = 0; i < kNumFaultDetectionGsgSignals; ++i) {
    CheckNoUpdateWithReset(status_updated, params->no_update_counts_limit[i],
                           &state->num_no_updates[i], &faults[i]);

    SetFault(kFaultTypeOutOfRange, ((gsg_arr[i] < params->signal_min[i]) ||
                                    (gsg_arr[i] > params->signal_max[i])),
             &faults[i]);
  }
}

void AvionicsFaultsCheckImuAccGyro(const FlightComputerImuMessage *imu_sensor,
                                   const ImuData *imu, bool cvt_updated,
                                   const FaultDetectionImuParams *params,
                                   AvionicsFaultsImuState *state,
                                   FaultMask faults[]) {
  const FaultDetectionImuSignalType acc_signal = kFaultDetectionImuSignalAcc;
  const FaultDetectionImuSignalType gyro_signal = kFaultDetectionImuSignalGyro;

  CheckImuSignal(acc_signal, &imu->acc, &state->acc_z1, cvt_updated, params,
                 state, faults);
  CheckImuSignal(gyro_signal, &imu->gyro, &state->gyro_z1, cvt_updated, params,
                 state, faults);

  state->acc_z1 = imu->acc;
  state->gyro_z1 = imu->gyro;

  // The current ADIS16488A TMS570 driver provides latency numbers (in
  // microseconds) indicating the delay between parsing IMU sensor
  // data and transmitting that data over the network.  On boot,
  // unreliable data may be transmitted along with a large latency
  // number.  To handle these cases we explicitly test these latency
  // numbers.
  //
  // TODO: Evaluate adding test based on the "self-test" error flags.
  if (imu_sensor->raw.latency < 0 || imu_sensor->raw.latency == INT32_MAX ||
      (double)imu_sensor->raw.latency > 1e6 * params->max_latency) {
    SetFault(kFaultTypeNoUpdate, true, &faults[acc_signal]);
    SetFault(kFaultTypeNoUpdate, true, &faults[gyro_signal]);
  }
}

void AvionicsFaultsCheckImuMag(const FlightComputerSensorMessage *mag_sensor,
                               const ImuData *imu, bool cvt_updated,
                               const FaultDetectionImuParams *params,
                               AvionicsFaultsImuState *state,
                               FaultMask faults[]) {
  const FaultDetectionImuSignalType mag_signal = kFaultDetectionImuSignalMag;

  CheckImuSignal(mag_signal, &imu->mag, &state->mag_z1, cvt_updated, params,
                 state, faults);
  state->mag_z1 = imu->mag;

  // The current ADIS16488A TMS570 driver provides latency numbers (in
  // microseconds) indicating the delay between parsing IMU sensor
  // data and transmitting that data over the network.  On boot,
  // unreliable data may be transmitted along with a large latency
  // number.  To handle these cases we explicitly test these latency
  // numbers.
  //
  // TODO: Evaluate adding test based on the "self-test" error flags.
  if (mag_sensor->aux.mag_latency < 0 ||
      mag_sensor->aux.mag_latency == INT32_MAX ||
      (double)mag_sensor->aux.mag_latency > 1e6 * params->mag_max_latency) {
    SetFault(kFaultTypeNoUpdate, true, &faults[mag_signal]);
  }

  double mag_norm = Vec3Norm(&imu->mag);
  SetFault(kFaultTypeImplausible, (mag_norm < params->mag_min_plausible ||
                                   mag_norm > params->mag_max_plausible),
           &faults[mag_signal]);
}

void AvionicsFaultsCheckJoystick(const TetherJoystick *joystick, bool updated,
                                 const FaultDetectionJoystickParams *params,
                                 AvionicsFaultsJoystickState *state,
                                 FaultMask *fault) {
  CheckNoUpdateWithReset(updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);

  // If the SetFault call below were run unconditionally, it could clear a
  // no-update fault set above.
  if (!HasFault(kFaultTypeNoUpdate, fault) && updated) {
    bool joystick_not_present = joystick->flags & kTetherJoystickFlagFault;
    SetFault(kFaultTypeNoUpdate, joystick_not_present, fault);
  }

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(joystick->no_update_count), fault);
}

void AvionicsFaultsCheckLevelwindEle(
    const TetherPlatform *platform_sensors, bool platform_sensors_updated,
    const FaultDetectionLevelwindEleParams *params,
    AvionicsFaultsLevelwindEleState *state, FaultMask *fault) {
  CheckNoUpdateWithReset(platform_sensors_updated,
                         params->no_update_counts_limit, &state->num_no_updates,
                         fault);

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(platform_sensors->no_update_count) ||
               (platform_sensors->flags &
                kTetherPlatformFlagLevelwindElevationFault),
           fault);
}

// Aggregate per-node faults into the tether release subsystem.
//
// We signal a fault only if both release mechanisms on a single side are
// faulted, in which case faults on that side propagate to the release
// subsystem.
static void SetTetherReleaseFault(const FaultMask loadcell_node_faults[],
                                  FaultMask *tether_release_fault) {
  bool no_update =
      HasFault(kFaultTypeNoUpdate, &loadcell_node_faults[kLoadcellNodePortA]) &&
      HasFault(kFaultTypeNoUpdate, &loadcell_node_faults[kLoadcellNodePortB]);
  no_update |= HasFault(kFaultTypeNoUpdate,
                        &loadcell_node_faults[kLoadcellNodeStarboardA]) &&
               HasFault(kFaultTypeNoUpdate,
                        &loadcell_node_faults[kLoadcellNodeStarboardB]);

  SetFault(kFaultTypeNoUpdate, no_update, tether_release_fault);
}

void AvionicsFaultsCheckLoadcells(const bool loadcell_messages_updated[],
                                  const LoadcellParams loadcell_params[],
                                  const FaultDetectionLoadcellParams *params,
                                  AvionicsFaultsLoadcellsState *state,
                                  FaultMask sensor_faults[],
                                  bool nodes_faulted[],
                                  FaultMask *tether_release_fault) {
  // Record faults on a per-node basis.
  FaultMask node_faults[kNumLoadcellNodes];
  for (int32_t node = 0; node < kNumLoadcellNodes; ++node) {
    ClearAllFaults(&node_faults[node]);
    CheckNoUpdateWithReset(loadcell_messages_updated[node],
                           params->no_update_counts_limit,
                           &state->num_no_updates[node], &node_faults[node]);
    nodes_faulted[node] = HasAnyFault(&node_faults[node]);
  }

  SetTetherReleaseFault(node_faults, tether_release_fault);

  // Record faults on a per-sensor basis.
  for (int32_t bridle = 0; bridle < kNumBridles; ++bridle) {
    for (int32_t channel = 0; channel < NUM_LOADCELL_CHANNELS; ++channel) {
      LoadcellSensorLabel sensor =
          BridleAndChannelToLoadcellSensorLabel((BridleLabel)bridle, channel);
      const FaultMask *node_fault =
          &node_faults
              [loadcell_params[bridle].channels[channel].strain_location.i_msg];
      SetFault(kFaultTypeNoUpdate, HasFault(kFaultTypeNoUpdate, node_fault),
               &sensor_faults[sensor]);
    }
  }
}

void AvionicsFaultsCheckMotor(const MotorStatusMessage *motor_status,
                              bool updated,
                              const FaultDetectionMotorParams *params,
                              AvionicsFaultsMotorState *state,
                              FaultMask *fault) {
  CheckNoUpdateWithReset(updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);

  SetFault(kFaultTypeThrownError,
           motor_status->motor_status == kMotorStatusError ||
               motor_status->motor_error != kMotorErrorNone,
           fault);
}

void AvionicsFaultsCheckPerchAzi(const TetherPlatform *platform_sensors,
                                 bool platform_sensors_updated,
                                 const FaultDetectionPerchAziParams *params,
                                 AvionicsFaultsPerchAziState *state,
                                 FaultMask *fault) {
  CheckNoUpdateWithReset(platform_sensors_updated,
                         params->no_update_counts_limit, &state->num_no_updates,
                         fault);

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(platform_sensors->no_update_count) ||
               (platform_sensors->flags & kTetherPlatformFlagPerchAzimuthFault),
           fault);
}

void AvionicsFaultsCheckPitot(const PitotSensor *pitot_sensor, bool updated,
                              const FaultDetectionPitotParams *params,
                              const StatusFlags *flags,
                              AvionicsFaultsPitotState *state,
                              FaultMask faults[]) {
  CountWithReset(updated, &state->num_no_updates);
  for (int32_t i = 0; i < kNumFaultDetectionPitotSignals; ++i) {
    SetFault(kFaultTypeNoUpdate,
             state->num_no_updates > params->no_update_counts_limit ||
                 pitot_sensor->latency_usec > 1e6 * params->max_latency,
             &faults[i]);
  }

  // The micro-controller responsible for polling the Pitot sensors
  // does not check if they are populated.  If a sensor value is
  // reported as all ones, this indicates that the serial line was
  // held high by a pull-up resistor and no data was communicated.
  //
  // TODO: Add a more general out of range check here.
  const uint16_t kAllOnes = 0xFFFF;
  SetFault(kFaultTypeOutOfRange, kAllOnes == pitot_sensor->altitude,
           &faults[kFaultDetectionPitotSignalStatic]);
  SetFault(kFaultTypeOutOfRange, kAllOnes == pitot_sensor->speed,
           &faults[kFaultDetectionPitotSignalDynamic]);
  SetFault(kFaultTypeOutOfRange, kAllOnes == pitot_sensor->pitch,
           &faults[kFaultDetectionPitotSignalAlpha]);
  SetFault(kFaultTypeOutOfRange, kAllOnes == pitot_sensor->yaw,
           &faults[kFaultDetectionPitotSignalBeta]);

  // Handle sensor faults.
  SetFault(kFaultTypeThrownError,
           CheckWarning(flags, kFlightComputerWarningPitotAltitude),
           &faults[kFaultDetectionPitotSignalStatic]);
  SetFault(kFaultTypeThrownError,
           CheckWarning(flags, kFlightComputerWarningPitotSpeed),
           &faults[kFaultDetectionPitotSignalDynamic]);
  SetFault(kFaultTypeThrownError,
           CheckWarning(flags, kFlightComputerWarningPitotPitch),
           &faults[kFaultDetectionPitotSignalAlpha]);
  SetFault(kFaultTypeThrownError,
           CheckWarning(flags, kFlightComputerWarningPitotYaw),
           &faults[kFaultDetectionPitotSignalBeta]);
}

void AvionicsFaultsCheckWinchSensor(const TetherPlc *plc_status,
                                    bool plc_status_updated,
                                    const FaultDetectionWinchParams *params,
                                    AvionicsFaultsWinchSensorState *state,
                                    FaultMask *fault) {
  CheckNoUpdateWithReset(plc_status_updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(plc_status->no_update_count) ||
               (plc_status->flags & kTetherPlcFlagPlcWarning) ||
               (plc_status->flags & kTetherPlcFlagPlcError) ||
               (plc_status->flags & kTetherPlcFlagDrumFault),
           fault);
}

void AvionicsFaultsCheckWindSensor(const TetherWind *wind_sensor,
                                   bool wind_sensor_updated,
                                   const FaultDetectionWindSensorParams *params,
                                   AvionicsFaultsWindSensorState *state,
                                   FaultMask *fault) {
  CheckNoUpdateWithReset(wind_sensor_updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(wind_sensor->no_update_count) ||
               (wind_sensor->status != kTetherWindStatusGood),
           fault);
}

void AvionicsFaultsCheckWeather(const TetherWeather *weather,
                                bool weather_updated,
                                const FaultDetectionWeatherParams *params,
                                const GsWeather *min, const GsWeather *max,
                                AvionicsFaultsWeatherState *state,
                                FaultMask *fault) {
  CheckNoUpdateWithReset(weather_updated, params->no_update_counts_limit,
                         &state->num_no_updates, fault);

  SetFault(kFaultTypeThrownError,
           !TetherIsNoUpdateCountValid(weather->no_update_count) ||
               (weather->flags != 0),
           fault);

  // TODO: These limits should be applied to the values in
  // control_input, not in the actual input message. This would
  // guarantee that both the structures (field names) and units would
  // match up properly, as in avionics_saturation.c.
  assert(min->pressure <= max->pressure);
  assert(min->temperature <= max->temperature);
  assert(min->humidity <= max->humidity);
  SetFault(kFaultTypeOutOfRange, !(min->pressure <= weather->pressure_pa &&
                                   weather->pressure_pa <= max->pressure &&
                                   min->temperature <= weather->temperature &&
                                   weather->temperature <= max->temperature &&
                                   min->humidity <= weather->humidity / 100.0 &&
                                   weather->humidity / 100.0 <= max->humidity),
           fault);
}

static void CheckWingGpsCommon(bool cvt_updated,
                               const FaultDetectionGpsParams *params,
                               AvionicsFaultsGpsState *state,
                               FaultMask faults[], GpsData *gps) {
  if (!state->initialized && cvt_updated) {
    for (int32_t i = 0; i < kNumFaultDetectionGpsSignals; ++i) {
      for (int32_t j = 0; j < 3; ++j) {
        state->num_no_updates[i][j] = 0;
      }
    }
    state->time_of_week_z1 = 24 * 7 * 3600 * 1000;
    state->initialized = true;
  }

  gps->new_data = cvt_updated && state->time_of_week_z1 != gps->time_of_week_ms;

  CheckAnyNoUpdateVec3(
      gps->new_data, &gps->pos, &state->pos_z1,
      gps->pos_sol_type != kGpsSolutionTypeFixedPosition,
      params->no_update_counts_limit[kFaultDetectionGpsSignalPos],
      state->num_no_updates[kFaultDetectionGpsSignalPos],
      &faults[kFaultDetectionGpsSignalPos]);

  CheckAnyNoUpdateVec3(
      gps->new_data, &gps->vel, &state->vel_z1,
      gps->vel_sol_type != kGpsSolutionTypeFixedPosition,
      params->no_update_counts_limit[kFaultDetectionGpsSignalVel],
      state->num_no_updates[kFaultDetectionGpsSignalVel],
      &faults[kFaultDetectionGpsSignalVel]);

  state->time_of_week_z1 = gps->time_of_week_ms;
  state->pos_z1 = gps->pos;
  state->vel_z1 = gps->vel;

  // TODO: Prior to go/mkcl/12068 we used rate limiting to
  // check if sudden changes in the GPS were physically plausible.
  // These checks resulted in many false positives in simulation, in part
  // because they did not account for the additive noise / uncertainties
  // on the estimates.
}

void AvionicsFaultsCheckWingGpsNovAtel(
    const NovAtelSolutionMessage *gps_message, bool cvt_updated,
    const FaultDetectionGpsParams *params, AvionicsFaultsGpsState *state,
    FaultMask faults[], GpsData *gps) {
  CheckWingGpsCommon(cvt_updated, params, state, faults, gps);

  // The current NovAtel TMS570 driver transmits GPS messages
  // regardless of whether updates have arrived, marking them with a
  // latency number.  After a reboot, all zero messages are
  // transmitted until the first updates are parsed.  These can be
  // misinterpreted as kNovAtelSolutionStatusSolComputed == 0.
  //
  // To handle these cases, we mark messages as kFaultTypeNoUpdate if
  // the latency is large, and kFaultTypeThrownError if the number of
  // satellites reported in a solution is zero.
  const double latency_sec = (double)gps_message->best_xyz_latency / 1e6 +
                             (double)gps_message->best_xyz.sol_age;
  if (latency_sec > params->max_latency) {
    SetFault(kFaultTypeNoUpdate, true, &faults[kFaultDetectionGpsSignalPos]);
    SetFault(kFaultTypeNoUpdate, true, &faults[kFaultDetectionGpsSignalVel]);
  }

  SetFault(kFaultTypeThrownError, gps_message->best_xyz.pos_sol_status !=
                                          kNovAtelSolutionStatusSolComputed ||
                                      gps_message->best_xyz.num_sol == 0,
           &faults[kFaultDetectionGpsSignalPos]);
  SetFault(kFaultTypeThrownError, gps_message->best_xyz.vel_sol_status !=
                                          kNovAtelSolutionStatusSolComputed ||
                                      gps_message->best_xyz.num_sol == 0,
           &faults[kFaultDetectionGpsSignalVel]);
}

void AvionicsFaultsCheckWingGpsSeptentrio(
    const SeptentrioSolutionMessage *gps_message, bool cvt_updated,
    const FaultDetectionGpsParams *params, AvionicsFaultsGpsState *state,
    FaultMask faults[], GpsData *gps) {
  CheckWingGpsCommon(cvt_updated, params, state, faults, gps);

  // Septentrio-specific fault detection.
  // TODO(b/27502531): Check Septentrio solution quality more thoroughly.
  if (gps_message->latency_usec * 1e-6 > params->max_latency) {
    SetFault(kFaultTypeNoUpdate, true, &faults[kFaultDetectionGpsSignalPos]);
    SetFault(kFaultTypeNoUpdate, true, &faults[kFaultDetectionGpsSignalVel]);
  }

  bool error = gps_message->pvt_cartesian.error != 0;
  bool no_solution =
      ((gps_message->pvt_cartesian.mode & kSeptentrioPvtModeBitSolutionMask) ==
       kSeptentrioPvtModeNoSolution);

  SetFault(kFaultTypeThrownError, error || no_solution,
           &faults[kFaultDetectionGpsSignalPos]);
  SetFault(kFaultTypeThrownError, error || no_solution,
           &faults[kFaultDetectionGpsSignalVel]);
}

void AvionicsFaultsCheckNovAtelCompass(
    const NovAtelCompassMessage *novatel_compass, bool cvt_updated,
    const FaultDetectionGsCompassParams *params, FaultMask faults[],
    GpsCompassData *compass) {
  compass->new_data = cvt_updated;
  // The current NovAtel TMS570 driver transmits GPS messages
  // regardless of whether updates have arrived, marking them with a
  // latency number.  After a reboot, all zero messages are
  // transmitted until the first updates are parsed.  These can be
  // misinterpreted as kNovAtelSolutionStatusSolComputed == 0.
  //
  // To handle these cases, we mark messages as kFaultTypeNoUpdate if
  // the latency is large, and kFaultTypeThrownError if the number of
  // satellites reported in a solution is zero.

  const double heading_latency_sec =
      (double)novatel_compass->heading_latency / 1e6;
  if (heading_latency_sec > params->max_latency) {
    SetFault(kFaultTypeNoUpdate, true,
             &faults[kFaultDetectionGpsCompassSignalAngles]);
  }

  const double heading_rate_latency_sec =
      (double)novatel_compass->heading_rate_latency / 1e6;
  if (heading_rate_latency_sec > params->max_latency) {
    SetFault(kFaultTypeNoUpdate, true,
             &faults[kFaultDetectionGpsCompassSignalAngularRates]);
  }

  SetFault(kFaultTypeThrownError,
           novatel_compass->heading.pos_sol_status !=
                   kNovAtelSolutionStatusSolComputed ||
               novatel_compass->heading.num_sol == 0 ||
               novatel_compass->heading.pos_type == kNovAtelSolutionTypeNone,
           &faults[kFaultDetectionGpsCompassSignalAngles]);
  SetFault(
      kFaultTypeThrownError,
      novatel_compass->heading_rate.pos_sol_status !=
              kNovAtelSolutionStatusSolComputed ||
          novatel_compass->heading_rate.pos_type == kNovAtelSolutionTypeNone,
      &faults[kFaultDetectionGpsCompassSignalAngularRates]);
}
