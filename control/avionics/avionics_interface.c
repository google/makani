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

#include "control/avionics/avionics_interface.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>  // For NULL.
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/pack_tether_message.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/servo_types.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_message.h"
#include "avionics/motor/firmware/flags.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/aio_node.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "control/actuator_util.h"
#include "control/avionics/avionics_conversion.h"
#include "control/avionics/avionics_faults.h"
#include "control/avionics/avionics_saturation.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "control/sensor_types.h"
#include "control/system_types.h"
#include "system/labels.h"
#include "system/labels_util.h"

void AvionicsInterfaceInit(AvionicsInterfaceState *state) {
  assert(state != NULL);

  memset(state, 0, sizeof(*state));

  state->last_used_gs_gps_position_no_update_count = INT32_MAX;
  state->last_used_gs_gps_status_no_update_count = INT32_MAX;

  // Sequence numbers are initialized to 0U by the memset.
  AvionicsFaultsInit(&state->faults_state);
  TetherUpMergeStateInit(&state->tether_up_merge_state);
}

void GroundvionicsInterfaceInit(GroundvionicsInterfaceState *state) {
  assert(state != NULL);

  memset(state, 0, sizeof(*state));

  state->last_used_gs_gps_position_no_update_count = INT32_MAX;
  state->last_used_gs_gps_status_no_update_count = INT32_MAX;

  // Sequence numbers are initialized to 0U by the memset.
  GroundvionicsFaultsInit(&state->faults_state);
}

// TODO: Make the "Handle" functions either avionics-centric or
// controller-input-centric once we've finished moving detection of input faults
// to this file.

static void HandleControllerSync(
    const FaultDetectionControllerParams *fault_params,
    ControllerSyncMessage controller_sync[], ControlSyncData sync[],
    uint16_t sequence_numbers[],
    AvionicsFaultsControllerSyncState faults_states[], FaultMask faults[]) {
  for (int32_t i = 0; i < kNumControllers; ++i) {
    bool updated = CvtGetControllerSyncMessage(
        ControllerLabelToControllerAioNode(i), &controller_sync[i],
        &sequence_numbers[i], NULL);
    ConvertControllerSync(&controller_sync[i], &sync[i]);
    AvionicsFaultsCheckControllerSync(&sync[i], updated, fault_params,
                                      &faults_states[i], &faults[i]);
  }
}

static bool CheckTetherUpdate(int32_t no_update_count_z1, uint16_t seq_z1,
                              int32_t no_update_count, uint16_t seq) {
  // Though tempting, we should not compare no_update_counts against each
  // other. The TetherUp messages come from multiple sources and we can not
  // assume the no_update_counts agree. We can, however, compare the
  // no_update_counts against a minimum required latency.
  if (TetherIsNoUpdateCountValid(no_update_count)) {
    if (TetherIsNoUpdateCountValid(no_update_count_z1)) {
      return TetherCompareSequence(seq, seq_z1) > 0;
    }
    return true;
  }
  return false;
}

static void HandleTetherUpToGsGpsData(
    const FaultDetectionGsGpsParams *fault_params,
    const TetherUpMessage *tether_up, GsGpsData *gs_gps_data,
    AvionicsInterfaceState *state, AvionicsFaultsGsGpsState *faults_state,
    FaultMask *fault) {
  const TetherGsGpsPosition *position = &tether_up->gps_position;
  const TetherGpsStatus *status = &tether_up->gps_status;

  // The position and status may not come from the same message. This
  // condition at least guarantees that they will be within
  // STALE_MESSAGE_TIMEOUT_US of each other.
  bool updated =
      CheckTetherUpdate(state->last_used_gs_gps_position_no_update_count,
                        state->last_used_gs_gps_position_seq,
                        position->no_update_count, position->sequence) &&
      CheckTetherUpdate(state->last_used_gs_gps_status_no_update_count,
                        state->last_used_gs_gps_status_seq,
                        status->no_update_count, status->sequence);
  if (updated) {
    state->last_used_gs_gps_position_no_update_count =
        position->no_update_count;
    state->last_used_gs_gps_position_seq = position->sequence;
    state->last_used_gs_gps_status_no_update_count = status->no_update_count;
    state->last_used_gs_gps_status_seq = status->sequence;
  }

  gs_gps_data->pos.x = position->ecef[0];
  gs_gps_data->pos.y = position->ecef[1];
  gs_gps_data->pos.z = position->ecef[2];
  gs_gps_data->pos_sigma = status->pos_sigma;

  AvionicsFaultsCheckGsGps(position, status, updated, fault_params,
                           faults_state, fault);
}

static void HandleWingGpsNovAtel(AioNode node,
                                 const FaultDetectionGpsParams *fault_params,
                                 NovAtelSolutionMessage *gps_message,
                                 GpsData *gps_data, uint16_t *sequence_number,
                                 AvionicsFaultsGpsState *faults_state,
                                 FaultMask faults[]) {
  bool updated =
      CvtGetNovAtelSolutionMessage(node, gps_message, sequence_number, NULL);
  ConvertNovAtelBestXyzToGps(&gps_message->best_xyz, gps_data);
  AvionicsFaultsCheckWingGpsNovAtel(gps_message, updated, fault_params,
                                    faults_state, faults, gps_data);
}

static void HandleWingGpsSeptentrio(AioNode node,
                                    const FaultDetectionGpsParams *fault_params,
                                    SeptentrioSolutionMessage *gps_message,
                                    GpsData *gps_data,
                                    uint16_t *sequence_number,
                                    AvionicsFaultsGpsState *faults_state,
                                    FaultMask faults[]) {
  bool updated =
      CvtGetSeptentrioSolutionMessage(node, gps_message, sequence_number, NULL);
  ConvertSeptentrioSolutionToGps(gps_message, gps_data);
  AvionicsFaultsCheckWingGpsSeptentrio(gps_message, updated, fault_params,
                                       faults_state, faults, gps_data);
}

static void HandleNovAtelGpsCompass(
    AioNode node, const FaultDetectionGsCompassParams *fault_params,
    NovAtelCompassMessage *novatel_compass, GpsCompassData *compass,
    uint16_t *sequence_number, FaultMask faults[]) {
  bool updated =
      CvtGetNovAtelCompassMessage(node, novatel_compass, sequence_number, NULL);
  ConvertNovAtelCompass(novatel_compass, compass);

  AvionicsFaultsCheckNovAtelCompass(novatel_compass, updated, fault_params,
                                    &faults[kSubsysGsCompassAngles], compass);
}

static void HandleFlightComputerImu(
    const AioNode node, const ImuParams *imu_params,
    const FaultDetectionImuParams *imu_fault_params,
    FlightComputerImuMessage *flight_comp_imu, ImuData *imu,
    uint16_t *sequence_number, AvionicsFaultsImuState *imu_faults_states,
    FaultMask *faults) {
  bool updated = CvtGetFlightComputerImuMessage(node, flight_comp_imu,
                                                sequence_number, NULL);

  ConvertImuAccGyro(flight_comp_imu, imu_params, imu);
  AvionicsFaultsCheckImuAccGyro(flight_comp_imu, imu, updated, imu_fault_params,
                                imu_faults_states, faults);
}

static void HandleFlightComputersImus(
    const ImuParams imu_params[],
    const FaultDetectionImuParams *imu_fault_params,
    FlightComputerImuMessage flight_comp_imu[], ImuData imus[],
    uint16_t sequence_numbers[], AvionicsFaultsImuState imu_faults_states[],
    FaultMask faults[]) {
  const SubsystemLabel imu_subsystems[] = {SUBSYS_IMU_A, SUBSYS_IMU_B,
                                           SUBSYS_IMU_C};
  assert(ARRAYSIZE(imu_subsystems) == kNumWingImus);
  assert((int32_t)kNumWingImus == (int32_t)kNumFlightComputers);

  for (int32_t i = 0; i < kNumFlightComputers; ++i) {
    HandleFlightComputerImu(FlightComputerLabelToFlightComputerAioNode(i),
                            &imu_params[i], imu_fault_params,
                            &flight_comp_imu[i], &imus[i], &sequence_numbers[i],
                            &imu_faults_states[i], &faults[imu_subsystems[i]]);
  }
}

static bool HandleFlightComputerSensorsExceptPitot(
    const AioNode node, const ImuParams *imu_params,
    const FaultDetectionImuParams *imu_fault_params,
    FlightComputerSensorMessage *flight_comp_sensors, ImuData *imu,
    uint16_t *sequence_number, AvionicsFaultsImuState *imu_faults_states,
    FaultMask *faults) {
  bool updated = CvtGetFlightComputerSensorMessage(node, flight_comp_sensors,
                                                   sequence_number, NULL);

  ConvertImuMag(flight_comp_sensors, imu_params, imu);
  AvionicsFaultsCheckImuMag(flight_comp_sensors, imu, updated, imu_fault_params,
                            imu_faults_states, faults);

  return updated;
}

static void HandleFlightComputersSensors(
    const ImuParams imu_params[],
    const FaultDetectionImuParams *imu_fault_params,
    const PitotParams *pitot_params,
    const FaultDetectionPitotParams *pitot_fault_params,
    const FlightComputerLabel pitot_fc_labels[],
    FlightComputerSensorMessage flight_comp_sensors[], ImuData imus[],
    PitotData pitot_datas[], uint16_t sequence_numbers[],
    AvionicsFaultsImuState imu_faults_states[],
    AvionicsFaultsPitotState pitot_fault_states[], FaultMask faults[]) {
  const SubsystemLabel imu_subsystems[] = {SUBSYS_IMU_A, SUBSYS_IMU_B,
                                           SUBSYS_IMU_C};
  assert(ARRAYSIZE(imu_subsystems) == kNumWingImus);
  assert((int32_t)kNumWingImus == (int32_t)kNumFlightComputers);

  bool updated[kNumFlightComputers];
  for (int32_t i = 0; i < kNumFlightComputers; ++i) {
    updated[i] = HandleFlightComputerSensorsExceptPitot(
        FlightComputerLabelToFlightComputerAioNode(i), &imu_params[i],
        imu_fault_params, &flight_comp_sensors[i], &imus[i],
        &sequence_numbers[i], &imu_faults_states[i],
        &faults[imu_subsystems[i]]);
  }

  for (int32_t i = 0; i < kNumPitotSensors; ++i) {
    const FlightComputerLabel pitot_fc = pitot_fc_labels[i];
    ConvertPitot(&flight_comp_sensors[pitot_fc].pitot,
                 &pitot_params->sensors[i], &pitot_datas[i]);
    SubsystemLabel pitot_subsystem;
    switch (i) {
      case kPitotSensorHighSpeed:
        pitot_subsystem = SUBSYS_PITOT_SENSOR_HIGH_SPEED;
        break;
      case kPitotSensorLowSpeed:
        pitot_subsystem = SUBSYS_PITOT_SENSOR_LOW_SPEED;
        break;
      default:
        assert(false);
        continue;
    }

    AvionicsFaultsCheckPitot(&flight_comp_sensors[pitot_fc].pitot,
                             updated[pitot_fc], pitot_fault_params,
                             &flight_comp_sensors[pitot_fc].flags,
                             &pitot_fault_states[i], &faults[pitot_subsystem]);
  }
}

static void HandleGroundEstimate(
    const FaultDetectionGroundEstimatorParams *ground_estimator_fault_params,
    GroundEstimateMessage *ground_estimate_in,
    GroundEstimateMessage *ground_estimate_out, uint16_t *sequence_number,
    AvionicsFaultsGroundEstimatorState *ground_estimator_fault_state,
    FaultMask *fault) {
  bool updated = CvtGetGroundEstimateMessage(
      kAioNodeGsEstimator, ground_estimate_in, sequence_number, NULL);

  *ground_estimate_out = *ground_estimate_in;

  AvionicsFaultsCheckGroundEstimate(ground_estimate_out, updated,
                                    ground_estimator_fault_params,
                                    ground_estimator_fault_state, fault);
}

static void HandleTetherUpToGsCompass(
    const FaultDetectionGsCompassParams *fault_params,
    const CalParams *heading_cal, TetherUpMessage *status_z1,
    const TetherUpMessage *status, double *perch_heading,
    AvionicsFaultsGsCompassState *state, FaultMask *fault) {
  assert(-PI <= heading_cal->bias && heading_cal->bias < PI);

  const TetherGsGpsCompass *compass = &status->gps_compass;

  bool updated = CheckTetherUpdate(status_z1->gps_compass.no_update_count,
                                   status_z1->gps_compass.sequence,
                                   compass->no_update_count, compass->sequence);

  // TODO: After input fault detection for the compass is
  // moved inside avionics_faults, catch out of range errors here.
  *perch_heading = Wrap(ApplyCal(compass->heading, heading_cal), 0.0, 2.0 * PI);

  AvionicsFaultsCheckGsCompass(compass, updated, fault_params, state, fault);
}

static void TetherDrumToGsgData(const GsgParams *gsg_params,
                                const FaultDetectionGsgParams *fault_params,
                                const TetherDrum *status_z1,
                                const TetherDrum *status, GsgData *gsg_data,
                                AvionicsFaultsGsgState *faults_state,
                                FaultMask faults[]) {
  bool updated =
      CheckTetherUpdate(status_z1->no_update_count, status_z1->sequence,
                        status->no_update_count, status->sequence);

  gsg_data->azi = ApplyCal(status->gsg_axis1, &gsg_params->azi_cal);
  gsg_data->ele = ApplyCal(status->gsg_axis2, &gsg_params->ele_cal);

  AvionicsFaultsCheckGsg(status, updated, gsg_data, fault_params, faults_state,
                         faults);
}

static void HandleTetherGroundStationToGsSensorData(
    const WinchParams *winch_params,
    const FaultDetectionGroundStationParams *fault_params,
    const TetherGroundStation *gs_z1, const TetherGroundStation *gs,
    GsSensorData *gs_sensors, AvionicsFaultsGroundStationState *faults_state,
    FaultMask *gs_fault, FaultMask *detwist_fault, FaultMask *drum_fault) {
  bool updated = CheckTetherUpdate(gs_z1->no_update_count, gs_z1->sequence,
                                   gs->no_update_count, gs->sequence);

  gs_sensors->mode = (GroundStationMode)gs->mode;
  gs_sensors->transform_stage = gs->transform_stage;
  gs_sensors->proximity = gs->proximity;
  gs_sensors->winch_pos = ApplyCal(gs->drum_angle, &winch_params->position_cal);
  gs_sensors->detwist_pos = Wrap(gs->detwist_angle, -PI, PI);

  AvionicsFaultsCheckGroundStation(gs, updated, fault_params, faults_state,
                                   gs_fault);
  AvionicsFaultsCheckDetwist(gs, detwist_fault);
  AvionicsFaultsCheckDrum(gs, drum_fault);
}

static void HandleTetherUpToGsgData(const GsgParams *gsg_params,
                                    const FaultDetectionGsgParams *fault_params,
                                    const TetherUpMessage *status_z1,
                                    const TetherUpMessage *status,
                                    GsgData gsg_data[],
                                    AvionicsFaultsGsgState faults_state[],
                                    FaultMask faults[]) {
  const SubsystemLabel gsg_subsystems[] = {SUBSYS_GSG_A, SUBSYS_GSG_B};
  assert(ARRAYSIZE(gsg_subsystems) == kNumDrums);

  TetherDrumToGsgData(gsg_params, fault_params, &status_z1->drum_a,
                      &status->drum_a, &gsg_data[kDrumSensorsA],
                      &faults_state[kDrumSensorsA],
                      &faults[gsg_subsystems[kDrumSensorsA]]);

  TetherDrumToGsgData(gsg_params, fault_params, &status_z1->drum_b,
                      &status->drum_b, &gsg_data[kDrumSensorsB],
                      &faults_state[kDrumSensorsB],
                      &faults[gsg_subsystems[kDrumSensorsB]]);
}

static void TetherPlatformToPerchData(PlatformLabel source,
                                      const FaultDetectionParams *fault_params,
                                      const TetherPlatform *status_z1,
                                      const TetherPlatform *status,
                                      PerchData *perch_data,
                                      AvionicsFaultsState *faults_state,
                                      FaultMask faults[]) {
  const SubsystemLabel levelwind_ele_subsystems[] = {kSubsysLevelwindEleA,
                                                     kSubsysLevelwindEleB};
  const SubsystemLabel perch_azi_subsystems[] = {kSubsysPerchAziA,
                                                 kSubsysPerchAziB};
  assert(ARRAYSIZE(levelwind_ele_subsystems) == kNumPlatforms);
  assert(ARRAYSIZE(perch_azi_subsystems) == kNumPlatforms);

  bool updated =
      CheckTetherUpdate(status_z1->no_update_count, status_z1->sequence,
                        status->no_update_count, status->sequence);

  perch_data->perch_azi[source] = status->perch_azi;
  perch_data->levelwind_ele[source] = status->levelwind_ele;

  AvionicsFaultsCheckLevelwindEle(status, updated, &fault_params->levelwind_ele,
                                  &faults_state->levelwind_ele[source],
                                  &faults[levelwind_ele_subsystems[source]]);
  AvionicsFaultsCheckPerchAzi(status, updated, &fault_params->perch_azi,
                              &faults_state->perch_azi[source],
                              &faults[perch_azi_subsystems[source]]);
}

static void HandleTetherUpToPerchData(const FaultDetectionParams *fault_params,
                                      const TetherUpMessage *status_z1,
                                      const TetherUpMessage *status,
                                      PerchData *perch_data,
                                      AvionicsFaultsState *faults_state,
                                      FaultMask faults[]) {
  TetherPlatformToPerchData(kPlatformSensorsA, fault_params,
                            &status_z1->platform_a, &status->platform_a,
                            perch_data, faults_state, faults);

  TetherPlatformToPerchData(kPlatformSensorsB, fault_params,
                            &status_z1->platform_b, &status->platform_b,
                            perch_data, faults_state, faults);
}

static void HandleTetherUpToWindData(const FaultDetectionParams *fault_params,
                                     const TetherUpMessage *status_z1,
                                     const TetherUpMessage *status,
                                     Vec3 *wind_ws, AvionicsFaultsState *state,
                                     FaultMask faults[]) {
  bool updated = CheckTetherUpdate(
      status_z1->wind.no_update_count, status_z1->wind.sequence,
      status->wind.no_update_count, status->wind.sequence);

  wind_ws->x = status->wind.velocity[0];
  wind_ws->y = status->wind.velocity[1];
  wind_ws->z = status->wind.velocity[2];

  AvionicsFaultsCheckWindSensor(&status->wind, updated, &fault_params->wind,
                                &state->wind_sensor,
                                &faults[kSubsysWindSensor]);
}

static void HandleTetherUpToWeatherData(
    const FaultDetectionParams *fault_params, const TetherUpMessage *status_z1,
    const TetherUpMessage *status, const GsWeather *min, const GsWeather *max,
    GsWeather *weather, AvionicsFaultsState *state, FaultMask faults[]) {
  bool updated = CheckTetherUpdate(
      status_z1->weather.no_update_count, status_z1->weather.sequence,
      status->weather.no_update_count, status->weather.sequence);

  weather->temperature = status->weather.temperature;
  weather->pressure = status->weather.pressure_pa;

  // Convert from percent to a fraction.
  weather->humidity = status->weather.humidity / 100.0;

  AvionicsFaultsCheckWeather(&status->weather, updated, &fault_params->weather,
                             min, max, &state->weather,
                             &faults[kSubsysWeather]);
}

static void HandleTetherPlcToPerchData(
    const WinchParams *winch_params, const FaultDetectionParams *fault_params,
    const TetherPlc *status_z1, const TetherPlc *status, PerchData *perch_data,
    AvionicsFaultsState *state, FaultMask faults[]) {
  bool updated =
      CheckTetherUpdate(status_z1->no_update_count, status_z1->sequence,
                        status->no_update_count, status->sequence);
  perch_data->winch_pos =
      ApplyCal(status->drum_angle, &winch_params->position_cal);

  AvionicsFaultsCheckWinchSensor(status, updated, &fault_params->winch,
                                 &state->winch_sensor, &faults[kSubsysWinch]);
}

static void HandleJoystickData(const JoystickParams *params,
                               const FaultDetectionJoystickParams *fault_params,
                               const TetherJoystick *tether_joystick_z1,
                               const TetherJoystick *tether_joystick,
                               bool tether_release_armed,
                               JoystickData *joystick_data,
                               AvionicsFaultsJoystickState *state,
                               FaultMask *fault) {
  bool updated = CheckTetherUpdate(
      tether_joystick_z1->no_update_count, tether_joystick_z1->sequence,
      tether_joystick->no_update_count, tether_joystick->sequence);

  ConvertJoystick(tether_joystick, tether_release_armed, params, joystick_data);
  AvionicsFaultsCheckJoystick(tether_joystick, updated, fault_params, state,
                              fault);
}

static void HandleLoadcellMessages(
    const LoadcellParams params[],
    const FaultDetectionLoadcellParams *fault_params,
    LoadcellMessage loadcell_messages[], double loadcells_out[],
    bool *tether_release_armed, bool *tether_released,
    uint16_t sequence_numbers[], AvionicsFaultsLoadcellsState *faults_state,
    FaultMask sensor_faults[], FaultMask *tether_release_fault) {
  bool updated[kNumLoadcellNodes];
  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    updated[i] = CvtGetLoadcellMessage(
        LoadcellNodeLabelToLoadcellNodeAioNode(i), &loadcell_messages[i],
        &sequence_numbers[i], NULL);
  }

  bool loadcell_nodes_faulted[kNumLoadcellNodes];
  AvionicsFaultsCheckLoadcells(updated, params, fault_params, faults_state,
                               sensor_faults, loadcell_nodes_faulted,
                               tether_release_fault);
  ConvertLoadcells(loadcell_messages, loadcell_nodes_faulted, params,
                   loadcells_out, tether_release_armed, tether_released);
}

static void HandleMotorStatuses(const RotorParams rotor_params[],
                                const RotorSensorParams rotor_sensor_params[],
                                const FaultDetectionMotorParams *fault_params,
                                MotorStatusMessage motor_statuses[],
                                double rotors[], int32_t *stacking_state,
                                uint16_t sequence_numbers[],
                                AvionicsFaultsMotorState faults_state[],
                                FaultMask faults[]) {
  for (int32_t i = 0; i < kNumMotors; ++i) {
    bool updated = CvtGetMotorStatusMessage(MotorLabelToMotorAioNode(i),
                                            &motor_statuses[i],
                                            &sequence_numbers[i], NULL);
    rotors[i] =
        (double)rotor_params[i].dir *
        ApplyCal(motor_statuses[i].omega, &rotor_sensor_params[i].omega_cal);
    AvionicsFaultsCheckMotor(&motor_statuses[i], updated, fault_params,
                             &faults_state[i], &faults[i]);
  }

  // Set the stacking state based on which motors are running.
  //
  // TODO: This is a basic first pass at handling motor
  // errors.  It does not handle some obvious situations like motors
  // not updating.
  if (!(motor_statuses[kMotorSbo].motor_status & kMotorStatusRunning) ||
      !(motor_statuses[kMotorPto].motor_status & kMotorStatusRunning)) {
    *stacking_state = kStackingStateFaultBlock1;
  } else if (!(motor_statuses[kMotorSbi].motor_status & kMotorStatusRunning) ||
             !(motor_statuses[kMotorPti].motor_status & kMotorStatusRunning)) {
    *stacking_state = kStackingStateFaultBlock2;
  } else if (!(motor_statuses[kMotorPbi].motor_status & kMotorStatusRunning) ||
             !(motor_statuses[kMotorSti].motor_status & kMotorStatusRunning)) {
    *stacking_state = kStackingStateFaultBlock3;
  } else if (!(motor_statuses[kMotorPbo].motor_status & kMotorStatusRunning) ||
             !(motor_statuses[kMotorSto].motor_status & kMotorStatusRunning)) {
    *stacking_state = kStackingStateFaultBlock4;
  } else {
    *stacking_state = kStackingStateNormal;
  }
}

static void HandleServoStatuses(ServoStatusMessage servo_statuses[],
                                const ServoParams params[], double flaps[],
                                uint16_t sequence_numbers[]) {
  for (int32_t i = 0; i < kNumServos; ++i) {
    CvtGetServoStatusMessage(ServoLabelToServoAioNode(i), &servo_statuses[i],
                             &sequence_numbers[i], NULL);
  }

  // TODO: Consider indicating a fault if the positions of
  // redundant servos disagree excessively.
  double servo_angles[kNumServos];
  for (int32_t i = 0; i < kNumServos; ++i) {
    servo_angles[i] = servo_statuses[i].angle_estimate;
  }
  ServoAnglesToFlapAngles(servo_angles, params, flaps);
}

static void HandleFlightCommand(ControlInput *control_input) {
  assert(control_input != NULL);

  control_input->force_hover_accel = false;
  control_input->force_high_tension = false;
  control_input->force_reel = false;
  control_input->gs_unpause_transform = false;
  control_input->force_detwist_turn_once = false;
  control_input->experiment_type = kExperimentTypeNoTest;
  control_input->experiment_case_id = 0U;
  FlightCommandMessage command;
  memset(&command, 0, sizeof(command));

  if (CvtGetFlightCommandMessage(kAioNodeOperator, &command, NULL, NULL) &&
      command.safety_code == FLIGHT_COMMAND_SIGNAL) {
    // This only executes each time a new message is available in the CVT. So if
    // FlightCommandMessage is sent at a lower rate than the controller
    // runs, the override will only be set on a fraction of controller
    // cycles. (At time of writing, that is 1 out of every 10 cycles.)  That
    // will still be sufficient to trigger the override, though.
    control_input->force_hover_accel = command.force_hover_accel;
    control_input->force_high_tension = command.force_high_tension;
    control_input->force_reel = command.force_reel;
    control_input->gs_unpause_transform = command.gs_unpause_transform;
    control_input->force_detwist_turn_once = command.force_detwist_turn_once;
    control_input->experiment_type = (ExperimentType)command.experiment_type;
    control_input->experiment_case_id = command.experiment_case_id;
  }
}

// Avionics to control.
//
// TODO: Rather than report "updated" flags,
// ConvertAvionicsToControl should likely report the time since the
// last sensor update.  This will allow it to combine the time at
// which the last message was updated with any latency numbers
// embedded in the messages themselves.
void ConvertAvionicsToControl(const SystemParams *system_params,
                              const SensorLimitsParams *limits_params,
                              const FaultDetectionParams *fault_params,
                              AvionicsInterfaceState *state,
                              ControlInputMessages *input_messages,
                              ControlInput *control_input, FaultMask faults[]) {
  assert(system_params != NULL);
  assert(limits_params != NULL);
  assert(fault_params != NULL);
  assert(input_messages != NULL);
  assert(control_input != NULL);

  memset(control_input, 0, sizeof(*control_input));

  // Clear faults that are owned by this module.
  for (int32_t i = 0; i < kNumSubsystems; ++i) {
    SetFault(kFaultTypeNoUpdate, false, &faults[i]);
    SetFault(kFaultTypeThrownError, false, &faults[i]);
    SetFault(kFaultTypeOutOfRange, false, &faults[i]);
    SetFault(kFaultTypeImplausible, false, &faults[i]);
  }

  AvionicsFaultsState *faults_state = &state->faults_state;
  AvionicsSequenceNumbers *sequence_numbers = &state->sequence_numbers;

  HandleControllerSync(&fault_params->control, input_messages->controller_sync,
                       control_input->sync, sequence_numbers->controller_sync,
                       faults_state->controller_sync,
                       &faults[SUBSYS_CONTROLLERS]);

  TetherUpMergeState *merge_state = &state->tether_up_merge_state;
  TetherUpMessage tether_up_z1 = merge_state->output_message;
  const TetherUpMessage *tether_up = TetherUpMergeCvtGet(merge_state);

  // Required by sim.
  assert(sizeof(merge_state->input_messages) ==
         sizeof(input_messages->tether_up_messages));
  for (int32_t i = 0; i < ARRAYSIZE(merge_state->input_messages); ++i) {
    if (merge_state->input_updated[i]) {
      input_messages->tether_up_messages[i] = merge_state->input_messages[i];
    }
  }
  if (merge_state->joystick_updated) {
    input_messages->joystick = merge_state->joystick_message;
  }

  HandleTetherGroundStationToGsSensorData(
      &system_params->winch, &fault_params->ground_station,
      &tether_up_z1.ground_station, &tether_up->ground_station,
      &control_input->gs_sensors, &faults_state->ground_station,
      &faults[kSubsysGroundStation], &faults[kSubsysDetwist],
      &faults[kSubsysDrum]);

  HandleTetherUpToGsgData(&system_params->gsg, &fault_params->gsg,
                          &tether_up_z1, tether_up, control_input->gsg,
                          faults_state->gsg, faults);

  HandleTetherUpToPerchData(fault_params, &tether_up_z1, tether_up,
                            &control_input->perch, faults_state, faults);

  HandleTetherUpToWeatherData(fault_params, &tether_up_z1, tether_up,
                              &limits_params->min.weather,
                              &limits_params->max.weather,
                              &control_input->weather, faults_state, faults);

  HandleTetherUpToWindData(fault_params, &tether_up_z1, tether_up,
                           &control_input->wind_ws, faults_state, faults);

  HandleFlightComputersImus(
      system_params->wing_imus, &fault_params->imu,
      input_messages->flight_comp_imus, control_input->imus,
      sequence_numbers->flight_comp_imus, faults_state->imus, faults);
  HandleFlightComputersSensors(
      system_params->wing_imus, &fault_params->imu, &system_params->pitot,
      &fault_params->pitot, system_params->sensor_layout.pitot_fc_labels,
      input_messages->flight_comp_sensors, control_input->imus,
      control_input->pitots, sequence_numbers->flight_comp_sensors,
      faults_state->imus, faults_state->pitots, faults);

  HandleTetherPlcToPerchData(&system_params->winch, fault_params,
                             &tether_up_z1.plc, &tether_up->plc,
                             &control_input->perch, faults_state, faults);

  HandleTetherUpToGsCompass(
      &fault_params->gs_compass, &system_params->gs_gps.heading_cal,
      &tether_up_z1, tether_up, &control_input->perch.perch_heading,
      &faults_state->gs_compass, &faults[kSubsysGsCompass]);

  HandleTetherUpToGsGpsData(&fault_params->gs_gps, tether_up,
                            &control_input->gs_gps, state,
                            &faults_state->gs_gps, &faults[kSubsysGsGpsPos]);

  bool tether_release_armed;
  HandleLoadcellMessages(
      system_params->loadcells, &fault_params->loadcell,
      input_messages->loadcell_messages, control_input->loadcells,
      &tether_release_armed, &control_input->tether_released,
      sequence_numbers->loadcell_messages, &faults_state->loadcells,
      &faults[SUBSYS_LOADCELLS], &faults[kSubsysTetherRelease]);

  // The controller receives redundant joystick data via TetherUpMessage and
  // JoystickStatusMessage.
  HandleJoystickData(&system_params->joystick, &fault_params->joystick,
                     &tether_up_z1.joystick, &tether_up->joystick,
                     tether_release_armed, &control_input->joystick,
                     &faults_state->joystick, &faults[kSubsysJoystick]);

  HandleMotorStatuses(system_params->rotors, system_params->rotor_sensors,
                      &fault_params->motor, input_messages->motor_statuses,
                      control_input->rotors, &control_input->stacking_state,
                      sequence_numbers->motor_statuses, faults_state->motors,
                      &faults[SUBSYS_MOTORS]);

  for (int32_t i_gps = 0; i_gps < kNumWingGpsReceivers; ++i_gps) {
    AioNode gps_node = WingGpsReceiverLabelToAioNode(i_gps);
    GpsReceiverType gps_type = WingGpsReceiverLabelToGpsReceiverType(i_gps);
    if (gps_type == kGpsReceiverTypeNovAtel) {
      HandleWingGpsNovAtel(gps_node, &fault_params->wing_gps,
                           &input_messages->wing_gps_novatel[i_gps],
                           &control_input->wing_gps[i_gps],
                           &sequence_numbers->wing_gps[i_gps],
                           &faults_state->wing_gps[i_gps],
                           GetWingGpsSubsysFaults(faults, i_gps));
    } else if (gps_type == kGpsReceiverTypeSeptentrio) {
      HandleWingGpsSeptentrio(gps_node, &fault_params->wing_gps,
                              &input_messages->wing_gps_septentrio[i_gps],
                              &control_input->wing_gps[i_gps],
                              &sequence_numbers->wing_gps[i_gps],
                              &faults_state->wing_gps[i_gps],
                              GetWingGpsSubsysFaults(faults, i_gps));
    } else {
      assert(false);

      // Set ThrownError faults and inject large sigmas to ensure the controller
      // doesn't use the GPS.
      FaultMask *gps_subsys = GetWingGpsSubsysFaults(faults, i_gps);
      for (int32_t i_fault = 0; i_fault < kNumFaultDetectionGpsSignals;
           ++i_fault) {
        SetFault(kFaultTypeThrownError, true, &gps_subsys[i_fault]);
      }
      GpsData *gps_data = &control_input->wing_gps[i_gps];
      memset(gps_data, 0, sizeof(*gps_data));
      Vec3 bad_value = {1e9, 1e9, 1e9};
      gps_data->pos_sigma = bad_value;
      gps_data->vel_sigma = bad_value;
    }
  }

  HandleGroundEstimate(
      &fault_params->ground_estimator, &input_messages->ground_estimate,
      &control_input->ground_estimate, &sequence_numbers->ground_estimate,
      &faults_state->ground_estimate, &faults[kSubsysGroundEstimatorPosition]);

  HandleServoStatuses(input_messages->servo_statuses, system_params->servos,
                      control_input->flaps, sequence_numbers->servo_statuses);

  HandleFlightCommand(control_input);

  AvionicsSaturateSensors(control_input, limits_params, faults, control_input);
}

// Control to avionics.

static void ConvertFlaps(const double flaps[], const ServoParams params[],
                         ControllerCommandMessage *command_message) {
  double servo_angle[kNumServos];
  FlapAnglesToServoAngles(flaps, params, servo_angle);
  for (int32_t i = 0; i < kNumServos; ++i) {
    command_message->servo_angle[i] = (float)servo_angle[i];
  }
}

void ConvertRotors(const double rotors_omega_upper[],
                   const double rotors_omega_lower[],
                   const double rotors_torque[],
                   const RotorParams rotor_params[],
                   const RotorSensorParams rotor_sensor_params[],
                   ControllerCommandMessage *command_message) {
  for (int32_t i = 0; i < kNumMotors; ++i) {
    // Compensate torque command for direction and calibration
    double torque_command = (double)rotor_params[i].dir * rotors_torque[i];
    command_message->motor_torque[i] =
        (float)InvertCal(torque_command, &rotor_sensor_params[i].torque_cal);

    // Generate upper and lower commands in motor frame and then swap
    // if necessary.
    float upper_vel_command =
        (float)InvertCal((double)rotor_params[i].dir * rotors_omega_upper[i],
                         &rotor_sensor_params[i].omega_cal);
    float lower_vel_command =
        (float)InvertCal((double)rotor_params[i].dir * rotors_omega_lower[i],
                         &rotor_sensor_params[i].omega_cal);

    if (upper_vel_command > lower_vel_command) {
      command_message->motor_speed_upper_limit[i] = upper_vel_command;
      command_message->motor_speed_lower_limit[i] = lower_vel_command;
    } else {
      command_message->motor_speed_upper_limit[i] = lower_vel_command;
      command_message->motor_speed_lower_limit[i] = upper_vel_command;
    }
  }
}

// Converts the controllers outputs (flap deflections, motor speeds,
// winch velocity, etc.) to the format expected by the
// avionics.
void ConvertControlToAvionics(const ControlOutput *control_output,
                              const SystemParams *params,
                              ControllerCommandMessage *command_message,
                              ControllerSyncMessage *sync_message) {
  ConvertFlaps(control_output->flaps, params->servos, command_message);
  ConvertRotors(control_output->motor_speed_upper_limit,
                control_output->motor_speed_lower_limit,
                control_output->motor_torque, params->rotors,
                params->rotor_sensors, command_message);
  command_message->winch_velocity = (float)InvertCal(
      control_output->winch_vel_cmd, &params->winch.velocity_cmd_cal);
  command_message->detwist_position = control_output->detwist_cmd;
  command_message->gs_mode_request = control_output->gs_mode_request;
  command_message->gs_unpause_transform = control_output->gs_unpause_transform;
  command_message->gs_azi_target = (float)control_output->gs_azi_cmd.target;
  command_message->gs_azi_dead_zone =
      (float)control_output->gs_azi_cmd.dead_zone;

  if (!control_output->stop_motors && control_output->run_motors) {
    command_message->motor_command = kMotorCommandRun;
  } else {
    command_message->motor_command = kMotorCommandNone;
  }

  command_message->tether_release = control_output->tether_release ? 1U : 0U;
  if (command_message->tether_release == 1U) {
    command_message->tether_release_safety_code = TETHER_RELEASE_SAFETY_CODE;
  } else {
    command_message->tether_release_safety_code = 0U;
  }

  sync_message->sequence = control_output->sync.sequence;
  sync_message->flight_mode = control_output->sync.flight_mode;
}

void ConvertGroundvionicsToGroundEstimator(
    const SystemParams *system_params,
    const GroundSensorLimitsParams *limits_params,
    const FaultDetectionParams *fault_params,
    GroundvionicsInterfaceState *state,
    GroundEstimatorInputMessages *input_messages,
    GroundEstimatorInput *ground_estimator_input, FaultMask faults[]) {
  assert(system_params != NULL);
  assert(limits_params != NULL);
  assert(fault_params != NULL);
  assert(input_messages != NULL);
  assert(ground_estimator_input != NULL);

  memset(ground_estimator_input, 0, sizeof(*ground_estimator_input));

  // Clear faults that are owned by this module.
  for (int32_t i = 0; i < kNumSubsystems; ++i) {
    SetFault(kFaultTypeNoUpdate, false, &faults[i]);
    SetFault(kFaultTypeThrownError, false, &faults[i]);
    SetFault(kFaultTypeOutOfRange, false, &faults[i]);
    SetFault(kFaultTypeImplausible, false, &faults[i]);
  }

  GroundvionicsFaultsState *faults_state = &state->faults_state;
  GroundvionicsSequenceNumbers *sequence_numbers = &state->sequence_numbers;
  HandleFlightComputerImu(
      kAioNodeGpsBaseStation, &system_params->gs_imus[kGsImuA],
      &fault_params->imu, &input_messages->ground_comp_imus[0],
      &ground_estimator_input->imu, &sequence_numbers->ground_comp_imu,
      &faults_state->imu, faults);
  HandleFlightComputerSensorsExceptPitot(
      kAioNodeGpsBaseStation, &system_params->gs_imus[kGsImuA],
      &fault_params->imu, &input_messages->ground_comp_sensors[0],
      &ground_estimator_input->imu, &sequence_numbers->ground_comp_sensor,
      &faults_state->imu, faults);

  // TODO: Replace wing_gps fault params with a ground specific one.
  HandleWingGpsNovAtel(kAioNodeGpsBaseStation, &fault_params->wing_gps,
                       &input_messages->ground_gps[0],
                       &ground_estimator_input->gs_gps,
                       &sequence_numbers->ground_gps, &faults_state->gs_gps,
                       &faults[kSubsysGsGpsPos]);

  HandleNovAtelGpsCompass(kAioNodeGpsBaseStation, &fault_params->gs_compass,
                          &input_messages->ground_compass[0],
                          &ground_estimator_input->gps_compass,
                          &sequence_numbers->ground_compass, faults);

  GroundvionicsSaturateSensors(ground_estimator_input, limits_params, faults,
                               ground_estimator_input);
}
