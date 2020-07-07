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

#include "control/avionics/avionics_saturation.h"

#include <stdint.h>

#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/control_types.h"
#include "control/fault_detection/fault_detection_util.h"
#include "system/labels.h"

// TODO: Add fault detection for saturated sensors.

// Utility functions for AvionicsSaturateSensors.
#define SATURATE_VEC3(field)                                                \
  do {                                                                      \
    SaturateVec3(&input_raw->field, &limits->min.field, &limits->max.field, \
                 &input->field);                                            \
  } while (0)

#define SATURATE(field)                                                   \
  do {                                                                    \
    input->field =                                                        \
        Saturate(input_raw->field, limits->min.field, limits->max.field); \
  } while (0)

#define CHECK_OUT_OF_RANGE(field)          \
  (limits->min.field > input_raw->field || \
   limits->max.field < input_raw->field) /* NOLINT */

#define CHECK_OUT_OF_RANGE_VEC3(field)                      \
  (limits->min.field.x > input_raw->field.x || /* NOLINT */ \
   limits->min.field.y > input_raw->field.y || /* NOLINT */ \
   limits->min.field.z > input_raw->field.z || /* NOLINT */ \
   limits->max.field.x < input_raw->field.x || /* NOLINT */ \
   limits->max.field.y < input_raw->field.y || /* NOLINT */ \
   limits->max.field.z < input_raw->field.z)   /* NOLINT */

void AvionicsSaturateSensors(const ControlInput *input_raw,
                             const SensorLimitsParams *limits,
                             FaultMask faults[], ControlInput *input) {
  assert(input_raw != NULL);
  assert(limits != NULL);
  assert(input != NULL);

  *input = *input_raw;

  // The sync field is skipped.
  for (int32_t i = 0; i < kNumDrums; ++i) {
    SATURATE(gsg[i].ele);
    SATURATE(gsg[i].azi);
  }

  assert(!CHECK_OUT_OF_RANGE(gs_sensors.mode));
  assert(!CHECK_OUT_OF_RANGE(gs_sensors.transform_stage));
  assert(!CHECK_OUT_OF_RANGE(gs_sensors.winch_pos));
  assert(!CHECK_OUT_OF_RANGE(gs_sensors.detwist_pos));
  assert(!CHECK_OUT_OF_RANGE(gs_sensors.proximity));

  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    FaultMask *gps_faults = GetWingGpsSubsysFaults(faults, i);

    SetFault(kFaultTypeOutOfRange,
             CHECK_OUT_OF_RANGE(wing_gps[i].pos_sol_type) ||
                 CHECK_OUT_OF_RANGE(wing_gps[i].time_of_week_ms) ||
                 CHECK_OUT_OF_RANGE_VEC3(wing_gps[i].pos) ||
                 CHECK_OUT_OF_RANGE_VEC3(wing_gps[i].pos_sigma),
             &gps_faults[kFaultDetectionGpsSignalPos]);

    SetFault(kFaultTypeOutOfRange,
             CHECK_OUT_OF_RANGE(wing_gps[i].vel_sol_type) ||
                 CHECK_OUT_OF_RANGE(wing_gps[i].time_of_week_ms) ||
                 CHECK_OUT_OF_RANGE_VEC3(wing_gps[i].vel) ||
                 CHECK_OUT_OF_RANGE_VEC3(wing_gps[i].vel_sigma),
             &gps_faults[kFaultDetectionGpsSignalVel]);
  }

  SATURATE(joystick.throttle);
  SATURATE(joystick.roll);
  SATURATE(joystick.pitch);
  SATURATE(joystick.yaw);

  for (int32_t i = 0; i < kNumLoadcellSensors; ++i) SATURATE(loadcells[i]);

  for (int32_t i = 0; i < kNumWingImus; ++i) {
    SATURATE_VEC3(imus[i].acc);
    SATURATE_VEC3(imus[i].gyro);
    SATURATE_VEC3(imus[i].mag);
  }

  for (int32_t i = 0; i < kNumPitotSensors; ++i) {
    SATURATE(pitots[i].stat_press);
    SATURATE(pitots[i].diff.alpha_press);
    SATURATE(pitots[i].diff.beta_press);
    SATURATE(pitots[i].diff.dyn_press);
  }

  for (int32_t i = 0; i < kNumFlaps; ++i) SATURATE(flaps[i]);
  for (int32_t i = 0; i < kNumMotors; ++i) SATURATE(rotors[i]);

  SATURATE_VEC3(wind_ws);

  // The gs_gps field is skipped.

  SATURATE(perch.winch_pos);
  SATURATE(perch.perch_heading);
  for (int32_t i = 0; i < kNumPlatforms; ++i) {
    SATURATE(perch.perch_azi[i]);
    SATURATE(perch.levelwind_ele[i]);
  }
}

void GroundvionicsSaturateSensors(const GroundEstimatorInput *input_raw,
                                  const GroundSensorLimitsParams *limits,
                                  FaultMask faults[],
                                  GroundEstimatorInput *input) {
  assert(input_raw != NULL);
  assert(limits != NULL);
  assert(input != NULL);

  *input = *input_raw;

  SATURATE_VEC3(imu.acc);
  SATURATE_VEC3(imu.gyro);
  SATURATE_VEC3(imu.mag);

  SetFault(kFaultTypeOutOfRange,
           CHECK_OUT_OF_RANGE(gs_gps.pos_sol_type) ||
               CHECK_OUT_OF_RANGE(gs_gps.time_of_week_ms) ||
               CHECK_OUT_OF_RANGE_VEC3(gs_gps.pos) ||
               CHECK_OUT_OF_RANGE_VEC3(gs_gps.pos_sigma),
           &faults[kSubsysGsGpsPos]);

  SetFault(kFaultTypeOutOfRange,
           CHECK_OUT_OF_RANGE(gs_gps.vel_sol_type) ||
               CHECK_OUT_OF_RANGE(gs_gps.time_of_week_ms) ||
               CHECK_OUT_OF_RANGE_VEC3(gs_gps.vel) ||
               CHECK_OUT_OF_RANGE_VEC3(gs_gps.vel_sigma),
           &faults[kSubsysGsGpsVel]);

  SetFault(kFaultTypeOutOfRange,
           CHECK_OUT_OF_RANGE(gps_compass.angle_sol_type) ||
               CHECK_OUT_OF_RANGE(gps_compass.heading) ||
               CHECK_OUT_OF_RANGE(gps_compass.heading_sigma) ||
               CHECK_OUT_OF_RANGE(gps_compass.pitch) ||
               CHECK_OUT_OF_RANGE(gps_compass.pitch_sigma),
           &faults[kSubsysGsCompassAngles]);

  SetFault(kFaultTypeOutOfRange,
           CHECK_OUT_OF_RANGE(gps_compass.rate_sol_type) ||
               CHECK_OUT_OF_RANGE(gps_compass.heading_rate) ||
               CHECK_OUT_OF_RANGE(gps_compass.pitch_rate),
           &faults[kSubsysGsCompassAngularRates]);
}

#undef SATURATE
#undef SATURATE_VEC3
#undef CHECK_OUT_OF_RANGE
#undef CHECK_OUT_OF_RANGE_VEC3
