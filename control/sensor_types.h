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

#ifndef CONTROL_SENSOR_TYPES_H_
#define CONTROL_SENSOR_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/novatel_types.h"
#include "avionics/common/plc_messages.h"
#include "avionics/network/aio_labels.h"
#include "common/c_math/vec3.h"
#include "system/labels.h"

typedef struct {
  double v;      // Airspeed [m/s] (always non-negative).
  double alpha;  // Angle-of-attack [rad].
  double beta;   // Sideslip angle [rad].
} ApparentWindSph;

// For general information about RTK positioning, see:
//     http://www.ngs.noaa.gov/PUBS_LIB/NGSRealTimeUserGuidelines.v2.1.pdf.
typedef enum {
  kGpsSolutionTypeForceSigned = -1,
  // No solution.
  kGpsSolutionTypeNone,
  // Differential solution based on corrections from the ground station.
  kGpsSolutionTypeDifferential,
  // Receiver operating in a 2D mode with height fixed.
  kGpsSolutionTypeFixedHeight,
  // Receiver operating in a fixed position mode.
  kGpsSolutionTypeFixedPosition,
  // RTK solution with floating point ambiguities.
  kGpsSolutionTypeRtkFloat,
  // RTK solution with integer point ambiguities.
  kGpsSolutionTypeRtkInt,
  // RTK solution with floating point ambiguities using dual frequency
  // operation to correct for the ionosphere.
  kGpsSolutionTypeRtkIonoFreeFloat,
  // RTK solution with floating point, narrow-lane ambiguities.
  kGpsSolutionTypeRtkNarrowFloat,
  // RTK solution with integer, narrow-lane ambiguities.
  kGpsSolutionTypeRtkNarrowInt,
  // RTK solution with integer, wide-lane ambiguities.
  kGpsSolutionTypeRtkWideInt,
  // Stand-alone (non-differential) solution.
  kGpsSolutionTypeStandAlone,
  // Unsupported type.  This does not necessarily indicate the need
  // for immediate action by the controller, as there could be a
  // solution type simply not supported by our drivers.
  kGpsSolutionTypeUnsupported,
  kNumGpsSolutionTypes
} GpsSolutionType;

typedef struct {
  Vec3 pos;
  double pos_sigma;
} GsGpsData;

typedef struct {
  bool new_data;
  double heading;        // [rad]
  double heading_sigma;  // [rad]
  double heading_rate;   // [rad/s]
  double pitch;          // [rad]
  double pitch_sigma;    // [rad]
  double pitch_rate;     // [rad/s]
  GpsSolutionType angle_sol_type;
  GpsSolutionType rate_sol_type;
} GpsCompassData;

typedef struct {
  bool new_data;
  int32_t time_of_week_ms;
  Vec3 pos;
  Vec3 vel;
  Vec3 pos_sigma;
  Vec3 vel_sigma;
  GpsSolutionType pos_sol_type;
  GpsSolutionType vel_sol_type;
} GpsData;

typedef struct {
  GroundStationMode mode;
  uint8_t transform_stage;
  double winch_pos;
  double detwist_pos;
  bool proximity;
} GsSensorData;

typedef struct {
  double azi;
  double ele;
} GsgData;

typedef struct {
  Vec3 acc;
  Vec3 gyro;
  Vec3 mag;
} ImuData;

typedef struct {
  double throttle;
  double roll;
  double pitch;
  double yaw;
  JoystickSwitchPositionLabel switch_position;
  bool release;
  bool engage_auto_glide;
} JoystickData;

typedef struct {
  double winch_pos;
  double perch_heading;
  double perch_azi[kNumPlatforms];
  double levelwind_ele[kNumPlatforms];
} PerchData;

typedef struct {
  double alpha_press;
  double beta_press;
  double dyn_press;
} PitotDifferentialData;

typedef struct {
  double stat_press;
  PitotDifferentialData diff;
} PitotData;

// Tether force in spherical coordinates.
typedef struct {
  // Tension [N] (always non-negative).
  double tension;

  // Angle [rad] the tether makes relative to the z-axis (rotated into
  // the bridle plane).  Positive means a positive rotation of the
  // tether about the body x-axis (rotated normal to the bridle
  // plane).
  double roll;

  // Angle [rad] the bridle plane makes relative to the body y-z
  // plane.  Positive means a positive rotation of the tether about
  // the body y-axis.
  double pitch;
} TetherForceSph;

#endif  // CONTROL_SENSOR_TYPES_H_
