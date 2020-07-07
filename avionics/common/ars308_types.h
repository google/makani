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

#ifndef AVIONICS_COMMON_ARS308_TYPES_H_
#define AVIONICS_COMMON_ARS308_TYPES_H_

#include <stdint.h>

#define NUM_RADAR_TARGETS      96
#define NUM_RADAR_NEAR_TARGETS 32
#define NUM_RADAR_FAR_TARGETS  (NUM_RADAR_TARGETS - NUM_RADAR_NEAR_TARGETS)
#define NUM_RADAR_OBJECTS      40

typedef enum {
  // See Table 3 RadarState message (0x201). The second byte is empty and
  // ignored here.
  kArs308StatusCurrentRadarPower   = 1 << 1,
  kArs308StatusSensTempErr         = 1 << 3,
  kArs308StatusSensDef             = 1 << 5,
  kArs308StatusSupVoltLow          = 1 << 6,
  kArs308StatusRadarPowerReduction = 1 << 7,
  kArs308StatusSpeedMissing        = 1 << 8,
  kArs308StatusYawRateMissing      = 1 << 9,
  kArs308StatusNvmReadSuccess      = 1 << 10,
  kArs308StatusNvmWriteSuccess     = 1 << 12
} Ars308Status;

typedef enum {
  kArs308TargetTypeNoTarget    = 0,
  kArs308TargetTypeOncoming    = 1,
  kArs308TargetTypeStationary  = 2,
  kArs308TargetTypeInvalidData = 3
} Ars308TargetType;

typedef enum {
  kArs308TargetAngleExpanded = 0,
  kArs308TargetAnglePoint    = 1,
  kArs308TargetAngleDigital  = 2,
  kArs308TargetAngleInvalid  = 3
} Ars308TargetAngle;

typedef struct {
  uint16_t status;    // See Ars308Status.
  uint8_t elev_cal;   // [0.25 deg]
  uint8_t range_cal;  // [m]
  uint8_t sw_major_version;
  uint8_t sw_minor_version;
  uint8_t sw_build_version;
} Ars308State;

typedef struct {
  uint8_t num_targets_near;
  uint8_t num_targets_far;
  uint8_t interface_version;  // [0.1 #]
} Ars308TargetStatus;

typedef struct {
  uint8_t target_id;     // [0, NUM_RADAR_TARGETS)
  uint8_t range_std;     // [0.1 m]
  uint8_t angle_std;     // [0.1 deg]
  uint16_t rel_vel_std;  // [0.03 m/s]
  int16_t rel_vel;       // [0.03 m/s]
  int16_t range;         // [0.1 m]
} Ars308Target1;

typedef struct {
  uint8_t target_id;         // [0, NUM_RADAR_TARGETS)
  uint8_t prob_false_alarm;  // [1 %]
  int16_t length;            // [0.1 m]
  int16_t width;             // [0.1 m]
  uint8_t type;              // See Ars308TargetType.
  uint8_t angle_status;
  int16_t angle;  // [0.1 deg]
  int16_t rcs;    // [0.1 dBm^2]
} Ars308Target2;

typedef struct {
  uint8_t num_objects;
  uint16_t meas_counter;
  uint8_t interface_version;
} Ars308ObjectStatus;

typedef struct {
  uint8_t object_id;     // [#]
  uint8_t roll_count;    // [#]
  uint16_t lon_disp;     // [0.1 m]
  uint16_t rel_lon_vel;  // [0.0625 m/s, -128 m/s offset]
  uint16_t rel_lon_acc;  // [0.0625 m/s, -16 m/s offset]
  uint16_t lat_disp;     // [0.1 m, -52 m offset]
  uint8_t dyn_property;  // [#]
  uint8_t prob_exist;    // [#]
  uint8_t meas_stat;     // [#]
  uint8_t width;         // [#]
  uint8_t length;        // [#]
} Ars308Object1;

typedef struct {
  uint8_t rcs;            // [0.5 dBm^2, -64 dBm^2 offset]
  uint8_t rel_lat_vel;    // [0.25 m/s, -32 m/s offset]
  uint8_t prob_obstacle;  // [1 %]
} Ars308Object2;

typedef struct {
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
} Ars308VersionId;

#endif  // AVIONICS_COMMON_ARS308_TYPES_H_
