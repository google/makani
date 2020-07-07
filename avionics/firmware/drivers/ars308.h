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

#ifndef AVIONICS_FIRMWARE_DRIVERS_ARS308_H_
#define AVIONICS_FIRMWARE_DRIVERS_ARS308_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/ars308_types.h"
#include "avionics/firmware/cpu/dcan.h"

#define ARS308_RANGE_MIN 50   // [m].
#define ARS308_RANGE_MAX 200  // [m].
#define ARS308_ELEV_MIN 0     // [0.25 deg].
#define ARS308_ELEV_MAX 128   // [0.25 deg].

typedef enum {
  kArs308ConfigRange  = 1 << 0,
  kArs308ConfigElev   = 1 << 1,
  kArs308ConfigPower  = 1 << 2,
  kArs308ConfigOutput = 1 << 3,
  kArs308ConfigAll    = 0x0F
} Ars308Config;

typedef enum {
  kArs308PowerReductionDisable = 0x00,
  kArs308PowerReductionEnable  = 0x01
} Ars308PowerReduction;

typedef enum {
  kArs308OutputTypeObjects = 0x01,
  kArs308OutputTypeTargets = 0x02
} Ars308OutputType;

typedef struct {
  DcanBus can1;  // Vehicle CAN bus (for control messages).
  DcanBus can2;  // Private CAN bus (for radar data).
} Ars308CanBus;

// Initialize CAN bus to send/receive radar messages.
void Ars308Init(const Ars308CanBus *bus);

// Set ARS308 configuration.
//
// Args:
//   bus: CAN bus configuration.
//   valid: Bitmask of settings to update. See Ars308Config.
//   range: Specify radar range length [meters], valid from [50, 200] m.
//   elev: Specify radar elevation angle [0.25 deg], valid from [0, 32] deg.
//   power_reduction: Specify power mode. See Ars308PowerReduction.
//   output_type: Specify output type. See Ars308OutputType.
//
// Return:
//   True upon success.
bool Ars308SendConfig(const Ars308CanBus *bus, int32_t valid, int32_t range,
                      int32_t elev, Ars308PowerReduction power_reduction,
                      Ars308OutputType output_type);

// Set ARS308 speed [0.02 m/s], valid from [-163.8, 163.8] m/s.
bool Ars308SendSpeed(const Ars308CanBus *bus, int32_t speed);

// Set ARS308 yaw rate [0.01 deg/s], valid from [-327.68, 327.67] deg/s.
bool Ars308SendYawRate(const Ars308CanBus *bus, int32_t yaw_rate);

// Poll ARS308 output data.
bool Ars308PollState(const Ars308CanBus *bus, Ars308State *state);
bool Ars308PollTargetStatus(const Ars308CanBus *bus,
                            Ars308TargetStatus *status);
bool Ars308PollTarget1(const Ars308CanBus *bus, Ars308Target1 *target1);
bool Ars308PollTarget2(const Ars308CanBus *bus, Ars308Target2 *target2);
bool Ars308PollObjectStatus(const Ars308CanBus *bus,
                            Ars308ObjectStatus *status);
bool Ars308PollObject1(const Ars308CanBus *bus, Ars308Object1 *object1);
bool Ars308PollObject2(const Ars308CanBus *bus, Ars308Object2 *object2);
bool Ars308PollVersionId(const Ars308CanBus *bus, Ars308VersionId *version_id);

#endif  // AVIONICS_FIRMWARE_DRIVERS_ARS308_H_
