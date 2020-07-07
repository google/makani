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

#include "avionics/firmware/drivers/ars308.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/ars308_types.h"
#include "avionics/firmware/cpu/dcan.h"
#include "common/macros.h"

// Mailboxes are ordered by priority with zero having the highest priority. TI
// recommends placing the transmit mailboxes first.
typedef enum {
  // Transmit mailboxes [0, 32).
  kRadarMailboxCan1Config = 0,
  kRadarMailboxCan1Speed,
  kRadarMailboxCan1YawRate,
  // Receive mailboxes [32, 64).
  kRadarMailboxCan1State = 32,
  kRadarMailboxCan2TargetStatus,
  kRadarMailboxCan2Target1,
  kRadarMailboxCan2Target2,
  kRadarMailboxCan2ObjectStatus,
  kRadarMailboxCan2Object1,
  kRadarMailboxCan2Object2,
  kRadarMailboxCan2VersionId
} RadarMailbox;

typedef enum {
  kArs308DirectionStandstill = 0,
  kArs308DirectionForward    = 1,
  kArs308DirectionReverse    = 2
} kArs308Direction;

static uint32_t SatStddev(uint32_t code, int32_t offset) {
  int32_t sat = (int32_t)code - offset;
  return (sat < 0) ? 0 : sat;
}

void Ars308Init(const Ars308CanBus *bus) {
  assert(bus != NULL);

  // Transmit mailboxes on CAN1.
  DcanSetTransmitMailbox(bus->can1, kRadarMailboxCan1Config, kDcanIdStandard,
                         DCAN_STANDARD_MASK, 0x200, 8);
  DcanSetTransmitMailbox(bus->can1, kRadarMailboxCan1Speed, kDcanIdStandard,
                         DCAN_STANDARD_MASK, 0x300, 8);
  DcanSetTransmitMailbox(bus->can1, kRadarMailboxCan1YawRate, kDcanIdStandard,
                         DCAN_STANDARD_MASK, 0x301, 8);

  // Receive mailboxes on CAN1.
  DcanSetReceiveMailbox(bus->can1, kRadarMailboxCan1State, kDcanIdStandard,
                        DCAN_STANDARD_MASK, 0x201, 8);

  // Receive mailboxes on CAN2.
  DcanSetReceiveMailbox(bus->can2, kRadarMailboxCan2TargetStatus,
                        kDcanIdStandard, DCAN_STANDARD_MASK, 0x600, 8);
  DcanSetReceiveMailbox(bus->can2, kRadarMailboxCan2Target1, kDcanIdStandard,
                        DCAN_STANDARD_MASK, 0x701, 8);
  DcanSetReceiveMailbox(bus->can2, kRadarMailboxCan2Target2, kDcanIdStandard,
                        DCAN_STANDARD_MASK, 0x702, 8);
  DcanSetReceiveMailbox(bus->can2, kRadarMailboxCan2ObjectStatus,
                        kDcanIdStandard, DCAN_STANDARD_MASK, 0x60A, 8);
  DcanSetReceiveMailbox(bus->can2, kRadarMailboxCan2Object1, kDcanIdStandard,
                        DCAN_STANDARD_MASK, 0x60B, 8);
  DcanSetReceiveMailbox(bus->can2, kRadarMailboxCan2Object2, kDcanIdStandard,
                        DCAN_STANDARD_MASK, 0x60C, 8);
  DcanSetReceiveMailbox(bus->can2, kRadarMailboxCan2VersionId, kDcanIdStandard,
                        DCAN_STANDARD_MASK, 0x700, 8);
}

bool Ars308SendConfig(const Ars308CanBus *bus, int32_t valid, int32_t range,
                      int32_t elev, Ars308PowerReduction power_reduction,
                      Ars308OutputType output_type) {
  assert(bus != NULL);
  assert((valid & kArs308ConfigAll) == valid);
  assert(50 <= range && range <= 200);
  assert(0 <= elev && elev <= 128);
  assert(power_reduction == kArs308PowerReductionDisable
         || power_reduction == kArs308PowerReductionEnable);
  assert(output_type == kArs308OutputTypeObjects
         || output_type == kArs308OutputTypeTargets);

  uint8_t data[8] = {(uint8_t)valid, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  if (valid & kArs308ConfigRange) {
    if (range < ARS308_RANGE_MIN) {
      data[1] = ARS308_RANGE_MIN;
    } else if (range > ARS308_RANGE_MAX) {
      data[1] = ARS308_RANGE_MAX;
    } else {
      data[1] = (uint8_t)range;
    }
  }
  if (valid & kArs308ConfigElev) {
    if (elev < ARS308_ELEV_MIN) {
      data[2] = ARS308_ELEV_MIN;
    } else if (elev > ARS308_ELEV_MAX) {
      data[2] = ARS308_ELEV_MAX;
    } else {
      data[2] = (uint8_t)elev;
    }
  }
  if (valid & kArs308ConfigPower) {
    data[3] |= power_reduction & 0x01;
  }
  if (valid & kArs308ConfigOutput) {
    data[3] |= (output_type << 1) & 0x06;
  }
  return DcanTransmit(bus->can1, kRadarMailboxCan1Config, ARRAYSIZE(data),
                      data);
}

bool Ars308SendSpeed(const Ars308CanBus *bus, int32_t speed) {
  assert(bus != NULL);
  uint8_t data[8] = {0U};
  int32_t out;
  if (speed < 0) {
    out = -speed;
    data[2] = kArs308DirectionReverse;
  } else if (speed > 0) {
    out = speed;
    data[2] = kArs308DirectionForward;
  } else {
    out = 0;
    data[2] = kArs308DirectionStandstill;
  }
  if (out > 0x1FFF) {
    out = 0x1FFF;
  }
  data[0] = (out >> 8) & 0x1F;
  data[1] = out & 0xFF;
  return DcanTransmit(bus->can1, kRadarMailboxCan1Speed, ARRAYSIZE(data), data);
}

bool Ars308SendYawRate(const Ars308CanBus *bus, int32_t yaw_rate) {
  assert(bus != NULL);
  uint8_t data[8] = {0U};
  int32_t out;
  if (yaw_rate > 0x7FFF) {
    out = 0xFFFF;
  } else if (yaw_rate < -0x8000) {
    out = 0;
  } else {
    out = 0x8000 + yaw_rate;
  }
  data[0] = out >> 8;
  data[1] = out & 0xFF;
  return DcanTransmit(bus->can1, kRadarMailboxCan1YawRate, ARRAYSIZE(data),
                      data);
}

bool Ars308PollState(const Ars308CanBus *bus, Ars308State *state) {
  assert(bus != NULL);
  assert(state != NULL);
  uint8_t data[8];
  if (DcanGetMailbox(bus->can1, kRadarMailboxCan1State, ARRAYSIZE(data), data,
                     NULL)) {
    // Ignore data[1].
    state->status = data[2] << 8 | data[0];
    state->elev_cal = data[3];
    state->range_cal = data[4];
    state->sw_major_version = data[5];
    state->sw_minor_version = data[6];
    state->sw_build_version = data[7];
    return true;
  }
  return false;
}

bool Ars308PollTargetStatus(const Ars308CanBus *bus,
                            Ars308TargetStatus *status) {
  assert(bus != NULL);
  assert(status != NULL);
  uint8_t data[8];
  if (DcanGetMailbox(bus->can2, kRadarMailboxCan2TargetStatus, ARRAYSIZE(data),
                     data, NULL)) {
    status->num_targets_near = data[0];
    status->num_targets_far = data[1];
    status->interface_version = data[2];
    return true;
  }
  return false;
}

bool Ars308PollTarget1(const Ars308CanBus *bus, Ars308Target1 *target1) {
  assert(bus != NULL);
  assert(target1 != NULL);
  uint8_t data[8];
  uint32_t id;
  if (DcanGetMailbox(bus->can2, kRadarMailboxCan2Target1, ARRAYSIZE(data),
                     data, &id)) {
    target1->target_id = data[0] - 1;  // Map id to [0, NUM_RADAR_TARGETS).
    target1->range_std = SatStddev(data[1], 0x64);  // Remove -10 m offset.
    target1->angle_std = data[2] >> 1;
    target1->rel_vel_std = SatStddev((data[2] & 0x01) << 8 | data[3], 0xA7);
    target1->rel_vel = data[4] << 4 | data[5] >> 4;
    target1->rel_vel -= 0xCBE;  // Remove erroneous offset (observed).
    target1->range = (data[5] & 0x07) << 8 | data[6];
    return true;
  }
  return false;
}

bool Ars308PollTarget2(const Ars308CanBus *bus, Ars308Target2 *target2) {
  assert(bus != NULL);
  assert(target2 != NULL);
  uint8_t data[8];
  if (DcanGetMailbox(bus->can2, kRadarMailboxCan2Target2, ARRAYSIZE(data),
                     data, NULL)) {
    target2->target_id = data[0] - 1;  // Map id to [0, NUM_RADAR_TARGETS).
    target2->prob_false_alarm = data[1] >> 1;
    target2->length = (data[1] & 0x01) << 8 | data[2];
    target2->width = (data[3] & 0x01) << 8 | data[4];
    target2->type = data[5] >> 6;
    target2->angle_status = (data[5] & 0x30) >> 4;
    target2->angle = (data[5] & 0x0F) << 6 | data[6] >> 2;
    target2->angle -= 300;  // Remove -30 deg offset.
    target2->rcs = (data[6] & 0x03) << 8 | data[7];
    target2->rcs -= 500;  // Remove -50 dBm^2 offset.
    return true;
  }
  return false;
}

bool Ars308PollObjectStatus(const Ars308CanBus *bus,
                            Ars308ObjectStatus *status) {
  assert(bus != NULL);
  assert(status != NULL);
  uint8_t data[8];
  if (DcanGetMailbox(bus->can2, kRadarMailboxCan2ObjectStatus, ARRAYSIZE(data),
                     data, NULL)) {
    status->num_objects = data[0];
    status->meas_counter = data[1] << 8 | data[2];
    status->interface_version = data[4] & 0x0F;
    return true;
  }
  return false;
}

bool Ars308PollObject1(const Ars308CanBus *bus, Ars308Object1 *object1) {
  assert(bus != NULL);
  assert(object1 != NULL);
  uint8_t data[8];
  if (DcanGetMailbox(bus->can2, kRadarMailboxCan2Object1, ARRAYSIZE(data),
                     data, NULL)) {
    object1->object_id = (data[0] >> 2) - 1;  // Map to [0, NUM_RADAR_TARGETS).
    object1->roll_count = data[0] & 0x03;
    object1->lon_disp = data[1] << 3 | data[2] >> 5;
    object1->rel_lon_vel = (data[2] & 0x1F) << 7 | data[3] >> 1;
    object1->rel_lon_acc = (data[3] & 0x01) << 8 | data[4];
    object1->lat_disp = data[5] << 2 | data[6] >> 6;
    object1->dyn_property = (data[6] & 0x38) >> 3;
    object1->prob_exist = data[6] & 0x07;
    object1->meas_stat = data[7] >> 6;
    object1->width = (data[7] & 0x38) >> 3;
    object1->length = data[7] & 0x07;
    return true;
  }
  return false;
}

bool Ars308PollObject2(const Ars308CanBus *bus, Ars308Object2 *object2) {
  assert(bus != NULL);
  assert(object2 != NULL);
  uint8_t data[8];
  if (DcanGetMailbox(bus->can2, kRadarMailboxCan2Object2, ARRAYSIZE(data),
                     data, NULL)) {
    object2->rcs = data[0];
    object2->rel_lat_vel = data[1];
    object2->prob_obstacle = data[2] & 0x7F;
    return true;
  }
  return false;
}

bool Ars308PollVersionId(const Ars308CanBus *bus, Ars308VersionId *version_id) {
  assert(bus != NULL);
  assert(version_id != NULL);
  uint8_t data[8];
  if (DcanGetMailbox(bus->can2, kRadarMailboxCan2VersionId, ARRAYSIZE(data),
                     data, NULL)) {
    version_id->major = data[0];
    version_id->minor = data[1];
    version_id->patch = data[2];
    return true;
  }
  return false;
}
