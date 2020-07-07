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

#include "avionics/common/pack_tether_message.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/aio_version.h"
#include "avionics/common/bits.h"
#include "avionics/common/crc.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/common/tether_op.h"
#include "common/macros.h"
#include "system/labels.h"

// See "Minimum kite/ground bandwidth" spreadsheet.

// We query the CVT in tether_cvt.c for each message. Upon update, we set
// the corresponding no_update_count field to zero. If not, we increment
// no_update_count until the maximum value. We then transmit the unpacked
// message over the IP network. Receiving parties can then determine the
// number of packets since the last update, or equivalently, the age of
// the data.
//
// To save bandwidth, the packed messages do not encode the no_update_count,
// but instead, encode an updated flag. The updated flag indicates if the
// corresponding field has been updated since the last radio transmission.
// Receiving parties then need to check the updated flag and increment the
// no_update_count locally. For each message with updated true, the receiver
// should zero the no_update_count. If not, the receiver should increment
// no_update_count until the maximum value.

#define CRC_BITS 16
#define AIO_NODE_BITS 7
#define TETHER_OP_TYPE_BITS 4
#define BATT_STATE_COMMAND_BITS 4
#define MVLV_STATE_COMMAND_BITS 4
#define ACTUATOR_STATE_BITS 3
#define ACTUATOR_STATE_COMMAND_BITS 4

static void HandleFill(BitOp op, int32_t expected_offset, int32_t *offset,
                       uint8_t *out) {
  assert(expected_offset >= *offset);
  if (op == kBitOpWrite) {
    WriteBitsUint32(0x0, expected_offset - *offset, offset, out);
  } else {
    *offset = expected_offset;
  }
}

static void HandleUpdated(BitOp op, int32_t decimation, int32_t *offset,
                          int32_t *no_update_count, uint8_t *out) {
  // Argument decimation indicates the number of frame_index updates per
  // field update. Since no_update_count increments at the frame_index rate,
  // we can compare no_update_count against decimation to determine if the
  // field was updated since the last transmission.
  uint8_t updated = (*no_update_count <= decimation);
  BitOpUint8(op, 1, offset, &updated, out);
  if (op == kBitOpRead && updated) {
    *no_update_count = 0;
  }
}

static void HandleBitFlag(BitOp op, int32_t mask, int32_t *offset,
                          uint8_t *flags, uint8_t *out) {
  if (op == kBitOpWrite) {
    WriteBitsUint32((*flags & mask) == mask, 1, offset, out);
  } else if (ReadBitsUint32(out, 1, offset)) {
    *flags |= (uint8_t)mask;
  } else {
    *flags &= (uint8_t)~mask;
  }
}

static void HandleSafetyCode(BitOp op, uint32_t expected_code, int32_t *offset,
                             uint32_t *code, uint8_t *out) {
  assert(expected_code != 0U);

  if (op == kBitOpWrite) {
    WriteBitsUint32(expected_code == *code, 1, offset, out);
  } else if (ReadBitsUint32(out, 1, offset)) {
    *code = expected_code;
  } else {
    *code = 0U;
  }
}

static bool HandleCrc16(BitOp op, int32_t *offset, uint8_t *out) {
  assert(*offset % 8 == 0);
  uint16_t computed = Crc16Ccitt(GetAioVersion(), *offset / 8, out);
  uint16_t transmitted = computed;
  BitOpUint16(op, 16, offset, &transmitted, out);
  return computed == transmitted;
}

static void HandleTetherGpsTime(BitOp op, int32_t decimation, int32_t *offset,
                                TetherGpsTime *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  BitOpUint32(op, 30, offset, (uint32_t *)&in->time_of_week, out);
}

static void HandleSequence(BitOp op, int32_t *offset, uint16_t *sequence,
                           uint8_t *out) {
  BitOpUint16(op, TETHER_SEQUENCE_BITS, offset, sequence, out);
}

static void HandleAngle(BitOp op, float min, float max, int32_t count,
                        int32_t *offset, float *in, uint8_t *out) {
  if (op == kBitOpWrite) {
    float in_wrapped = WrapAngle(*in);
    BitOpRangeFloat(op, min, max, count, offset, &in_wrapped, out);
  } else {
    BitOpRangeFloat(op, min, max, count, offset, in, out);
  }
}

static void HandleMultiturnAngle(BitOp op, float max, int32_t count,
                                 int32_t *offset, float *in, uint8_t *out) {
  if (op == kBitOpWrite) {
    float in_wrapped = fmodf(*in, max);
    if (in_wrapped < 0.0f) {
      in_wrapped += max;
    }
    BitOpRangeFloat(op, 0.0f, max, count, offset, &in_wrapped, out);
  } else {
    BitOpRangeFloat(op, 0.0f, max, count, offset, in, out);
  }
}

static void HandleDetwistAngle(BitOp op, int32_t *offset, float *in,
                               uint8_t *out) {
  const float max = TETHER_DETWIST_REVS * 2.0f * PI_F;
  HandleMultiturnAngle(op, max, TETHER_DETWIST_BITS, offset, in, out);
}

static void HandleDrumAngle(BitOp op, int32_t *offset, float *in,
                            uint8_t *out) {
  const float min = -TETHER_DRUM_REVS * 2.0f * PI_F;
  BitOpRangeFloat(op, min, 0.0f, TETHER_DRUM_BITS, offset, in, out);
}

static void HandleTemperature(BitOp op, int32_t *offset, int16_t *in,
                              uint8_t *out) {
  BitOpScaledOffsetInt16(op, 1, -40, 8, offset, in, out);
}

static void HandleJoystickSwitchPosition(BitOp op, int32_t count,
                                         int32_t *offset,
                                         uint8_t *switch_position,
                                         uint8_t *out) {
  // Pack up/down into 1 bit, up/middle/down into 2 bits.
  if (op == kBitOpWrite) {
    if (*switch_position == kJoystickSwitchPositionUp) {
      WriteBitsUint32(1U, count, offset, out);
    } else if (*switch_position == kJoystickSwitchPositionMiddle) {
      WriteBitsUint32(2U, count, offset, out);
    } else {
      WriteBitsUint32(0U, count, offset, out);  // Down.
    }
  } else {
    uint32_t state = ReadBitsUint32(out, count, offset);
    if (state == 1U) {
      *switch_position = kJoystickSwitchPositionUp;
    } else if (state == 2U) {
      *switch_position = kJoystickSwitchPositionMiddle;
    } else {
      *switch_position = kJoystickSwitchPositionDown;
    }
  }
}

static void HandleTetherJoystick(BitOp op, int32_t decimation,
                                 int32_t *offset, TetherJoystick *in,
                                 uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherJoystickFlagFault, offset, &in->flags, out);
  HandleJoystickSwitchPosition(op, 1, offset, &in->momentary_switch, out);
  HandleJoystickSwitchPosition(op, 2, offset, &in->tri_switch, out);
  BitOpRangeFloat(op, -1.0f, 1.0f, 8, offset, &in->roll, out);
  BitOpRangeFloat(op, -1.0f, 1.0f, 8, offset, &in->pitch, out);
  BitOpRangeFloat(op, -1.0f, 1.0f, 8, offset, &in->yaw, out);
  BitOpRangeFloat(op, 0.0f, 1.0f, 8, offset, &in->throttle, out);
  HandleSafetyCode(op, TETHER_RELEASE_INTERLOCK_SAFETY_CODE, offset,
                   &in->tether_release_interlock_code, out);
  HandleSafetyCode(op, SCUTTLE_SAFETY_CODE, offset, &in->scuttle_code, out);
}

static void HandleTetherPlatform(BitOp op, int32_t decimation, int32_t *offset,
                                 TetherPlatform *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherPlatformFlagPerchAzimuthFault, offset, &in->flags,
                out);
  HandleBitFlag(op, kTetherPlatformFlagLevelwindElevationFault, offset,
                &in->flags, out);
  HandleAngle(op, -PI_F, PI_F, 16, offset, &in->perch_azi, out);
  HandleAngle(op, -PI_F, PI_F, 16, offset, &in->levelwind_ele, out);
}

static void HandleTetherDrum(BitOp op, int32_t decimation, int32_t *offset,
                             TetherDrum *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherDrumFlagGsgAxis1Fault, offset, &in->flags, out);
  HandleBitFlag(op, kTetherDrumFlagGsgAxis2Fault, offset, &in->flags, out);
  HandleAngle(op, -PI_F, PI_F, 16, offset, &in->gsg_axis1, out);
  HandleAngle(op, -PI_F / 2.0f, PI_F / 2.0f, 16, offset, &in->gsg_axis2, out);
}

static void HandleGpsRtcm(BitOp op, int32_t *offset, int32_t length,
                          uint8_t *rtcm, uint8_t *data) {
  for (int32_t i = 0; i < length; ++i) {
    BitOpUint8(op, 8, offset, &rtcm[i], data);
  }
}

static void HandleTetherGroundStationStatus(
    BitOp op, int32_t decimation, int32_t *offset, TetherGroundStation *in,
    uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherGroundStationFlagError, offset, &in->flags, out);
  HandleBitFlag(
      op, kTetherGroundStationFlagDetwistError, offset, &in->flags, out);
  HandleBitFlag(op, kTetherGroundStationFlagDrumError, offset, &in->flags, out);
  BitOpUint8(op, 2, offset, &in->mode, out);
  BitOpUint8(op, 3, offset, &in->transform_stage, out);
  BitOpRangeFloat(op, -250.0f, 0.0f, 16, offset, &in->drum_angle, out);
  HandleAngle(op, -PI_F, PI_F, 15, offset, &in->detwist_angle, out);
  BitOpUint8(op, 1, offset, &in->proximity, out);
  BitOpUint8(op, 1, offset, &in->tether_engaged, out);
}

static void HandleTetherPlc(BitOp op, int32_t decimation, int32_t *offset,
                            TetherPlc *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherPlcFlagPlcWarning, offset, &in->flags, out);
  HandleBitFlag(op, kTetherPlcFlagPlcError, offset, &in->flags, out);
  HandleBitFlag(op, kTetherPlcFlagDetwistFault, offset, &in->flags, out);
  HandleBitFlag(op, kTetherPlcFlagDrumFault, offset, &in->flags, out);
  HandleBitFlag(op, kTetherPlcFlagProximityFault, offset, &in->flags, out);
  BitOpUint8(op, 4, offset, &in->proximity, out);
  HandleDetwistAngle(op, offset, &in->detwist_angle, out);
  HandleDrumAngle(op, offset, &in->drum_angle, out);
}

static void HandleTetherGpsStatus(BitOp op, int32_t decimation, int32_t *offset,
                                  TetherGpsStatus *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  BitOpUint8(op, 3, offset, &in->status, out);
  BitOpSaturateUint8(op, 5, offset, &in->satellites, out);
  BitOpUnsignedFloat(op, 0.1f, 10, offset, &in->pos_sigma, out);
  BitOpScaledOffsetInt8(op, 1, 0, 6, offset, &in->avg_cn0, out);
}

static void HandleTetherGpsStatuses(BitOp op, int32_t num_gpses,
                                    int32_t decimation, int32_t index,
                                    int32_t *offset, TetherGpsStatus *gpses,
                                    uint8_t *out) {
  HandleTetherGpsStatus(op, num_gpses * decimation, offset,
                        &gpses[index % num_gpses], out);
}

static void HandleTetherGsGpsCompass(BitOp op, int32_t decimation,
                                     int32_t *offset, TetherGsGpsCompass *in,
                                     uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherGsGpsCompassFlagFault, offset, &in->flags, out);
  HandleAngle(op, -PI_F, PI_F, 16, offset, &in->heading, out);
  BitOpUnsignedFloat(op, 0.1f, 11, offset, &in->heading_sigma, out);
  BitOpScaledFloat(op, 0.1f, 16, offset, &in->heading_rate, out);
}

static void HandleTetherGsGpsPosition(BitOp op, int32_t decimation,
                                      int32_t *offset, TetherGsGpsPosition *in,
                                      uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherGsGpsPositionFlagFault, offset, &in->flags, out);
  BitOpScaledDouble(op, 0.01, 31, offset, &in->ecef[0], out);
  BitOpScaledDouble(op, 0.01, 31, offset, &in->ecef[1], out);
  BitOpScaledDouble(op, 0.01, 31, offset, &in->ecef[2], out);
}

static void HandleTetherWind(BitOp op, int32_t decimation, int32_t *offset,
                             TetherWind *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  BitOpUint8(op, 2, offset, &in->status, out);
  BitOpScaledFloat(op, 0.01f, 14, offset, &in->velocity[0], out);
  BitOpScaledFloat(op, 0.01f, 14, offset, &in->velocity[1], out);
  BitOpScaledFloat(op, 0.01f, 14, offset, &in->velocity[2], out);
}

static void HandleTetherWeather(BitOp op, int32_t decimation, int32_t *offset,
                                TetherWeather *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  HandleBitFlag(op, kTetherWeatherFlagFault, offset, &in->flags, out);
  BitOpRangeFloat(op, 60000.0f, 110000.0f, 20, offset, &in->pressure_pa, out);
  BitOpRangeFloat(op, -40.0f, 125.0f, 11, offset, &in->temperature, out);
  BitOpRangeFloat(op, 0.0f, 100.0f, 6, offset, &in->humidity, out);
}

static void HandleAioNode(BitOp op, int32_t *offset, AioNode *in,
                          uint8_t *out) {
  const int32_t bits = AIO_NODE_BITS;
  if (op == kBitOpWrite) {
    assert((*in & ((1 << bits) - 1)) == *in);
    WriteBitsUint32((uint32_t)*in, bits, offset, out);
  } else {
    *in = (AioNode)ReadBitsUint32(out, bits, offset);
  }
}

static void HandleTetherOpType(BitOp op, int32_t *offset, TetherOpType *in,
                               uint8_t *out) {
  const int32_t bits = TETHER_OP_TYPE_BITS;
  if (op == kBitOpWrite) {
    assert((*in & ((1 << bits) - 1)) == *in);
    WriteBitsUint32((uint32_t)*in, bits, offset, out);
  } else {
    *in = (TetherOpType)ReadBitsUint32(out, bits, offset);
  }
}

static void HandleActuatorState(BitOp op, int32_t *offset, uint8_t *in,
                                uint8_t *out) {
  const int32_t bits = ACTUATOR_STATE_BITS;
  assert(op != kBitOpWrite || (*in & ((1U << bits) - 1U)) == *in);
  BitOpUint8(op, bits, offset, in, out);
}

static void HandleActuatorStateCommand(BitOp op, int32_t *offset,
                                       ActuatorStateCommand *in, uint8_t *out) {
  const int32_t bits = ACTUATOR_STATE_COMMAND_BITS;
  if (op == kBitOpWrite) {
    assert((*in & ((1U << bits) - 1U)) == *in);
    WriteBitsUint32((uint32_t)*in, bits, offset, out);
  } else {
    *in = (ActuatorStateCommand)ReadBitsUint32(out, bits, offset);
  }
}

static void HandleBattStateCommand(BitOp op, int32_t *offset,
                                   BattStateCommand *in, uint8_t *out) {
  const int32_t bits = BATT_STATE_COMMAND_BITS;
  if (op == kBitOpWrite) {
    assert((*in & ((1U << bits) - 1U)) == *in);
    WriteBitsUint32((uint32_t)*in, bits, offset, out);
  } else {
    *in = (BattStateCommand)ReadBitsUint32(out, bits, offset);
  }
}

static void HandleMvlvStateCommand(BitOp op, int32_t *offset,
                                   MvlvStateCommand *in, uint8_t *out) {
  const int32_t bits = MVLV_STATE_COMMAND_BITS;
  if (op == kBitOpWrite) {
    assert((*in & ((1U << bits) - 1U)) == *in);
    WriteBitsUint32((uint32_t)*in, bits, offset, out);
  } else {
    *in = (MvlvStateCommand)ReadBitsUint32(out, bits, offset);
  }
}

static bool HandleBattCommandMessage(
    BitOp op, int32_t total_length, int32_t *offset, BattCommandMessage *in,
    uint8_t *out) {
  const int32_t required_length = BATT_STATE_COMMAND_BITS + 32;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    HandleBattStateCommand(op, offset, &in->state_command, out);
    BitOpUint32(op, 32, offset, &in->batt_signal, out);
  }
  return *offset - start == required_length;
}

static bool HandleFlightCommandMessage(
    BitOp op, int32_t total_length, int32_t *offset,
    FlightCommandMessage *in, uint8_t *out) {
  const int32_t required_length = 46;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint8(op, 1, offset, &in->force_hover_accel, out);
    BitOpUint8(op, 1, offset, &in->force_high_tension, out);
    BitOpUint8(op, 1, offset, &in->force_reel, out);
    BitOpUint8(op, 1, offset, &in->gs_unpause_transform, out);
    BitOpUint8(op, 1, offset, &in->force_detwist_turn_once, out);
    BitOpUint32(op, 32, offset, &in->safety_code, out);
    BitOpUint8(op, 3, offset, &in->experiment_type, out);
    BitOpUint8(op, 6, offset, &in->experiment_case_id, out);
  }
  return *offset - start == required_length;
}

static bool HandleFpvSetStateMessage(
    BitOp op, int32_t total_length, int32_t *offset, FpvSetStateMessage *in,
    uint8_t *out) {
  const int32_t required_length = 33;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint8(op, 1, offset, &in->enable, out);
    BitOpUint32(op, 32, offset, &in->safety_code, out);
  }
  return *offset - start == required_length;
}

static bool HandleMotorAckParamMessage(
    BitOp op, int32_t total_length, int32_t *offset, MotorAckParamMessage *in,
    uint8_t *out) {
  const int32_t required_length = 40;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint8(op, 8, offset, &in->id, out);
    BitOpUint32(op, 32, offset, (uint32_t *)&in->value, out);
  }
  return *offset - start == required_length;
}

static bool HandleMotorGetParamMessage(
    BitOp op, int32_t total_length, int32_t *offset, MotorGetParamMessage *in,
    uint8_t *out) {
  const int32_t required_length = 8 + kNumMotors;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint8(op, kNumMotors, offset, &in->selected_motors, out);
    BitOpUint8(op, 8, offset, &in->id, out);
  }
  return *offset - start == required_length;
}

static bool HandleMotorSetParamMessage(
    BitOp op, int32_t total_length, int32_t *offset, MotorSetParamMessage *in,
    uint8_t *out) {
  const int32_t required_length = 40 + kNumMotors;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint8(op, kNumMotors, offset, &in->selected_motors, out);
    BitOpUint8(op, 8, offset, &in->id, out);
    BitOpUint32(op, 32, offset, (uint32_t *)&in->value, out);
  }
  return *offset - start == required_length;
}

static bool HandleMotorSetStateMessage(
    BitOp op, int32_t total_length, int32_t *offset, MotorSetStateMessage *in,
    uint8_t *out) {
  const int32_t required_length = ACTUATOR_STATE_COMMAND_BITS + 32 + kNumMotors;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    HandleActuatorStateCommand(op, offset, &in->command, out);
    BitOpUint32(op, 32, offset, &in->command_data, out);
    BitOpUint8(op, kNumMotors, offset, &in->selected_motors, out);
  }
  return *offset - start == required_length;
}

static bool HandleMvlvCommandMessage(
    BitOp op, int32_t total_length, int32_t *offset, MvlvCommandMessage *in,
    uint8_t *out) {
  const int32_t required_length = MVLV_STATE_COMMAND_BITS + 32;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    HandleMvlvStateCommand(op, offset, &in->state_command, out);
    BitOpUint32(op, 32, offset, &in->mvlv_signal, out);
  }
  return *offset - start == required_length;
}

static bool HandlePitotSetStateMessage(
    BitOp op, int32_t total_length, int32_t *offset, PitotSetStateMessage *in,
    uint8_t *out) {
  const int32_t required_length = 33;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint8(op, 1, offset, &in->cover, out);
    BitOpUint32(op, 32, offset, &in->safety_code, out);
  }
  return *offset - start == required_length;
}

static bool HandleServoAckParamMessage(
    BitOp op, int32_t total_length, int32_t *offset, ServoAckParamMessage *in,
    uint8_t *out) {
  const int32_t required_length = 48;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpInt16(op, 16, offset, &in->param, out);
    BitOpUint32(op, 32, offset, &in->value, out);
  }
  return *offset - start == required_length;
}

static bool HandleServoGetParamMessage(
    BitOp op, int32_t total_length, int32_t *offset, ServoGetParamMessage *in,
    uint8_t *out) {
  const int32_t required_length = 16 + kNumServos;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint16(op, kNumServos, offset, &in->selected_servos, out);
    BitOpInt16(op, 16, offset, &in->param, out);
  }
  return *offset - start == required_length;
}

static bool HandleServoSetParamMessage(
    BitOp op, int32_t total_length, int32_t *offset, ServoSetParamMessage *in,
    uint8_t *out) {
  const int32_t required_length = 48 + kNumServos;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    BitOpUint16(op, kNumServos, offset, &in->selected_servos, out);
    BitOpInt16(op, 16, offset, &in->param, out);
    BitOpUint32(op, 32, offset, &in->value, out);
  }
  return *offset - start == required_length;
}

static bool HandleServoSetStateMessage(
    BitOp op, int32_t total_length, int32_t *offset, ServoSetStateMessage *in,
    uint8_t *out) {
  const int32_t required_length = ACTUATOR_STATE_COMMAND_BITS + 32 + kNumServos;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    HandleActuatorStateCommand(op, offset, &in->state_command, out);
    BitOpUint32(op, 32, offset, &in->servo_arming_signal, out);
    BitOpUint16(op, kNumServos, offset, &in->selected_servos, out);
  }
  return *offset - start == required_length;
}

static bool HandleTetherReleaseSetStateMessage(
    BitOp op, int32_t total_length, int32_t *offset,
    TetherReleaseSetStateMessage *in, uint8_t *out) {
  const int32_t required_length =
      ACTUATOR_STATE_COMMAND_BITS + 32 + kNumLoadcellNodes;
  const int32_t start = *offset;
  if (total_length - *offset >= required_length) {
    HandleActuatorStateCommand(op, offset, &in->state_command, out);
    BitOpUint32(op, 32, offset, &in->arming_signal, out);
    BitOpUint8(op, kNumLoadcellNodes, offset, &in->selected_loadcells, out);
  }
  return *offset - start == required_length;
}

static bool HandleTetherOpData(BitOp op, TetherOpType type,
                               int32_t total_length, int32_t *offset,
                               TetherOpData *in, uint8_t *out) {
  // TetherOps are multiplexed according to their message type.
  if (op == kBitOpRead) {
    memset(in, 0, sizeof(*in));
  }
  switch (type) {
    case kTetherOpTypeNone:
      return false;
    case kTetherOpTypeBattCommand:
      return HandleBattCommandMessage(op, total_length, offset,
                                      &in->batt_command, out);
    case kTetherOpTypeFlightCommand:
      return HandleFlightCommandMessage(op, total_length, offset,
                                        &in->flight_command, out);
    case kTetherOpTypeFpvSetState:
      return HandleFpvSetStateMessage(op, total_length, offset,
                                      &in->fpv_set_state, out);
    case kTetherOpTypeMotorAckParam:
      return HandleMotorAckParamMessage(op, total_length, offset,
                                        &in->motor_ack_param, out);
    case kTetherOpTypeMotorGetParam:
      return HandleMotorGetParamMessage(op, total_length, offset,
                                        &in->motor_get_param, out);
    case kTetherOpTypeMotorSetParam:
      return HandleMotorSetParamMessage(op, total_length, offset,
                                        &in->motor_set_param, out);
    case kTetherOpTypeMotorSetState:
      return HandleMotorSetStateMessage(op, total_length, offset,
                                        &in->motor_set_state, out);
    case kTetherOpTypeMvlvCommand:
      return HandleMvlvCommandMessage(op, total_length, offset,
                                      &in->mvlv_command, out);
    case kTetherOpTypePitotSetState:
      return HandlePitotSetStateMessage(op, total_length, offset,
                                        &in->pitot_set_state, out);
    case kTetherOpTypeServoAckParam:
      return HandleServoAckParamMessage(op, total_length, offset,
                                        &in->servo_ack_param, out);
    case kTetherOpTypeServoGetParam:
      return HandleServoGetParamMessage(op, total_length, offset,
                                        &in->servo_get_param, out);
    case kTetherOpTypeServoSetParam:
      return HandleServoSetParamMessage(op, total_length, offset,
                                        &in->servo_set_param, out);
    case kTetherOpTypeServoSetState:
      return HandleServoSetStateMessage(op, total_length, offset,
                                        &in->servo_set_state, out);
    case kTetherOpTypeTetherReleaseSetState:
      return HandleTetherReleaseSetStateMessage(op, total_length, offset,
                                                &in->tether_release_set_state,
                                                out);
    case kTetherOpTypeForceSigned:
    case kNumTetherOpTypes:
    default:
      assert(false);
      return false;
  }
}

static bool HandleTetherOp(BitOp op, bool encode_source, int32_t total_length,
                           int32_t *offset, TetherOpQueue *in, uint8_t *out) {
  AioNode source = kAioNodeOperator;
  TetherOpType type = kTetherOpTypeNone;
  TetherOpData *data = NULL;
  uint16_t sequence = 0U;

  // TetherOps are stored in a ring buffer. When writing, we obtain the
  // next pending command from the tail position. When reading, we obtain the
  // next available command from the new_head position.

  // Check for sufficient header length.
  const int32_t required_length =
      encode_source * AIO_NODE_BITS + TETHER_OP_TYPE_BITS + 16;
  const int32_t start = *offset;
  if (total_length - *offset < required_length) {
    return false;
  }

  // Handle header.
  if (op == kBitOpWrite) {
    data = TetherOpGetTail(in, &source, &type, &sequence);
  }
  if (encode_source) {
    HandleAioNode(op, offset, &source, out);
  }
  HandleTetherOpType(op, offset, &type, out);
  BitOpUint16(op, 16, offset, &sequence, out);  // From AioHeader.
  if (op == kBitOpRead) {
    data = TetherOpGetNewHead(source, type, sequence, in);
  }
  if (*offset - start != required_length) {
    assert(false);
  }

  // Handle data.
  if (data != NULL
      && HandleTetherOpData(op, type, total_length, offset, data, out)) {
    // Advance ring buffer.
    if (op == kBitOpWrite) {
      TetherOpPopTail(in);
    } else {
      TetherOpPushNewHead(in);
    }
    return true;
  }
  return false;
}

static void HandleTetherUpSubframe1(BitOp op, int32_t decimation,
                                    int32_t total_length, int32_t *offset,
                                    TetherOpQueue *op_commands,
                                    TetherUpMessage *in, uint8_t *out) {
  // The first bit in subframe 1 indicates if a TetherOp is present.
  uint8_t op_command_present =
      op_commands != NULL && TetherOpIsPending(op_commands);
  BitOpUint8(op, 1, offset, &op_command_present, out);

  if (op_command_present) {
    const bool encode_source = false;  // kAioNodeOperator assumed.
    while (HandleTetherOp(op, encode_source, total_length, offset, op_commands,
                          out)) {}
  } else {
    HandleTetherGsGpsCompass(op, decimation, offset, &in->gps_compass, out);
  }
}

static void HandleTetherUpSubframe(BitOp op, int32_t decimation,
                                   int32_t total_length, int32_t *offset,
                                   TetherOpQueue *op_commands,
                                   TetherUpMessage *in, uint8_t *out) {
  const int32_t index = in->frame_index / TETHER_RADIO_DECIMATION;
  const int32_t parts = 4;
  const int32_t dec = parts * decimation;
  switch (index % parts) {
    case 0:
      HandleTetherGpsTime(op, dec, offset, &in->gps_time, out);
      HandleTetherGpsStatus(op, dec, offset, &in->gps_status, out);
      break;
    case 1:
      HandleTetherUpSubframe1(op, dec, total_length, offset, op_commands, in,
                              out);
      break;
    case 2:
      HandleTetherGsGpsPosition(op, dec, offset, &in->gps_position, out);
      break;
    case 3:
      HandleTetherGsGpsCompass(op, dec, offset, &in->gps_compass, out);
      HandleTetherWeather(op, dec, offset, &in->weather, out);
      break;
    default:
      assert(false);
      break;
  }
}

static bool HandleTetherUp(BitOp op, TetherOpQueue *op_commands,
                           TetherUpMessage *in, uint16_t *frame_index,
                           TetherUpPackedMessage *out) {
  const int32_t total_length = 8 * ARRAYSIZE(out->data) - CRC_BITS;
  int32_t offset = 0;
  int32_t decimation = TETHER_RADIO_DECIMATION;

  BitOpUint16(op, TETHER_FRAME_INDEX_BITS, &offset, frame_index, out->data);
  if (in != NULL) {
    HandleTetherJoystick(op, decimation, &offset, &in->joystick, out->data);
    HandleTetherPlatform(op, decimation, &offset, &in->platform_a, out->data);
    HandleTetherPlatform(op, decimation, &offset, &in->platform_b, out->data);
    HandleTetherDrum(op, decimation, &offset, &in->drum_a, out->data);
    HandleTetherDrum(op, decimation, &offset, &in->drum_b, out->data);
    HandleTetherWind(op, decimation, &offset, &in->wind, out->data);
    HandleTetherPlc(op, decimation, &offset, &in->plc, out->data);
    HandleTetherGroundStationStatus(op, decimation, &offset,
                                    &in->ground_station, out->data);
    HandleGpsRtcm(op, &offset, ARRAYSIZE(in->rtcm), in->rtcm, out->data);

    // Handle subframe data last since it may fill the remaining buffer with
    // operator commands.
    HandleTetherUpSubframe(op, decimation, total_length, &offset, op_commands,
                           in, out->data);
  }
  HandleFill(op, total_length, &offset, out->data);
  return HandleCrc16(op, &offset, out->data);
}

static void HandleTetherControlCommand(BitOp op, int32_t decimation,
                                       int32_t *offset,
                                       TetherControlCommand *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  BitOpUint8(op, 2, offset, &in->controller_label, out);
  HandleDetwistAngle(op, offset, &in->detwist_angle, out);
  BitOpScaledFloat(op, 0.001f, 14, offset, &in->winch_velocity, out);
  BitOpUint8(op, 2, offset, &in->gs_mode_request, out);
  BitOpUint8(op, 1, offset, &in->gs_unpause_transform, out);

  // Azimuth: [-pi, pi) at ~0.01 deg resolution.
  HandleAngle(op, -PI_F, PI_F, 15, offset, &in->gs_azi_target, out);

  // Azimuth dead zone: [0 deg, 12.75 deg) at 0.05 deg resolution.
  BitOpRangeFloat(op, 0.0f, GS_AZI_DEAD_ZONE_MAX, GS_AZI_DEAD_ZONE_BITS, offset,
                  &in->gs_azi_dead_zone, out);
}

static void HandleTetherCommsStatus(BitOp op, int32_t decimation,
                                    int32_t *offset, TetherCommsStatus *in,
                                    uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleBitFlag(op, kTetherCommsLinkPof, offset, &in->links_up, out);
  HandleBitFlag(op, kTetherCommsLinkEop, offset, &in->links_up, out);
  HandleBitFlag(op, kTetherCommsLinkWifi, offset, &in->links_up, out);
  HandleBitFlag(op, kTetherCommsLinkLongRange, offset, &in->links_up, out);
  HandleBitFlag(op, kTetherCommsLinkJoystick, offset, &in->links_up, out);
  BitOpScaledOffsetInt16(op, 1, -127, 7, offset, &in->received_signal_strength,
                         out);
}

static void HandleTetherControlTelemetry(BitOp op, int32_t decimation,
                                         int32_t index,
                                         uint8_t controller_label,
                                         int32_t *offset,
                                         TetherControlTelemetry *in,
                                         uint8_t *out) {
  assert(decimation == TETHER_CONTROL_TELEMETRY_DECIMATION);
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleSequence(op, offset, &in->sequence, out);
  in->controller_label = controller_label;
  BitOpUint8(op, 3, offset, &in->flags, out);

  // Rotates through all subsystems with a fault indication.
  BitOpUint8(op, 7, offset, &in->subsystem, out);
  BitOpUint8(op, 5, offset, &in->subsystem_faults, out);

  // Current flight mode and gates for the next flight mode.
  BitOpUint8(op, 4, offset, &in->flight_mode, out);
  BitOpUint16(op, 12, offset, &in->flight_mode_gates, out);

  // Experiment config.
  BitOpUint8(op, 3, offset, &in->experiment_type, out);
  BitOpUint8(op, 6, offset, &in->experiment_case_id, out);

  // Timers.
  // Multiplex flight_mode_time with loop_time and loop_angle.
  const int32_t loop_time_bits = 12;
  const int32_t loop_angle_bits = 8;
  const int32_t flight_mode_time_bits = loop_time_bits + loop_angle_bits;
  const int32_t loop_count_bits = loop_time_bits + loop_angle_bits;
  if (index % 3 == 0) {
    // Stored as an unsigned integer to allow for rollovers.
    BitOpUint32(op, flight_mode_time_bits, offset, &in->flight_mode_time, out);
  } else if (index % 3 == 1) {
    BitOpUnsignedFloat(op, 1e-5f, loop_time_bits, offset, &in->loop_time, out);
    HandleAngle(op, -PI_F, PI_F, loop_angle_bits, offset, &in->loop_angle, out);
  } else {
    BitOpUint32(op, loop_count_bits, offset, &in->loop_count, out);
  }

  // Multiplex pilot data with crosswind status.
  if (index % 2 == 0) {
    // Pilot data.
    BitOpScaledFloat(op, 0.1f, 11, offset, &in->airspeed, out);
    HandleAngle(op, -PI_F / 2.0f, PI_F / 2.0f, 8, offset, &in->alpha, out);
    HandleAngle(op, -PI_F / 2.0f, PI_F / 2.0f, 8, offset, &in->beta, out);
    HandleAngle(op, -PI_F, PI_F, 8, offset, &in->roll, out);
    HandleAngle(op, -PI_F / 2.0f, PI_F / 2.0f, 8, offset, &in->pitch, out);
    HandleAngle(op, -PI_F, PI_F, 8, offset, &in->yaw, out);
    BitOpScaledFloat(op, 0.01f, 10, offset, &in->pqr[0], out);
    BitOpScaledFloat(op, 0.01f, 10, offset, &in->pqr[1], out);
    BitOpScaledFloat(op, 0.01f, 10, offset, &in->pqr[2], out);
  } else {
    // Crosswind status.
    BitOpScaledFloat(op, 0.5f, 12, offset, &in->target_pos_cw[0], out);
    BitOpScaledFloat(op, 0.5f, 12, offset, &in->target_pos_cw[1], out);
    BitOpScaledFloat(op, 0.5f, 12, offset, &in->current_pos_cw[0], out);
    BitOpScaledFloat(op, 0.5f, 12, offset, &in->current_pos_cw[1], out);
    BitOpScaledFloat(op, 0.01f, 8, offset, &in->delta_aileron, out);
    BitOpScaledFloat(op, 0.01f, 8, offset, &in->delta_elevator, out);
    BitOpScaledFloat(op, 0.01f, 8, offset, &in->delta_rudder, out);
    BitOpScaledFloat(op, 0.01f, 8, offset, &in->delta_inboard_flap, out);
  }

  // Multiplex horizontal position.
  BitOpScaledFloat(op, 0.1f, 18, offset, &in->pos_g[index % 2], out);
  BitOpScaledFloat(op, 0.1f, 14, offset, &in->pos_g[2], out);

  // Multiplex horizontal velocity.
  BitOpScaledFloat(op, 0.1f, 12, offset, &in->vel_g[index % 2], out);
  BitOpScaledFloat(op, 0.1f, 12, offset, &in->vel_g[2], out);

  // Multiplex tension and tension_command.
  const int32_t tension_bits = 12;
  if (index % 2 == 0) {
    BitOpUnsignedFloat(op, 100.0f, tension_bits, offset, &in->tension, out);
  } else {
    BitOpUnsignedFloat(op, 100.0f, tension_bits, offset, &in->tension_command,
                       out);
  }

  // Multiplex thrust and thrust_avail.
  const int32_t thrust_bits = 7;
  if (index % 2 == 0) {
    BitOpScaledFloat(op, 500.0f, thrust_bits, offset, &in->thrust, out);
  } else {
    BitOpScaledFloat(op, 500.0f, thrust_bits, offset, &in->thrust_avail, out);
  }

  // Multiplex moment[i], and moment_avail[i].
  BitOpScaledFloat(op, 500.0f, 7, offset, &in->moment[index % 3], out);
  BitOpScaledFloat(op, 500.0f, 7, offset, &in->moment_avail[index % 3], out);

  BitOpRangeFloat(op, 0.0f, 1.0f, 6, offset, &in->gain_ramp_scale, out);

  BitOpUint8(op, 1, offset, &in->force_detwist_turn_once, out);
}

static void HandleTetherMotorStatuses(BitOp op, int32_t num_motors,
                                      int32_t decimation, int32_t index,
                                      int32_t *offset,
                                      TetherMotorStatus motors[],
                                      uint8_t *out) {
  // Always send this information for each update.
  for (int32_t i = 0; i < num_motors; ++i) {
    TetherMotorStatus *s = &motors[i];
    BitOpUint8(op, 1, offset, &s->warning, out);
    BitOpUint8(op, 1, offset, &s->error, out);
    HandleUpdated(op, decimation, offset, &s->no_update_count, out);
    HandleActuatorState(op, offset, &s->state, out);
    BitOpScaledOffsetInt16(op, 1, -256, 9, offset, &s->speed, out);
  }

  // Multiplex by motor id.
  int32_t i = index % num_motors;
  TetherMotorStatus *s = &motors[i];
  BitOpScaledOffsetInt16(op, 2, -256, 8, offset, &s->iq, out);
  BitOpScaledOffsetInt16(op, 2, -256, 8, offset, &s->id, out);
  BitOpScaledOffsetInt16(op, 2, 0, 10, offset, &s->bus_voltage, out);
  BitOpScaledOffsetInt16(op, 2, -256, 8, offset, &s->bus_current, out);

  // Multiplex by motor id, then by motor temp id.
  i = (index / num_motors) % ARRAYSIZE(s->motor_temps);
  HandleTemperature(op, offset, &s->motor_temps[i], out);

  // Multiplex by motor id, then by controller temp id.
  i = (index / num_motors) % ARRAYSIZE(s->controller_temps);
  HandleTemperature(op, offset, &s->controller_temps[i], out);
}

static void HandleTetherNodeStatus(BitOp op, int32_t decimation,
                                   int32_t *offset, TetherNodeStatus *in,
                                   uint8_t *out) {
  assert(decimation == TETHER_NODE_STATUS_DECIMATION);
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  BitOpUint8(op, 7, offset, &in->node, out);
  BitOpUint8(op, 6, offset, &in->flags, out);
  HandleTemperature(op, offset, &in->board_temp, out);
  BitOpUint8(op, 7, offset, &in->board_humidity, out);
}

static void HandleTetherFlightComputer(BitOp op, int32_t decimation,
                                       int32_t *offset,
                                       TetherFlightComputer *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  BitOpUint8(op, 4, offset, &in->flags, out);
}

static void HandleTetherFlightComputers(BitOp op, int32_t num_flight_computers,
                                        int32_t decimation, int32_t index,
                                        int32_t *offset,
                                        TetherFlightComputer *flight_computers,
                                        uint8_t *out) {
  HandleTetherFlightComputer(op, num_flight_computers * decimation, offset,
                             &flight_computers[index % num_flight_computers],
                             out);
}

static void HandleTetherReleaseStatus(BitOp op, int32_t decimation,
                                      int32_t *offset, TetherReleaseStatus *in,
                                      uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  HandleActuatorState(op, offset, &in->state, out);
  BitOpUint8(op, 1, offset, &in->interlock_switched, out);
  BitOpUint8(op, 1, offset, &in->released, out);
}

static void HandleTetherReleaseStatuses(BitOp op, int32_t num_releases,
                                        int32_t decimation, int32_t *offset,
                                        TetherReleaseStatus *releases,
                                        uint8_t *out) {
  for (int32_t i = 0; i < num_releases; ++i) {
    HandleTetherReleaseStatus(op, decimation, offset, &releases[i], out);
  }
}

static void HandleTetherServoStatuses(BitOp op, int32_t num_servos,
                                      int32_t decimation, int32_t index,
                                      int32_t *offset,
                                      TetherServoStatus servos[],
                                      uint8_t *out) {
  // Always send this information for each update.
  for (int32_t i = 0; i < num_servos; ++i) {
    TetherServoStatus *s = &servos[i];
    HandleUpdated(op, decimation, offset, &s->no_update_count, out);
    HandleActuatorState(op, offset, &s->state, out);
    HandleAngle(op, -PI_F, PI_F, 8, offset, &s->angle, out);
  }

  // Multiplex by servo id.
  TetherServoStatus *s = &servos[index % num_servos];
  HandleTemperature(op, offset, &s->r22_temp, out);
}

static void HandleTetherBatteryStatus(BitOp op, int32_t decimation,
                                      int32_t index, int32_t *offset,
                                      TetherBatteryStatus *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  BitOpUint8(op, 1, offset, &in->warning, out);
  BitOpUint8(op, 1, offset, &in->error, out);

  // Multiplex voltages.
  const int32_t voltage_bits = 10;
  switch (index % 4) {
    case 0:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset, &in->lv_a,
                             out);
      break;
    case 1:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset, &in->lv_b,
                             out);
      break;
    case 2:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset, &in->v_lv_or,
                             out);
      break;
    case 3:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset,
                             &in->v_charger, out);
      break;
    default:
      assert(false);
      break;
  }

  // Multiplex currents.
  const int32_t current_bits = 10;
  switch (index % 2) {
    case 0:
      BitOpScaledOffsetFloat(op, 0.1f, -1.0f, current_bits, offset, &in->i_hall,
                             out);
      break;
    case 1:
      BitOpScaledOffsetFloat(op, 0.025f, 0.0f, current_bits, offset,
                             &in->i_charger, out);
      break;
    default:
      assert(false);
      break;
  }

  // Multiplex temperatures.
  HandleTemperature(op, offset, &in->temps[index % ARRAYSIZE(in->temps)], out);
}

static void HandleTetherMvlvStatus(BitOp op, int32_t decimation,
                                   int32_t index, int32_t *offset,
                                   TetherMvlvStatus *in, uint8_t *out) {
  HandleUpdated(op, decimation, offset, &in->no_update_count, out);
  BitOpUint8(op, 1, offset, &in->warning, out);
  BitOpUint8(op, 1, offset, &in->error, out);

  // Multiplex voltages.
  const int32_t voltage_bits = 10;
  switch (index % 4) {
    case 0:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset, &in->v_lv,
                             out);
      break;
    case 1:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset, &in->v_lv_or,
                             out);
      break;
    case 2:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset,
                             &in->v_lv_pri, out);
      break;
    case 3:
      BitOpScaledOffsetFloat(op, 0.1f, 0.0f, voltage_bits, offset,
                             &in->v_lv_sec, out);
      break;
    default:
      assert(false);
      break;
  }

  // Multiplex current and lower 10 bits of MVLV status.
  const int32_t current_bits = 10;
  if (index % 2) {
    BitOpScaledOffsetFloat(op, 0.1f, -1.0f, current_bits, offset, &in->i_hall,
                           out);
  } else {
    BitOpUint16(op, current_bits, offset, &in->status, out);
  }

  // Multiplex temperatures.
  HandleTemperature(op, offset, &in->temps[index % ARRAYSIZE(in->temps)], out);
}

static void HandleTetherLvSystemStatuses(BitOp op, int32_t decimation,
                                         int32_t index, int32_t *offset,
                                         TetherBatteryStatus *in_a,
                                         TetherBatteryStatus *in_b,
                                         TetherMvlvStatus *mvlv,
                                         uint8_t *out) {
  const int32_t dec  = 3 * decimation;
  switch (index % 3) {
    case 0:
      HandleTetherBatteryStatus(op, dec, index / 3, offset, in_a, out);
      break;
    case 1:
      HandleTetherBatteryStatus(op, dec, index / 3, offset, in_b, out);
      break;
    case 2:
      HandleTetherMvlvStatus(op, dec, index / 3, offset, mvlv, out);
      break;
    default:
      assert(false);
      break;
  }
}

static void HandleTetherDownSubframe1(BitOp op, int32_t decimation,
                                      int32_t index, int32_t total_length,
                                      int32_t *offset,
                                      TetherOpQueue *command_replies,
                                      TetherDownMessage *in, uint8_t *out) {
  // The first bit in subframe 1 indicates if a TetherOp is present.
  uint8_t command_reply_present =
      command_replies != NULL && TetherOpIsPending(command_replies);
  BitOpUint8(op, 1, offset, &command_reply_present, out);

  if (command_reply_present) {
    const bool encode_source = true;
    while (HandleTetherOp(op, encode_source, total_length, offset,
                          command_replies, out)) {}
  } else {
    HandleTetherControlTelemetry(op, decimation, index,
                                 in->control_command.controller_label, offset,
                                 &in->control_telemetry, out);
  }
}

static void HandleTetherDownSubframe(BitOp op, int32_t decimation,
                                     int32_t total_length, int32_t *offset,
                                     TetherOpQueue *command_replies,
                                     TetherDownMessage *in, uint8_t *out) {
  const int32_t index = in->frame_index / TETHER_RADIO_DECIMATION;
  const int32_t parts = 4;
  const int32_t dec = parts * decimation;
  switch (index % parts) {
    case 0:
      HandleTetherGpsTime(op, dec, offset, &in->gps_time, out);
      HandleTetherGpsStatuses(op, ARRAYSIZE(in->gps_statuses), dec,
                              index / parts, offset, in->gps_statuses, out);
      HandleTetherServoStatuses(op, ARRAYSIZE(in->servo_statuses), dec,
                                index / parts, offset, in->servo_statuses, out);
      HandleTetherLvSystemStatuses(op, dec, index / parts, offset,
                                   &in->batt_a, &in->batt_b, &in->mvlv, out);
      HandleTetherCommsStatus(op, dec, offset, &in->comms_status, out);
      break;
    case 1:
      // Index parameter accounts for two updates per cycle.
      HandleTetherDownSubframe1(op, dec / 2, (2 * index) / parts, total_length,
                                offset, command_replies, in, out);
      break;
    case 2:
      HandleTetherMotorStatuses(op, ARRAYSIZE(in->motor_statuses), dec,
                                index / parts, offset, in->motor_statuses, out);
      HandleTetherNodeStatus(op, dec, offset, &in->node_status, out);
      HandleTetherFlightComputers(op, ARRAYSIZE(in->flight_computers), dec,
                                  index / parts, offset, in->flight_computers,
                                  out);
      HandleTetherReleaseStatuses(op, ARRAYSIZE(in->release_statuses), dec,
                                  offset, in->release_statuses, out);
      break;
    case 3:
      // Index parameter accounts for two updates per cycle.
      HandleTetherControlTelemetry(op, dec / 2, (2 * index) / parts,
                                   in->control_command.controller_label,
                                   offset, &in->control_telemetry, out);
      break;
    default:
      assert(false);
      break;
  }
}

static bool HandleTetherDown(BitOp op, TetherOpQueue *command_replies,
                             TetherDownMessage *in, uint16_t *frame_index,
                             TetherDownPackedMessage *out) {
  const int32_t total_length = 8 * ARRAYSIZE(out->data) - CRC_BITS;
  int32_t offset = 0;
  int32_t decimation = TETHER_RADIO_DECIMATION;

  BitOpUint16(op, TETHER_FRAME_INDEX_BITS, &offset, frame_index, out->data);
  if (in != NULL) {
    BitOpUint16(op, TETHER_FRAME_INDEX_BITS, &offset, &in->received_frame_index,
                out->data);
    HandleTetherControlCommand(op, decimation, &offset, &in->control_command,
                               out->data);

    // Handle subframe data last since it may fill the remaining buffer with
    // operator command replies.
    HandleTetherDownSubframe(op, decimation, total_length, &offset,
                             command_replies, in, out->data);
  }
  HandleFill(op, total_length, &offset, out->data);
  return HandleCrc16(op, &offset, out->data);
}

bool PackTetherUp(TetherOpQueue *op_commands, TetherUpMessage *in,
                  TetherUpPackedMessage *out) {
  return HandleTetherUp(kBitOpWrite, op_commands, in, &in->frame_index, out);
}

bool UnpackTetherUp(TetherOpQueue *op_commands, TetherUpPackedMessage *in,
                    TetherUpMessage *out) {
  return HandleTetherUp(kBitOpRead, op_commands, out, &out->frame_index, in);
}

bool UnpackTetherUpFrameIndex(TetherUpPackedMessage *in,
                              uint16_t *frame_index) {
  return HandleTetherUp(kBitOpRead, NULL, NULL, frame_index, in);
}

bool PackTetherDown(TetherOpQueue *command_replies, TetherDownMessage *in,
                    TetherDownPackedMessage *out) {
  return HandleTetherDown(kBitOpWrite, command_replies, in, &in->frame_index,
                          out);
}

bool UnpackTetherDown(TetherOpQueue *command_replies,
                      TetherDownPackedMessage *in, TetherDownMessage *out) {
  return HandleTetherDown(kBitOpRead, command_replies, out, &out->frame_index,
                          in);
}

bool UnpackTetherDownFrameIndex(TetherDownPackedMessage *in,
                                uint16_t *frame_index) {
  return HandleTetherDown(kBitOpRead, NULL, NULL, frame_index, in);
}
