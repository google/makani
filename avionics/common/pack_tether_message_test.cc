// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <float.h>
#include <stdint.h>
#include <string.h>

#include <vector>

#include "avionics/common/actuator_types.h"
#include "avionics/common/avionics_messages.h"
#include "avionics/common/fast_math/fast_math.h"
#include "avionics/common/pack_tether_message.h"
#include "avionics/common/safety_codes.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/common/tether_op.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "control/crosswind/crosswind_types.h"
#include "control/experiments/experiment_types.h"
#include "control/fault_detection/fault_detection_types.h"
#include "control/hover/hover_types.h"
#include "control/trans_in/trans_in_types.h"

namespace {

const ActuatorState kActuatorStates[] = {
  kActuatorStateInit,
  kActuatorStateReady,
  kActuatorStateArmed,
  kActuatorStateRunning,
  kActuatorStateError,
  kActuatorStateTest,
};

template<typename T> bool PackUnpackTetherMessage(T *in, T *out) {
  return false;
}

template <> bool PackUnpackTetherMessage<TetherUpMessage>(
    TetherUpMessage *in, TetherUpMessage *out) {
  TetherUpPackedMessage packed;
  return PackTetherUp(nullptr, in, &packed)
      && UnpackTetherUp(nullptr, &packed, out);
}

template <> bool PackUnpackTetherMessage<TetherDownMessage>(
    TetherDownMessage *in, TetherDownMessage *out) {
  TetherDownPackedMessage packed;
  return PackTetherDown(nullptr, in, &packed)
      && UnpackTetherDown(nullptr, &packed, out);
}

template<typename T> bool PackUnpackTetherMessage(TetherOpQueue *q_in, T *in,
                                                  TetherOpQueue *q_out,
                                                  T *out) {
  return false;
}

template <> bool PackUnpackTetherMessage<TetherUpMessage>(
    TetherOpQueue *q_in, TetherUpMessage *in, TetherOpQueue *q_out,
    TetherUpMessage *out) {
  TetherUpPackedMessage packed;
  return PackTetherUp(q_in, in, &packed) && UnpackTetherUp(q_out, &packed, out);
}

template <> bool PackUnpackTetherMessage<TetherDownMessage>(
    TetherOpQueue *q_in, TetherDownMessage *in, TetherOpQueue *q_out,
    TetherDownMessage *out) {
  TetherDownPackedMessage packed;
  return PackTetherDown(q_in, in, &packed)
      && UnpackTetherDown(q_out, &packed, out);
}

template<typename T> void PushOpQueue(AioNode source, TetherOpType type,
                                      uint16_t sequence, const T &data,
                                      TetherOpQueue *q) {
  TetherOpData *u = TetherOpGetNewHead(source, type, sequence, q);
  ASSERT_NE(nullptr, u);
  memcpy(u, &data, sizeof(data));
  TetherOpPushNewHead(q);
}

template<typename T> void PopOpQueue(TetherOpQueue *q, AioNode *source,
                                     TetherOpType *type, uint16_t *sequence,
                                     T *data) {
  TetherOpData *u = TetherOpGetTail(q, source, type, sequence);
  ASSERT_NE(nullptr, u);
  memcpy(data, u, sizeof(*data));
  TetherOpPopTail(q);
}

template<typename TypeMessage, typename TypeData>
void PackUnpackOpQueue(uint16_t frame_index, AioNode source_in,
                       TetherOpType type_in, uint16_t sequence_in,
                       const TypeData &data_in, TypeData *data_out) {
  TypeMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));
  orig.frame_index = frame_index;

  TetherOpQueue q_in, q_out;
  TetherOpInit(&q_in);
  TetherOpInit(&q_out);

  // Do not allow commands of type none.
  EXPECT_EQ(nullptr,
            TetherOpGetNewHead(kAioNodeOperator, kTetherOpTypeNone, 0, &q_in));
  EXPECT_EQ(nullptr,
            TetherOpGetNewHead(kAioNodeOperator, kTetherOpTypeNone, 1, &q_in));

  // Pack/unpack.
  PushOpQueue(source_in, type_in, sequence_in, data_in, &q_in);
  EXPECT_TRUE(PackUnpackTetherMessage(&q_in, &orig, &q_out, &unpacked));
  AioNode source_out;
  TetherOpType type_out;
  uint16_t sequence_out;
  PopOpQueue(&q_out, &source_out, &type_out, &sequence_out, data_out);

  // Compare.
  EXPECT_EQ(source_in, source_out);
  EXPECT_EQ(type_in, type_out);
  EXPECT_EQ(sequence_in, sequence_out);
}

template<typename T> void PackUnpackOpCommand(TetherOpType type_in,
                                              uint16_t sequence_in,
                                              const T &data_in, T *data_out) {
  // Decimated according to (frame_index / dec) % 4 == 1.
  uint16_t frame_index = 33U * TETHER_RADIO_DECIMATION;
  PackUnpackOpQueue<TetherUpMessage>(frame_index, kAioNodeOperator, type_in,
                                     sequence_in, data_in, data_out);
}

template<typename T> void PackUnpackOpReply(AioNode source_in,
                                            TetherOpType type_in,
                                            uint16_t sequence_in,
                                            const T &data_in, T *data_out) {
  // Decimated according to (frame_index / dec) % 4 == 1.
  uint16_t frame_index = 33U * TETHER_RADIO_DECIMATION;
  PackUnpackOpQueue<TetherDownMessage>(frame_index, source_in, type_in,
                                       sequence_in, data_in, data_out);
}

template<typename T> int32_t GetDecimation(size_t offsetof_field,
                                           size_t sizeof_field) {
  T in, out;
  memset(&in, 0x0, sizeof(in));
  memset(&out, 0x0, sizeof(out));
  int32_t offset = -1;
  bool radio_match = false;

  // Iterate over each frame_index to determine the relationship between
  // field_index and field update.
  for (int32_t i = 0; i < TETHER_FRAME_INDEX_ROLLOVER; ++i) {
    uint8_t *in_field = (uint8_t *)&in + offsetof_field;
    uint8_t *out_field = (uint8_t *)&out + offsetof_field;

    // Initialize in_field=0x00, out_field=0xFF.
    in.frame_index = static_cast<uint16_t>(i);
    memset(in_field, 0x00, sizeof_field);
    memset(out_field, 0xFF, sizeof_field);

    // Pack/unpack message.
    EXPECT_TRUE(PackUnpackTetherMessage(&in, &out));

    // Since many fields do not pack/unpack the same number of bits as the
    // field width, we cannot compare in_field and out_field directly. Instead,
    // we check if out_field changed or not. We reuse in_field and set it to
    // the unchanged out_field value.
    memset(in_field, 0xFF, sizeof_field);
    bool match = (memcmp(in_field, out_field, sizeof_field) != 0x0);

    // Decimated fields should only update according to a multiple of the
    // radio decimation period. (We want to transmit all data.) Non-decimated
    // fields, such as comm_status.received_signal_strength update faster
    // than the radio decimation period.
    if (i % TETHER_RADIO_DECIMATION == 0) {
      radio_match = match;
    }
    EXPECT_EQ(radio_match, match);

    // The first match corresponds to the decimation offset. The second match
    // determines the decimation period.
    if (radio_match) {
      if (offset < 0) {
        offset = i;
      } else {
        return i - offset;
      }
    }
  }
  return -1;
}

template<typename T> bool CheckDecimation(size_t offsetof_field,
                                          size_t sizeof_field) {
  int32_t decimation = GetDecimation<T>(offsetof_field, sizeof_field);
  return decimation > 0
      && (TETHER_FRAME_INDEX_ROLLOVER / decimation) * decimation
      == TETHER_FRAME_INDEX_ROLLOVER;
}

}  // namespace

TEST(PackTetherUp, Frame) {
  TetherUpMessage orig, unpacked;
  TetherUpPackedMessage packed;

  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));
  memset(&packed, 0xC3, sizeof(packed));

  // Test frame index. Iterating through multiple frame indicies since the
  // packing multiplexes streams according to the frame index.
  for (int32_t i = 100; i < 200; ++i) {
    orig.frame_index = (uint8_t)i;
    EXPECT_TRUE(PackTetherUp(NULL, &orig, &packed));
    EXPECT_TRUE(UnpackTetherUp(NULL, &packed, &unpacked));
    EXPECT_EQ(orig.frame_index, unpacked.frame_index);
  }

  // Invalidate CRC.
  packed.data[0] = (uint8_t)~packed.data[0];
  EXPECT_FALSE(UnpackTetherUp(NULL, &packed, &unpacked));
}

TEST(PackTetherUp, TetherJoystick) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, joystick.sequence),
      SIZEOF(TetherUpMessage, joystick.sequence)));

  // Test sequence.
  orig.joystick.sequence = 10U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(10U, unpacked.joystick.sequence);

  // Test flags.
  orig.joystick.flags = kTetherJoystickFlagFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherJoystickFlagFault & unpacked.joystick.flags);
  orig.joystick.flags &= (uint8_t)~kTetherJoystickFlagFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherJoystickFlagFault & unpacked.joystick.flags);

  // Test roll/pitch/yaw/throttle saturation (lower bound).
  orig.joystick.roll = -1.1f;
  orig.joystick.pitch = -1.1f;
  orig.joystick.yaw = -1.1f;
  orig.joystick.throttle = -0.1f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(-1.0f, unpacked.joystick.roll, FLT_EPSILON);
  EXPECT_NEAR(-1.0f, unpacked.joystick.pitch, FLT_EPSILON);
  EXPECT_NEAR(-1.0f, unpacked.joystick.yaw, FLT_EPSILON);
  EXPECT_NEAR(0.0f, unpacked.joystick.throttle, FLT_EPSILON);

  // Test roll/pitch/yaw/throttle saturation (upper bound).
  orig.joystick.roll = 1.1f;
  orig.joystick.pitch = 1.1f;
  orig.joystick.yaw = 1.1f;
  orig.joystick.throttle = 1.1f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(1.0f, unpacked.joystick.roll, FLT_EPSILON);
  EXPECT_NEAR(1.0f, unpacked.joystick.pitch, FLT_EPSILON);
  EXPECT_NEAR(1.0f, unpacked.joystick.yaw, FLT_EPSILON);
  EXPECT_NEAR(1.0f, unpacked.joystick.throttle, FLT_EPSILON);

  // Test roll/pitch/yaw/throttle mid-range (8-bit quantization).
  orig.joystick.roll = -0.21f;
  orig.joystick.pitch = 0.15f;
  orig.joystick.yaw = 0.23f;
  orig.joystick.throttle = 0.19f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(-0.21f, unpacked.joystick.roll, 2.0f / 256.0f);
  EXPECT_NEAR(0.15f, unpacked.joystick.pitch, 2.0f / 256.0f);
  EXPECT_NEAR(0.23f, unpacked.joystick.yaw, 2.0f / 256.0f);
  EXPECT_NEAR(0.19f, unpacked.joystick.throttle, 1.0f / 256.0f);

  // Test tri-stale switch (up).
  orig.joystick.tri_switch = kJoystickSwitchPositionUp;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(kJoystickSwitchPositionUp, unpacked.joystick.tri_switch);

  // Test tri-stale switch (middle).
  orig.joystick.tri_switch = kJoystickSwitchPositionMiddle;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(kJoystickSwitchPositionMiddle, unpacked.joystick.tri_switch);

  // Test tri-stale switch (down).
  orig.joystick.tri_switch = kJoystickSwitchPositionDown;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(kJoystickSwitchPositionDown, unpacked.joystick.tri_switch);

  // Test momentary switch (up).
  orig.joystick.momentary_switch = kJoystickSwitchPositionUp;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(kJoystickSwitchPositionUp, unpacked.joystick.momentary_switch);

  // Test momentary switch (down).
  orig.joystick.momentary_switch = kJoystickSwitchPositionDown;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(kJoystickSwitchPositionDown, unpacked.joystick.momentary_switch);

  // Test tether release interlock.
  orig.joystick.tether_release_interlock_code =
      TETHER_RELEASE_INTERLOCK_SAFETY_CODE;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.joystick.tether_release_interlock_code,
            unpacked.joystick.tether_release_interlock_code);
  orig.joystick.tether_release_interlock_code =
      TETHER_RELEASE_INTERLOCK_SAFETY_CODE - 1U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(0U, unpacked.joystick.tether_release_interlock_code);

  // Test scuttle.
  orig.joystick.scuttle_code = SCUTTLE_SAFETY_CODE;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.joystick.scuttle_code, unpacked.joystick.scuttle_code);
  orig.joystick.scuttle_code = SCUTTLE_SAFETY_CODE - 1U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(0U, unpacked.joystick.scuttle_code);
}

void TestTetherPlatform(uint16_t frame_index, TetherUpMessage *orig,
                        TetherPlatform *orig_platform,
                        TetherUpMessage *unpacked,
                        TetherPlatform *unpacked_platform) {
  memset(orig, 0, sizeof(*orig));
  memset(unpacked, 0x5A, sizeof(*unpacked));

  // Test sequence.
  orig->frame_index = frame_index;
  orig_platform->sequence = 20U;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(20U, unpacked_platform->sequence);

  // Test kTetherPlatformFlagPerchAzimuthFault flag.
  orig_platform->flags = kTetherPlatformFlagPerchAzimuthFault;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_TRUE(kTetherPlatformFlagPerchAzimuthFault & unpacked_platform->flags);
  orig_platform->flags &= (uint8_t)~kTetherPlatformFlagPerchAzimuthFault;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_FALSE(
      kTetherPlatformFlagPerchAzimuthFault & unpacked_platform->flags);

  // Test kTetherPlatformFlagLevelwindElevationFault flag.
  orig_platform->flags = kTetherPlatformFlagLevelwindElevationFault;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_TRUE(kTetherPlatformFlagLevelwindElevationFault
              & unpacked_platform->flags);
  orig_platform->flags &= (uint8_t)~kTetherPlatformFlagLevelwindElevationFault;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_FALSE(
      kTetherPlatformFlagLevelwindElevationFault & unpacked_platform->flags);

  // Test perch azimuth.
  orig_platform->perch_azi = 1.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_NEAR(1.3f, unpacked_platform->perch_azi, 1e-3f);
  orig_platform->perch_azi = -1.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_NEAR(-1.3f, unpacked_platform->perch_azi, 1e-3f);

  // Test levelwind elevation.
  orig_platform->levelwind_ele = 1.5f;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_NEAR(1.5f, unpacked_platform->levelwind_ele, 1e-3f);
  orig_platform->levelwind_ele = 0.5f;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_NEAR(0.5f, unpacked_platform->levelwind_ele, 1e-3f);
}

TEST(PackTetherUp, TetherPlatformA) {
  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, platform_a.sequence),
      SIZEOF(TetherUpMessage, platform_a.sequence)));

  // Decimated according to frame_index / dec == 0.
  TetherUpMessage orig, unpacked;
  TestTetherPlatform(20U * TETHER_RADIO_DECIMATION, &orig, &orig.platform_a,
                     &unpacked, &unpacked.platform_a);
}

TEST(PackTetherUp, TetherPlatformB) {
  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, platform_b.sequence),
      SIZEOF(TetherUpMessage, platform_b.sequence)));

  // Decimated according to frame_index / dec == 0.
  TetherUpMessage orig, unpacked;
  TestTetherPlatform(20U * TETHER_RADIO_DECIMATION, &orig, &orig.platform_b,
                     &unpacked, &unpacked.platform_b);
}

TEST(PackTetherUp, TetherDrumA) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, drum_a.sequence),
      SIZEOF(TetherUpMessage, drum_a.sequence)));

  // Test sequence.
  // Decimated according to frame_index / dec == 0.
  orig.frame_index = 20U * TETHER_RADIO_DECIMATION;
  orig.drum_a.sequence = 20U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(20U, unpacked.drum_a.sequence);

  // Test kTetherDrumFlagGsgAxis1Fault flag.
  orig.drum_a.flags = kTetherDrumFlagGsgAxis1Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherDrumFlagGsgAxis1Fault & unpacked.drum_a.flags);
  orig.drum_a.flags &= (uint8_t)~kTetherDrumFlagGsgAxis1Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherDrumFlagGsgAxis1Fault & unpacked.drum_a.flags);

  // Test kTetherDrumFlagGsgAxis2Fault flag.
  orig.drum_a.flags = kTetherDrumFlagGsgAxis2Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherDrumFlagGsgAxis2Fault & unpacked.drum_a.flags);
  orig.drum_a.flags &= (uint8_t)~kTetherDrumFlagGsgAxis2Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherDrumFlagGsgAxis2Fault & unpacked.drum_a.flags);

  // Test GSG axis 1.
  orig.drum_a.gsg_axis1 = 1.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(1.3f, unpacked.drum_a.gsg_axis1, 1e-3f);

  // Test GSG axis 2.
  orig.drum_a.gsg_axis2 = 0.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(0.3f, unpacked.drum_a.gsg_axis2, 1e-3f);
}

TEST(PackTetherUp, TetherDrumB) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, drum_b.sequence),
      SIZEOF(TetherUpMessage, drum_b.sequence)));

  // Test sequence.
  // Decimated according to frame_index / dec == 0.
  orig.frame_index = 21U * TETHER_RADIO_DECIMATION;
  orig.drum_b.sequence = 20U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(20U, unpacked.drum_b.sequence);

  // Test kTetherDrumFlagGsgAxis1Fault flag.
  orig.drum_b.flags = kTetherDrumFlagGsgAxis1Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherDrumFlagGsgAxis1Fault & unpacked.drum_b.flags);
  orig.drum_b.flags &= (uint8_t)~kTetherDrumFlagGsgAxis1Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherDrumFlagGsgAxis1Fault & unpacked.drum_b.flags);

  // Test kTetherDrumFlagGsgAxis2Fault flag.
  orig.drum_b.flags = kTetherDrumFlagGsgAxis2Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherDrumFlagGsgAxis2Fault & unpacked.drum_b.flags);
  orig.drum_b.flags &= (uint8_t)~kTetherDrumFlagGsgAxis2Fault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherDrumFlagGsgAxis2Fault & unpacked.drum_b.flags);

  // Test GSG axis 1.
  orig.drum_b.gsg_axis1 = 1.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(1.3f, unpacked.drum_b.gsg_axis1, 1e-3f);

  // Test GSG axis 2.
  orig.drum_b.gsg_axis2 = 0.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(0.3f, unpacked.drum_b.gsg_axis2, 1e-3f);
}

TEST(PackTetherUp, TetherPlc) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, plc.sequence),
      SIZEOF(TetherUpMessage, plc.sequence)));

  // Test sequence.
  // Decimated according to frame_index / dec == 0.
  orig.frame_index = 20U * TETHER_RADIO_DECIMATION;
  orig.plc.sequence = 20U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(20U, unpacked.plc.sequence);

  // Test kTetherPlcFlagPlcWarning flag.
  orig.plc.flags = kTetherPlcFlagPlcWarning;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherPlcFlagPlcWarning & unpacked.plc.flags);
  orig.plc.flags &= (uint8_t)~kTetherPlcFlagPlcWarning;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherPlcFlagPlcWarning & unpacked.plc.flags);

  // Test kTetherPlcFlagPlcError flag.
  orig.plc.flags = kTetherPlcFlagPlcError;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherPlcFlagPlcError & unpacked.plc.flags);
  orig.plc.flags &= (uint8_t)~kTetherPlcFlagPlcError;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherPlcFlagPlcError & unpacked.plc.flags);

  // Test kTetherPlcFlagDetwistFault flag.
  orig.plc.flags = kTetherPlcFlagDetwistFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherPlcFlagDetwistFault & unpacked.plc.flags);
  orig.plc.flags &= (uint8_t)~kTetherPlcFlagDetwistFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherPlcFlagDetwistFault & unpacked.plc.flags);

  // Test kTetherPlcFlagDrumFault flag.
  orig.plc.flags = kTetherPlcFlagDrumFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherPlcFlagDrumFault & unpacked.plc.flags);
  orig.plc.flags &= (uint8_t)~kTetherPlcFlagDrumFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherPlcFlagDrumFault & unpacked.plc.flags);

  // Test kTetherPlcFlagProximityFault flag.
  orig.plc.flags = kTetherPlcFlagProximityFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherPlcFlagProximityFault & unpacked.plc.flags);
  orig.plc.flags &= (uint8_t)~kTetherPlcFlagProximityFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherPlcFlagProximityFault & unpacked.plc.flags);

  // Test kTetherPlcProximity flags.
  orig.plc.proximity = (kTetherPlcProximityFinalA | kTetherPlcProximityFinalB
                        | kTetherPlcProximityEarlyA
                        | kTetherPlcProximityEarlyB);
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.plc.proximity, unpacked.plc.proximity);
  orig.plc.proximity = (kTetherPlcProximityFinalA | kTetherPlcProximityFinalB);
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.plc.proximity, unpacked.plc.proximity);
  orig.plc.proximity = (kTetherPlcProximityEarlyA | kTetherPlcProximityEarlyB);
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.plc.proximity, unpacked.plc.proximity);
  orig.plc.proximity = 0x0;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.plc.proximity, unpacked.plc.proximity);

  // Test detwist angle.
  orig.plc.detwist_angle = 2.0f * PI_F * TETHER_DETWIST_REVS - 0.1f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(orig.plc.detwist_angle, unpacked.plc.detwist_angle, 1e-3f);
  orig.plc.detwist_angle = 2.0f * PI_F * (TETHER_DETWIST_REVS + 0.5f);
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(PI_F, unpacked.plc.detwist_angle, 1e-3f);
  orig.plc.detwist_angle = 0.1f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(orig.plc.detwist_angle, unpacked.plc.detwist_angle, 1e-3f);
  orig.plc.detwist_angle = -2.0f * PI_F;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(2.0f * PI_F * (TETHER_DETWIST_REVS - 1.0f),
              unpacked.plc.detwist_angle, 1e-3f);

  // Test drum angle.
  orig.plc.drum_angle = -2.0f * PI_F * TETHER_DRUM_REVS + 0.2f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(orig.plc.drum_angle, unpacked.plc.drum_angle, 2e-3f);
  orig.plc.drum_angle = -0.2f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(orig.plc.drum_angle, unpacked.plc.drum_angle, 2e-3f);
}

TEST(PackTetherUp, TetherGroundStation) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, plc.sequence),
      SIZEOF(TetherUpMessage, plc.sequence)));

  // Test sequence.
  // Decimated according to frame_index / dec == 0.
  orig.frame_index = 20U * TETHER_RADIO_DECIMATION;
  orig.plc.sequence = 20U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(20U, unpacked.plc.sequence);

  // Test kTetherGroundStationFlag* flag.
  const int num_flags = 3;
  TetherGroundStationFlag flags[] = {
    kTetherGroundStationFlagError,
    kTetherGroundStationFlagDetwistError,
    kTetherGroundStationFlagDrumError
  };
  for (int n = 0; n < num_flags; n++) {
    TetherGroundStationFlag flag = flags[n];
    orig.ground_station.flags = flag;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_TRUE(flag & unpacked.ground_station.flags);
    orig.ground_station.flags &= (uint8_t)~flag;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_FALSE(flag & unpacked.ground_station.flags);
  }

  // Test mode.
  orig.ground_station.mode = kGroundStationModeHighTension;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.ground_station.mode, unpacked.ground_station.mode);

  // Test transform_stage.
  orig.ground_station.transform_stage = 3U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.ground_station.transform_stage,
            unpacked.ground_station.transform_stage);

  {  // Drum angle.
    std::vector<float> ins = {-251.0f, -250.0f, -240.0f, -10.0f, 0.0f, 1.0f};
    std::vector<float> outs = {-250.0f, -250.0f, -240.0f, -10.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < ins.size(); ++i) {
      orig.ground_station.drum_angle = ins[i];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(outs[i], unpacked.ground_station.drum_angle, 2e-3f);
    }
  }

  {  // Detwist azimuth.
    std::vector<float> ins = {-1.1f, 1.1f};
    std::vector<float> outs = {-1.1f, 1.1f};
    for (size_t i = 0; i < ins.size(); ++i) {
      orig.ground_station.detwist_angle = ins[i];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(outs[i], unpacked.ground_station.detwist_angle, 2e-3f);
    }
  }

  {  // Proximity.
    std::vector<uint8_t> ins = {0U, 1U};
    std::vector<uint8_t> outs = {0U, 1U};
    for (size_t i = 0; i < ins.size(); ++i) {
      orig.ground_station.proximity = ins[i];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(outs[i], unpacked.ground_station.proximity);
    }
  }

  {  // Tether engagement.
    std::vector<uint8_t> ins = {0U, 1U};
    std::vector<uint8_t> outs = {0U, 1U};
    for (size_t i = 0; i < ins.size(); ++i) {
      orig.ground_station.tether_engaged = ins[i];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(outs[i], unpacked.ground_station.tether_engaged);
    }
  }
}

template<typename T>
void TestTetherGpsTime(uint16_t frame_index) {
  T orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  orig.frame_index = frame_index;

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<T>(OFFSETOF(T, gps_time.time_of_week),
                                 SIZEOF(T, gps_time.time_of_week)));

  // Test GPS time of week.
  orig.gps_time.time_of_week = 1;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(1, unpacked.gps_time.time_of_week);
  orig.gps_time.time_of_week = 2;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(2, unpacked.gps_time.time_of_week);
}

template<typename T>
void TestTetherGpsStatus(uint16_t frame_index, T *orig,
                         TetherGpsStatus *orig_gps_status, T *unpacked,
                         TetherGpsStatus *unpacked_gps_status) {
  memset(orig, 0, sizeof(*orig));
  memset(unpacked, 0x5A, sizeof(*unpacked));

  // Test sequence.
  orig->frame_index = frame_index;
  orig_gps_status->sequence = 50U;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(50U, unpacked_gps_status->sequence);

  // Test kTetherGpsSolutionStatus.
  orig_gps_status->status = kTetherGpsSolutionStatusRtkFloat;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(kTetherGpsSolutionStatusRtkFloat, unpacked_gps_status->status);
  orig_gps_status->status = kTetherGpsSolutionStatusNone;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(kTetherGpsSolutionStatusNone, unpacked_gps_status->status);

  // Test number of satellites.
  orig_gps_status->satellites = 1U;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(1U, unpacked_gps_status->satellites);
  orig_gps_status->satellites = 31U;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(31U, unpacked_gps_status->satellites);

  // Test position sigma.
  orig_gps_status->pos_sigma = 0.1f;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_NEAR(0.1f, unpacked_gps_status->pos_sigma, 1e-2f);
  orig_gps_status->pos_sigma = 50.0f;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_NEAR(50.0f, unpacked_gps_status->pos_sigma, 1e-2f);

  // Test C/N0.
  orig_gps_status->avg_cn0 = 1;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(1, unpacked_gps_status->avg_cn0);
  orig_gps_status->avg_cn0 = 52;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(52, unpacked_gps_status->avg_cn0);
}

TEST(PackTetherUp, TetherGpsTime) {
  // Decimated according to (frame_index / dec) % 4 == 0.
  TestTetherGpsTime<TetherUpMessage>(32U * TETHER_RADIO_DECIMATION);
}

TEST(PackTetherUp, TetherGpsStatus) {
  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, gps_status.sequence),
      SIZEOF(TetherUpMessage, gps_status.sequence)));

  // Decimated according to (frame_index / dec) % 4 == 0.
  TetherUpMessage orig, unpacked;
  TestTetherGpsStatus(32U * TETHER_RADIO_DECIMATION, &orig, &orig.gps_status,
                      &unpacked, &unpacked.gps_status);
}

TEST(PackTetherUp, TetherGsGpsCompass) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, gps_compass.sequence),
      SIZEOF(TetherUpMessage, gps_compass.sequence)));

  // Test sequence.
  // Decimated according to (frame_index / dec) % 4 == 1, 3.
  orig.frame_index = 33U * TETHER_RADIO_DECIMATION;
  orig.gps_compass.sequence = 50U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(50U, unpacked.gps_compass.sequence);

  // Test kTetherGsGpsCompassFlagFault flag.
  orig.gps_compass.flags = kTetherGsGpsCompassFlagFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherGsGpsCompassFlagFault & unpacked.gps_compass.flags);
  orig.gps_compass.flags &= (uint8_t)~kTetherGsGpsCompassFlagFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherGsGpsCompassFlagFault & unpacked.gps_compass.flags);

  // Test heading.
  orig.gps_compass.heading = 1.2f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(1.2f, unpacked.gps_compass.heading, 1e-3f);
  orig.gps_compass.heading = -1.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(-1.3f, unpacked.gps_compass.heading, 1e-3f);

  // Test heading sigma.
  orig.gps_compass.heading_sigma = 1.2f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(1.2f, unpacked.gps_compass.heading_sigma, 1e-3f);
  orig.gps_compass.heading_sigma = 1.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(1.3f, unpacked.gps_compass.heading_sigma, 1e-3f);

  // Test heading rate.
  orig.gps_compass.heading_rate = 1.2f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(1.2f, unpacked.gps_compass.heading_rate, 1e-3f);
  orig.gps_compass.heading_rate = -1.3f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(-1.3f, unpacked.gps_compass.heading_rate, 1e-3f);
}

TEST(PackTetherUp, TetherWind) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, wind.sequence),
      SIZEOF(TetherUpMessage, wind.sequence)));

  // Test sequence.
  // Decimated according to frame_index / dec == 0.
  orig.frame_index = 20U * TETHER_RADIO_DECIMATION;
  orig.wind.sequence = 50U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(50U, unpacked.wind.sequence);

  // Test status.
  orig.wind.status = kTetherWindStatusGood;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(kTetherWindStatusGood, unpacked.wind.status);
  orig.wind.status = kTetherWindStatusFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(kTetherWindStatusFault, unpacked.wind.status);

  // Test velocity.
  for (int32_t i = 0; i < 3; ++i) {
    orig.wind.velocity[i] = 1.2f;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(1.2f, unpacked.wind.velocity[i], 1e-3f);
    orig.wind.velocity[i] = -1.3f;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(-1.3f, unpacked.wind.velocity[i], 1e-3f);
  }
}

TEST(PackTetherUp, TetherWeather) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherUpMessage>(
      OFFSETOF(TetherUpMessage, weather.sequence),
      SIZEOF(TetherUpMessage, weather.sequence)));

  // Test sequence.
  // Decimated according to (frame_index / dec) % 4 == 3.
  orig.frame_index = 35U * TETHER_RADIO_DECIMATION;
  orig.weather.sequence = 50U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(50U, unpacked.weather.sequence);

  // Test kTetherFlagFault flag.
  orig.weather.flags = kTetherWeatherFlagFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_TRUE(kTetherWeatherFlagFault & unpacked.weather.flags);
  orig.weather.flags &= (uint8_t)~kTetherWeatherFlagFault;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_FALSE(kTetherWeatherFlagFault & unpacked.weather.flags);

  // Test pressure.
  orig.weather.pressure_pa = 103000.0f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(103000.0f, unpacked.weather.pressure_pa, 0.1f);
  orig.weather.pressure_pa = 90000.0f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(90000.0f, unpacked.weather.pressure_pa, 0.1f);

  // Test temperature.
  orig.weather.temperature = -5.0f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(-5.0f, unpacked.weather.temperature, 0.1f);
  orig.weather.temperature = 90.0f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(90.0f, unpacked.weather.temperature, 0.1f);

  // Test humidity.
  orig.weather.humidity = 50.0f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(50.0f, unpacked.weather.humidity, 1.6f);
  orig.weather.humidity = 95.0f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(95.0f, unpacked.weather.humidity, 1.6f);
}

TEST(PackTetherUp, TetherOpBattCommand) {
  BattCommandMessage m_in, m_out;

  // Test disconnect A.
  m_in.batt_signal = BATT_DISCONNECT_A_SIGNAL;
  m_in.state_command = kBattStateCommandDisconnectA;
  PackUnpackOpCommand(kTetherOpTypeBattCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.batt_signal, m_out.batt_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);

  // Test disconnect B.
  m_in.batt_signal = BATT_DISCONNECT_B_SIGNAL;
  m_in.state_command = kBattStateCommandDisconnectB;
  PackUnpackOpCommand(kTetherOpTypeBattCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.batt_signal, m_out.batt_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);

  // Test clear errors.
  m_in.batt_signal = BATT_CLEAR_ERRORS_SIGNAL;
  m_in.state_command = kBattStateCommandClearErrors;
  PackUnpackOpCommand(kTetherOpTypeBattCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.batt_signal, m_out.batt_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);

  // Test connect.
  m_in.batt_signal = BATT_CONNECT_SIGNAL;
  m_in.state_command = kBattStateCommandConnect;
  PackUnpackOpCommand(kTetherOpTypeBattCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.batt_signal, m_out.batt_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);
}

TEST(PackTetherUp, TetherOpFlightCommand) {
  FlightCommandMessage m_in, m_out;

  // Test force hover accel.
  m_in.safety_code = FLIGHT_COMMAND_SIGNAL;
  m_in.force_hover_accel = true;
  m_in.force_high_tension = false;
  m_in.force_reel = false;
  m_in.gs_unpause_transform = false;
  m_in.force_detwist_turn_once = false;
  m_in.experiment_type = kExperimentTypeNoTest;
  m_in.experiment_case_id = 0U;
  PackUnpackOpCommand(kTetherOpTypeFlightCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.force_hover_accel, m_out.force_hover_accel);
  EXPECT_EQ(m_in.force_high_tension, m_out.force_high_tension);
  EXPECT_EQ(m_in.force_reel, m_out.force_reel);
  EXPECT_EQ(m_in.gs_unpause_transform, m_out.gs_unpause_transform);
  EXPECT_EQ(m_in.force_detwist_turn_once, m_out.force_detwist_turn_once);
  EXPECT_EQ(m_in.experiment_type, m_out.experiment_type);
  EXPECT_EQ(m_in.experiment_case_id, m_out.experiment_case_id);

  // Test no forcing.
  m_in.safety_code = FLIGHT_COMMAND_SIGNAL;
  m_in.force_hover_accel = false;
  m_in.force_high_tension = false;
  m_in.force_reel = false;
  m_in.gs_unpause_transform = false;
  m_in.force_detwist_turn_once = false;
  m_in.experiment_type = kExperimentTypeNoTest;
  m_in.experiment_case_id = 0U;
  PackUnpackOpCommand(kTetherOpTypeFlightCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.force_hover_accel, m_out.force_hover_accel);
  EXPECT_EQ(m_in.force_high_tension, m_out.force_high_tension);
  EXPECT_EQ(m_in.force_reel, m_out.force_reel);
  EXPECT_EQ(m_in.gs_unpause_transform, m_out.gs_unpause_transform);
  EXPECT_EQ(m_in.force_detwist_turn_once, m_out.force_detwist_turn_once);
  EXPECT_EQ(m_in.experiment_type, m_out.experiment_type);
  EXPECT_EQ(m_in.experiment_case_id, m_out.experiment_case_id);

  // Test force high tension.
  m_in.safety_code = FLIGHT_COMMAND_SIGNAL;
  m_in.force_hover_accel = false;
  m_in.force_high_tension = true;
  m_in.force_reel = false;
  m_in.gs_unpause_transform = false;
  m_in.force_detwist_turn_once = false;
  m_in.experiment_type = kExperimentTypeNoTest;
  m_in.experiment_case_id = 0U;
  PackUnpackOpCommand(kTetherOpTypeFlightCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.force_hover_accel, m_out.force_hover_accel);
  EXPECT_EQ(m_in.force_high_tension, m_out.force_high_tension);
  EXPECT_EQ(m_in.force_reel, m_out.force_reel);
  EXPECT_EQ(m_in.gs_unpause_transform, m_out.gs_unpause_transform);
  EXPECT_EQ(m_in.force_detwist_turn_once, m_out.force_detwist_turn_once);
  EXPECT_EQ(m_in.experiment_type, m_out.experiment_type);
  EXPECT_EQ(m_in.experiment_case_id, m_out.experiment_case_id);

  // Test force reel.
  m_in.safety_code = FLIGHT_COMMAND_SIGNAL;
  m_in.force_hover_accel = false;
  m_in.force_high_tension = false;
  m_in.force_reel = true;
  m_in.gs_unpause_transform = false;
  m_in.force_detwist_turn_once = false;
  m_in.experiment_type = kExperimentTypeNoTest;
  m_in.experiment_case_id = 0U;
  PackUnpackOpCommand(kTetherOpTypeFlightCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.force_hover_accel, m_out.force_hover_accel);
  EXPECT_EQ(m_in.force_high_tension, m_out.force_high_tension);
  EXPECT_EQ(m_in.force_reel, m_out.force_reel);
  EXPECT_EQ(m_in.gs_unpause_transform, m_out.gs_unpause_transform);
  EXPECT_EQ(m_in.force_detwist_turn_once, m_out.force_detwist_turn_once);
  EXPECT_EQ(m_in.experiment_type, m_out.experiment_type);
  EXPECT_EQ(m_in.experiment_case_id, m_out.experiment_case_id);

  // Test unpause.
  m_in.safety_code = FLIGHT_COMMAND_SIGNAL;
  m_in.force_hover_accel = false;
  m_in.force_high_tension = false;
  m_in.force_reel = false;
  m_in.gs_unpause_transform = true;
  m_in.force_detwist_turn_once = false;
  m_in.experiment_type = kExperimentTypeNoTest;
  m_in.experiment_case_id = 0U;
  PackUnpackOpCommand(kTetherOpTypeFlightCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.force_hover_accel, m_out.force_hover_accel);
  EXPECT_EQ(m_in.force_high_tension, m_out.force_high_tension);
  EXPECT_EQ(m_in.force_reel, m_out.force_reel);
  EXPECT_EQ(m_in.gs_unpause_transform, m_out.gs_unpause_transform);
  EXPECT_EQ(m_in.force_detwist_turn_once, m_out.force_detwist_turn_once);
  EXPECT_EQ(m_in.experiment_type, m_out.experiment_type);
  EXPECT_EQ(m_in.experiment_case_id, m_out.experiment_case_id);

  // Test detwist align.
  m_in.safety_code = FLIGHT_COMMAND_SIGNAL;
  m_in.force_hover_accel = false;
  m_in.force_high_tension = false;
  m_in.force_reel = false;
  m_in.gs_unpause_transform = false;
  m_in.force_detwist_turn_once = true;
  m_in.experiment_type = kExperimentTypeNoTest;
  m_in.experiment_case_id = 0U;
  PackUnpackOpCommand(kTetherOpTypeFlightCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.force_hover_accel, m_out.force_hover_accel);
  EXPECT_EQ(m_in.force_high_tension, m_out.force_high_tension);
  EXPECT_EQ(m_in.force_reel, m_out.force_reel);
  EXPECT_EQ(m_in.gs_unpause_transform, m_out.gs_unpause_transform);
  EXPECT_EQ(m_in.force_detwist_turn_once, m_out.force_detwist_turn_once);
  EXPECT_EQ(m_in.experiment_type, m_out.experiment_type);
  EXPECT_EQ(m_in.experiment_case_id, m_out.experiment_case_id);

  // Test experiment cases.
  m_in.safety_code = FLIGHT_COMMAND_SIGNAL;
  m_in.force_hover_accel = false;
  m_in.force_high_tension = false;
  m_in.force_reel = false;
  m_in.gs_unpause_transform = false;
  m_in.force_detwist_turn_once = true;
  m_in.experiment_type = kExperimentTypeHoverElevator;
  m_in.experiment_case_id = 2U;
  PackUnpackOpCommand(kTetherOpTypeFlightCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.force_hover_accel, m_out.force_hover_accel);
  EXPECT_EQ(m_in.force_high_tension, m_out.force_high_tension);
  EXPECT_EQ(m_in.force_reel, m_out.force_reel);
  EXPECT_EQ(m_in.gs_unpause_transform, m_out.gs_unpause_transform);
  EXPECT_EQ(m_in.force_detwist_turn_once, m_out.force_detwist_turn_once);
  EXPECT_EQ(m_in.experiment_type, m_out.experiment_type);
  EXPECT_EQ(m_in.experiment_case_id, m_out.experiment_case_id);
}

TEST(PackTetherUp, TetherOpFpvSetState) {
  FpvSetStateMessage m_in, m_out;

  // Test enable.
  m_in.safety_code = FPV_ENABLE_SIGNAL;
  m_in.enable = true;
  PackUnpackOpCommand(kTetherOpTypeFpvSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.enable, m_out.enable);

  // Test disable.
  m_in.safety_code = FPV_DISABLE_SIGNAL;
  m_in.enable = false;
  PackUnpackOpCommand(kTetherOpTypeFpvSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.enable, m_out.enable);
}

TEST(PackTetherUp, TetherOpMotorGetParam) {
  MotorGetParamMessage m_in, m_out;

  m_in.selected_motors = (1U << kNumMotors) - 1U;
  m_in.id = 0x32;
  PackUnpackOpCommand(kTetherOpTypeMotorGetParam, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.selected_motors, m_out.selected_motors);
  EXPECT_EQ(m_in.id, m_out.id);
}

TEST(PackTetherUp, TetherOpMotorSetParam) {
  MotorSetParamMessage m_in, m_out;

  m_in.selected_motors = (1U << kNumMotors) - 1U;
  m_in.id = 0x12;
  m_in.value = 123.31f;
  PackUnpackOpCommand(kTetherOpTypeMotorSetParam, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.selected_motors, m_out.selected_motors);
  EXPECT_EQ(m_in.id, m_out.id);
  EXPECT_EQ(m_in.value, m_out.value);
}

TEST(PackTetherUp, TetherOpMotorSetState) {
  MotorSetStateMessage m_in, m_out;

  // Test arming.
  m_in.command = kActuatorStateCommandArm;
  m_in.command_data = MOTOR_ARMING_SIGNAL;
  m_in.selected_motors = 0x0F;
  PackUnpackOpCommand(kTetherOpTypeMotorSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.command, m_out.command);
  EXPECT_EQ(m_in.command_data, m_out.command_data);
  EXPECT_EQ(m_in.selected_motors, m_out.selected_motors);

  // Test disarm.
  m_in.command = kActuatorStateCommandDisarm;
  m_in.command_data = MOTOR_DISARMING_SIGNAL;
  m_in.selected_motors = 0xF0;
  PackUnpackOpCommand(kTetherOpTypeMotorSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.command, m_out.command);
  EXPECT_EQ(m_in.command_data, m_out.command_data);
  EXPECT_EQ(m_in.selected_motors, m_out.selected_motors);
}

TEST(PackTetherUp, TetherOpMvlvCommand) {
  MvlvCommandMessage m_in, m_out;

  // Test enable.
  m_in.mvlv_signal = MVLV_ENABLE_SIGNAL;
  m_in.state_command = kMvlvStateCommandEnable;
  PackUnpackOpCommand(kTetherOpTypeMvlvCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.mvlv_signal, m_out.mvlv_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);

  // Test disable.
  m_in.mvlv_signal = MVLV_DISABLE_SIGNAL;
  m_in.state_command = kMvlvStateCommandDisable;
  PackUnpackOpCommand(kTetherOpTypeMvlvCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.mvlv_signal, m_out.mvlv_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);

  // Test connect.
  m_in.mvlv_signal = MVLV_CONNECT_SIGNAL;
  m_in.state_command = kMvlvStateCommandConnect;
  PackUnpackOpCommand(kTetherOpTypeMvlvCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.mvlv_signal, m_out.mvlv_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);

  // Test disconnect.
  m_in.mvlv_signal = MVLV_DISCONNECT_SIGNAL;
  m_in.state_command = kMvlvStateCommandDisconnect;
  PackUnpackOpCommand(kTetherOpTypeMvlvCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.mvlv_signal, m_out.mvlv_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);

  // Test clear errors.
  m_in.mvlv_signal = MVLV_CLEAR_ERRORS_SIGNAL;
  m_in.state_command = kMvlvStateCommandClearErrors;
  PackUnpackOpCommand(kTetherOpTypeMvlvCommand, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.mvlv_signal, m_out.mvlv_signal);
  EXPECT_EQ(m_in.state_command, m_out.state_command);
}

TEST(PackTetherUp, TetherOpPitotSetState) {
  PitotSetStateMessage m_in, m_out;

  // Test cover.
  m_in.safety_code = PITOT_COVER_SIGNAL;
  m_in.cover = true;
  PackUnpackOpCommand(kTetherOpTypePitotSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.cover, m_out.cover);

  // Test uncover.
  m_in.safety_code = PITOT_UNCOVER_SIGNAL;
  m_in.cover = false;
  PackUnpackOpCommand(kTetherOpTypePitotSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.safety_code, m_out.safety_code);
  EXPECT_EQ(m_in.cover, m_out.cover);
}

TEST(PackTetherUp, TetherOpServoGetParam) {
  ServoGetParamMessage m_in, m_out;

  m_in.selected_servos = (1U << kNumServos) - 1U;
  m_in.param = 0x1232;
  PackUnpackOpCommand(kTetherOpTypeServoGetParam, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.selected_servos, m_out.selected_servos);
  EXPECT_EQ(m_in.param, m_out.param);
}

TEST(PackTetherUp, TetherOpServoSetParam) {
  ServoSetParamMessage m_in, m_out;

  m_in.selected_servos = (1U << kNumServos) - 1U;
  m_in.param = 0x1232;
  m_in.value = 0xDEADC0DE;
  PackUnpackOpCommand(kTetherOpTypeServoSetParam, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.selected_servos, m_out.selected_servos);
  EXPECT_EQ(m_in.param, m_out.param);
  EXPECT_EQ(m_in.value, m_out.value);
}

TEST(PackTetherUp, TetherOpServoSetState) {
  ServoSetStateMessage m_in, m_out;

  // Test arming.
  m_in.state_command = kActuatorStateCommandArm;
  m_in.servo_arming_signal = SERVO_ARMING_SIGNAL;
  m_in.selected_servos = 0x0F;
  PackUnpackOpCommand(kTetherOpTypeServoSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.state_command, m_out.state_command);
  EXPECT_EQ(m_in.servo_arming_signal, m_out.servo_arming_signal);
  EXPECT_EQ(m_in.selected_servos, m_out.selected_servos);

  // Test disarm.
  m_in.state_command = kActuatorStateCommandDisarm;
  m_in.servo_arming_signal = SERVO_DISARM_SIGNAL;
  m_in.selected_servos = 0x03FF;
  PackUnpackOpCommand(kTetherOpTypeServoSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.state_command, m_out.state_command);
  EXPECT_EQ(m_in.servo_arming_signal, m_out.servo_arming_signal);
  EXPECT_EQ(m_in.selected_servos, m_out.selected_servos);
}

TEST(PackTetherUp, TetherOpTetherReleaseSetState) {
  TetherReleaseSetStateMessage m_in, m_out;

  // Test arming.
  m_in.state_command = kActuatorStateCommandArm;
  m_in.arming_signal = TETHER_RELEASE_ARMING_SIGNAL;
  m_in.selected_loadcells = 0x0F;
  PackUnpackOpCommand(kTetherOpTypeTetherReleaseSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.state_command, m_out.state_command);
  EXPECT_EQ(m_in.arming_signal, m_out.arming_signal);
  EXPECT_EQ(m_in.selected_loadcells, m_out.selected_loadcells);

  // Test disarm.
  m_in.state_command = kActuatorStateCommandDisarm;
  m_in.arming_signal = TETHER_RELEASE_DISARM_SIGNAL;
  m_in.selected_loadcells = 0x03;
  PackUnpackOpCommand(kTetherOpTypeTetherReleaseSetState, 3259U, m_in, &m_out);
  EXPECT_EQ(m_in.state_command, m_out.state_command);
  EXPECT_EQ(m_in.arming_signal, m_out.arming_signal);
  EXPECT_EQ(m_in.selected_loadcells, m_out.selected_loadcells);
}

TEST(PackTetherUp, TetherOpQueueMultiple) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Decimated according to (frame_index / dec) % 4 == 1.
  orig.frame_index = 33U * TETHER_RADIO_DECIMATION;

  TetherOpQueue q_in, q_out;
  TetherOpInit(&q_in);
  TetherOpInit(&q_out);

  // We expect to transmit at least two MotorGetParamMessages (an arbitrary
  // small message) per TetherUpPackedMessage.
  AioNode source_in = kAioNodeOperator;
  TetherOpType type_in = kTetherOpTypeMotorGetParam;
  MotorGetParamMessage m_in;
  m_in.selected_motors = 1U;
  m_in.id = 1U;
  PushOpQueue(source_in, type_in, 1U, m_in, &q_in);
  m_in.selected_motors = 2U;
  m_in.id = 2U;
  PushOpQueue(source_in, type_in, 2U, m_in, &q_in);

  // Each pack/unpack counts as one transmit.
  EXPECT_TRUE(PackUnpackTetherMessage(&q_in, &orig, &q_out, &unpacked));

  // Check first message.
  AioNode source_out;
  TetherOpType type_out;
  uint16_t sequence_out;
  MotorGetParamMessage m_out;
  PopOpQueue(&q_out, &source_out, &type_out, &sequence_out, &m_out);
  EXPECT_EQ(source_in, source_out);
  EXPECT_EQ(type_in, type_in);
  EXPECT_EQ(1U, sequence_out);
  EXPECT_EQ(1U, m_out.selected_motors);
  EXPECT_EQ(1U, m_out.id);

  // Check second message.
  PopOpQueue(&q_out, &source_out, &type_out, &sequence_out, &m_out);
  EXPECT_EQ(source_in, source_out);
  EXPECT_EQ(type_in, type_in);
  EXPECT_EQ(2U, sequence_out);
  EXPECT_EQ(2U, m_out.selected_motors);
  EXPECT_EQ(2U, m_out.id);
}

TEST(PackTetherUp, TetherOpQueueOverflow) {
  TetherUpMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Decimated according to (frame_index / dec) % 4 == 1.
  orig.frame_index = 33U * TETHER_RADIO_DECIMATION;

  TetherOpQueue q_in, q_out;
  TetherOpInit(&q_in);
  TetherOpInit(&q_out);

  // Create input messages.
  MotorGetParamMessage m_in;
  AioNode source_in = kAioNodeOperator;
  TetherOpType type_in = kTetherOpTypeMotorGetParam;
  for (int32_t i = 0; i < TETHER_OP_QUEUE_SIZE; ++i) {
    m_in.selected_motors = 1U;
    m_in.id = static_cast<uint8_t>(i);
    PushOpQueue(source_in, type_in, static_cast<uint16_t>(i), m_in, &q_in);
  }

  uint16_t expected_sequence = 0U;
  while (TetherOpIsPending(&q_in)) {
    // Transmit as many messages as possible per pack/unpack.
    EXPECT_TRUE(PackUnpackTetherMessage(&q_in, &orig, &q_out, &unpacked));

    // Check output messages.
    while (TetherOpIsPending(&q_out)) {
      AioNode source_out;
      TetherOpType type_out;
      uint16_t sequence_out;
      MotorGetParamMessage m_out;
      PopOpQueue(&q_out, &source_out, &type_out, &sequence_out, &m_out);

      EXPECT_EQ(source_in, source_out);
      EXPECT_EQ(type_in, type_out);
      EXPECT_EQ(expected_sequence, sequence_out);

      ++expected_sequence;
    }
  }
  EXPECT_EQ(TETHER_OP_QUEUE_SIZE, expected_sequence);
}

TEST(PackTetherDown, Frame) {
  TetherDownMessage orig, unpacked;
  TetherDownPackedMessage packed;

  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));
  memset(&packed, 0xC3, sizeof(packed));

  // Test frame index. Iterating through multiple frame indicies since the
  // packing multiplexes streams according to the frame index.
  for (int32_t i = 100; i < 200; ++i) {
    orig.frame_index = (uint8_t)i;
    EXPECT_TRUE(PackTetherDown(nullptr, &orig, &packed));
    EXPECT_TRUE(UnpackTetherDown(nullptr, &packed, &unpacked));
    EXPECT_EQ(orig.frame_index, unpacked.frame_index);
  }

  // Test received frame_index.
  orig.received_frame_index = 50U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(50U, unpacked.received_frame_index);
  orig.received_frame_index = 200U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(200U, unpacked.received_frame_index);

  // Invalidate CRC.
  packed.data[0] = (uint8_t)~packed.data[0];
  EXPECT_FALSE(UnpackTetherDown(nullptr, &packed, &unpacked));
}

TEST(PackTetherDown, TetherControlCommand) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_command.sequence),
      SIZEOF(TetherDownMessage, control_command.sequence)));

  // Test detwist command.
  orig.control_command.detwist_angle = 3.2f;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(3.2f, unpacked.control_command.detwist_angle, 1e-3f);
  orig.control_command.detwist_angle
      = 2.0f * PI_F * (TETHER_DETWIST_REVS - 0.1f);
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(2.0f * PI_F * (TETHER_DETWIST_REVS - 0.1f),
              unpacked.control_command.detwist_angle, 1e-3f);

  // Winch velocity command.
  orig.control_command.winch_velocity = PI_F;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(orig.control_command.winch_velocity,
              unpacked.control_command.winch_velocity, 1e-3f);
  orig.control_command.winch_velocity = -PI_F;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_NEAR(orig.control_command.winch_velocity,
              unpacked.control_command.winch_velocity, 1e-3f);

  {  // GS target azimuth.
    std::vector<float> ins = {-PI_F - 0.1f, -PI_F + 0.1f, PI_F - 0.1f,
                              PI_F + 0.1f};
    std::vector<float> outs = {PI_F - 0.1f, -PI_F + 0.1f, PI_F - 0.1f,
                               -PI_F + 0.1};
    for (size_t i = 0U; i < ins.size(); ++i) {
      orig.control_command.gs_azi_target = ins[i];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(outs[i], unpacked.control_command.gs_azi_target,
                  0.01f);
    }
  }

  // GS azimuth dead zone. The tighter tolerance reflects the fact that we want
  // to "almost exactly" reproduce the angles of interest.
  {
    std::vector<double> ins_deg = {-0.05, 0.25, 1.0, 12.0, 13.0};
    std::vector<double> outs_deg = {0.0, 0.25, 1.0, 12.0, 12.75};

    for (size_t i = 0U; i < ins_deg.size(); ++i) {
      orig.control_command.gs_azi_dead_zone = static_cast<float>(
          DegToRad(ins_deg[i]));
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(static_cast<float>(DegToRad(outs_deg[i])),
                  unpacked.control_command.gs_azi_dead_zone, 1e-6f);
    }
  }

  // GS mode command.
  orig.control_command.gs_mode_request = kGroundStationModeManual;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.control_command.gs_mode_request,
            unpacked.control_command.gs_mode_request);
  orig.control_command.gs_mode_request = kGroundStationModeHighTension;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.control_command.gs_mode_request,
            unpacked.control_command.gs_mode_request);
  orig.control_command.gs_mode_request = kGroundStationModeReel;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.control_command.gs_mode_request,
            unpacked.control_command.gs_mode_request);
  orig.control_command.gs_mode_request = kGroundStationModeTransform;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.control_command.gs_mode_request,
            unpacked.control_command.gs_mode_request);
}

TEST(PackTetherDown, TetherControlTelemetry) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_telemetry.sequence),
      SIZEOF(TetherDownMessage, control_telemetry.sequence)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_telemetry.flight_mode_time),
      SIZEOF(TetherDownMessage, control_telemetry.flight_mode_time)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_telemetry.pos_g[0]),
      SIZEOF(TetherDownMessage, control_telemetry.pos_g[0])));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_telemetry.vel_g[0]),
      SIZEOF(TetherDownMessage, control_telemetry.vel_g[0])));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_telemetry.tension),
      SIZEOF(TetherDownMessage, control_telemetry.tension)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_telemetry.thrust),
      SIZEOF(TetherDownMessage, control_telemetry.thrust)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, control_telemetry.moment[0]),
      SIZEOF(TetherDownMessage, control_telemetry.moment[0])));

  // TetherControlTelemetry updates twice per cycle. We multiplex several
  // fields according to each update.
  for (int32_t part = 0; part < 50; ++part) {
    // Decimated according to (frame_index / dec) % 4 == 1, 3.
    orig.frame_index = (uint16_t)(TETHER_RADIO_DECIMATION * (1 + 2 * part));

    // Test sequence.
    orig.control_telemetry.sequence = 50U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.sequence,
              unpacked.control_telemetry.sequence);

    // Test controller label.
    orig.control_command.controller_label = 2;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_command.controller_label,
              unpacked.control_command.controller_label);

    // Test flags.
    orig.control_telemetry.flags = 0x0;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(0x0, unpacked.control_telemetry.flags);
    orig.control_telemetry.flags |= kTetherControlTelemetryFlagAutoGlideActive;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_TRUE(unpacked.control_telemetry.flags
                & kTetherControlTelemetryFlagAutoGlideActive);
    orig.control_telemetry.flags |= kTetherControlTelemetryFlagReleaseLatched;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_TRUE(unpacked.control_telemetry.flags
                & kTetherControlTelemetryFlagReleaseLatched);
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));

    // Test subsystem.
    orig.control_telemetry.subsystem = kSubsysControllerA;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.subsystem,
              unpacked.control_telemetry.subsystem);
    orig.control_telemetry.subsystem = kNumSubsystems - 1;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.subsystem,
              unpacked.control_telemetry.subsystem);

    // Test subsystem faults.
    orig.control_telemetry.subsystem_faults = 0x1A;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.subsystem_faults,
              unpacked.control_telemetry.subsystem_faults);
    orig.control_telemetry.subsystem_faults = 0x01;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.subsystem_faults,
              unpacked.control_telemetry.subsystem_faults);
    orig.control_telemetry.subsystem_faults = (1U << kNumFaultTypes) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.subsystem_faults,
              unpacked.control_telemetry.subsystem_faults);

    // Test flight mode.
    orig.control_telemetry.flight_mode = 5U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode,
              unpacked.control_telemetry.flight_mode);
    orig.control_telemetry.flight_mode = 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode,
              unpacked.control_telemetry.flight_mode);

    // Test flight mode gates.
    orig.control_telemetry.flight_mode_gates = 0x04A5;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates = 0x0100;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);

    // Test hover flight mode gates.
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumHoverPerchedGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumHoverAscendGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumHoverPayOutGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumHoverFullLengthGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumHoverAccelGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumHoverReelInGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumHoverDescendGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);

    // Test trans-in flight mode gates.
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumTransInGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);

    // Test crosswind flight mode gates.
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumCrosswindNormalGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumCrosswindPrepTransOutGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);
    orig.control_telemetry.flight_mode_gates =
        (1U << kNumCrosswindHoverTransOutGates) - 1U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.control_telemetry.flight_mode_gates,
              unpacked.control_telemetry.flight_mode_gates);

    if (part % 3 == 0) {
      // Test flight mode time.
      orig.control_telemetry.flight_mode_time = (1 << 20) - 1;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(orig.control_telemetry.flight_mode_time,
                unpacked.control_telemetry.flight_mode_time);
      orig.control_telemetry.flight_mode_time = 1;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(orig.control_telemetry.flight_mode_time,
                unpacked.control_telemetry.flight_mode_time);
    } else if (part % 3 == 1) {
      // Test loop time.
      orig.control_telemetry.loop_time = 0.01234f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.loop_time,
                  unpacked.control_telemetry.loop_time, 1e-6f);
      orig.control_telemetry.loop_time = 0.0f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.loop_time,
                  unpacked.control_telemetry.loop_time, 1e-6f);

      // Test loop angle.
      orig.control_telemetry.loop_angle = 2.0f * PI_F;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(0.0f, unpacked.control_telemetry.loop_angle, 1.5e-2f);
      orig.control_telemetry.loop_angle = -PI_F + 0.1f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.loop_angle,
                  unpacked.control_telemetry.loop_angle, 1.5e-2f);
      orig.control_telemetry.loop_angle = PI_F - 0.1f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.loop_angle,
                  unpacked.control_telemetry.loop_angle, 1.5e-2f);
    } else {
      // Test loop count.
      orig.control_telemetry.loop_count = 38242U;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(orig.control_telemetry.loop_count,
                unpacked.control_telemetry.loop_count);
      orig.control_telemetry.loop_count = 1U;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(orig.control_telemetry.loop_count,
                unpacked.control_telemetry.loop_count);
    }

    if (part % 2 == 0) {
      // Test airspeed.
      orig.control_telemetry.airspeed = -50.0f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.airspeed,
                  unpacked.control_telemetry.airspeed, 1e-3f);
      orig.control_telemetry.airspeed = 50.0f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.airspeed,
                  unpacked.control_telemetry.airspeed, 1e-3f);

      // Test alpha.
      orig.control_telemetry.alpha = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.alpha,
                  unpacked.control_telemetry.alpha, 1.5e-2f);
      orig.control_telemetry.alpha = 1.3f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.alpha,
                  unpacked.control_telemetry.alpha, 1.5e-2f);

      // Test beta.
      orig.control_telemetry.beta = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.beta,
                  unpacked.control_telemetry.beta, 1.5e-2f);
      orig.control_telemetry.beta = 1.3f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.beta,
                  unpacked.control_telemetry.beta, 1.5e-2f);

      // Test roll.
      orig.control_telemetry.roll = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.roll,
                  unpacked.control_telemetry.roll, 1.5e-2f);
      orig.control_telemetry.roll = 1.3f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.roll,
                  unpacked.control_telemetry.roll, 1.5e-2f);

      // Test pitch.
      orig.control_telemetry.pitch = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.pitch,
                  unpacked.control_telemetry.pitch, 1.5e-2f);
      orig.control_telemetry.pitch = 1.3f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.pitch,
                  unpacked.control_telemetry.pitch, 1.5e-2f);

      // Test yaw.
      orig.control_telemetry.yaw = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.yaw,
                  unpacked.control_telemetry.yaw, 1.5e-2f);
      orig.control_telemetry.yaw = 1.3f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.yaw,
                  unpacked.control_telemetry.yaw, 1.5e-2f);

      // Test angular rates.
      for (int32_t i = 0; i < 3; ++i) {
        orig.control_telemetry.pqr[i] = -5.0f;
        EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
        EXPECT_NEAR(orig.control_telemetry.pqr[i],
                    unpacked.control_telemetry.pqr[i], 1.5e-2f);
        orig.control_telemetry.pqr[i] = 5.0f;
        EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
        EXPECT_NEAR(orig.control_telemetry.pqr[i],
                    unpacked.control_telemetry.pqr[i], 1.5e-2f);
      }
    } else {
      // Test target position.
      for (int32_t i = 0;
           i < ARRAYSIZE(orig.control_telemetry.target_pos_cw); ++i) {
        orig.control_telemetry.target_pos_cw[i] = -500.0f;
        EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
        EXPECT_NEAR(orig.control_telemetry.target_pos_cw[i],
                    unpacked.control_telemetry.target_pos_cw[i], 1.0f);
        orig.control_telemetry.target_pos_cw[i] = 500.0f;
        EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
        EXPECT_NEAR(orig.control_telemetry.target_pos_cw[i],
                    unpacked.control_telemetry.target_pos_cw[i], 1.0f);
      }

      // Test current position.
      for (int32_t i = 0;
           i < ARRAYSIZE(orig.control_telemetry.current_pos_cw); ++i) {
        orig.control_telemetry.current_pos_cw[i] = -500.0f;
        EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
        EXPECT_NEAR(orig.control_telemetry.current_pos_cw[i],
                    unpacked.control_telemetry.current_pos_cw[i], 1.0f);
        orig.control_telemetry.current_pos_cw[i] = 500.0f;
        EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
        EXPECT_NEAR(orig.control_telemetry.current_pos_cw[i],
                    unpacked.control_telemetry.current_pos_cw[i], 1.0f);
      }

      // Test delta aileron.
      orig.control_telemetry.delta_aileron = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_aileron,
                  unpacked.control_telemetry.delta_aileron, 1.5e-2f);
      orig.control_telemetry.delta_aileron = 1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_aileron,
                  unpacked.control_telemetry.delta_aileron, 1.5e-2f);

      // Test delta elevator.
      orig.control_telemetry.delta_elevator = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_elevator,
                  unpacked.control_telemetry.delta_elevator, 1.5e-2f);
      orig.control_telemetry.delta_elevator = 1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_elevator,
                  unpacked.control_telemetry.delta_elevator, 1.5e-2f);

      // Test delta rudder.
      orig.control_telemetry.delta_rudder = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_rudder,
                  unpacked.control_telemetry.delta_rudder, 1.5e-2f);
      orig.control_telemetry.delta_rudder = 1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_rudder,
                  unpacked.control_telemetry.delta_rudder, 1.5e-2f);

      // Test delta inboard_flap.
      orig.control_telemetry.delta_inboard_flap = -1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_inboard_flap,
                  unpacked.control_telemetry.delta_inboard_flap, 1.5e-2f);
      orig.control_telemetry.delta_inboard_flap = 1.2f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.delta_inboard_flap,
                  unpacked.control_telemetry.delta_inboard_flap, 1.5e-2f);

      // Test experiment configs.
      orig.control_telemetry.experiment_type = kExperimentTypeNoTest;
      orig.control_telemetry.experiment_case_id = 0U;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(orig.control_telemetry.experiment_type,
                unpacked.control_telemetry.experiment_type);
      EXPECT_EQ(orig.control_telemetry.experiment_case_id,
                unpacked.control_telemetry.experiment_case_id);
      orig.control_telemetry.experiment_type = 1U;
      orig.control_telemetry.experiment_case_id = 4U;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(orig.control_telemetry.experiment_type,
                unpacked.control_telemetry.experiment_type);
      EXPECT_EQ(orig.control_telemetry.experiment_case_id,
                unpacked.control_telemetry.experiment_case_id);
    }

    // Test position.
    const int32_t pos_index[] = {part % 2, 2};
    for (int32_t i = 0; i < ARRAYSIZE(pos_index); ++i) {
      int32_t j = pos_index[i];
      orig.control_telemetry.pos_g[j] = -800.1f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.pos_g[j],
                  unpacked.control_telemetry.pos_g[j], 0.1f);
      orig.control_telemetry.pos_g[j] = 800.1f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.pos_g[j],
                  unpacked.control_telemetry.pos_g[j], 0.1f);
    }

    // Test velocity.
    const int32_t vel_index[] = {part % 2, 2};
    for (int32_t i = 0; i < ARRAYSIZE(vel_index); ++i) {
      int32_t j = vel_index[i];
      orig.control_telemetry.vel_g[j] = -100.1f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.vel_g[j],
                  unpacked.control_telemetry.vel_g[j], 0.1f);
      orig.control_telemetry.vel_g[j] = 100.1f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.vel_g[j],
                  unpacked.control_telemetry.vel_g[j], 0.1f);
    }

    if (part % 2 == 0) {
      // Test tension.
      orig.control_telemetry.tension = 300.9e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.tension,
                  unpacked.control_telemetry.tension, 10.0f);
      orig.control_telemetry.tension = 100.3e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.tension,
                  unpacked.control_telemetry.tension, 10.0f);
      orig.control_telemetry.tension = 0.0f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.tension,
                  unpacked.control_telemetry.tension, 10.0f);
    } else {
      // Test tension command.
      orig.control_telemetry.tension_command = 350.7e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.tension_command,
                  unpacked.control_telemetry.tension_command, 10.0f);
      orig.control_telemetry.tension_command = 150.1e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.tension_command,
                  unpacked.control_telemetry.tension_command, 10.0f);
      orig.control_telemetry.tension_command = 0.0f;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.tension_command,
                  unpacked.control_telemetry.tension_command, 10.0f);
    }

    if (part % 2 == 0) {
      // Test thrust.
      orig.control_telemetry.thrust = 10.7e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.thrust,
                  unpacked.control_telemetry.thrust, 499.0f);
      orig.control_telemetry.thrust = -10.1e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.thrust,
                  unpacked.control_telemetry.thrust, 499.0f);

    } else {
      // Test thrust available.
      orig.control_telemetry.thrust_avail = 30.7e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.thrust_avail,
                  unpacked.control_telemetry.thrust_avail, 499.0f);
      orig.control_telemetry.thrust_avail = -30.1e3;
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(orig.control_telemetry.thrust_avail,
                  unpacked.control_telemetry.thrust_avail, 499.0f);
    }

    // Test moment.
    orig.control_telemetry.moment[part % 3] = 10.7e3;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(orig.control_telemetry.moment[part % 3],
                unpacked.control_telemetry.moment[part % 3], 499.0f);
    orig.control_telemetry.moment[part % 3] = -10.1e3;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(orig.control_telemetry.moment[part % 3],
                unpacked.control_telemetry.moment[part % 3], 499.0f);

    // Test moment available.
    orig.control_telemetry.moment_avail[part % 3] = 30.7e3;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(orig.control_telemetry.moment_avail[part % 3],
                unpacked.control_telemetry.moment_avail[part % 3], 499.0f);
    orig.control_telemetry.moment_avail[part % 3] = -30.1e3;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(orig.control_telemetry.moment_avail[part % 3],
                unpacked.control_telemetry.moment_avail[part % 3], 499.0f);

    // Test gain ramp scale.
    orig.control_telemetry.gain_ramp_scale = 0.1f;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(orig.control_telemetry.gain_ramp_scale,
                unpacked.control_telemetry.gain_ramp_scale, 1e-2f);
    orig.control_telemetry.gain_ramp_scale = 1.0f;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_NEAR(orig.control_telemetry.gain_ramp_scale,
                unpacked.control_telemetry.gain_ramp_scale, 1e-2f);
  }
}

TEST(PackTetherDown, TetherGpsTime) {
  // Decimated according to (frame_index / dec) % 4 == 0.
  TestTetherGpsTime<TetherDownMessage>(32U * TETHER_RADIO_DECIMATION);
}

TEST(PackTetherDown, TetherGpsStatus) {
  for (int32_t i = 0; i < kNumWingGpsReceivers; ++i) {
    // Test decimation.
    EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
        OFFSETOF(TetherDownMessage, gps_statuses[i].sequence),
        SIZEOF(TetherDownMessage, gps_statuses[i].sequence)));

    // Decimated according to (frame_index / dec) % 4 == 0.
    uint16_t frame_index =
        static_cast<uint16_t>((32U + 4U * i) * TETHER_RADIO_DECIMATION);
    TetherDownMessage orig, unpacked;
    TestTetherGpsStatus(frame_index, &orig, &orig.gps_statuses[i], &unpacked,
                        &unpacked.gps_statuses[i]);
  }
}

TEST(PackTetherDown, TetherCommsStatus) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, comms_status.received_signal_strength),
      SIZEOF(TetherDownMessage, comms_status.received_signal_strength)));

  // Only updated when (frame_index / dec) % 4 == 0.
  orig.frame_index = 32U * TETHER_RADIO_DECIMATION;

  // Test network links.
  orig.comms_status.links_up = 0U;
  unpacked.comms_status.links_up = 0U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.comms_status.links_up, unpacked.comms_status.links_up);
  orig.comms_status.links_up = kTetherCommsLinkPof;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.comms_status.links_up, unpacked.comms_status.links_up);
  orig.comms_status.links_up = kTetherCommsLinkEop;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.comms_status.links_up, unpacked.comms_status.links_up);
  orig.comms_status.links_up = kTetherCommsLinkWifi;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.comms_status.links_up, unpacked.comms_status.links_up);
  orig.comms_status.links_up = kTetherCommsLinkLongRange;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.comms_status.links_up, unpacked.comms_status.links_up);
  orig.comms_status.links_up = kTetherCommsLinkJoystick;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.comms_status.links_up, unpacked.comms_status.links_up);

  // Test received signal strength.
  orig.comms_status.received_signal_strength = -112;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(-112, unpacked.comms_status.received_signal_strength);
  orig.comms_status.received_signal_strength = 0;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(0, unpacked.comms_status.received_signal_strength);
}

TEST(PackTetherDown, TetherMotorStatus) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  const int32_t num_motors = ARRAYSIZE(orig.motor_statuses);

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, motor_statuses[0].iq),
      SIZEOF(TetherDownMessage, motor_statuses[0].iq)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, motor_statuses[0].id),
      SIZEOF(TetherDownMessage, motor_statuses[0].id)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, motor_statuses[0].bus_voltage),
      SIZEOF(TetherDownMessage, motor_statuses[0].bus_voltage)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, motor_statuses[0].bus_current),
      SIZEOF(TetherDownMessage, motor_statuses[0].bus_current)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, motor_statuses[0].motor_temps[0]),
      SIZEOF(TetherDownMessage, motor_statuses[0].motor_temps[0])));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, motor_statuses[0].controller_temps[0]),
      SIZEOF(TetherDownMessage, motor_statuses[0].controller_temps[0])));

  // Only updated when (frame_index / dec) % 4 == 2.
  orig.frame_index = 34U * TETHER_RADIO_DECIMATION;

  for (int32_t motor = 0; motor < num_motors; ++motor) {
    // Test warning.
    orig.motor_statuses[motor].warning = 1;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(1, unpacked.motor_statuses[motor].warning);
    orig.motor_statuses[motor].warning = 0;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(0, unpacked.motor_statuses[motor].warning);

    // Test error.
    orig.motor_statuses[motor].error = 1;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(1, unpacked.motor_statuses[motor].error);
    orig.motor_statuses[motor].error = 0;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(0, unpacked.motor_statuses[motor].error);
  }

  // Test state.
  for (int32_t s = 0; s < ARRAYSIZE(kActuatorStates); ++s) {
    for (int32_t motor = 0; motor < num_motors; ++motor) {
      orig.motor_statuses[motor].state = kActuatorStates[s];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(kActuatorStates[s], unpacked.motor_statuses[motor].state);
    }
  }

  // Test speed.
  const int16_t speeds[] = {-256, -200, -100, 0, 100, 200, 255};
  for (int32_t s = 0; s < ARRAYSIZE(speeds); ++s) {
    for (int32_t motor = 0; motor < num_motors; ++motor) {
      orig.motor_statuses[motor].speed = speeds[s];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(speeds[s], unpacked.motor_statuses[motor].speed);
    }
  }

  // We multiplex the TetherMotorState structure per each motor, then
  // multiplex each motor temperature within this structure. Here, we
  // determine the number of radio transmissions to transmit the complete
  // structure.
  const int32_t num_motor_temps =
      ARRAYSIZE(orig.motor_statuses[0].motor_temps);
  const int32_t num_controller_temps =
      ARRAYSIZE(orig.motor_statuses[0].controller_temps);
  const int32_t num_updates =
      num_motors * MaxInt32(num_motor_temps, num_controller_temps);

  // Set state.
  for (int32_t m = 0; m < num_motors; ++m) {
    orig.motor_statuses[m].iq =
        static_cast<int16_t>(2 * (num_motors * m + 2));
    orig.motor_statuses[m].id =
        static_cast<int16_t>(2 * (num_motors * m + 3));
    orig.motor_statuses[m].bus_voltage =
        static_cast<int16_t>(2 * (num_motors * m + 4));
    orig.motor_statuses[m].bus_current =
        static_cast<int16_t>(2 * (num_motors * m + 5));

    for (int32_t mt = 0; mt < num_motor_temps; ++mt) {
      orig.motor_statuses[m].motor_temps[mt] =
          static_cast<int16_t>(16 + 2 * (num_motors * m + mt));
    }
    for (int32_t ct = 0; ct < num_controller_temps; ++ct) {
      orig.motor_statuses[m].controller_temps[ct] =
          static_cast<int16_t>(18 + 2 * (num_motors * m + ct));
    }
  }

  // Pack.
  for (int32_t i = 0; i < num_updates; ++i) {
    orig.frame_index =
        static_cast<uint16_t>(orig.frame_index + 4U * TETHER_RADIO_DECIMATION);
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  }

  // Verify state.
  for (int32_t m = 0; m < num_motors; ++m) {
    EXPECT_EQ(static_cast<int16_t>(2 * (num_motors * m + 2)),
              unpacked.motor_statuses[m].iq);
    EXPECT_EQ(static_cast<int16_t>(2 * (num_motors * m + 3)),
              unpacked.motor_statuses[m].id);
    EXPECT_EQ(static_cast<int16_t>(2 * (num_motors * m + 4)),
              unpacked.motor_statuses[m].bus_voltage);
    EXPECT_EQ(static_cast<int16_t>(2 * (num_motors * m + 5)),
              unpacked.motor_statuses[m].bus_current);

    for (int32_t mt = 0; mt < num_motor_temps; ++mt) {
      EXPECT_EQ(static_cast<int16_t>(16 + 2 * (num_motors * m + mt)),
                unpacked.motor_statuses[m].motor_temps[mt]);
    }
    for (int32_t ct = 0; ct < num_controller_temps; ++ct) {
      EXPECT_EQ(static_cast<int16_t>(18 + 2 * (num_motors * m + ct)),
                unpacked.motor_statuses[m].controller_temps[ct]);
    }
  }
}

TEST(PackTetherDown, TetherNodeStatus) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, node_status.node),
      SIZEOF(TetherDownMessage, node_status.node)));

  // Only updated when (frame_index / dec) % 4 == 2.
  orig.frame_index = 34U * TETHER_RADIO_DECIMATION;

  // Test node.
  orig.node_status.node = 0;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.node, unpacked.node_status.node);
  orig.node_status.node = kNumAioNodes - 1;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.node, unpacked.node_status.node);

  // Test flags.
  orig.node_status.flags = 0U;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.flags, unpacked.node_status.flags);
  orig.node_status.flags = kTetherNodeFlagSelfTestFailure;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.flags, unpacked.node_status.flags);
  orig.node_status.flags = kTetherNodeFlagPowerGood;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.flags, unpacked.node_status.flags);
  orig.node_status.flags = kTetherNodeFlagNetworkAGood;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.flags, unpacked.node_status.flags);
  orig.node_status.flags = kTetherNodeFlagNetworkBGood;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.flags, unpacked.node_status.flags);
  orig.node_status.flags = kTetherNodeFlagAnyWarning;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.flags, unpacked.node_status.flags);
  orig.node_status.flags = kTetherNodeFlagAnyError;
  EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
  EXPECT_EQ(orig.node_status.flags, unpacked.node_status.flags);

  // Test board temperature.
  const int16_t temps[] = {-40, -20, 0, 20, 40, 60, 80, 100, 120};
  for (int32_t i = 0; i < ARRAYSIZE(temps); ++i) {
    orig.node_status.board_temp = temps[i];
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.node_status.board_temp, unpacked.node_status.board_temp);
  }

  // Test board humidity.
  const uint8_t humidities[] = {0, 20, 40, 60, 80, 100};
  for (int32_t i = 0; i < ARRAYSIZE(humidities); ++i) {
    orig.node_status.board_humidity = humidities[i];
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig.node_status.board_humidity,
              unpacked.node_status.board_humidity);
  }
}

TEST(PackTetherDown, TetherFlightComputer) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  for (int32_t i = 0; i < kNumFlightComputers; ++i) {
    TetherFlightComputer *orig_fc = &orig.flight_computers[i];
    TetherFlightComputer *unpacked_fc = &unpacked.flight_computers[i];

    // Test decimation.
    EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
        OFFSETOF(TetherDownMessage, flight_computers[i].flags),
        SIZEOF(TetherDownMessage, flight_computers[i].flags)));

    // Only updated when (frame_index / dec) % 4 == 2.
    orig.frame_index =
        static_cast<uint16_t>((38U + 4U * i) * TETHER_RADIO_DECIMATION);

    // Test flags.
    orig_fc->flags = 0U;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_fc->flags, unpacked_fc->flags);
    orig_fc->flags = kTetherFlightComputerFlagImuGood;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_fc->flags, unpacked_fc->flags);
    orig_fc->flags = kTetherFlightComputerFlagGpsGood;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_fc->flags, unpacked_fc->flags);
    orig_fc->flags = kTetherFlightComputerFlagPitotGood;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_fc->flags, unpacked_fc->flags);
    orig_fc->flags = kTetherFlightComputerFlagFpvEnabled;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_fc->flags, unpacked_fc->flags);
  }
}

TEST(PackTetherDown, TetherReleaseStatus) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  for (int32_t i = 0; i < kNumLoadcellNodes; ++i) {
    TetherReleaseStatus *orig_rs = &orig.release_statuses[i];
    TetherReleaseStatus *unpacked_rs = &unpacked.release_statuses[i];

    // Test decimation.
    EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
        OFFSETOF(TetherDownMessage, release_statuses[i].state),
        SIZEOF(TetherDownMessage, release_statuses[i].state)));

    // Only updated when (frame_index / dec) % 4 == 2.
    orig.frame_index = 34U * TETHER_RADIO_DECIMATION;

    // Test state.
    for (int32_t s = 0; s < ARRAYSIZE(kActuatorStates); ++s) {
      orig_rs->state = kActuatorStates[s];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(kActuatorStates[s], unpacked_rs->state);
    }

    // Test interlock_switched.
    orig_rs->interlock_switched = 0;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_rs->interlock_switched, unpacked_rs->interlock_switched);
    orig_rs->interlock_switched = 1;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_rs->interlock_switched, unpacked_rs->interlock_switched);

    // Test released.
    orig_rs->released = 0;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_rs->released, unpacked_rs->released);
    orig_rs->released = 1;
    EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    EXPECT_EQ(orig_rs->released, unpacked_rs->released);
  }
}

TEST(PackTetherDown, TetherOpMotorAckParam) {
  MotorAckParamMessage m_in, m_out;

  m_in.id = 0x12;
  m_in.value = 0.32f;
  PackUnpackOpReply(kAioNodeMotorPbo, kTetherOpTypeMotorAckParam, 3259U, m_in,
                    &m_out);
  EXPECT_EQ(m_in.id, m_out.id);
  EXPECT_EQ(m_in.value, m_out.value);
}

TEST(PackTetherDown, TetherOpServoAckParam) {
  ServoAckParamMessage m_in, m_out;

  m_in.param = 0x1232;
  m_in.value = 0xDEADC0DE;
  PackUnpackOpReply(kAioNodeServoA1, kTetherOpTypeServoAckParam, 3259U, m_in,
                    &m_out);
  EXPECT_EQ(m_in.param, m_out.param);
  EXPECT_EQ(m_in.value, m_out.value);
}

TEST(PackTetherDown, TetherOpQueueMultiple) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Decimated according to (frame_index / dec) % 4 == 1.
  orig.frame_index = 33U * TETHER_RADIO_DECIMATION;

  TetherOpQueue q_in, q_out;
  TetherOpInit(&q_in);
  TetherOpInit(&q_out);

  // We expect to transmit at least two MotorAckParamMessages (an arbitrary
  // small message) per TetherDownPackedMessage.
  AioNode source_in = kAioNodeMotorPbo;
  TetherOpType type_in = kTetherOpTypeMotorAckParam;
  MotorAckParamMessage m_in;
  m_in.id = 1U;
  m_in.value = 0.1f;
  PushOpQueue(source_in, type_in, 1U, m_in, &q_in);
  m_in.id = 2U;
  m_in.value = 0.2f;
  PushOpQueue(source_in, type_in, 2U, m_in, &q_in);

  // Each pack/unpack counts as one transmit.
  EXPECT_TRUE(PackUnpackTetherMessage(&q_in, &orig, &q_out, &unpacked));

  // Check first message.
  AioNode source_out;
  TetherOpType type_out;
  uint16_t sequence_out;
  MotorAckParamMessage m_out;
  PopOpQueue(&q_out, &source_out, &type_out, &sequence_out, &m_out);
  EXPECT_EQ(source_in, source_out);
  EXPECT_EQ(type_in, type_in);
  EXPECT_EQ(1U, sequence_out);
  EXPECT_EQ(1U, m_out.id);
  EXPECT_EQ(0.1f, m_out.value);

  // Check second message.
  PopOpQueue(&q_out, &source_out, &type_out, &sequence_out, &m_out);
  EXPECT_EQ(source_in, source_out);
  EXPECT_EQ(type_in, type_in);
  EXPECT_EQ(2U, sequence_out);
  EXPECT_EQ(2U, m_out.id);
  EXPECT_EQ(0.2f, m_out.value);
}

TEST(PackTetherDown, TetherOpQueueOverflow) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Decimated according to (frame_index / dec) % 4 == 1.
  orig.frame_index = 33U * TETHER_RADIO_DECIMATION;

  TetherOpQueue q_in, q_out;
  TetherOpInit(&q_in);
  TetherOpInit(&q_out);

  // Create input messages.
  MotorAckParamMessage m_in;
  AioNode source_in = kAioNodeMotorPbo;
  TetherOpType type_in = kTetherOpTypeMotorAckParam;
  for (int32_t i = 0; i < TETHER_OP_QUEUE_SIZE; ++i) {
    m_in.id = static_cast<uint8_t>(i);
    m_in.value = 0.123f;
    PushOpQueue(source_in, type_in, static_cast<uint16_t>(i), m_in, &q_in);
  }

  uint16_t expected_sequence = 0U;
  while (TetherOpIsPending(&q_in)) {
    // Transmit as many messages as possible per pack/unpack.
    EXPECT_TRUE(PackUnpackTetherMessage(&q_in, &orig, &q_out, &unpacked));

    // Check output messages.
    while (TetherOpIsPending(&q_out)) {
      AioNode source_out;
      TetherOpType type_out;
      uint16_t sequence_out;
      MotorAckParamMessage m_out;
      PopOpQueue(&q_out, &source_out, &type_out, &sequence_out, &m_out);

      EXPECT_EQ(source_in, source_out);
      EXPECT_EQ(type_in, type_out);
      EXPECT_EQ(expected_sequence, sequence_out);

      ++expected_sequence;
    }
  }
  EXPECT_EQ(TETHER_OP_QUEUE_SIZE, expected_sequence);
}

TEST(PackTetherDown, TetherServoStatus) {
  TetherDownMessage orig, unpacked;
  memset(&orig, 0, sizeof(orig));
  memset(&unpacked, 0x5A, sizeof(unpacked));

  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, servo_statuses[0].angle),
      SIZEOF(TetherDownMessage, servo_statuses[0].angle)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, servo_statuses[0].r22_temp),
      SIZEOF(TetherDownMessage, servo_statuses[0].r22_temp)));

  const int32_t num_servos = ARRAYSIZE(orig.servo_statuses);

  // Only updated when (frame_index / dec) % 4 == 0.
  orig.frame_index = 32U * TETHER_RADIO_DECIMATION;

  // Test state.
  for (int32_t s = 0; s < ARRAYSIZE(kActuatorStates); ++s) {
    for (int32_t servo = 0; servo < num_servos; ++servo) {
      orig.servo_statuses[servo].state = kActuatorStates[s];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_EQ(kActuatorStates[s], unpacked.servo_statuses[servo].state);
    }
  }

  // Test angle.
  const float angles[] = {-PI_F, -PI_F / 2.0f, -PI_F / 4.0f, 0.0f,
                          PI_F / 4.0f, PI_F / 2.0f, PI_F - 0.01f};
  for (int32_t a = 0; a < ARRAYSIZE(angles); ++a) {
    for (int32_t servo = 0; servo < num_servos; ++servo) {
      orig.servo_statuses[servo].angle = angles[a];
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
      EXPECT_NEAR(angles[a], unpacked.servo_statuses[servo].angle, 0.03f);
    }
  }

  // Test temperature.
  const int32_t temps[] = {-40, -20, 0, 20, 40, 60, 80, 100, 120};
  for (int32_t t = 0; t < ARRAYSIZE(temps); ++t) {
    // We multiplex temperatures by servo id.
    for (int32_t i = 0; i < num_servos; ++i) {
      orig.servo_statuses[i].r22_temp = static_cast<int16_t>(temps[t] + i);
    }
    for (int32_t i = 0; i < num_servos; ++i) {
      orig.frame_index = static_cast<uint16_t>(orig.frame_index
                                               + 4U * TETHER_RADIO_DECIMATION);
      EXPECT_TRUE(PackUnpackTetherMessage(&orig, &unpacked));
    }
    for (int32_t i = 0; i < num_servos; ++i) {
      EXPECT_EQ(temps[t] + i, unpacked.servo_statuses[i].r22_temp);
    }
  }
}

void TestTetherBatteryStatus(uint16_t frame_index, TetherDownMessage *orig,
                             TetherBatteryStatus *orig_batt,
                             TetherDownMessage *unpacked,
                             TetherBatteryStatus *unpacked_batt) {
  memset(orig, 0, sizeof(*orig));
  memset(unpacked, 0x5A, sizeof(*unpacked));

  orig->frame_index = frame_index;

  // Test warning flag.
  orig_batt->warning = true;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_batt->warning, unpacked_batt->warning);
  orig_batt->warning = false;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_batt->warning, unpacked_batt->warning);

  // Test error flag.
  orig_batt->error = true;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_batt->error, unpacked_batt->error);
  orig_batt->error = false;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_batt->error, unpacked_batt->error);

  // Test voltages.
  const float voltages[] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 60.0f, 80.0f};
  for (int32_t i = 0; i < ARRAYSIZE(voltages); ++i) {
    // We multiplex voltages by voltage id.
    orig_batt->lv_a = voltages[i] + 1.0f;
    orig_batt->lv_b = voltages[i] + 2.0f;
    orig_batt->v_lv_or = voltages[i] + 3.0f;
    orig_batt->v_charger = voltages[i] + 4.0f;

    for (int32_t j = 0; j < 4; ++j) {
      orig->frame_index =
          static_cast<uint16_t>(frame_index + 12U * TETHER_RADIO_DECIMATION
                                * j);
      EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
    }

    EXPECT_NEAR(orig_batt->lv_a, unpacked_batt->lv_a, 0.1f);
    EXPECT_NEAR(orig_batt->lv_b, unpacked_batt->lv_b, 0.1f);
    EXPECT_NEAR(orig_batt->v_lv_or, unpacked_batt->v_lv_or, 0.1f);
    EXPECT_NEAR(orig_batt->v_charger, unpacked_batt->v_charger, 0.1f);
  }

  // Test currents.
  const float currents[] = {0.0f, 5.0f, 10.0f, 15.0f, 20.0f};
  for (int32_t i = 0; i < ARRAYSIZE(currents); ++i) {
    // We multiplex currents by current id.
    orig_batt->i_hall = currents[i] + 2.0f;
    orig_batt->i_charger = currents[i] + 3.0f;

    for (int32_t j = 0; j < 4; ++j) {
      orig->frame_index =
          static_cast<uint16_t>(frame_index + 12U * TETHER_RADIO_DECIMATION
                                * j);
      EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
    }

    EXPECT_NEAR(orig_batt->i_hall, unpacked_batt->i_hall, 0.1f);
    EXPECT_NEAR(orig_batt->i_charger, unpacked_batt->i_charger, 0.1f);
  }

  // Test temperature.
  const int32_t temps[] = {-40, -20, 0, 20, 40, 60, 80, 100, 120};
  for (int32_t i = 0; i < ARRAYSIZE(temps); ++i) {
    // We multiplex temperatures by temperature id.
    for (int32_t j = 0; j < ARRAYSIZE(orig_batt->temps); ++j) {
      orig_batt->temps[j] = static_cast<int16_t>(temps[i] + j);
    }
    for (int32_t j = 0; j < ARRAYSIZE(orig_batt->temps); ++j) {
      orig->frame_index =
          static_cast<uint16_t>(frame_index + 12U * TETHER_RADIO_DECIMATION
                                * j);
      EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
    }
    for (int32_t j = 0; j < ARRAYSIZE(orig_batt->temps); ++j) {
      EXPECT_EQ(orig_batt->temps[j], unpacked_batt->temps[j]);
    }
  }
}

TEST(PackTetherDown, TetherBatteryStatusA) {
  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, batt_a.v_lv_or),
      SIZEOF(TetherDownMessage, batt_a.v_lv_or)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, batt_a.i_hall),
      SIZEOF(TetherDownMessage, batt_a.i_hall)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, batt_a.temps[0]),
      SIZEOF(TetherDownMessage, batt_a.temps[0])));

  // Decimated according to (frame_index / dec) % 3 = 0.
  TetherDownMessage orig, unpacked;
  TestTetherBatteryStatus(36U * TETHER_RADIO_DECIMATION, &orig, &orig.batt_a,
                          &unpacked, &unpacked.batt_a);
}

TEST(PackTetherDown, TetherBatteryStatusB) {
  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, batt_b.v_lv_or),
      SIZEOF(TetherDownMessage, batt_b.v_lv_or)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, batt_b.i_hall),
      SIZEOF(TetherDownMessage, batt_b.i_hall)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, batt_b.temps[0]),
      SIZEOF(TetherDownMessage, batt_b.temps[0])));

  // Decimated according to (frame_index / dec) % 3 = 1.
  TetherDownMessage orig, unpacked;
  TestTetherBatteryStatus(40U * TETHER_RADIO_DECIMATION, &orig, &orig.batt_b,
                          &unpacked, &unpacked.batt_b);
}

void TestTetherMvlvStatus(uint16_t frame_index, TetherDownMessage *orig,
                          TetherMvlvStatus *orig_mvlv,
                          TetherDownMessage *unpacked,
                          TetherMvlvStatus *unpacked_mvlv) {
  memset(orig, 0, sizeof(*orig));
  memset(unpacked, 0x5A, sizeof(*unpacked));

  orig->frame_index = frame_index;

  // Test warning flag.
  orig_mvlv->warning = true;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_mvlv->warning, unpacked_mvlv->warning);
  orig_mvlv->warning = false;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_mvlv->warning, unpacked_mvlv->warning);

  // Test error flag.
  orig_mvlv->error = true;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_mvlv->error, unpacked_mvlv->error);
  orig_mvlv->error = false;
  EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
  EXPECT_EQ(orig_mvlv->error, unpacked_mvlv->error);

  // Test voltages.
  const float voltages[] = {0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 60.0f, 80.0f};
  for (int32_t i = 0; i < ARRAYSIZE(voltages); ++i) {
    // We multiplex voltages by voltage id.
    orig_mvlv->v_lv = voltages[i] + 1.0f;
    orig_mvlv->v_lv_or = voltages[i] + 2.0f;
    orig_mvlv->v_lv_pri = voltages[i] + 3.0f;
    orig_mvlv->v_lv_sec = voltages[i] + 4.0f;

    for (int32_t j = 0; j < 4; ++j) {
      orig->frame_index =
          static_cast<uint16_t>(frame_index + 12U * TETHER_RADIO_DECIMATION
                                * j);
      EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
    }

    EXPECT_NEAR(orig_mvlv->v_lv, unpacked_mvlv->v_lv, 0.1f);
    EXPECT_NEAR(orig_mvlv->v_lv_or, unpacked_mvlv->v_lv_or, 0.1f);
    EXPECT_NEAR(orig_mvlv->v_lv_pri, unpacked_mvlv->v_lv_pri, 0.1f);
    EXPECT_NEAR(orig_mvlv->v_lv_sec, unpacked_mvlv->v_lv_sec, 0.1f);
  }

  // Test currents.
  const float currents[] = {0.0f, 5.0f, 10.0f, 15.0f, 20.0f};
  for (int32_t i = 0; i < ARRAYSIZE(currents); ++i) {
    // We multiplex currents by current id.
    orig_mvlv->i_hall = currents[i] + 2.0f;

    for (int32_t j = 0; j < 4; ++j) {
      orig->frame_index =
          static_cast<uint16_t>(frame_index + 12U * TETHER_RADIO_DECIMATION
                                * j);
      EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
    }

    EXPECT_NEAR(orig_mvlv->i_hall, unpacked_mvlv->i_hall, 0.1f);
  }

  // Test mvlv status.
  const MvlvMonitorStatus statuses[] = {
    kMvlvMonitorStatusEnabled, kMvlvMonitorStatusConnected,
    kMvlvMonitorStatusFaultRetry, kMvlvMonitorStatusCmdReceived,
    kMvlvMonitorStatusCmdProcessed};
  for (int32_t i = 0; i < ARRAYSIZE(statuses); ++i) {
    // We multiplex currents by current id.
    orig_mvlv->status = statuses[i];

    for (int32_t j = 0; j < 4; ++j) {
      orig->frame_index =
          static_cast<uint16_t>(frame_index + 12U * TETHER_RADIO_DECIMATION
                                * j);
      EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
    }

    EXPECT_EQ(orig_mvlv->status, unpacked_mvlv->status);
  }

  // Test temperature.
  const int32_t temps[] = {-40, -20, 0, 20, 40, 60, 80, 100, 120};
  for (int32_t i = 0; i < ARRAYSIZE(temps); ++i) {
    // We multiplex temperatures by temperature id.
    for (int32_t j = 0; j < ARRAYSIZE(orig_mvlv->temps); ++j) {
      orig_mvlv->temps[j] = static_cast<int16_t>(temps[i] + j);
    }
    for (int32_t j = 0; j < ARRAYSIZE(orig_mvlv->temps); ++j) {
      orig->frame_index =
          static_cast<uint16_t>(frame_index + 12U * TETHER_RADIO_DECIMATION
                                * j);
      EXPECT_TRUE(PackUnpackTetherMessage(orig, unpacked));
    }
    for (int32_t j = 0; j < ARRAYSIZE(orig_mvlv->temps); ++j) {
      EXPECT_EQ(orig_mvlv->temps[j], unpacked_mvlv->temps[j]);
    }
  }
}

TEST(PackTetherDown, TetherMvlvStatus) {
  // Test decimation.
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, mvlv.v_lv_pri),
      SIZEOF(TetherDownMessage, mvlv.v_lv_pri)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, mvlv.i_hall),
      SIZEOF(TetherDownMessage, mvlv.i_hall)));
  EXPECT_TRUE(CheckDecimation<TetherDownMessage>(
      OFFSETOF(TetherDownMessage, mvlv.temps[0]),
      SIZEOF(TetherDownMessage, mvlv.temps[0])));

  // Decimated according to (frame_index / dec) % 3 = 2.
  TetherDownMessage orig, unpacked;
  TestTetherMvlvStatus(44U * TETHER_RADIO_DECIMATION, &orig, &orig.mvlv,
                       &unpacked, &unpacked.mvlv);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
