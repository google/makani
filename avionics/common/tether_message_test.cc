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

#include "avionics/common/tether_message.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/network/message_stats.h"
#include "common/macros.h"

extern "C" {

const void *GetMessage(const TetherMessageInfo *info, const void *messages,
                       int32_t index);

uint16_t GetMessageFrameIndex(const TetherMessageInfo *info, const void *m);

bool GetMessageGpsTime(const TetherMessageInfo *info, const void *m,
                       TetherGpsTime *value);

bool GetFieldNoUpdateCount(const TetherFieldInfo *info, const void *m,
                           int32_t *value);

void SetFieldNoUpdateCount(const TetherFieldInfo *info, int32_t value, void *m);

uint16_t GetFieldSequence(const TetherFieldInfo *info, const void *m);

bool MergeFieldBySequence(const TetherMessageInfo *message,
                          const TetherFieldInfo *field, int32_t num_inputs,
                          const void *input_messages, void *output);

bool MergeFieldByFrameIndex(const TetherMessageInfo *message,
                            const TetherFieldInfo *field, int32_t num_inputs,
                            const void *input_messages, void *output);

bool MergeFieldByGpsTime(const TetherMessageInfo *message,
                         const TetherFieldInfo *field, int32_t num_inputs,
                         const void *input_messages, void *output);

}  // extern "C"

namespace {

const TetherFieldInfo *FindTetherMessageField(const TetherMessageInfo *message,
                                              int32_t offsetof_field) {
  for (int32_t i = 0; i < message->num_fields; ++i) {
    const TetherFieldInfo *field = &message->fields[i];
    if (field->offsetof_field == offsetof_field) {
      return field;
    }
  }
  return nullptr;
}

bool TetherMessageFieldExists(const TetherMessageInfo *message,
                              int32_t offsetof_field) {
  return FindTetherMessageField(message, offsetof_field) != nullptr;
}

}  // namespace

TEST(TetherDownFields, Arrays) {
  TetherDownMessage m;
  const uint8_t *mp = reinterpret_cast<const uint8_t *>(&m);

  ASSERT_EQ(kNumFlightComputers, ARRAYSIZE(m.flight_computers));
  for (int32_t i = 0; i < ARRAYSIZE(m.flight_computers); ++i) {
    const uint8_t *fp =
        reinterpret_cast<const uint8_t *>(&m.flight_computers[i]);
    int32_t offset = static_cast<int32_t>(fp - mp);
    EXPECT_TRUE(TetherMessageFieldExists(TetherDownGetMessageInfo(), offset));
  }

  ASSERT_EQ(kNumWingGpsReceivers, ARRAYSIZE(m.gps_statuses));
  for (int32_t i = 0; i < ARRAYSIZE(m.gps_statuses); ++i) {
    const uint8_t *fp = reinterpret_cast<const uint8_t *>(&m.gps_statuses[i]);
    int32_t offset = static_cast<int32_t>(fp - mp);
    EXPECT_TRUE(TetherMessageFieldExists(TetherDownGetMessageInfo(), offset));
  }

  ASSERT_EQ(kNumMotors, ARRAYSIZE(m.motor_statuses));
  for (int32_t i = 0; i < ARRAYSIZE(m.motor_statuses); ++i) {
    const uint8_t *fp = reinterpret_cast<const uint8_t *>(&m.motor_statuses[i]);
    int32_t offset = static_cast<int32_t>(fp - mp);
    EXPECT_TRUE(TetherMessageFieldExists(TetherDownGetMessageInfo(), offset));
  }

  ASSERT_EQ(kNumLoadcellNodes, ARRAYSIZE(m.release_statuses));
  for (int32_t i = 0; i < ARRAYSIZE(m.release_statuses); ++i) {
    const uint8_t *fp =
        reinterpret_cast<const uint8_t *>(&m.release_statuses[i]);
    int32_t offset = static_cast<int32_t>(fp - mp);
    EXPECT_TRUE(TetherMessageFieldExists(TetherDownGetMessageInfo(), offset));
  }

  ASSERT_EQ(kNumServos, ARRAYSIZE(m.servo_statuses));
  for (int32_t i = 0; i < ARRAYSIZE(m.servo_statuses); ++i) {
    const uint8_t *fp = reinterpret_cast<const uint8_t *>(&m.servo_statuses[i]);
    int32_t offset = static_cast<int32_t>(fp - mp);
    EXPECT_TRUE(TetherMessageFieldExists(TetherDownGetMessageInfo(), offset));
  }
}

TEST(GetMessage, Down) {
  TetherDownMessage down[3];
  for (int32_t i = 0; i < ARRAYSIZE(down); ++i) {
    EXPECT_EQ(&down[i], GetMessage(TetherDownGetMessageInfo(), down, i));
  }
}

TEST(GetMessage, Up) {
  TetherUpMessage up[3];
  for (int32_t i = 0; i < ARRAYSIZE(up); ++i) {
    EXPECT_EQ(&up[i], GetMessage(TetherUpGetMessageInfo(), up, i));
  }
}

TEST(GetMessageFrameIndex, Normal) {
  TetherUpMessage up;

  memset(&up, 0, sizeof(up));
  up.frame_index = UINT16_MAX;
  EXPECT_EQ(up.frame_index,
            GetMessageFrameIndex(TetherUpGetMessageInfo(), &up));

  memset(&up, 0xFF, sizeof(up));
  up.frame_index = 0U;
  EXPECT_EQ(up.frame_index,
            GetMessageFrameIndex(TetherUpGetMessageInfo(), &up));
}

TEST(GetMessageGpsTime, Normal) {
  TetherUpMessage up;
  struct {
    int32_t before;
    TetherGpsTime gps_time;
    int32_t after;
  } orig, out;

  // Test for possible memory corruption.
  const uint8_t init[] = {0x00, 0xFF};
  const int32_t value[] = {0, -1};
  for (int32_t i = 0; i < ARRAYSIZE(init); ++i) {
    for (int32_t j = 0; j < ARRAYSIZE(init); ++j) {
      memset(&up, init[j], sizeof(up));
      up.gps_time.no_update_count = value[j];
      up.gps_time.time_of_week = value[j];
      memset(&orig, init[i], sizeof(orig));
      out = orig;
      GetMessageGpsTime(TetherUpGetMessageInfo(), &up, &out.gps_time);
      EXPECT_EQ(up.gps_time.no_update_count, out.gps_time.no_update_count);
      EXPECT_EQ(up.gps_time.time_of_week, out.gps_time.time_of_week);
      memset(&out.gps_time, init[i], sizeof(out.gps_time));
      EXPECT_EQ(0, memcmp(&orig, &out, sizeof(orig)));
    }
  }

  // Test valid time.
  up.gps_time.no_update_count = 0;
  up.gps_time.time_of_week = 0;
  EXPECT_TRUE(GetMessageGpsTime(TetherUpGetMessageInfo(), &up, &out.gps_time));

  // Test invalid no_update_count.
  up.gps_time.no_update_count = INT32_MAX;
  up.gps_time.time_of_week = 0;
  EXPECT_FALSE(GetMessageGpsTime(TetherUpGetMessageInfo(), &up, &out.gps_time));

  // Test invalid time.
  up.gps_time.no_update_count = 0;
  up.gps_time.time_of_week = INT32_MAX;
  EXPECT_FALSE(GetMessageGpsTime(TetherUpGetMessageInfo(), &up, &out.gps_time));
}

TEST(GetFieldNoUpdateCount, Normal) {
  TetherUpMessage up;
  struct {
    int32_t before;
    int32_t no_update_count;
    int32_t after;
  } orig, out;

  const TetherMessageInfo *message = TetherUpGetMessageInfo();
  ASSERT_NE(nullptr, message);

  const TetherFieldInfo *field =
      FindTetherMessageField(message, OFFSETOF(TetherUpMessage, joystick));
  ASSERT_NE(nullptr, field);

  // Test for possible memory corruption.
  const uint8_t init[] = {0x00, 0xFF};
  const int32_t value[] = {0, -1};
  for (int32_t i = 0; i < ARRAYSIZE(init); ++i) {
    for (int32_t j = 0; j < ARRAYSIZE(init); ++j) {
      memset(&up, init[j], sizeof(up));
      up.joystick.no_update_count = value[j];
      memset(&orig, init[i], sizeof(orig));
      out = orig;
      GetFieldNoUpdateCount(field, &up, &out.no_update_count);
      EXPECT_EQ(up.joystick.no_update_count, out.no_update_count);
      memset(&out.no_update_count, init[i], sizeof(out.no_update_count));
      EXPECT_EQ(0, memcmp(&orig, &out, sizeof(orig)));
    }
  }

  // Test valid no_update_count.
  up.joystick.no_update_count = 0;
  EXPECT_TRUE(GetFieldNoUpdateCount(field, &up, &out.no_update_count));
  EXPECT_EQ(up.joystick.no_update_count, out.no_update_count);
  up.joystick.no_update_count = INT32_MAX;
  EXPECT_TRUE(GetFieldNoUpdateCount(field, &up, &out.no_update_count));
  EXPECT_EQ(up.joystick.no_update_count, out.no_update_count);

  // Test invalid no_update_count.
  field = FindTetherMessageField(message,
                                 OFFSETOF(TetherUpMessage, frame_index));
  EXPECT_FALSE(GetFieldNoUpdateCount(field, &up, &out.no_update_count));
}

TEST(SetFieldNoUpdateCount, Normal) {
  TetherUpMessage up;

  const TetherMessageInfo *message = TetherUpGetMessageInfo();
  ASSERT_NE(nullptr, message);

  const TetherFieldInfo *field =
      FindTetherMessageField(message, OFFSETOF(TetherUpMessage, joystick));
  ASSERT_NE(nullptr, field);

  memset(&up, 0, sizeof(up));
  SetFieldNoUpdateCount(field, INT32_MAX, &up);
  EXPECT_EQ(INT32_MAX, up.joystick.no_update_count);

  memset(&up, 0xFF, sizeof(up));
  SetFieldNoUpdateCount(field, 0, &up);
  EXPECT_EQ(0, up.joystick.no_update_count);
}

TEST(GetFieldSequence, Normal) {
  TetherUpMessage up;

  const TetherMessageInfo *message = TetherUpGetMessageInfo();
  ASSERT_NE(nullptr, message);

  const TetherFieldInfo *field =
      FindTetherMessageField(message, OFFSETOF(TetherUpMessage, joystick));
  ASSERT_NE(nullptr, field);

  memset(&up, 0, sizeof(up));
  up.joystick.sequence = UINT16_MAX;
  EXPECT_EQ(up.joystick.sequence, GetFieldSequence(field, &up));

  memset(&up, 0xFF, sizeof(up));
  up.joystick.sequence = 0U;
  EXPECT_EQ(up.joystick.sequence, GetFieldSequence(field, &up));
}

TEST(MergeTetherBySequence, Normal) {
  const TetherMessageInfo *message = TetherUpGetMessageInfo();
  ASSERT_NE(nullptr, message);

  const TetherFieldInfo *field =
      FindTetherMessageField(message, OFFSETOF(TetherUpMessage, joystick));
  ASSERT_NE(nullptr, field);

  TetherUpMessage input_messages[3], output;
  for (int32_t i = 0; i < ARRAYSIZE(input_messages); ++i) {
    TetherUpInit(&input_messages[i]);
    input_messages[i].joystick.no_update_count = 0;
    input_messages[i].joystick.throttle = static_cast<float>(i);
  }
  memset(&output, 0x5A, sizeof(output));

  // Define input to select message index 1 when all are valid.
  input_messages[0].joystick.sequence = TETHER_SEQUENCE_ROLLOVER - 1;
  input_messages[1].joystick.sequence = 2;
  input_messages[2].joystick.sequence = 1;

  // Test all messages valid.
  input_messages[0].joystick.no_update_count = 0;
  input_messages[1].joystick.no_update_count = 0;
  input_messages[2].joystick.no_update_count = 0;
  EXPECT_TRUE(MergeFieldBySequence(message, field, ARRAYSIZE(input_messages),
                                   input_messages, &output));
  EXPECT_EQ(2, output.joystick.sequence);
  EXPECT_NEAR(1.0f, output.joystick.throttle, FLT_EPSILON);

  // Test invalid no_update_count.
  input_messages[0].joystick.no_update_count = 0;
  input_messages[1].joystick.no_update_count = INT32_MAX;
  input_messages[2].joystick.no_update_count = 0;
  EXPECT_TRUE(MergeFieldBySequence(message, field, ARRAYSIZE(input_messages),
                                   input_messages, &output));
  EXPECT_EQ(1, output.joystick.sequence);
  EXPECT_NEAR(2.0f, output.joystick.throttle, FLT_EPSILON);

  // Test all invalid inputs, according to no_update_count.
  input_messages[0].joystick.no_update_count = INT32_MAX;
  input_messages[1].joystick.no_update_count = INT32_MAX;
  input_messages[2].joystick.no_update_count = INT32_MAX;
  EXPECT_FALSE(MergeFieldBySequence(message, field, ARRAYSIZE(input_messages),
                                    input_messages, &output));

  // Test zero inputs.
  EXPECT_FALSE(MergeFieldBySequence(message, field, 0, input_messages,
                                    &output));
  EXPECT_FALSE(MergeFieldBySequence(message, field, 0, nullptr, &output));
}

TEST(MergeTetherByFrameIndex, Normal) {
  const TetherMessageInfo *message = TetherUpGetMessageInfo();
  ASSERT_NE(nullptr, message);

  const TetherFieldInfo *field =
      FindTetherMessageField(message, OFFSETOF(TetherUpMessage, joystick));
  ASSERT_NE(nullptr, field);

  TetherUpMessage input_messages[3], output;
  for (int32_t i = 0; i < ARRAYSIZE(input_messages); ++i) {
    TetherUpInit(&input_messages[i]);
    input_messages[i].joystick.no_update_count = 0;
    input_messages[i].joystick.throttle = static_cast<float>(i);
  }
  memset(&output, 0x5A, sizeof(output));

  // Define input to select message index 1 when all are valid and all
  // no_update_counts are equal.
  input_messages[0].frame_index = TETHER_FRAME_INDEX_ROLLOVER - 1;
  input_messages[1].frame_index = 2;
  input_messages[2].frame_index = 1;

  // Test all messages valid and equal no_update_counts.
  input_messages[0].joystick.no_update_count = 1;
  input_messages[1].joystick.no_update_count = 1;
  input_messages[2].joystick.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByFrameIndex(message, field, ARRAYSIZE(input_messages),
                                     input_messages, &output));
  EXPECT_NEAR(1.0f, output.joystick.throttle, FLT_EPSILON);

  // Test all messages valid and not equal no_update_counts.
  input_messages[0].joystick.no_update_count = 1;
  input_messages[1].joystick.no_update_count = 3;
  input_messages[2].joystick.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByFrameIndex(message, field, ARRAYSIZE(input_messages),
                                     input_messages, &output));
  EXPECT_NEAR(2.0f, output.joystick.throttle, FLT_EPSILON);

  // Test invalid no_update_count.
  input_messages[0].joystick.no_update_count = 0;
  input_messages[1].joystick.no_update_count = INT32_MAX;
  input_messages[2].joystick.no_update_count = 0;
  EXPECT_TRUE(MergeFieldByFrameIndex(message, field, ARRAYSIZE(input_messages),
                                     input_messages, &output));
  EXPECT_NEAR(2.0f, output.joystick.throttle, FLT_EPSILON);

  // Test all invalid inputs, according to no_update_count.
  input_messages[0].joystick.no_update_count = INT32_MAX;
  input_messages[1].joystick.no_update_count = INT32_MAX;
  input_messages[2].joystick.no_update_count = INT32_MAX;
  EXPECT_FALSE(MergeFieldByFrameIndex(message, field, ARRAYSIZE(input_messages),
                                      input_messages, &output));

  // Test zero inputs.
  EXPECT_FALSE(MergeFieldByFrameIndex(message, field, 0, input_messages,
                                      &output));
  EXPECT_FALSE(MergeFieldByFrameIndex(message, field, 0, nullptr, &output));
}

TEST(MergeTetherByGpsTime, WithNoUpdateCount) {
  const TetherMessageInfo *message = TetherUpGetMessageInfo();
  ASSERT_NE(nullptr, message);

  const TetherFieldInfo *field =
      FindTetherMessageField(message, OFFSETOF(TetherUpMessage, joystick));
  ASSERT_NE(nullptr, field);
  ASSERT_LE(0, field->offsetof_no_update_count);

  TetherUpMessage input_messages[3], output;
  for (int32_t i = 0; i < ARRAYSIZE(input_messages); ++i) {
    TetherUpInit(&input_messages[i]);
    input_messages[i].gps_time.no_update_count = 0;
    input_messages[i].joystick.no_update_count = 0;
    input_messages[i].joystick.throttle = static_cast<float>(i);
  }
  memset(&output, 0x5A, sizeof(output));

  // Define input to select message index 1 when all are valid and all
  // no_update_counts are equal.
  input_messages[0].gps_time.time_of_week =
      TETHER_GPS_TIME_OF_WEEK_ROLLOVER - 1;
  input_messages[1].gps_time.time_of_week =
      2 * TETHER_DOWN_PERIOD_US / 1000 - 1;
  input_messages[2].gps_time.time_of_week =
      1 * TETHER_DOWN_PERIOD_US / 1000 - 1;

  // Test all messages valid and equal no_update_counts.
  input_messages[0].gps_time.no_update_count = 1;
  input_messages[0].joystick.no_update_count = 1;
  input_messages[1].gps_time.no_update_count = 1;
  input_messages[1].joystick.no_update_count = 1;
  input_messages[2].gps_time.no_update_count = 1;
  input_messages[2].joystick.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_NEAR(1.0f, output.joystick.throttle, FLT_EPSILON);

  // Test all messages valid and not equal no_update_counts.
  input_messages[0].gps_time.no_update_count = 1;
  input_messages[0].joystick.no_update_count = 1;
  input_messages[1].gps_time.no_update_count = 0;
  input_messages[1].joystick.no_update_count = 2;
  input_messages[2].gps_time.no_update_count = 1;
  input_messages[2].joystick.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_NEAR(2.0f, output.joystick.throttle, FLT_EPSILON);

  // Test all messages valid and not equal no_update_counts.
  input_messages[0].gps_time.no_update_count = 1;
  input_messages[0].joystick.no_update_count = 1;
  input_messages[1].gps_time.no_update_count = 1;
  input_messages[1].joystick.no_update_count = 3;
  input_messages[2].gps_time.no_update_count = 1;
  input_messages[2].joystick.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_NEAR(2.0f, output.joystick.throttle, FLT_EPSILON);

  // Test invalid no_update_count.
  input_messages[0].gps_time.no_update_count = 1;
  input_messages[0].joystick.no_update_count = 1;
  input_messages[1].gps_time.no_update_count = 1;
  input_messages[1].joystick.no_update_count = INT32_MAX;
  input_messages[2].gps_time.no_update_count = 1;
  input_messages[2].joystick.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_NEAR(2.0f, output.joystick.throttle, FLT_EPSILON);

  // Test invalid no_update_count.
  input_messages[0].gps_time.no_update_count = 1;
  input_messages[0].joystick.no_update_count = 1;
  input_messages[1].gps_time.no_update_count = INT32_MAX;
  input_messages[1].joystick.no_update_count = 1;
  input_messages[2].gps_time.no_update_count = 1;
  input_messages[2].joystick.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_NEAR(2.0f, output.joystick.throttle, FLT_EPSILON);

  // Test all invalid inputs, according to no_update_count.
  input_messages[0].joystick.no_update_count = INT32_MAX;
  input_messages[1].joystick.no_update_count = INT32_MAX;
  input_messages[2].joystick.no_update_count = INT32_MAX;
  EXPECT_FALSE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                   input_messages, &output));

  // Test zero inputs.
  EXPECT_FALSE(MergeFieldByGpsTime(message, field, 0, input_messages, &output));
  EXPECT_FALSE(MergeFieldByGpsTime(message, field, 0, nullptr, &output));
}

TEST(MergeTetherByGpsTime, WithoutNoUpdateCount) {
  const TetherMessageInfo *message = TetherUpGetMessageInfo();
  ASSERT_NE(nullptr, message);

  const TetherFieldInfo *field = FindTetherMessageField(
      message, OFFSETOF(TetherUpMessage, received_signal_strength));
  ASSERT_NE(nullptr, field);
  ASSERT_GT(0, field->offsetof_no_update_count);

  TetherUpMessage input_messages[3], output;
  for (int32_t i = 0; i < ARRAYSIZE(input_messages); ++i) {
    TetherUpInit(&input_messages[i]);
    input_messages[i].gps_time.no_update_count = 0;
    input_messages[i].received_signal_strength = static_cast<int16_t>(-i);
  }
  memset(&output, 0x5A, sizeof(output));

  // Define input to select message index 1 when all are valid and all
  // no_update_counts are equal.
  input_messages[0].gps_time.time_of_week =
      TETHER_GPS_TIME_OF_WEEK_ROLLOVER - 1;
  input_messages[1].gps_time.time_of_week =
      2 * TETHER_DOWN_PERIOD_US / 1000 - 1;
  input_messages[2].gps_time.time_of_week =
      1 * TETHER_DOWN_PERIOD_US / 1000 - 1;

  // Test all messages valid and equal no_update_counts.
  input_messages[0].gps_time.no_update_count = 1;
  input_messages[1].gps_time.no_update_count = 1;
  input_messages[2].gps_time.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_EQ(-1, output.received_signal_strength);

  // Test all messages valid and not equal no_update_counts.
  input_messages[0].gps_time.no_update_count = 2;
  input_messages[1].gps_time.no_update_count = 0;
  input_messages[2].gps_time.no_update_count = 2;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_EQ(-2, output.received_signal_strength);

  // Test invalid no_update_count.
  input_messages[0].gps_time.no_update_count = 1;
  input_messages[1].gps_time.no_update_count = INT32_MAX;
  input_messages[2].gps_time.no_update_count = 1;
  EXPECT_TRUE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                  input_messages, &output));
  EXPECT_EQ(-2, output.received_signal_strength);

  // Test all invalid inputs, according to no_update_count.
  input_messages[0].gps_time.no_update_count = INT32_MAX;
  input_messages[1].gps_time.no_update_count = INT32_MAX;
  input_messages[2].gps_time.no_update_count = INT32_MAX;
  EXPECT_FALSE(MergeFieldByGpsTime(message, field, ARRAYSIZE(input_messages),
                                   input_messages, &output));

  // Test zero inputs.
  EXPECT_FALSE(MergeFieldByGpsTime(message, field, 0, input_messages, &output));
  EXPECT_FALSE(MergeFieldByGpsTime(message, field, 0, nullptr, &output));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
