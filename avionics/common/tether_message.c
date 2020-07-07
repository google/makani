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

#include "avionics/common/tether_message.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/cvt_avionics_messages.h"
#include "avionics/common/tether_convert.h"
#include "avionics/common/tether_cvt.h"
#include "avionics/common/tether_message_types.h"
#include "avionics/network/aio_labels.h"
#include "avionics/network/message_stats.h"
#include "common/c_math/util.h"
#include "common/macros.h"
#include "system/labels.h"

// TETHER_UP_PERIOD_US and TETHER_DOWN_PERIOD_US represent the Ethernet
// transmit period as defined by network.yaml.
COMPILE_ASSERT(TETHER_UP_PERIOD_US == TETHER_DOWN_PERIOD_US,
               TetherUp_and_TetherDown_must_operate_at_the_same_frequency);

#define MAX_NO_UPDATE_COUNT TETHER_FRAME_INDEX_ACCEPTANCE_WINDOW

#define TETHER_SEQUENCE_UPDATED(type, field) {STR(field),       \
        SIZEOF(type, field),                                    \
        OFFSETOF(type, field),                                  \
        OFFSETOF(type, field.no_update_count),                  \
        OFFSETOF(type, field.sequence)}

#define TETHER_TIME_UPDATED(type, field) {STR(field),   \
        SIZEOF(type, field),                            \
        OFFSETOF(type, field),                          \
        OFFSETOF(type, field.no_update_count),          \
        -1}  /* Set offsetof_sequence invalid. */

#define TETHER_TIME(type, field) {STR(field),                   \
        SIZEOF(type, field),                                    \
        OFFSETOF(type, field),                                  \
        -1,  /* Set offsetof_no_update_count invalid. */        \
        -1}  /* Set offsetof_sequence invalid. */

#define TETHER_UP_SEQUENCE_UPDATED(field)               \
  TETHER_SEQUENCE_UPDATED(TetherUpMessage, field)
#define TETHER_UP_TIME_UPDATED(field)           \
  TETHER_TIME_UPDATED(TetherUpMessage, field)
#define TETHER_UP_TIME(field) TETHER_TIME(TetherUpMessage, field)

#define TETHER_DOWN_SEQUENCE_UPDATED(field)             \
  TETHER_SEQUENCE_UPDATED(TetherDownMessage, field)
#define TETHER_DOWN_TIME_UPDATED(field)         \
  TETHER_TIME_UPDATED(TetherDownMessage, field)
#define TETHER_DOWN_TIME(field) TETHER_TIME(TetherDownMessage, field)

static const TetherFieldInfo kTetherUpFields[] = {
  TETHER_UP_SEQUENCE_UPDATED(drum_a),
  TETHER_UP_SEQUENCE_UPDATED(drum_b),
  TETHER_UP_SEQUENCE_UPDATED(gps_compass),
  TETHER_UP_SEQUENCE_UPDATED(gps_position),
  TETHER_UP_SEQUENCE_UPDATED(gps_status),
  TETHER_UP_SEQUENCE_UPDATED(ground_station),
  TETHER_UP_SEQUENCE_UPDATED(joystick),
  TETHER_UP_SEQUENCE_UPDATED(platform_a),
  TETHER_UP_SEQUENCE_UPDATED(platform_b),
  TETHER_UP_SEQUENCE_UPDATED(plc),
  TETHER_UP_SEQUENCE_UPDATED(weather),
  TETHER_UP_SEQUENCE_UPDATED(wind),
  TETHER_UP_TIME(frame_index),
  TETHER_UP_TIME(received_signal_strength),
  TETHER_UP_TIME_UPDATED(gps_time),
};

static const TetherMessageInfo kTetherUpMessageInfo = {
  .sizeof_message = sizeof(TetherUpMessage),
  .offsetof_frame_index = OFFSETOF(TetherUpMessage, frame_index),
  .offsetof_gps_time = OFFSETOF(TetherUpMessage, gps_time),
  .fields = kTetherUpFields,
  .num_fields = ARRAYSIZE(kTetherUpFields),
};

static const TetherFieldInfo kTetherDownFields[] = {
  // We cannot de-duplicate TetherControlCommand or TetherControlTelemetry
  // using sequence numbers because the core switches make independent
  // arbitration decisions on data from the controllers.
  TETHER_DOWN_SEQUENCE_UPDATED(gps_statuses[kWingGpsReceiverCrosswind]),
  TETHER_DOWN_SEQUENCE_UPDATED(gps_statuses[kWingGpsReceiverHover]),
  TETHER_DOWN_SEQUENCE_UPDATED(gps_statuses[kWingGpsReceiverPort]),
  TETHER_DOWN_SEQUENCE_UPDATED(gps_statuses[kWingGpsReceiverStar]),
  TETHER_DOWN_TIME(frame_index),
  TETHER_DOWN_TIME(received_signal_strength),
  TETHER_DOWN_TIME_UPDATED(batt_a),
  TETHER_DOWN_TIME_UPDATED(batt_b),
  TETHER_DOWN_TIME_UPDATED(comms_status),
  TETHER_DOWN_TIME_UPDATED(control_command),
  TETHER_DOWN_TIME_UPDATED(control_telemetry),
  TETHER_DOWN_TIME_UPDATED(flight_computers[kFlightComputerA]),
  TETHER_DOWN_TIME_UPDATED(flight_computers[kFlightComputerB]),
  TETHER_DOWN_TIME_UPDATED(flight_computers[kFlightComputerC]),
  TETHER_DOWN_TIME_UPDATED(gps_time),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorPbi]),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorPbo]),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorPti]),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorPto]),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorSbi]),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorSbo]),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorSti]),
  TETHER_DOWN_TIME_UPDATED(motor_statuses[kMotorSto]),
  TETHER_DOWN_TIME_UPDATED(mvlv),
  TETHER_DOWN_TIME_UPDATED(node_status),
  TETHER_DOWN_TIME_UPDATED(release_statuses[kLoadcellNodePortA]),
  TETHER_DOWN_TIME_UPDATED(release_statuses[kLoadcellNodePortB]),
  TETHER_DOWN_TIME_UPDATED(release_statuses[kLoadcellNodeStarboardA]),
  TETHER_DOWN_TIME_UPDATED(release_statuses[kLoadcellNodeStarboardB]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoA1]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoA2]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoA4]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoA5]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoA7]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoA8]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoE1]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoE2]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoR1]),
  TETHER_DOWN_TIME_UPDATED(servo_statuses[kServoR2]),
};

static const TetherMessageInfo kTetherDownMessageInfo = {
  .sizeof_message = sizeof(TetherDownMessage),
  .offsetof_frame_index = OFFSETOF(TetherDownMessage, frame_index),
  .offsetof_gps_time = OFFSETOF(TetherDownMessage, gps_time),
  .fields = kTetherDownFields,
  .num_fields = ARRAYSIZE(kTetherDownFields),
};

static void IncGpsTime(int32_t inc_ms, TetherGpsTime *time) {
  if (TetherIsGpsTimeValid(time)) {
    time->time_of_week = WrapInt32(time->time_of_week + inc_ms, 0,
                                   TETHER_GPS_TIME_OF_WEEK_ROLLOVER);
  }
}

static void IncFrameIndex(int32_t inc, uint16_t *frame_index) {
  *frame_index = (uint16_t)WrapInt32(*frame_index + inc, 0,
                                     TETHER_FRAME_INDEX_ROLLOVER);
}

static const void *GetMessage(const TetherMessageInfo *info,
                              const void *messages, int32_t index) {
  assert(info->sizeof_message > 0);

  return (const uint8_t *)messages + index * (int32_t)info->sizeof_message;
}

static uint16_t GetMessageFrameIndex(const TetherMessageInfo *info,
                                     const void *m) {
  if (info->offsetof_frame_index >= 0) {
    const void *p = (const uint8_t *)m + info->offsetof_frame_index;

    uint16_t frame_index;
    memcpy(&frame_index, p, sizeof(frame_index));
    return frame_index;
  }
  return 0U;
}

static bool GetMessageGpsTime(const TetherMessageInfo *info, const void *m,
                              TetherGpsTime *value) {
  if (info->offsetof_gps_time >= 0) {
    assert(value != NULL);
    const void *p = (const uint8_t *)m + info->offsetof_gps_time;

    memcpy(value, p, sizeof(*value));
    return TetherIsGpsTimeValid(value);
  }
  return false;
}

// Argument 'value' may be NULL.
static bool GetFieldNoUpdateCount(const TetherFieldInfo *info, const void *m,
                                  int32_t *value) {
  if (info->offsetof_no_update_count >= 0) {
    const void *p = (const uint8_t *)m + info->offsetof_no_update_count;

    int32_t no_update_count;
    memcpy(&no_update_count, p, sizeof(no_update_count));
    if (value != NULL) {
      *value = no_update_count;
    }
    return true;
  }
  return false;
}

static void SetFieldNoUpdateCount(const TetherFieldInfo *info, int32_t value,
                                  void *m) {
  if (info->offsetof_no_update_count >= 0) {
    void *p = (uint8_t *)m + info->offsetof_no_update_count;

    memcpy(p, &value, sizeof(value));
  }
}

static void IncrementFieldNoUpdateCount(const TetherFieldInfo *info,
                                        int32_t inc, void *m) {
  int32_t no_update_count;
  if (GetFieldNoUpdateCount(info, m, &no_update_count)) {
    TetherIncrementNoUpdateCount(inc, &no_update_count);
  } else {
    no_update_count = INT32_MAX;
  }
  SetFieldNoUpdateCount(info, no_update_count, m);
}

static void IncrementMessageNoUpdateCounts(const TetherMessageInfo *info,
                                           int32_t count, void *message) {
  for (int32_t i = 0; i < info->num_fields; ++i) {
    IncrementFieldNoUpdateCount(&info->fields[i], count, message);
  }
}

static uint16_t GetFieldSequence(const TetherFieldInfo *info, const void *m) {
  if (info->offsetof_sequence >= 0) {
    const void *p = (const uint8_t *)m + info->offsetof_sequence;

    uint16_t seq;
    memcpy(&seq, p, sizeof(seq));
    return seq;
  }
  return 0U;
}

// Argument 'in' may be NULL.
static bool CopyField(const TetherFieldInfo *info, const void *in, void *out) {
  assert(info->offsetof_field >= 0);
  assert(info->sizeof_field > 0);

  if (in != NULL) {
    memcpy((uint8_t *)out + info->offsetof_field,
           (const uint8_t *)in + info->offsetof_field, info->sizeof_field);
    return true;
  }
  return false;
}

// Merge field using the originating node's sequence number.
static bool MergeFieldBySequence(const TetherMessageInfo *message,
                                 const TetherFieldInfo *field,
                                 int32_t num_inputs, const void *input_messages,
                                 void *output) {
  const void *best_in = NULL;
  uint16_t best_sequence = 0U;

  for (int32_t i = 0; i < num_inputs; ++i) {
    const void *in = GetMessage(message, input_messages, i);

    // Use no_update_count to determine if sequence number is valid.
    int32_t in_no_update_count;
    if (GetFieldNoUpdateCount(field, in, &in_no_update_count)
        && TetherIsNoUpdateCountValid(in_no_update_count)) {
      uint16_t in_sequence = GetFieldSequence(field, in);

      if (best_in == NULL
          || TetherCompareSequence(in_sequence, best_sequence) > 0) {
        best_in = in;
        best_sequence = in_sequence;
      }
    }
  }
  return CopyField(field, best_in, output);
}

// Merge fields using the frame_index and no_update_count. We can subtract
// no_update_count from frame_index to determine the frame_index when the
// field updated.
static bool MergeFieldByFrameIndex(const TetherMessageInfo *message,
                                   const TetherFieldInfo *field,
                                   int32_t num_inputs,
                                   const void *input_messages, void *output) {
  const void *best_in = NULL;
  uint16_t best_index = 0U;

  for (int32_t i = 0; i < num_inputs; ++i) {
    const void *in = GetMessage(message, input_messages, i);

    // Use no_update_count to determine if frame_index is valid.
    int32_t in_no_update_count;
    if (GetFieldNoUpdateCount(field, in, &in_no_update_count)
        && TetherIsNoUpdateCountValid(in_no_update_count)) {
      uint16_t in_index = GetMessageFrameIndex(message, in);
      IncFrameIndex(-in_no_update_count, &in_index);

      if (best_in == NULL
          || TetherCompareFrameIndex(in_index, best_index) > 0) {
        best_in = in;
        best_index = in_index;
      }
    }
  }
  return CopyField(field, best_in, output);
}

// Merge field using the message's GPS time-stamp. This approach assumes we
// have GPS time-of-week information in all messages.
static bool MergeFieldByGpsTime(const TetherMessageInfo *message,
                                const TetherFieldInfo *field,
                                int32_t num_inputs, const void *input_messages,
                                void *output) {
  const void *best_in = NULL;
  TetherGpsTime best_time = {
    .no_update_count = INT32_MAX,
    .time_of_week = TETHER_GPS_TIME_OF_WEEK_INVALID
  };

  for (int32_t i = 0; i < num_inputs; ++i) {
    const void *in = GetMessage(message, input_messages, i);

    int32_t in_no_update_count = 0;
    bool valid_no_update_count = field->offsetof_no_update_count < 0
        || (GetFieldNoUpdateCount(field, in, &in_no_update_count)
            && TetherIsNoUpdateCountValid(in_no_update_count));

    TetherGpsTime in_time;
    if (GetMessageGpsTime(message, in, &in_time) && valid_no_update_count) {
      // Compute GpsTime of current field using last TetherGpsTime update.
      int32_t no_updates = in_no_update_count - in_time.no_update_count;
      int32_t age_ms = no_updates * TETHER_DOWN_PERIOD_US / 1000;
      IncGpsTime(-age_ms, &in_time);

      if (best_in == NULL || TetherCompareGpsTime(&in_time, &best_time) > 0) {
        best_in = in;
        best_time = in_time;
      }
    }
  }
  return CopyField(field, best_in, output);
}

// Merge field using the message's receive time-stamp. This approach assumes
// messages arrive without transport delay.
static bool MergeFieldByReceiveTime(const TetherMessageInfo *message,
                                    const TetherFieldInfo *field,
                                    int32_t num_inputs,
                                    const int64_t input_timestamps[],
                                    const void *input_messages, void *output) {
  const void *best_in = NULL;
  int64_t best_time = 0;

  for (int32_t i = 0; i < num_inputs; ++i) {
    const void *in = GetMessage(message, input_messages, i);

    int32_t no_update_count = 0;
    if (field->offsetof_no_update_count < 0
        || GetFieldNoUpdateCount(field, in, &no_update_count)) {
      int64_t age_us = no_update_count * TETHER_DOWN_PERIOD_US;
      int64_t in_time = input_timestamps[i] - age_us;

      if (best_in == NULL || in_time > best_time) {
        best_in = in;
        best_time = in_time;
      }
    }
  }
  return CopyField(field, best_in, output);
}

static void InitMessage(const TetherMessageInfo *message, void *m) {
  memset(m, 0, message->sizeof_message);

  for (int32_t i = 0; i < message->num_fields; ++i) {
    SetFieldNoUpdateCount(&message->fields[i], INT32_MAX, m);
  }
}

static int64_t GetMaxTimestamp(int32_t num_messages,
                               const int64_t timestamps[]) {
  int64_t max_timestamp = 0;
  for (int32_t i = 0; i < num_messages; ++i) {
    if (timestamps[i] > max_timestamp) {
      max_timestamp = timestamps[i];
    }
  }
  return max_timestamp;
}

static bool MergeField(const TetherMessageInfo *message,
                       const TetherFieldInfo *field, bool one_source,
                       int32_t num_inputs, const int64_t input_timestamps[],
                       const void *input_messages, void *output_message) {
  // Sort merging approaches according to their reliability.
  if (field->offsetof_sequence >= 0
      && MergeFieldBySequence(message, field, num_inputs, input_messages,
                              output_message)) {
    return true;
  }

  // We can only compare frame indices from messages originating from the
  // same source. On network A, CsA generates the message, then sends it
  // over the network and long range radio. CsGsA receives the long range
  // radio transmission, then unpacks the message and sends the message on
  // the network. This message originated from CsA and therefore we can
  // compare CsA's and CsGsA's TetherDown. We, however, cannot compare CsA
  // with CsB.
  if (field->offsetof_no_update_count >= 0 && one_source
      && MergeFieldByFrameIndex(message, field, num_inputs, input_messages,
                                output_message)) {
    return true;
  }

  if (message->offsetof_gps_time >= 0
      && MergeFieldByGpsTime(message, field, num_inputs, input_messages,
                             output_message)) {
    return true;
  }

  if (MergeFieldByReceiveTime(message, field, num_inputs, input_timestamps,
                              input_messages, output_message)) {
    return true;
  }

  // Unable to merge inputs. Output should default to invalid.
  return false;
}

static void MergeFields(const TetherMessageInfo *message, bool one_source,
                        int32_t num_inputs, const int64_t input_timestamps[],
                        const void *input_messages, int64_t *output_timestamp,
                        void *output_message) {
  InitMessage(message, output_message);
  for (int32_t i = 0; i < message->num_fields; ++i) {
    MergeField(message, &message->fields[i], one_source, num_inputs,
               input_timestamps, input_messages, output_message);
  }
  *output_timestamp = GetMaxTimestamp(num_inputs, input_timestamps);
}

static bool IsSameTetherDownTrunk(TetherMergeTrunk trunk, int32_t source) {
  switch (trunk) {
    case kTetherMergeTrunkA:
      return source == kTetherDownSourceCsA || source == kTetherDownSourceCsGsA;
    case kTetherMergeTrunkB:
      return source == kTetherDownSourceCsB;
    case kTetherMergeTrunkForceSigned:
    case kNumTetherMergeTrunks:
    default:
      assert(false);
      return false;
  }
}

static bool IsSameTetherUpTrunk(TetherMergeTrunk trunk, int32_t source) {
  switch (trunk) {
    case kTetherMergeTrunkA:
      return source == kTetherUpSourceCsA || source == kTetherUpSourceCsGsA;
    case kTetherMergeTrunkB:
      return source == kTetherUpSourceCsGsB;
    case kTetherMergeTrunkForceSigned:
    case kNumTetherMergeTrunks:
    default:
      assert(false);
      return false;
  }
}

// TetherDown functions.

void TetherDownInit(TetherDownMessage *message) {
  InitMessage(&kTetherDownMessageInfo, message);
}

void TetherDownMergeStateInit(TetherDownMergeState *state) {
  memset(state, 0, sizeof(*state));

  for (int32_t i = 0; i < ARRAYSIZE(state->input_messages); ++i) {
    TetherDownInit(&state->input_messages[i]);
  }
  for (int32_t i = 0; i < ARRAYSIZE(state->trunk_messages); ++i) {
    TetherDownInit(&state->trunk_messages[i]);
  }
  TetherDownInit(&state->output_message);
}

const TetherDownMessage *TetherDownMergeInputs(TetherDownMergeState *state) {
  // Merge by trunk link, then merge the result of all trunk links. Each trunk
  // link sends the same message, but may multiplex fields such that some
  // fields are more current than others. We can use the frame_index and field
  // no_update_count to compute the frame_index corresponding to the field
  // update, then determine the most recent field by comparing the result
  // across messages from the same trunk link. We can then merge messages from
  // multiple trunk links using GPS time or sequence numbers.
  for (int32_t trunk = 0; trunk < ARRAYSIZE(state->trunk_messages); ++trunk) {
    IncrementMessageNoUpdateCounts(&kTetherDownMessageInfo, 1,
                                   &state->trunk_messages[trunk]);

    TetherDownMessage in_messages[1 + kNumTetherDownSources];
    int64_t in_timestamps[1 + kNumTetherDownSources];

    int32_t num_messages = 1;
    in_messages[0] = state->trunk_messages[trunk];
    in_timestamps[0] = state->trunk_timestamps[trunk];

    for (int32_t input = 0; input < ARRAYSIZE(state->input_messages); ++input) {
      if (state->input_updated[input] && IsSameTetherDownTrunk(trunk, input)) {
        in_messages[num_messages] = state->input_messages[input];
        in_timestamps[num_messages] = state->input_timestamps[input];
        ++num_messages;
      }
    }

    MergeFields(&kTetherDownMessageInfo, true, num_messages, in_timestamps,
                in_messages, &state->trunk_timestamps[trunk],
                &state->trunk_messages[trunk]);
  }

  // This allows nodes to receive ControllerController messages when there is
  // no TetherDown message processing.  An example of this is commanding the
  // ground station from the command center when there is no wing connected.
  if (state->controller_updated) {
    TetherDownMessage in_messages[2];
    int64_t in_timestamps[2];

    in_timestamps[0] = state->trunk_timestamps[0];
    in_messages[0] = state->trunk_messages[0];

    in_timestamps[1] = state->controller_timestamp;
    TetherDownInit(&in_messages[1]);
    ControllerCommandMessageToTetherControlCommand(
        kControllerA, &state->controller_message,
        state->controller_sequence_number, &in_messages[1].control_command);

    MergeFields(&kTetherDownMessageInfo, false, ARRAYSIZE(in_messages),
                in_timestamps, in_messages, &state->trunk_timestamps[0],
                &state->trunk_messages[0]);
  }

  // Merge all trunk links to determine the final result.
  MergeFields(&kTetherDownMessageInfo, false, kNumTetherMergeTrunks,
              state->trunk_timestamps, state->trunk_messages,
              &state->output_timestamp, &state->output_message);

  return &state->output_message;
}

const TetherDownMessage *TetherDownMergeCvtGet(TetherDownMergeState *state) {
  TetherCvtGetTetherDownMergeInputs(state);
  return TetherDownMergeInputs(state);
}

const TetherDownMessage *TetherDownMergeCvtPeek(TetherDownMergeState *state) {
  TetherCvtPeekTetherDownMergeInputs(state);
  return TetherDownMergeInputs(state);
}

const TetherDownMessage *TetherDownGetMergeInput(
    const TetherDownMergeState *state, TetherDownSource source) {
  assert(0 <= source && source < kNumTetherDownSources);
  return &state->input_messages[source];
}

const TetherDownMessage *TetherDownGetMergeTrunk(
    const TetherDownMergeState *state, TetherMergeTrunk trunk) {
  assert(0 <= trunk && trunk < kNumTetherMergeTrunks);
  return &state->trunk_messages[trunk];
}

const TetherDownMessage *TetherDownGetMergeOutput(
    const TetherDownMergeState *state) {
  return &state->output_message;
}

const TetherMessageInfo *TetherDownGetMessageInfo(void) {
  return &kTetherDownMessageInfo;
}

// TetherUp functions.

void TetherUpInit(TetherUpMessage *message) {
  InitMessage(&kTetherUpMessageInfo, message);
}

void TetherUpMergeStateInit(TetherUpMergeState *state) {
  memset(state, 0, sizeof(*state));

  for (int32_t i = 0; i < ARRAYSIZE(state->input_messages); ++i) {
    TetherUpInit(&state->input_messages[i]);
  }
  for (int32_t i = 0; i < ARRAYSIZE(state->trunk_messages); ++i) {
    TetherUpInit(&state->trunk_messages[i]);
  }
  TetherUpInit(&state->output_message);
}

const TetherUpMessage *TetherUpMergeInputs(TetherUpMergeState *state) {
  // Merge by trunk link, then merge the result of all trunk links. Each trunk
  // link sends the same message, but may multiplex fields such that some
  // fields are more current than others. We can use the frame_index and field
  // no_update_count to compute the frame_index corresponding to the field
  // update, then determine the most recent field by comparing the result
  // across messages from the same trunk link. We can then merge messages from
  // multiple trunk links using GPS time or sequence numbers.
  for (int32_t trunk = 0; trunk < ARRAYSIZE(state->trunk_messages); ++trunk) {
    IncrementMessageNoUpdateCounts(&kTetherUpMessageInfo, 1,
                                   &state->trunk_messages[trunk]);

    TetherUpMessage in_messages[1 + kNumTetherUpSources];
    int64_t in_timestamps[1 + kNumTetherUpSources];

    int32_t num_messages = 1;
    in_messages[0] = state->trunk_messages[trunk];
    in_timestamps[0] = state->trunk_timestamps[trunk];

    for (int32_t input = 0; input < ARRAYSIZE(state->input_messages); ++input) {
      if (state->input_updated[input] && IsSameTetherUpTrunk(trunk, input)) {
        in_messages[num_messages] = state->input_messages[input];
        in_timestamps[num_messages] = state->input_timestamps[input];
        ++num_messages;
      }
    }

    MergeFields(&kTetherUpMessageInfo, true, num_messages, in_timestamps,
                in_messages, &state->trunk_timestamps[trunk],
                &state->trunk_messages[trunk]);
  }

  // kAioNodeJoystickA transmits the JoystickStatusMessage to the controller
  // and core switches CsGsA and CsGsB over multiple links. The core switches
  // translate JoystickStatusMessage into TetherJoystick and transmit it as
  // part of TetherUpMessage. Here, we deduplicate against the TetherUp
  // message.
  if (state->joystick_updated) {
    TetherUpMessage in_messages[2];
    int64_t in_timestamps[2];

    in_timestamps[0] = state->trunk_timestamps[0];
    in_messages[0] = state->trunk_messages[0];

    in_timestamps[1] = state->joystick_timestamp;
    TetherUpInit(&in_messages[1]);
    JoystickStatusMessageToTetherJoystick(&state->joystick_message,
                                          state->joystick_sequence_number,
                                          &in_messages[1].joystick);

    MergeFields(&kTetherUpMessageInfo, false, ARRAYSIZE(in_messages),
                in_timestamps, in_messages, &state->trunk_timestamps[0],
                &state->trunk_messages[0]);
  }

  // Merge all trunk links to determine the final result.
  MergeFields(&kTetherUpMessageInfo, false, ARRAYSIZE(state->trunk_messages),
              state->trunk_timestamps, state->trunk_messages,
              &state->output_timestamp, &state->output_message);

  return &state->output_message;
}

const TetherUpMessage *TetherUpMergeCvtGet(TetherUpMergeState *state) {
  TetherCvtGetTetherUpMergeInputs(state);
  return TetherUpMergeInputs(state);
}

const TetherUpMessage *TetherUpMergeCvtPeek(TetherUpMergeState *state) {
  TetherCvtPeekTetherUpMergeInputs(state);
  return TetherUpMergeInputs(state);
}

const TetherUpMessage *TetherUpGetMergeInput(const TetherUpMergeState *state,
                                             TetherUpSource source) {
  assert(0 <= source && source < kNumTetherUpSources);
  return &state->input_messages[source];
}

const TetherUpMessage *TetherUpGetMergeTrunk(const TetherUpMergeState *state,
                                             TetherMergeTrunk trunk) {
  assert(0 <= trunk && trunk < kNumTetherMergeTrunks);
  return &state->trunk_messages[trunk];
}

const TetherUpMessage *TetherUpGetMergeOutput(const TetherUpMergeState *state) {
  return &state->output_message;
}

const TetherMessageInfo *TetherUpGetMessageInfo(void) {
  return &kTetherUpMessageInfo;
}

// Helper functions.

int32_t TetherCompareFrameIndex(uint16_t a, uint16_t b) {
  assert(a < TETHER_FRAME_INDEX_ROLLOVER);
  assert(b < TETHER_FRAME_INDEX_ROLLOVER);
  return WrapInt32(a - b, -TETHER_FRAME_INDEX_ROLLOVER / 2,
                   TETHER_FRAME_INDEX_ROLLOVER / 2);
}

int32_t TetherCompareSequence(uint16_t a, uint16_t b) {
  assert(a < TETHER_SEQUENCE_ROLLOVER);
  assert(b < TETHER_SEQUENCE_ROLLOVER);
  return WrapInt32(a - b, -TETHER_SEQUENCE_ROLLOVER / 2,
                   TETHER_SEQUENCE_ROLLOVER / 2);  // [counts]
}

int32_t TetherCompareGpsTime(const TetherGpsTime *a, const TetherGpsTime *b) {
  assert(0 <= a->time_of_week
         && a->time_of_week < TETHER_GPS_TIME_OF_WEEK_INVALID);
  assert(0 <= b->time_of_week
         && b->time_of_week < TETHER_GPS_TIME_OF_WEEK_INVALID);
  int32_t delta = a->time_of_week - b->time_of_week;

  return WrapInt32(delta, -TETHER_GPS_TIME_OF_WEEK_INVALID / 2,
                   TETHER_GPS_TIME_OF_WEEK_INVALID / 2);  // [ms]
}

bool TetherIsGpsTimeValid(const TetherGpsTime *t) {
  return 0 <= t->time_of_week
      && t->time_of_week < TETHER_GPS_TIME_OF_WEEK_INVALID
      && TetherIsNoUpdateCountValid(t->no_update_count);
}

void TetherUpIncrementNoUpdateCounts(int32_t inc, TetherUpMessage *message) {
  IncrementMessageNoUpdateCounts(&kTetherUpMessageInfo, inc, message);
}

void TetherDownIncrementNoUpdateCounts(int32_t inc,
                                       TetherDownMessage *message) {
  IncrementMessageNoUpdateCounts(&kTetherDownMessageInfo, inc, message);
}

void TetherIncrementNoUpdateCount(int32_t inc, int32_t *no_update_count) {
  if (inc >= 0 && *no_update_count >= 0 && *no_update_count < INT32_MAX - inc) {
    *no_update_count += inc;
  } else {
    *no_update_count = INT32_MAX;
  }
}

bool TetherIsNoUpdateCountValid(int32_t no_update_count) {
  return 0 <= no_update_count && no_update_count < MAX_NO_UPDATE_COUNT;
}
