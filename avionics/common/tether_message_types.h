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

#ifndef AVIONICS_COMMON_TETHER_MESSAGE_TYPES_H_
#define AVIONICS_COMMON_TETHER_MESSAGE_TYPES_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "avionics/common/avionics_messages.h"

// Modulus to apply to frame_index fields in TetherUp and TetherDown messages.
// Define rollover value to ensure consistent multiplexing for all fields.
#define TETHER_FRAME_INDEX_BITS 12
#define TETHER_FRAME_INDEX_ROLLOVER 4080

// Number of frame indices beyond the current frame_index to accept as valid
// messages.
#define TETHER_FRAME_INDEX_ACCEPTANCE_WINDOW 500

// Decimate network message rate to determine radio message rate.
#define TETHER_RADIO_DECIMATION 4

// Decimate the node_status field in TetherDown.
#define TETHER_NODE_STATUS_DECIMATION (TETHER_RADIO_DECIMATION * 4)

// Decimate control telemetry field in TetherDown.
#define TETHER_CONTROL_TELEMETRY_DECIMATION (TETHER_RADIO_DECIMATION * 2)

// Modulus to apply to AIO sequence number. Sequence numbers greater than or
// equal to TETHER_SEQUENCE_ROLLOVER are not transmitted over the long range
// radio.
#define TETHER_SEQUENCE_BITS 12
#define TETHER_SEQUENCE_ROLLOVER (1 << TETHER_SEQUENCE_BITS)

// Invalid value to mark stale GPS time data.
#define TETHER_GPS_TIME_OF_WEEK_ROLLOVER (3600 * 24 * 7 * 1000)
#define TETHER_GPS_TIME_OF_WEEK_INVALID TETHER_GPS_TIME_OF_WEEK_ROLLOVER

typedef enum {
  kTetherMergeTrunkForceSigned = -1,
  kTetherMergeTrunkA,
  kTetherMergeTrunkB,
  kNumTetherMergeTrunks,
} TetherMergeTrunk;

// Do not reference the internals of this structure!
typedef struct {
  // Inputs correspond to a particular TetherDownSource.
  TetherDownMessage input_messages[kNumTetherDownSources];
  uint16_t input_sequence_numbers[kNumTetherDownSources];
  int64_t input_timestamps[kNumTetherDownSources];
  bool input_updated[kNumTetherDownSources];

  // Controller inputs correspond to the ControllerCommand message.
  ControllerCommandMessage controller_message;
  uint16_t controller_sequence_number;
  int64_t controller_timestamp;
  bool controller_updated;

  // Trunks corresponds to a particular TetherMergeTrunk.
  TetherDownMessage trunk_messages[kNumTetherMergeTrunks];
  int64_t trunk_timestamps[kNumTetherMergeTrunks];

  // Output.
  TetherDownMessage output_message;
  int64_t output_timestamp;
} TetherDownMergeState;

// Do not reference the internals of this structure!
typedef struct {
  // Inputs correspond to a particular TetherUpSource.
  TetherUpMessage input_messages[kNumTetherUpSources];
  uint16_t input_sequence_numbers[kNumTetherUpSources];
  int64_t input_timestamps[kNumTetherUpSources];
  bool input_updated[kNumTetherUpSources];

  // Joystick inputs correspond to the JoystickStatus message.
  JoystickStatusMessage joystick_message;
  uint16_t joystick_sequence_number;
  int64_t joystick_timestamp;
  bool joystick_updated;

  // Trunks corresponds to a particular TetherMergeTrunk.
  TetherUpMessage trunk_messages[kNumTetherMergeTrunks];
  int64_t trunk_timestamps[kNumTetherMergeTrunks];

  // Output.
  TetherUpMessage output_message;
  int64_t output_timestamp;
} TetherUpMergeState;

typedef struct {
  const char *name;
  size_t sizeof_field;
  int32_t offsetof_field;
  int32_t offsetof_no_update_count;  // Set to -1 to disable.
  int32_t offsetof_sequence;  // Set to -1 to disable.
} TetherFieldInfo;

typedef struct {
  size_t sizeof_message;
  int32_t offsetof_frame_index;
  int32_t offsetof_gps_time;
  const TetherFieldInfo *fields;
  int32_t num_fields;
} TetherMessageInfo;

#endif  // AVIONICS_COMMON_TETHER_MESSAGE_TYPES_H_
