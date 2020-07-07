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

#ifndef NAV_INS_MESSAGES_SEQ_NUM_H_
#define NAV_INS_MESSAGES_SEQ_NUM_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// This structure provides a compact means to communicate the sequence numbers
// processed by the algorithm.
typedef struct {
  // The most recent sequence number, valid when the LSB of seq_bitmask is set.
  uint16_t seq_num;

  // The seq_bitmask variable describes a shift register of sequence number
  // updates, where the bit number describes the offset from seq_num. If
  // seq_num occurs at time k, then seq_bitmask indicates updates for times
  // k-0, k-1, ..., k-63, where k-0 is the LSB.
  uint64_t seq_bitmask;
} InsSequenceSnapshot;

static inline bool InsSequenceGe(uint16_t a, uint16_t b) {
  return (int16_t)(a - b) >= 0;
}

static inline bool InsSequenceGt(uint16_t a, uint16_t b) {
  return (int16_t)(a - b) > 0;
}

static inline bool InsSequenceLe(uint16_t a, uint16_t b) {
  return (int16_t)(a - b) <= 0;
}

static inline bool InsSequenceLt(uint16_t a, uint16_t b) {
  return (int16_t)(a - b) < 0;
}

static inline int16_t InsSequenceDiff(uint16_t a, uint16_t b) {
  return (int16_t)(a - b);
}

void InsSequenceZeroSnapshot(InsSequenceSnapshot *snapshot);
void InsSequenceUpdateSnapshot(uint16_t seq_num, InsSequenceSnapshot *snapshot);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // NAV_INS_MESSAGES_SEQ_NUM_H_
