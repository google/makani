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

#include "nav/ins/messages/seq_num.h"

#include <stdint.h>

void InsSequenceZeroSnapshot(InsSequenceSnapshot *snapshot) {
  snapshot->seq_bitmask = 0U;
}

void InsSequenceUpdateSnapshot(uint16_t seq_num,
                               InsSequenceSnapshot *snapshot) {
  if (snapshot->seq_bitmask & 1U) {
    int16_t delta = InsSequenceDiff(seq_num, snapshot->seq_num);
    if (0 <= delta && delta < 8 * (int32_t)sizeof(snapshot->seq_bitmask)) {
      snapshot->seq_bitmask <<= delta;
      snapshot->seq_bitmask |= 1U;
    } else {
      snapshot->seq_bitmask = 1U;
    }
  } else {
    snapshot->seq_bitmask = 1U;
  }
  snapshot->seq_num = seq_num;
}
