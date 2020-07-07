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

#ifndef AVIONICS_COMMON_TETHER_MESSAGE_H_
#define AVIONICS_COMMON_TETHER_MESSAGE_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/tether_message_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// TetherDown functions.

void TetherDownInit(TetherDownMessage *message);
void TetherDownMergeStateInit(TetherDownMergeState *state);
const TetherDownMessage *TetherDownMergeInputs(TetherDownMergeState *state);
const TetherDownMessage *TetherDownMergeCvtGet(TetherDownMergeState *state);
const TetherDownMessage *TetherDownMergeCvtPeek(TetherDownMergeState *state);
const TetherDownMessage *TetherDownGetMergeInput(
    const TetherDownMergeState *state, TetherDownSource source);
const TetherDownMessage *TetherDownGetMergeTrunk(
    const TetherDownMergeState *state, TetherMergeTrunk trunk);
const TetherDownMessage *TetherDownGetMergeOutput(
    const TetherDownMergeState *state);
const TetherMessageInfo *TetherDownGetMessageInfo(void);

// TetherUp functions.

void TetherUpInit(TetherUpMessage *message);
void TetherUpMergeStateInit(TetherUpMergeState *state);
const TetherUpMessage *TetherUpMergeInputs(TetherUpMergeState *state);
const TetherUpMessage *TetherUpMergeCvtGet(TetherUpMergeState *state);
const TetherUpMessage *TetherUpMergeCvtPeek(TetherUpMergeState *state);
const TetherUpMessage *TetherUpGetMergeInput(const TetherUpMergeState *state,
                                             TetherUpSource source);
const TetherUpMessage *TetherUpGetMergeTrunk(const TetherUpMergeState *state,
                                             TetherMergeTrunk trunk);
const TetherUpMessage *TetherUpGetMergeOutput(const TetherUpMergeState *state);
const TetherMessageInfo *TetherUpGetMessageInfo(void);

// Helper functions.

int32_t TetherCompareFrameIndex(uint16_t a, uint16_t b);
int32_t TetherCompareSequence(uint16_t a, uint16_t b);
int32_t TetherCompareGpsTime(const TetherGpsTime *a, const TetherGpsTime *b);
bool TetherIsGpsTimeValid(const TetherGpsTime *t);
void TetherUpIncrementNoUpdateCounts(int32_t inc, TetherUpMessage *message);
void TetherDownIncrementNoUpdateCounts(int32_t inc, TetherDownMessage *message);
void TetherIncrementNoUpdateCount(int32_t inc, int32_t *no_update_count);
bool TetherIsNoUpdateCountValid(int32_t no_update_count);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_TETHER_MESSAGE_H_
