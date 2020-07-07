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

#ifndef AVIONICS_COMMON_PACK_TETHER_MESSAGE_H_
#define AVIONICS_COMMON_PACK_TETHER_MESSAGE_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/avionics_messages.h"
#include "avionics/common/tether_op.h"

#ifdef __cplusplus
extern "C" {
#endif

bool PackTetherUp(TetherOpQueue *op_commands, TetherUpMessage *in,
                  TetherUpPackedMessage *out);
bool UnpackTetherUp(TetherOpQueue *op_commands,
                    TetherUpPackedMessage *in, TetherUpMessage *out);
bool UnpackTetherUpFrameIndex(TetherUpPackedMessage *in, uint16_t *frame_index);

bool PackTetherDown(TetherOpQueue *command_replies, TetherDownMessage *in,
                    TetherDownPackedMessage *out);
bool UnpackTetherDown(TetherOpQueue *command_replies,
                      TetherDownPackedMessage *in, TetherDownMessage *out);
bool UnpackTetherDownFrameIndex(TetherDownPackedMessage *in,
                                uint16_t *frame_index);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_PACK_TETHER_MESSAGE_H_
