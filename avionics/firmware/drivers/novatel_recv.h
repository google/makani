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

#ifndef AVIONICS_FIRMWARE_DRIVERS_NOVATEL_RECV_H_
#define AVIONICS_FIRMWARE_DRIVERS_NOVATEL_RECV_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/novatel_serial.h"
#include "avionics/common/novatel_types.h"
#include "avionics/firmware/drivers/novatel.h"

void NovAtelReceiveInit(void);
bool NovAtelReceivePoll(NovAtelProto *proto);
void NovAtelInsertDataFromThisPort(int32_t length, const uint8_t *data);
void NovAtelInsertDataFromOtherPort(int32_t length, const uint8_t *data);
bool NovAtelReceivedResponse(NovAtelMessageId message_id);
bool NovAtelReceivedMessage(NovAtelMessageId message_id);
uint32_t NovAtelGetAbbreviatedIndex(void);
uint32_t NovAtelGetPromptIndex(void);

// Output valid until next NovAtelReceivePoll() call.
bool NovAtelGetBinary(const NovAtelHeader **hdr, NovAtelLog *log);
bool NovAtelGetRtcm3(uint16_t *message_number, int32_t *length,
                     const uint8_t **data);

#endif  // AVIONICS_FIRMWARE_DRIVERS_NOVATEL_RECV_H_
