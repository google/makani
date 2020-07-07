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

#ifndef AVIONICS_COMMON_NOVATEL_BINARY_H_
#define AVIONICS_COMMON_NOVATEL_BINARY_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/common/novatel_types.h"

// NovAtel message format.
#define NOVATEL_SYNC_LENGTH   3
#define NOVATEL_SYNC0         0xAA
#define NOVATEL_SYNC1         0x44
#define NOVATEL_SYNC2         0x12
#define NOVATEL_HEADER_LENGTH 28
#define NOVATEL_CRC_LENGTH    4
#define NOVATEL_CRC_INIT      0xFFFFFFFFU
#define NOVATEL_WRITE_LENGTH  256

bool NovAtelBinaryDecodeHeader(int32_t length, const uint8_t *data,
                               NovAtelHeader *hdr);
bool NovAtelBinaryDecode(const NovAtelHeader *hdr, const uint8_t *data,
                         NovAtelLog *out);

int32_t NovAtelBinaryWriteCom(NovAtelPort port, uint32_t baud,
                              bool enable_break, uint8_t *data);
int32_t NovAtelBinaryWriteInterfaceMode(NovAtelPort port, NovAtelPortMode rx,
                                        NovAtelPortMode tx, uint8_t *data);
int32_t NovAtelBinaryWriteLog(NovAtelPort output_port,
                              NovAtelMessageId message_id, NovAtelFormat format,
                              NovAtelTrigger trigger, double period,
                              double offset, bool hold, uint8_t *data);

#endif  // AVIONICS_COMMON_NOVATEL_BINARY_H_
