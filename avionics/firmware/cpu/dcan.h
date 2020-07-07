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

#ifndef AVIONICS_FIRMWARE_CPU_DCAN_H_
#define AVIONICS_FIRMWARE_CPU_DCAN_H_

#include <stdbool.h>
#include <stdint.h>

#define DCAN_MAILBOXES 64
#define DCAN_STANDARD_MASK 0x07FF      // 11 bits.
#define DCAN_EXTENDED_MASK 0x1FFFFFFF  // 29 bits.

typedef enum {
  kDcanBus1 = 1,  // Values should match memory offsets in registers.h.
  kDcanBus2 = 2,
  kDcanBus3 = 3
} DcanBus;

typedef enum {
  kDcanBitRate1000kbps,
  kDcanBitRate500kbps,
  kDcanBitRate125kbps,
} DcanBitRate;

typedef enum {
  kDcanIdStandard,
  kDcanIdExtended
} DcanIdType;

void DcanInit(DcanBus bus, DcanBitRate bit_rate);
void DcanSetTransmitMailbox(DcanBus bus, int32_t mailbox, DcanIdType type,
                            uint32_t mask, uint32_t id, int32_t dlc);
bool DcanTransmit(DcanBus bus, int32_t mailbox, int32_t length,
                  const uint8_t *data);
void DcanSetReceiveMailbox(DcanBus bus, int32_t mailbox, DcanIdType type,
                           uint32_t mask, uint32_t id, int32_t dlc);
int32_t DcanGetMailbox(DcanBus bus, int32_t mailbox, int32_t length,
                       uint8_t *data, uint32_t *id);

#endif  // AVIONICS_FIRMWARE_CPU_DCAN_H_
