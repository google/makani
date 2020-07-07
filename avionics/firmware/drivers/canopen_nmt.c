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

#include "avionics/firmware/drivers/canopen_nmt.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define NMT_RESPONSE_VALUE 0x700
#define NMT_RESPONSE_MASK 0x780
#define NMT_SYNC_ID 0x80

// CANopen NMT summary at http://www.can-cia.org/index.php?id=155
// Referenced description from http://www.a-m-c.com/download/sw/dw300_3-0-3/CAN_Manual300_3-0-3.pdf

void CanopenNmtInit(const CanopenNmtConfig *config) {
  assert(config != NULL);

  DcanSetTransmitMailbox(config->bus, config->transmit_mailbox, kDcanIdStandard,
                         DCAN_STANDARD_MASK, 0, 2);
  DcanSetReceiveMailbox(config->bus, config->receive_mailbox, kDcanIdStandard,
                        DCAN_STANDARD_MASK & NMT_RESPONSE_MASK,
                        NMT_RESPONSE_VALUE, 1);
  DcanSetTransmitMailbox(config->bus, config->transmit_sync_mailbox,
                         kDcanIdStandard, DCAN_STANDARD_MASK, NMT_SYNC_ID, 0);
}

bool CanopenNmtSend(const CanopenNmtConfig *config, uint8_t node_id,
                    CanopenNmtMessage message) {
  assert(config != NULL);

  uint8_t can_buf[2];
  can_buf[0] = (uint8_t)message;
  can_buf[1] = node_id;

  return DcanTransmit(config->bus, config->transmit_mailbox, 2, can_buf);
}

bool CanopenNmtGetState(const CanopenNmtConfig *config, uint8_t *node_id,
                        CanopenNmtState *state) {
  assert(config != NULL);
  assert(node_id != NULL && state != NULL);

  uint32_t cob_id;
  uint8_t response;

  if (DcanGetMailbox(config->bus, config->receive_mailbox, 1, &response,
                     &cob_id)) {
    *state = (CanopenNmtState)(response & 0x7F);
    // cob_id = 0x700 + node_id (7 bits).
    *node_id = (uint8_t)(cob_id & 0x7F);
    return true;
  }
  return false;
}

bool CanopenNmtSendSync(const CanopenNmtConfig *config) {
  assert(config != NULL);

  uint8_t null = {0};

  return DcanTransmit(config->bus, config->transmit_sync_mailbox, 0, &null);
}
