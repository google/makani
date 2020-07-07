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

#ifndef AVIONICS_FIRMWARE_DRIVERS_BCM_H_
#define AVIONICS_FIRMWARE_DRIVERS_BCM_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/network/switch_types.h"

typedef enum {
  kPortStatisticForceSigned = -1,
  kPortStatisticRxAlignmentError,
  kPortStatisticRxDiscard,
  kPortStatisticRxDroppedPackets,
  kPortStatisticRxFcsError,
  kPortStatisticRxFragmentError,
  kPortStatisticRxGoodOctets,
  kPortStatisticRxInRangeError,
  kPortStatisticRxJabberError,
  kPortStatisticRxMulticastPackets,
  kPortStatisticRxOctets,
  kPortStatisticRxPausePackets,
  kPortStatisticRxSymbolError,
  kPortStatisticTxDroppedPackets,
  kPortStatisticTxMulticastPackets,
  kPortStatisticTxOctets,
  kPortStatisticTxPausePackets,
  kNumPortStatistics
} PortStatistic;

// Selected Management Information Base (MIB) counters are read from
// access switches and core switches and stored in the MIBCounts structure.
// The structure elements are named after the hardware counters.
// See Tables 160 and 161 in 53101-DS05-R and Tables 271 and 272 in
// 5328XM-DS303-R, respectively, for access switch and core switch MIB
// counter addresses.
typedef struct {
  uint32_t rx_alignment_errors;
  uint32_t rx_dropped_packets;
  uint32_t rx_fcs_errors;
  uint32_t rx_fragment_errors;
  uint64_t rx_good_octets;
  uint32_t rx_in_range_errors;
  uint32_t rx_jabber_errors;
  uint32_t rx_multicast_packets;
  uint64_t rx_octets;
  uint32_t rx_pause_packets;
  uint32_t rx_route_discard;
  uint32_t rx_symbol_errors;
  uint32_t tx_dropped_packets;
  uint32_t tx_multicast_packets;
  uint64_t tx_octets;
  uint32_t tx_pause_packets;
} MibCounts;

void BcmInit(void);
void BcmReset(bool reset);
void BcmPoll(void);
bool BcmIsIdle(void);
bool BcmRead(uint8_t page, uint8_t address, int32_t len, void *buf);
bool BcmWrite(uint8_t page, uint8_t address, int32_t len, const void *buf);

bool BcmReadUint8(uint8_t page, uint8_t address, uint8_t *value);
bool BcmReadUint16(uint8_t page, uint8_t address, uint16_t *value);
bool BcmReadUint32(uint8_t page, uint8_t address, uint32_t *value);
bool BcmReadUint48(uint8_t page, uint8_t address, uint64_t *value);
bool BcmReadUint64(uint8_t page, uint8_t address, uint64_t *value);

bool BcmWriteUint8(uint8_t page, uint8_t address, uint8_t value);
bool BcmWriteUint16(uint8_t page, uint8_t address, uint16_t value);
bool BcmWriteUint32(uint8_t page, uint8_t address, uint32_t value);
bool BcmWriteUint48(uint8_t page, uint8_t address, uint64_t value);
bool BcmWriteUint64(uint8_t page, uint8_t address, uint64_t value);

bool BcmDetectSwitchType(SwitchType *type);

#endif  // AVIONICS_FIRMWARE_DRIVERS_BCM_H_
