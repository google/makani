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

#include "avionics/firmware/cpu/dcan.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/cpu/memory.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

// Standard ID (11 bits) stored left justified in extended ID (29 bits).
#define STD_ID_SHIFT 18

// TODO: Implement IF3/DMA to improve receive performance.
// TODO: Implement error and status monitoring.

static inline bool IsInterface1Busy(DcanBus bus) {
  return DCAN(bus).IF1CMD.BUSY;
}

static inline bool IsInterface2Busy(DcanBus bus) {
  return DCAN(bus).IF2CMD.BUSY;
}

static inline bool IsInInitializationMode(DcanBus bus) {
  return DCAN(bus).CTL.INIT;
}

static inline bool IsInSoftwareReset(DcanBus bus) {
  return DCAN(bus).CTL.SWR;
}

static void SoftwareReset(DcanBus bus) {
  assert(bus == kDcanBus1 || bus == kDcanBus2 || bus == kDcanBus3);

  // Shutdown CAN bus.
  DCAN(bus).CTL.raw = DCAN_CTL_INIT;
  while (!IsInInitializationMode(bus)) {}

  // Software reset.
  DCAN(bus).CTL.raw = DCAN_CTL_INIT | DCAN_CTL_SWR;
  while (IsInSoftwareReset(bus)) {}

  // Initialize DCAN RAM.
  switch (bus) {
    case kDcanBus1:
      MemoryHardwareInit(kMemoryInitDcan1Ram);
      break;
    case kDcanBus2:
      MemoryHardwareInit(kMemoryInitDcan2Ram);
      break;
    case kDcanBus3:
      MemoryHardwareInit(kMemoryInitDcan3Ram);
      break;
    default:
      assert(false);
      return;
  }

  // Initialize mailbox update configuration.
  for (int32_t i = 0; i < ARRAYSIZE(DCAN(bus).IF3UPD); ++i) {
    DCAN(bus).IF3UPD[i].raw = 0x0;
  }
}

// Arguments brp, tseg1, tseg2, and sjw specify values to program to the
// DCAN.BTR register (one less than their respective functional values).
// Argument brp specifies the baud rate prescale (minus 1), tseg1 specifies
// the time quanta before the sample point (minus 1), tseg2 specifies the
// time quanta after the sample point (minus 1), and sjw specifies the
// synchronization jump width time quanta (minus 1). See "CAN Bit Timing" in
// the TRM to determine proper settings.
static void SetBitRate(DcanBus bus, int32_t brp, int32_t tseg1, int32_t tseg2,
                       int32_t sjw) {
  assert(bus == kDcanBus1 || bus == kDcanBus2 || bus == kDcanBus3);
  assert(0 <= brp && brp < 1024);
  assert(1 <= tseg1 && tseg1 < 16);
  assert(0 <= tseg2 && tseg2 < 8);
  assert(0 <= sjw && sjw < 4);
  assert(tseg1 >= sjw && tseg2 >= sjw);

  // Enter initialization mode.
  DCAN(bus).CTL.raw |= DCAN_CTL_INIT;
  DCAN(bus).CTL.raw |= DCAN_CTL_CCE;
  while (!IsInInitializationMode(bus)) {}

  // Set bit rate.
  DCAN(bus).BTR.raw = (((brp << 10) & DCAN_BTR_BRPE_MASK)
                       | ((tseg1 << DCAN_BTR_TSEG1_SHIFT) & DCAN_BTR_TSEG1_MASK)
                       | ((tseg2 << DCAN_BTR_TSEG2_SHIFT) & DCAN_BTR_TSEG2_MASK)
                       | ((sjw << DCAN_BTR_SJW_SHIFT) & DCAN_BTR_SJW_MASK)
                       | ((brp << DCAN_BTR_BRP_SHIFT) & DCAN_BTR_BRP_MASK));

  // Leave initialization mode.
  DCAN(bus).CTL.raw &= ~DCAN_CTL_CCE;
  DCAN(bus).CTL.raw &= ~DCAN_CTL_INIT;
  while (IsInInitializationMode(bus)) {}
}

static uint32_t PackData(int32_t length, const uint8_t *data) {
  // Length bounds handled by for loop.
  assert(data != NULL);
  uint32_t out = 0x0;
  for (int32_t i = 0; i < 4 && i < length; ++i) {
    out |= data[i] << (8 * i);
  }
  return out;
}

static void UnpackData(uint32_t data, int32_t length, uint8_t *out) {
  // Length bounds handled by for loop.
  assert(out != NULL);
  for (int32_t i = 0; i < 4 && i < length; ++i) {
    out[i] = data >> (8 * i);
  }
}

void DcanInit(DcanBus bus, DcanBitRate bit_rate) {
  assert(bus == kDcanBus1 || bus == kDcanBus2 || bus == kDcanBus3);

  // Enable clocks.
  Peripheral per = kPeripheralDcan1;
  switch (bus) {
    case kDcanBus1:
      per = kPeripheralDcan1;
      break;
    case kDcanBus2:
      per = kPeripheralDcan2;
      break;
    case kDcanBus3:
      per = kPeripheralDcan3;
      break;
    default:
      assert(false);
      return;
  }
  PeripheralEnable(per);

  // Initialize module to known state.
  SoftwareReset(bus);

  // Setup for bit rate.
  int32_t brp10M = PeripheralGetClockPrescale(per, 10000000) - 1;
  int32_t brp1M = PeripheralGetClockPrescale(per, 1000000) - 1;
  switch (bit_rate) {
    case kDcanBitRate1000kbps:
      SetBitRate(bus, brp10M, 7, 0, 0);
      break;
    case kDcanBitRate500kbps:
      SetBitRate(bus, brp10M, 15, 2, 1);
      break;
    case kDcanBitRate125kbps:
      SetBitRate(bus, brp1M, 3, 2, 1);
      break;
    default:
      assert(false);
      return;
  }

  // Configure for normal operation.
  DCAN(bus).CTL.raw = DCAN_CTL_ABO | DCAN_CTL_EIE | DCAN_CTL_SIE | DCAN_CTL_IE0;

  // Clear pending error flags and reset status.
  (void)DCAN(bus).ES.raw;  // Read to clear.

  // Set auto-bus on timer.
  DCAN(bus).ABOTR.raw = 0x0;
}

void DcanSetTransmitMailbox(DcanBus bus, int32_t mailbox, DcanIdType type,
                            uint32_t mask, uint32_t id, int32_t dlc) {
  assert(bus == kDcanBus1 || bus == kDcanBus2 || bus == kDcanBus3);
  assert(0 <= mailbox && mailbox < DCAN_MAILBOXES);
  assert(0 <= dlc && dlc <= 8);

  while (IsInterface1Busy(bus)) {}

  // Use IF1 for transmit mailboxes.
  if (type == kDcanIdExtended) {
    DCAN(bus).IF1MSK.raw = (DCAN_IF1MSK_MXTD | DCAN_IF1MSK_MDIR
                            | (mask & DCAN_IF1MSK_MSK_MASK));
    DCAN(bus).IF1ARB.raw = (DCAN_IF1ARB_MSGVAL | DCAN_IF1ARB_XTD
                            | DCAN_IF1ARB_DIR | (id & DCAN_IF1ARB_ID_MASK));
  } else {
    DCAN(bus).IF1MSK.raw = (DCAN_IF1MSK_MDIR
                            | ((mask << STD_ID_SHIFT) & DCAN_IF1MSK_MSK_MASK));
    DCAN(bus).IF1ARB.raw = (DCAN_IF1ARB_MSGVAL | DCAN_IF1ARB_DIR
                            | ((id << STD_ID_SHIFT) & DCAN_IF1ARB_ID_MASK));
  }
  DCAN(bus).IF1MCTL.raw = (DCAN_IF1MCTL_UMASK | DCAN_IF1MCTL_TXIE
                           | DCAN_IF1MCTL_RXIE | DCAN_IF1MCTL_EOB
                           | (dlc & DCAN_IF1MCTL_DLC_MASK));
  DCAN(bus).IF1CMD.raw = (DCAN_IF1CMD_WR | DCAN_IF1CMD_MASK | DCAN_IF1CMD_ARB
                          | DCAN_IF1CMD_CONTROL
                          | (mailbox & DCAN_IF1CMD_MESSAGE_NUMBER_MASK));
}

bool DcanTransmit(DcanBus bus, int32_t mailbox, int32_t length,
                  const uint8_t *data) {
  assert(bus == kDcanBus1 || bus == kDcanBus2 || bus == kDcanBus3);
  assert(0 <= mailbox && mailbox < DCAN_MAILBOXES);
  assert(data != NULL);

  // Check for pending transmit request.
  uint32_t reg = (mailbox - 1) >> 5;
  uint32_t bit = 1 << ((mailbox - 1) & 0x1F);
  if (DCAN(bus).TXRQ[reg].raw & bit) {
    return false;
  }

  // Use IF1 for transmit mailboxes.
  while (IsInterface1Busy(bus)) {}

  // Copy data.
  DCAN(bus).IF1DATA.raw = PackData(length, &data[0]);
  DCAN(bus).IF1DATB.raw = PackData(length - 4, &data[4]);

  // Transfer to mailbox and send.
  DCAN(bus).IF1CMD.raw = (DCAN_IF1CMD_WR | DCAN_IF1CMD_TXRQST_NEWDAT
                          | DCAN_IF1CMD_DATA_A | DCAN_IF1CMD_DATA_B
                          | (mailbox & DCAN_IF1CMD_MESSAGE_NUMBER_MASK));
  return true;
}

void DcanSetReceiveMailbox(DcanBus bus, int32_t mailbox, DcanIdType type,
                           uint32_t mask, uint32_t id, int32_t dlc) {
  assert(bus == kDcanBus1 || bus == kDcanBus2 || bus == kDcanBus3);
  assert(0 <= mailbox && mailbox < DCAN_MAILBOXES);
  assert(0 <= dlc && dlc <= 8);

  while (IsInterface2Busy(bus)) {}

  // Use IF2 for receive mailboxes.
  if (type == kDcanIdExtended) {
    DCAN(bus).IF2MSK.raw = (DCAN_IF2MSK_MXTD | DCAN_IF2MSK_MDIR
                            | (mask & DCAN_IF2MSK_MSK_MASK));
    DCAN(bus).IF2ARB.raw = (DCAN_IF2ARB_MSGVAL | DCAN_IF2ARB_XTD
                            | (id & DCAN_IF2ARB_ID_MASK));
  } else {
    DCAN(bus).IF2MSK.raw = (DCAN_IF2MSK_MDIR
                            | ((mask << STD_ID_SHIFT) & DCAN_IF2MSK_MSK_MASK));
    DCAN(bus).IF2ARB.raw = (DCAN_IF2ARB_MSGVAL
                            | ((id << STD_ID_SHIFT) & DCAN_IF2ARB_ID_MASK));
  }
  DCAN(bus).IF2MCTL.raw = (DCAN_IF2MCTL_UMASK | DCAN_IF2MCTL_TXIE
                           | DCAN_IF2MCTL_RXIE | DCAN_IF2MCTL_EOB
                           | (dlc & DCAN_IF2MCTL_DLC_MASK));
  DCAN(bus).IF2CMD.raw = (DCAN_IF2CMD_WR | DCAN_IF2CMD_MASK | DCAN_IF2CMD_ARB
                          | DCAN_IF2CMD_CONTROL
                          | (mailbox & DCAN_IF2CMD_MESSAGE_NUMBER_MASK));
}

int32_t DcanGetMailbox(DcanBus bus, int32_t mailbox, int32_t length,
                       uint8_t *data, uint32_t *id) {
  assert(bus == kDcanBus1 || bus == kDcanBus2 || bus == kDcanBus3);
  assert(0 <= mailbox && mailbox < DCAN_MAILBOXES);
  assert(data != NULL);

  // Check for new data.
  uint32_t reg = (mailbox - 1) >> 5;
  uint32_t bit = 1 << ((mailbox - 1) & 0x1F);
  if ((DCAN(bus).NWDAT[reg].raw & bit) == 0) {
    return 0;
  }

  // Use IF2 for receive mailboxes.
  while (IsInterface2Busy(bus)) {}

  // Transfer data to IF2.
  DCAN(bus).IF2CMD.raw = (DCAN_IF2CMD_ARB | DCAN_IF2CMD_CONTROL
                          | DCAN_IF2CMD_TXRQST_NEWDAT
                          | DCAN_IF2CMD_DATA_A | DCAN_IF2CMD_DATA_B
                          | (mailbox & DCAN_IF2CMD_MESSAGE_NUMBER_MASK));
  while (IsInterface2Busy(bus)) {}

  // Copy output data.
  UnpackData(DCAN(bus).IF2DATA.raw, length, &data[0]);
  UnpackData(DCAN(bus).IF2DATB.raw, length - 4, &data[4]);
  if (id) {
    uint32_t arb = DCAN(bus).IF2ARB.raw;
    if (arb & DCAN_IF2ARB_XTD) {
      *id = arb & DCAN_IF2ARB_ID_MASK;
    } else {
      *id = (arb & DCAN_IF2ARB_ID_MASK) >> STD_ID_SHIFT;
    }
  }

  // Return actual number of bytes received.
  return DCAN(bus).IF2MCTL.DLC;
}
