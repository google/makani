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

#include "avionics/firmware/drivers/bcm.h"

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/endian.h"
#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/mibspi.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

#define BCM_BUF_SIZE 8

typedef enum {
  kBcmStateInit,
  kBcmStateIdle,
  kBcmStateReadInit,
  kBcmStateReadPage,
  kBcmStateReadAddress,
  kBcmStateReadWaitReady,
  kBcmStateReadReceive,
  kBcmStateReadComplete,
  kBcmStateWriteInit,
  kBcmStateWritePage,
  kBcmStateWriteData,
  kBcmStateWriteComplete,
} BcmState;

static struct {
  int32_t device;
  BcmState state;
  uint8_t page;
  uint8_t address;
  int32_t len;
  uint8_t buf[BCM_BUF_SIZE];
} g_state;

static bool SpiTransferAsync(int32_t len, void *buf) {
  return MibSPITransferUint8Async(1, g_state.device, 1, len, buf, buf);
}

static bool StatusCommand(uint8_t *status) {
  uint8_t status_command[3] = {0x60, 0xFE};
  if (SpiTransferAsync(ARRAYSIZE(status_command), status_command)) {
    *status = status_command[2];
    return true;
  }
  return false;
}

static void BcmStateInit(BcmState next) {
  // Initialize MibSPI1 to interface with the switch.
  g_state.device = MibSPIAddDevice(1, 1 << 2, 12.5e6, 0, true, true, 8, 10);

  // Initialize MibSPI5 for GPIO functionality on MIBSPI5NCS[0], which is used
  // for resetting the switch.

  // Enable clocks.
  PeripheralEnable(kPeripheralMibSpi5);

  // Reset.
  SPI(5).GCR0.NRESET = 0;
  SPI(5).GCR0.NRESET = 1;

  // Set direction to output.
  SPI(5).PC1.SCSDIR = 1 << 0;

  // Enable open-drain.
  SPI(5).PC6.SCSPDR = 1 << 0;

  g_state.state = next;
}

// Poll SPI read/write complete flag.
static void BcmCheckStatus(BcmState next) {
  uint8_t status;
  if (StatusCommand(&status) && ((status & 0x80) == 0)) {
    g_state.state = next;
  }
}

// Set page.
static void BcmStateSetPage(BcmState next) {
  uint8_t page_command[3] = {0x61, 0xFF, g_state.page};
  if (SpiTransferAsync(ARRAYSIZE(page_command), page_command)) {
    g_state.state = next;
  }
}

// Set address.
static void BcmStateReadAddress(BcmState next) {
  uint8_t address_command[3] = {0x60, g_state.address};
  if (SpiTransferAsync(ARRAYSIZE(address_command), address_command)) {
    g_state.state = next;
  }
}

// Wait for read ready flag.
static void BcmStateReadWaitReady(BcmState next) {
  uint8_t status;
  if (StatusCommand(&status) && (status & 0x20) != 0) {
    g_state.state = next;
  }
}

static void BcmStateReadReceive(BcmState next) {
  uint8_t data_command[10] = {0x60, 0xF0};
  if (SpiTransferAsync(2 + g_state.len, data_command)) {
    for (int32_t i = 0; i < g_state.len; ++i) {
      g_state.buf[i] = data_command[2 + g_state.len - 1 - i];
    }
    g_state.state = next;
  }
}

static void BcmStateWriteData(BcmState next) {
  uint8_t data_command[10] = {0x61, g_state.address};
  for (int32_t i = 0; i < g_state.len; ++i) {
    data_command[2 + g_state.len - 1 - i] = g_state.buf[i];
  }
  if (SpiTransferAsync(2 + g_state.len, data_command)) {
    g_state.state = next;
  }
}

static void BcmStateComplete(void) {
}

void BcmInit() {
  memset(&g_state, 0, sizeof(g_state));
  g_state.state = kBcmStateInit;
}

void BcmReset(bool reset) {
  SPI(5).PC3.SCSDOUT = !reset << 0;
}

void BcmPoll() {
  switch (g_state.state) {
    case kBcmStateInit:
      BcmStateInit(kBcmStateIdle);
      break;
    case kBcmStateReadInit:
      BcmCheckStatus(kBcmStateReadPage);
      break;
    case kBcmStateReadPage:
      BcmStateSetPage(kBcmStateReadAddress);
      break;
    case kBcmStateReadAddress:
      BcmStateReadAddress(kBcmStateReadWaitReady);
      break;
    case kBcmStateReadWaitReady:
      BcmStateReadWaitReady(kBcmStateReadReceive);
      break;
    case kBcmStateReadReceive:
      BcmStateReadReceive(kBcmStateReadComplete);
      break;
    case kBcmStateReadComplete:
      BcmStateComplete();
      break;
    case kBcmStateWriteInit:
      BcmCheckStatus(kBcmStateWritePage);
      break;
    case kBcmStateWritePage:
      BcmStateSetPage(kBcmStateWriteData);
      break;
    case kBcmStateWriteData:
      BcmStateWriteData(kBcmStateWriteComplete);
      break;
    case kBcmStateWriteComplete:
      BcmStateComplete();
      break;
    case kBcmStateIdle:
      break;
    default:
      assert(false);
      BcmInit();
  }
}

bool BcmIsIdle() {
  return g_state.state == kBcmStateIdle;
}

bool BcmRead(uint8_t page, uint8_t address, int32_t len, void *buf) {
  assert(1 <= len && len <= BCM_BUF_SIZE);

  if (g_state.state == kBcmStateReadComplete) {
    g_state.state = kBcmStateIdle;
    memcpy(buf, g_state.buf, len);
    return true;
  } else if (BcmIsIdle()) {
    g_state.state = kBcmStateReadInit;
    g_state.page = page;
    g_state.address = address;
    g_state.len = len;
  }
  return false;
}

bool BcmReadUint8(uint8_t page, uint8_t address, uint8_t *value) {
  uint8_t buf[1];
  if (!BcmRead(page, address, ARRAYSIZE(buf), buf)) {
    return false;
  }
  ReadUint8Be(buf, value);
  return true;
}

bool BcmReadUint16(uint8_t page, uint8_t address, uint16_t *value) {
  uint8_t buf[2];
  if (!BcmRead(page, address, ARRAYSIZE(buf), buf)) {
    return false;
  }
  ReadUint16Be(buf, value);
  return true;
}

bool BcmReadUint32(uint8_t page, uint8_t address, uint32_t *value) {
  uint8_t buf[4];
  if (!BcmRead(page, address, ARRAYSIZE(buf), buf)) {
    return false;
  }
  ReadUint32Be(buf, value);
  return true;
}

bool BcmReadUint48(uint8_t page, uint8_t address, uint64_t *value) {
  uint8_t buf[6];
  if (!BcmRead(page, address, ARRAYSIZE(buf), buf)) {
    return false;
  }
  ReadUint48Be(buf, value);
  return true;
}

bool BcmReadUint64(uint8_t page, uint8_t address, uint64_t *value) {
  uint8_t buf[8];
  if (!BcmRead(page, address, ARRAYSIZE(buf), buf)) {
    return false;
  }
  ReadUint64Be(buf, value);
  return true;
}

bool BcmWrite(uint8_t page, uint8_t address, int32_t len, const void *buf) {
  assert(buf != NULL);
  assert(1 <= len && len <= BCM_BUF_SIZE);

  if (g_state.state == kBcmStateWriteComplete) {
    g_state.state = kBcmStateIdle;
    return true;
  } else if (BcmIsIdle()) {
    g_state.state = kBcmStateWriteInit;
    g_state.page = page;
    g_state.address = address;
    g_state.len = len;
    memcpy(g_state.buf, buf, len);
  }
  return false;
}

bool BcmWriteUint8(uint8_t page, uint8_t address, uint8_t value) {
  uint8_t buf[1];
  WriteUint8Be(value, buf);
  return BcmWrite(page, address, ARRAYSIZE(buf), buf);
}

bool BcmWriteUint16(uint8_t page, uint8_t address, uint16_t value) {
  uint8_t buf[2];
  WriteUint16Be(value, buf);
  return BcmWrite(page, address, ARRAYSIZE(buf), buf);
}

bool BcmWriteUint32(uint8_t page, uint8_t address, uint32_t value) {
  uint8_t buf[4];
  WriteUint32Be(value, buf);
  return BcmWrite(page, address, ARRAYSIZE(buf), buf);
}

bool BcmWriteUint48(uint8_t page, uint8_t address, uint64_t value) {
  uint8_t buf[6];
  WriteUint48Be(value, buf);
  return BcmWrite(page, address, ARRAYSIZE(buf), buf);
}

bool BcmWriteUint64(uint8_t page, uint8_t address, uint64_t value) {
  uint8_t buf[8];
  WriteUint64Be(value, buf);
  return BcmWrite(page, address, ARRAYSIZE(buf), buf);
}

// Detection values are derived from pg. 275 of 5328XM-DS303-RDS.pdf and
// pg. 158 of 53101M-DS05-RDS.pdf.  Reads are byte-by-byte because 0xE8 is an
// undefined address in the BCM53101 and will cause the read to return zero.
#define VALUE_EE_53101 0x3
#define VALUE_EE_53284 0x0
#define VALUE_E8_53284 0x7
bool BcmDetectSwitchType(SwitchType *type) {
  static bool first_entry = true;
  static int32_t state = 0;
  if (first_entry) {
    first_entry = false;
    state = 0;
  }
  uint8_t value;
  switch (state) {
    case 0:
      if (BcmReadUint8(0x00, 0xEE, &value)) {
        switch (value) {
          case VALUE_EE_53101:
            first_entry = true;
            *type = kSwitchTypeBcm53101;
            break;
          case VALUE_EE_53284:
            ++state;
            break;
          default:
            first_entry = true;
            *type = kSwitchTypeUnknown;
        }
      }
      break;
    case 1:
      if (BcmReadUint8(0x00, 0xE8, &value)) {
        switch (value) {
          case VALUE_E8_53284:
            first_entry = true;
            *type = kSwitchTypeBcm53284;
            break;
          default:
            first_entry = true;
            *type = kSwitchTypeUnknown;
        }
      }
      break;
    default:
      assert(false);
  }
  return first_entry;
}
