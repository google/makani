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

#include "avionics/firmware/cpu/i2c.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"

// SMBus 2.0: 25 ms max cumulative clock low extend time for a slave device.
#define I2C_BUSY_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(25)
#define I2C_BYTE_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(25)
#define I2C_BIT_PERIOD_CYCLES   CLOCK32_USEC_TO_CYCLES(10)
#define I2C_BITS_TO_RELEASE_SDA 16
#define I2C_MDR_MASTER (I2C_MDR_FREE | I2C_MDR_NIRS | I2C_MDR_MST)

typedef enum {
  kBusStateCheckBusy,
  kBusStateTransferStart,
  kBusStateTransferWait,
  kBusStateSlaveFailure,
  kBusStateIdle
} BusState;

static struct {
  BusState state;
  bool first_entry;
  int32_t index;
  uint32_t timeout;
  bool error;
} g_bus;

static struct {
  uint8_t addr;
  const uint8_t *write_ptr;
  const uint8_t *write_end;
  uint8_t *read_ptr;
  const uint8_t *read_end;
} g_xfer;

static bool g_i2c_initialized = false;

static bool BusAccess(void) {
  // Check busy status.
  return !(I2C.STR.BB || I2C.MDR.MST);
}

static void AbortTransfer(void) {
  // Place I2C into reset.
  I2C.MDR.raw = 0x0;

  // Report result.
  g_bus.state = kBusStateIdle;
  g_bus.error = true;
}

static void SlaveFailure(void) {
  // Place I2C into reset.
  I2C.MDR.raw = 0x0;

  // Report result.
  g_bus.state = kBusStateSlaveFailure;
  g_bus.error = true;
}

static void TransferComplete(void) {
  // Place I2C into reset.
  I2C.MDR.raw = 0x0;

  // Report result.
  g_bus.state = kBusStateIdle;
  g_bus.error = false;
}

static void StartTransfer(uint8_t addr, uint16_t cnt, uint32_t mdr) {
  assert(addr != I2C.OAR.OA);
  assert(addr <= 127U);

  // Clear status register.
  I2C.STR.raw = 0xFFFF;

  // Start transfer.
  I2C.SAR.raw = addr;
  I2C.CNT.raw = cnt;
  I2C.MDR.raw = I2C_MDR_MASTER | mdr;
}

static void BusStateCheckBusy(BusState next) {
  uint32_t now = Clock32GetCycles();
  if (g_bus.first_entry) {
    g_bus.timeout = now + I2C_BUSY_TIMEOUT_CYCLES;
  }
  if (BusAccess()) {
    g_bus.state = next;
  } else if (CLOCK32_GT(now, g_bus.timeout)) {
    g_bus.state = kBusStateSlaveFailure;
  }
}

static void BusStateTransferStart(BusState next) {
  uint16_t cnt, mdr;
  if (g_xfer.write_end > g_xfer.write_ptr) {
    cnt = (uint16_t)(g_xfer.write_end - g_xfer.write_ptr);
    mdr = I2C_MDR_TRX | I2C_MDR_STT;
    if (g_xfer.read_end <= g_xfer.read_ptr) {
      mdr |= I2C_MDR_STP;
    }
  } else {
    cnt = (uint16_t)(g_xfer.read_end - g_xfer.read_ptr);
    mdr = I2C_MDR_STT | I2C_MDR_STP;
  }
  if (cnt > 0) {
    StartTransfer(g_xfer.addr, cnt, mdr);
    g_bus.state = next;
  } else {
    g_bus.state = kBusStateIdle;
    g_bus.error = true;
    assert(false);
  }
}

static void BusStateTransferWait(void) {
  uint32_t now = Clock32GetCycles();

  // Initialize transfer timeout.
  if (g_bus.first_entry) {
    g_bus.timeout = now + I2C_BYTE_TIMEOUT_CYCLES;
  }

  // Poll status registers in an interrupt-like fashion.
  union I2C_STR str = I2C.STR;
  if (str.AL) {
    // Arbitration lost.
    I2C.STR.raw |= I2C_STR_AL;
    AbortTransfer();

  } else if (str.NACK) {
    // Negative acknowledgment.
    I2C.STR.raw |= I2C_STR_NACK;
    AbortTransfer();

  } else if (str.ARDY) {
    // Address ready.
    if (!I2C.MDR.STP) {
      I2C.CNT.raw = (uint16_t)(g_xfer.read_end - g_xfer.read_ptr);
      I2C.MDR.raw = I2C_MDR_MASTER | I2C_MDR_STT | I2C_MDR_STP;
    }
    g_bus.timeout = now + I2C_BYTE_TIMEOUT_CYCLES;

  } else if (str.RXRDY) {
    // Receive ready.
    if (g_xfer.read_ptr < g_xfer.read_end) {
      *g_xfer.read_ptr++ = I2C.DRR.DATARX;
    } else {
      I2C.MDR.raw |= I2C_MDR_NACKMOD;  // Generate NACK on next cycle.
      (void)I2C.DRR.DATARX;
    }
    g_bus.timeout = now + I2C_BYTE_TIMEOUT_CYCLES;

  } else if (str.TXRDY) {
    // Transmit ready.
    if (g_xfer.write_ptr < g_xfer.write_end) {
      I2C.DXR.raw = *g_xfer.write_ptr++;
    } else {
      I2C.DXR.raw = 0x0;
    }
    g_bus.timeout = now + I2C_BYTE_TIMEOUT_CYCLES;

  } else if (str.SCD) {
    // Stop condition received.
    I2C.STR.raw |= I2C_STR_SCD;
    TransferComplete();

  } else if (CLOCK32_GT(now, g_bus.timeout)) {
    // Timeout.
    SlaveFailure();
  }
}

static void BusStateSlaveFailure(BusState next) {
  uint32_t now = Clock32GetCycles();

  // Initialize pins for I/O function.
  if (g_bus.first_entry) {
    I2C.MDR.raw = 0x0;  // Place I2C into reset.
    I2C.DSET.raw = I2C_DSET_SCLSET;   // Set SCL high.
    I2C.PDIR.raw = I2C_PDIR_SCLDIR;   // Set SCL as output, SDA as input.
    I2C.PFNC.raw = I2C_PFNC_PINFUNC;  // Configure as I/O pins.
    g_bus.index = 0;
    g_bus.timeout = now;
  }

  if (g_bus.index >= 2 * I2C_BITS_TO_RELEASE_SDA + 1) {
    // Configure pins for I2C function.
    I2C.PFNC.raw = 0x0;
    g_bus.state = next;
  } else if (CLOCK32_GE(now, g_bus.timeout)) {
    // Toggle SCL.
    if (g_bus.index % 2 == 0) {
      I2C.DSET.raw = I2C_DSET_SCLSET;
    } else {
      I2C.DCLR.raw = I2C_DCLR_SCLCLR;
    }
    ++g_bus.index;
    g_bus.timeout = now + I2C_BIT_PERIOD_CYCLES;
  }
}

static bool SetupTransfer(uint8_t addr, int32_t wr_len, const void *wr_data,
                          int32_t rd_len, void *rd_data) {
  assert(addr <= 127U);
  assert(0 <= wr_len && wr_len <= 65536);  // In I2C.CNT, 0 is treated as 65536.
  assert(0 <= rd_len && rd_len <= 65536);  // In I2C.CNT, 0 is treated as 65536.

  // Wait for completion.
  if (g_bus.state != kBusStateIdle && !I2cPoll(NULL)) {
    return false;
  }
  assert(g_bus.state == kBusStateIdle);

  // Prepare transfer.
  g_xfer.addr = addr;
  if (wr_len > 0) {
    g_xfer.write_ptr = (const uint8_t *)wr_data;
    g_xfer.write_end = &g_xfer.write_ptr[wr_len];
  } else {
    g_xfer.write_ptr = NULL;
    g_xfer.write_end = NULL;
  }
  if (rd_len > 0) {
    g_xfer.read_ptr = (uint8_t *)rd_data;
    g_xfer.read_end = &g_xfer.read_ptr[rd_len];
  } else {
    g_xfer.read_ptr = NULL;
    g_xfer.read_end = NULL;
  }
  g_bus.state = kBusStateCheckBusy;
  g_bus.error = false;

  return true;
}

// Setup I2C as master with 7 bit addressing and 8 bit data.
void I2cInit(double freq) {
  if (g_i2c_initialized) {
    return;
  }

  // Enable peripheral clock.
  PeripheralEnable(kPeripheralI2c);

  // Reset.
  I2C.MDR.raw = 0;

  // Reset transfer state.
  memset(&g_xfer, 0, sizeof(g_xfer));
  memset(&g_bus, 0, sizeof(g_bus));
  g_bus.state = kBusStateSlaveFailure;
  g_bus.first_entry = true;

  // Setup pinmux.
  IommSetPinmux(0, 17);
  IommSetPinmux(0, 25);

  // Setup I2C module clock.  The maximum frequency is 13.3 MHz.
  float f_i2c = 13.3e6;
  float f_vclk = PeripheralGetClockFreq(kPeripheralI2c);
  float prescale = roundf(f_vclk / f_i2c);
  I2C.PSC.PSC = (int32_t)prescale - 1;

  // Setup I2C data transfer frequency.
  // Determine fudge factor (see TMS570LS12x TRM 31.1.3).
  int32_t fudge;
  if (prescale == 0) {
    fudge = 7;
  } else if (prescale == 1) {
    fudge = 6;
  } else {
    fudge = 5;
  }

  int32_t master_prescale = (int32_t)ceil(f_vclk / (prescale * freq));
  assert(master_prescale >= 2 * fudge
         && master_prescale <= 2 * (65535 + fudge));

  // Set low and high time prescalers.
  I2C.CKL.CLKL = (int32_t)(master_prescale - 2 * fudge) / 2;
  I2C.CKH.CLKH = (int32_t)(master_prescale - 2 * fudge) - I2C.CKL.CLKL;

  // Turn off DMA (it's on by default!).
  I2C.DMACR.raw = 0;

  // Set own address. This address should not conflict with another slave or
  // the general call address (0x00).
  I2C.OAR.raw = 0x3FF;

  g_i2c_initialized = true;
}

bool I2cPoll(bool *error) {
  BusState current = g_bus.state;
  switch (current) {
    case kBusStateCheckBusy:
      BusStateCheckBusy(kBusStateTransferStart);
      break;
    case kBusStateTransferStart:
      BusStateTransferStart(kBusStateTransferWait);
      break;
    case kBusStateTransferWait:
      BusStateTransferWait();
      break;
    case kBusStateSlaveFailure:
      BusStateSlaveFailure(kBusStateIdle);
      break;
    case kBusStateIdle:
      g_bus.error = false;
      break;
    default:
      g_bus.state = kBusStateIdle;
      g_bus.error = false;
      assert(false);
      break;
  }
  g_bus.first_entry = (current != g_bus.state);

  if (error != NULL) {
    *error = g_bus.error;
  }
  return g_bus.state == kBusStateIdle;
}

bool I2cIsBusIdle(void) {
  return g_bus.state == kBusStateIdle;
}

bool I2cRead(uint8_t addr, int32_t len, void *data) {
  bool error;
  while (!I2cReadAsync(addr, len, data)) {}
  while (!I2cPoll(&error)) {}
  return error;
}

bool I2cReadAsync(uint8_t addr, int32_t len, void *data) {
  assert(addr <= 127U);
  assert(1 <= len && len <= 65536);  // In I2C.CNT, 0 is treated as 65536.
  assert(data != NULL);

  return SetupTransfer(addr, 0, NULL, len, data);
}

bool I2cWrite(uint8_t addr, int32_t len, const void *data) {
  bool error;
  while (!I2cWriteAsync(addr, len, data)) {}
  while (!I2cPoll(&error)) {}
  return error;
}

bool I2cWriteAsync(uint8_t addr, int32_t len, const void *data) {
  assert(addr <= 127U);
  assert(1 <= len && len <= 65536);  // In I2C.CNT, 0 is treated as 65536.
  assert(data != NULL);

  return SetupTransfer(addr, len, data, 0, NULL);
}

bool I2cWriteRead(uint8_t addr, int32_t wr_len, const void *wr_data,
                  int32_t rd_len, void *rd_data) {
  bool error;
  while (!I2cWriteReadAsync(addr, wr_len, wr_data, rd_len, rd_data)) {}
  while (!I2cPoll(&error)) {}
  return error;
}

bool I2cWriteReadAsync(uint8_t addr, int32_t wr_len, const void *wr_data,
                       int32_t rd_len, void *rd_data) {
  assert(addr <= 127U);
  assert(1 <= wr_len && wr_len <= 65536);  // In I2C.CNT, 0 is treated as 65536.
  assert(wr_data != NULL);
  assert(1 <= rd_len && rd_len <= 65536);  // In I2C.CNT, 0 is treated as 65536.
  assert(rd_data != NULL);

  return SetupTransfer(addr, wr_len, wr_data, rd_len, rd_data);
}
