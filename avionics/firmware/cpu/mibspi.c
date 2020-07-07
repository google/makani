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

// This driver enables the MibSPI to asynchronously communicate with up to four
// slave devices without CPU intervention.
//
// The slave devices are numbered 0 to 3. The TMS570 MibSPI supports four data
// formats and eight transfer groups. For device i:
// - Data format i contains the device configuration.
// - Transfer group 2 * i points to the start of the buffers allocated to the
//   device and is fixed at initialization.
// - Transfer group 2 * i + 1 points to the end of the buffers used for the
//   current transfer to the device and is variable.
// - Transfer group 2 * i + 2 implicitly points to the end of the buffers
//   allocated to the device and is fixed at initialization. The user is
//   responsible for declaring the worst-case size.

#include "avionics/firmware/cpu/mibspi.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/spi.h"

#define MIB_SPI_BUFFERS 128
#define MIB_SPI_DEVICES 4

static void AssignBuffers(int32_t spi, int32_t device, uint8_t csmask,
                          bool wdelay, int32_t num_buffers) {
  assert(spi == 1 || spi == 3);
  assert(0 <= device && device < MIB_SPI_DEVICES);
  assert(1 <= num_buffers && num_buffers < MIB_SPI_BUFFERS);

  // Get transfer group.
  int32_t tg = 2 * device;

  // Disable triggering.
  SPI(spi).TGCTRL[tg].TRIGEVT = 0;

  // Get transfer group bounds.
  int32_t start = SPI(spi).TGCTRL[tg].PSTART;
  int32_t end   = start + num_buffers;

  // Not enough buffers available.
  assert(end < MIB_SPI_BUFFERS);

  // Set transfer group end.
  if (device < MIB_SPI_DEVICES - 1) {
    SPI(spi).TGCTRL[tg + 2].PSTART = end;
  } else {
    SPI(spi).LTGPEND.LPEND = end - 1;
  }

  // Set current transfer end.
  SPI(spi).TGCTRL[tg + 1].PSTART = end;

  // Configure transmit buffers.
  for (int32_t i = start; i < end; ++i) {
    // Always transmit when triggered.
    SPIRAM(spi).TXRAM[i].BUFMODE = 4;
    // Prevent interruption during transfer.
    SPIRAM(spi).TXRAM[i].LOCK = 1;
    // Set data format.
    SPIRAM(spi).TXRAM[i].DFSEL = device;
    // Set chip select.
    SPIRAM(spi).TXRAM[i].CSNR = ~csmask;
    // Set chip select hold.
    SPIRAM(spi).TXRAM[i].CSHOLD = 0;
    // Enable word delay.
    SPIRAM(spi).TXRAM[i].WDEL = wdelay;
  }
}

static bool SetTransferBuffers(int32_t spi, int32_t device, int32_t num_buffers,
                               int32_t *start, int32_t *end) {
  assert(spi == 1 || spi == 3);
  assert(0 <= device && device < MIB_SPI_DEVICES);
  assert(1 <= num_buffers && num_buffers < MIB_SPI_BUFFERS);
  assert(start != NULL && end != NULL);

  // Get transfer group.
  int32_t tg = 2 * device;
  union SPI_TGCTRL tgctrl = SPI(spi).TGCTRL[tg];

  // Check for in-progress transfer.
  if (tgctrl.TGENA != 0) {
    return false;
  }

  // Get current transfer bounds.
  *start = tgctrl.PSTART;
  *end   = *start + num_buffers;

  // Bounds check current transfer end.
  if (device < MIB_SPI_DEVICES - 1) {
    assert(*end <= SPI(spi).TGCTRL[tg + 2].PSTART);
  } else {
    assert(*end <= SPI(spi).LTGPEND.LPEND + 1);
  }

  // Set current transfer end.
  SPI(spi).TGCTRL[tg + 1].PSTART = *end;

  return true;
}

static bool GetTransferBuffers(int32_t spi, int32_t device, int32_t num_buffers,
                               int32_t *start, int32_t *end) {
  (void)num_buffers;
  assert(spi == 1 || spi == 3);
  assert(0 <= device && device < MIB_SPI_DEVICES);
  assert(1 <= num_buffers && num_buffers < MIB_SPI_BUFFERS);
  assert(start != NULL && end != NULL);

  // Get transfer group.
  int32_t tg = 2 * device;

  // Get current transfer bounds.
  *start = SPI(spi).TGCTRL[tg].PSTART;
  *end   = SPI(spi).TGCTRL[tg + 1].PSTART;

  // Check for consistent length.
  assert(num_buffers == *end - *start);

  // Check for transfer completion interrupt flag.
  bool ready = (SPI(spi).TGINTFLG.INTFLGRDY & (1U << tg)) != 0;
  if (ready) {
    SPI(spi).TGINTFLG.INTFLGRDY = 1U << tg;  // Clear flag.
  }
  return ready;
}

void MibSPIInit(int32_t spi, SpiPinmux spi_pins) {
  assert(spi == 1 || spi == 3);

  // Initialize standard SPI.
  SpiInit(spi, spi_pins);

  // Enable Mib mode.
  SPI(spi).MIBSPIE.MSPIENA = 1;

  // Wait for MibSPI RAM initialization.
  while (SPI(spi).FLG.BUFINITACTIVE) {}
}

// TODO: Create SPI mode enumeration to replace polarity and phase
// parameters.
int32_t MibSPIAddDevice(int32_t spi, uint8_t csmask, int32_t freq_hz,
                        int32_t wdelay_ns, bool polarity, bool phase,
                        int32_t charlen, int32_t num_buffers) {
  assert(spi == 1 || spi == 3);

  // Get transfer device.
  int32_t device = SpiAddDevice(spi, csmask, freq_hz, wdelay_ns, charlen,
                                polarity, phase);

  // Assign SPI TX/RXRAM buffers to this device.
  AssignBuffers(spi, device, csmask, (wdelay_ns > 0), num_buffers);

  return device;
}

// Honeywell HSC pressure sensors require chip select held low during the
// entire transfer, then released after completion.
void MibSPISetChipSelect(int32_t spi, int32_t device, uint8_t csmask,
                         bool cshold, int32_t offset, int32_t num_buffers) {
  assert(spi == 1 || spi == 3);
  assert(0 <= device && device < MIB_SPI_DEVICES);
  assert(0 <= offset && offset < MIB_SPI_BUFFERS);
  assert(1 <= num_buffers && num_buffers < MIB_SPI_BUFFERS);

  // Get transfer group.
  int32_t tg = 2 * device;
  union SPI_TGCTRL tgctrl = SPI(spi).TGCTRL[tg];

  // Check for in-progress transfer.
  assert(tgctrl.TGENA == 0);

  // Get current transfer bounds.
  int32_t start = tgctrl.PSTART + offset;
  int32_t end   = start + num_buffers;

  // Bounds check current transfer end.
  if (device < MIB_SPI_DEVICES - 1) {
    assert(end <= SPI(spi).TGCTRL[tg + 2].PSTART);
  } else {
    assert(end <= SPI(spi).LTGPEND.LPEND + 1);
  }

  // Update chip select mask.
  for (int32_t i = start; i < end; ++i) {
    // Set chip select.
    SPIRAM(spi).TXRAM[i].CSNR = ~csmask;
    // Set chip select hold.
    SPIRAM(spi).TXRAM[i].CSHOLD = cshold && (i + 1 != end);
  }
}

bool MibSPITrigger(int32_t spi, int32_t device, MibSpiSource source,
                   MibSpiEvent event, bool one_shot, bool enable) {
  assert(spi == 1 || spi == 3);
  assert(0 <= device && device < MIB_SPI_DEVICES);
  assert((source & 0x0F) == source);
  assert((event & 0x0F) == event);
  assert(SPI(spi).FMT[device].CHARLEN > 0);

  // Get transfer group.
  int32_t tg = 2 * device;
  union SPI_TGCTRL tgctrl = SPI(spi).TGCTRL[tg];

  // Wait for current transfer to complete before re-triggering.
  if (tgctrl.TGENA && enable) {
    return false;
  }

  // Configure.
  tgctrl.TRIGSRC = source;
  tgctrl.TRIGEVT = event;
  tgctrl.ONESHOT = one_shot;
  tgctrl.TGENA = enable;
  SPI(spi).TGINTFLG.INTFLGRDY = 1U << tg;  // Clear pending flag.
  SPI(spi).TGCTRL[tg] = tgctrl;

  return true;
}

bool MibSPITriggerBySoftware(int32_t spi, int32_t device) {
  return MibSPITrigger(spi, device, kMibSpiSourceDisabled, kMibSpiEventAlways,
                       true, true);
}

bool MibSPITriggerDisable(int32_t spi, int32_t device) {
  return MibSPITrigger(spi, device, kMibSpiSourceDisabled, kMibSpiEventNever,
                       false, false);
}

bool MibSPIWriteUint8(int32_t spi, int32_t device, int32_t count,
                      int32_t length, const uint8_t *data) {
  assert(spi == 1 || spi == 3);

  if (data == NULL) {
    return MibSPIWriteConst(spi, device, count, length, 0);
  }

  int32_t start, end;
  if (!SetTransferBuffers(spi, device, count * length, &start, &end)) {
    return false;
  }

  int32_t j = length;
  const uint8_t *p = data;
  for (int32_t i = start; i < end; ++i, ++p) {
    SPIRAM(spi).TXRAM[i].TXDATA = *p;  // TXDATA is 16-bits wide.
    if (--j > 0) {
      SPIRAM(spi).TXRAM[i].CSHOLD = 1;
    } else {
      SPIRAM(spi).TXRAM[i].CSHOLD = 0;
      j = length;
    }
  }
  return true;
}

bool MibSPIWriteUint16(int32_t spi, int32_t device, int32_t count,
                       int32_t length, const uint16_t *data) {
  assert(spi == 1 || spi == 3);

  if (data == NULL) {
    return MibSPIWriteConst(spi, device, count, length, 0);
  }

  int32_t start, end;
  if (!SetTransferBuffers(spi, device, count * length, &start, &end)) {
    return false;
  }

  int32_t j = length;
  const uint16_t *p = data;
  for (int32_t i = start; i < end; ++i, ++p) {
    SPIRAM(spi).TXRAM[i].TXDATA = *p;  // TXDATA is 16-bits wide.
    if (--j > 0) {
      SPIRAM(spi).TXRAM[i].CSHOLD = 1;
    } else {
      SPIRAM(spi).TXRAM[i].CSHOLD = 0;
      j = length;
    }
  }
  return true;
}

bool MibSPIWriteConst(int32_t spi, int32_t device, int32_t count,
                      int32_t length, uint16_t ch) {
  assert(spi == 1 || spi == 3);

  int32_t start, end;
  if (!SetTransferBuffers(spi, device, count * length, &start, &end)) {
    return false;
  }

  int32_t j = length;
  for (int32_t i = start; i < end; ++i) {
    SPIRAM(spi).TXRAM[i].TXDATA = ch;
    if (--j > 0) {
      SPIRAM(spi).TXRAM[i].CSHOLD = 1;
    } else {
      SPIRAM(spi).TXRAM[i].CSHOLD = 0;
      j = length;
    }
  }
  return true;
}

bool MibSPIReadUint8(int32_t spi, int32_t device, int32_t count,
                     int32_t length, uint8_t *data) {
  assert(spi == 1 || spi == 3);

  // Get current transfer bounds.
  int32_t start, end;
  if (!GetTransferBuffers(spi, device, count * length, &start, &end)) {
    return false;
  }

  // Read receive buffers.
  if (data != NULL) {
    uint8_t *p = data;
    for (int32_t i = start; i < end; ++i, ++p) {
      *p = SPIRAM(spi).RXRAM[i].RXDATA;  // RXDATA is 16-bits wide.
    }
  }
  return true;
}

bool MibSPIReadUint16(int32_t spi, int32_t device, int32_t count,
                      int32_t length, uint16_t *data) {
  assert(spi == 1 || spi == 3);

  // Get current transfer bounds.
  int32_t start, end;
  if (!GetTransferBuffers(spi, device, count * length, &start, &end)) {
    return false;
  }

  // Read receive buffers.
  if (data != NULL) {
    uint16_t *p = data;
    for (int32_t i = start; i < end; ++i, ++p) {
      *p = SPIRAM(spi).RXRAM[i].RXDATA;  // RXDATA is 16-bits wide.
    }
  }
  return true;
}

// Call this function repeatedly until it returns true.
bool MibSPITransferUint8Async(int32_t spi, int32_t device, int32_t count,
                              int32_t length, const uint8_t *mosi,
                              uint8_t *miso) {
  assert(spi == 1 || spi == 3);
  assert(0 <= device && device < MIB_SPI_DEVICES);
  assert(mosi != NULL);

  // Check for in-progress transfer.
  int32_t start, end;
  if (!SetTransferBuffers(spi, device, count * length, &start, &end)) {
    return false;
  }

  // Transfer group.
  int32_t tg = 2 * device;

  // No transfer in progress. Determine if transfer started or completed.
  bool complete = (SPI(spi).TGINTFLG.INTFLGRDY & (1U << tg)) != 0;
  if (complete) {
    // Transfer completion. Read output buffers.
    if (miso != NULL) {
      uint8_t *p = miso;
      for (int32_t i = start; i < end; ++i, ++p) {
        *p = SPIRAM(spi).RXRAM[i].RXDATA;  // RXDATA is 16-bits wide.
      }
    }
    SPI(spi).TGINTFLG.INTFLGRDY = 1U << tg;  // Clear flag.
  } else {
    // Prepare to start a new transfer.
    int32_t j = length;
    const uint8_t *p = mosi;
    for (int32_t i = start; i < end; ++i, ++p) {
      SPIRAM(spi).TXRAM[i].TXDATA = *p;  // TXDATA is 16-bits wide.
      if (--j > 0) {
        SPIRAM(spi).TXRAM[i].CSHOLD = 1;
      } else {
        SPIRAM(spi).TXRAM[i].CSHOLD = 0;
        j = length;
      }
    }
    MibSPITriggerBySoftware(spi, device);
  }
  return complete;
}

void MibSPITransferUint8(int32_t spi, int32_t device, int32_t count,
                         int32_t length, const uint8_t *mosi, uint8_t *miso) {
  while (!MibSPITransferUint8Async(spi, device, count, length, mosi, miso)) {}
}

void MibSPITransferUint16(int32_t spi, int32_t device, int32_t count,
                          int32_t length, const uint16_t *mosi,
                          uint16_t *miso) {
  while (!MibSPIWriteUint16(spi, device, count, length, mosi)) {}
  while (!MibSPITriggerBySoftware(spi, device)) {}
  while (!MibSPIReadUint16(spi, device, count, length, miso)) {}
}
