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

#include "avionics/firmware/cpu/spi.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/spi_pin.h"
#include "common/macros.h"

#define SPI_BUSES 3
#define SPI_FORMAT_DEVICES 4  // Array size of SPI(spi).FMT[].

typedef enum {
  kSpiStateIdle,
  kSpiStateTransfer8Bit,
  kSpiStateTransfer16Bit
} SpiState;

typedef struct {
  SpiState state;
  union SPI_DAT1 dat1;    // Transmit object.
  bool cs_hold;           // Hold chip select after each object transfer.
  int32_t count;          // Number of objects remaining in transfer.
  int32_t length;         // Number of bytes in each object.
  int32_t index;          // Number of words remaining in current object.
  const void *mosi_addr;  // Current master-out-slave-in data address.
  void *miso_addr;        // Current master-in-slave-out data address.
} SpiStatus;

static SpiStatus g_bus_status[SPI_BUSES];

static void EnableSpiById(int32_t spi, SpiPinmux spi_pins) {
  switch (spi) {
    case 1:
      PeripheralEnable(kPeripheralMibSpi1);
      SpiPinConfigureForSpi(kSpi1PinEna);
      break;
    case 3:
      PeripheralEnable(kPeripheralMibSpi3);
      SpiPinConfigureForSpi(kSpi3PinClk);
      if ((spi_pins & kSpiPinmuxMosi) == kSpiPinmuxMosi) {
        SpiPinConfigureForSpi(kSpi3PinMosi0);
      }
      if ((spi_pins & kSpiPinmuxMiso) == kSpiPinmuxMiso) {
        SpiPinConfigureForSpi(kSpi3PinMiso0);
      }
      break;
    case 4:
      PeripheralEnable(kPeripheralSpi4);
      SpiPinConfigureForSpi(kSpi4PinClk);
      if ((spi_pins & kSpiPinmuxMosi) == kSpiPinmuxMosi) {
        SpiPinConfigureForSpi(kSpi4PinMosi0);
      }
      if ((spi_pins & kSpiPinmuxMiso) == kSpiPinmuxMiso) {
        SpiPinConfigureForSpi(kSpi4PinMiso0);
      }
      SpiPinConfigureForSpi(kSpi4PinEna);
      break;
    default:
      assert(false);
      break;
  }
}

static int32_t GetClockFreq(int32_t spi) {
  switch (spi) {
    case 1: return PeripheralGetClockFreq(kPeripheralMibSpi1);
    case 2: return PeripheralGetClockFreq(kPeripheralSpi2);
    case 3: return PeripheralGetClockFreq(kPeripheralMibSpi3);
    case 4: return PeripheralGetClockFreq(kPeripheralSpi4);
    case 5: return PeripheralGetClockFreq(kPeripheralMibSpi5);
    default:
      assert(false);
      return -1;
  }
}

static SpiStatus *GetSpiStatus(int32_t spi) {
  switch (spi) {
    case 1:
      return &g_bus_status[0];
    case 3:
      return &g_bus_status[1];
    case 4:
      return &g_bus_status[2];
    default:
      assert(false);
      return NULL;
  }
}

static bool SetupTransfer(SpiState state, int32_t spi, int32_t device,
                          uint8_t cs_mask, bool cs_hold, int32_t count,
                          int32_t length, const void *mosi_addr,
                          void *miso_addr) {
  assert(SpiBusIsValid(spi));
  assert(0 <= device && device < SPI_FORMAT_DEVICES);
  assert(count > 0);
  assert(length > 0);

  // Wait for completion.
  SpiStatus *s = GetSpiStatus(spi);
  if (s == NULL || (s->state != kSpiStateIdle && !SpiPoll(spi))) {
    return false;
  }

  // Prepare transfer.
  s->dat1.raw = 0x0;
  s->dat1.DFSEL = device;
  s->dat1.CSNR = ~cs_mask;  // Chip selects are active low.
  s->cs_hold = cs_hold;
  s->count = count;
  s->length = length;
  s->index = length - 1;
  s->mosi_addr = mosi_addr;
  s->miso_addr = miso_addr;
  s->state = state;
  return true;
}

static void WriteWord8Bit(int32_t spi, SpiStatus *s) {
  s->dat1.CSHOLD = (s->index != 0 || s->cs_hold);
  if (s->mosi_addr != NULL) {
    s->dat1.TXDATA = *(const uint8_t *)s->mosi_addr;
    s->mosi_addr = (const uint8_t *)s->mosi_addr + 1;
  }
  SPI(spi).DAT1 = s->dat1;
}

static void WriteWord16Bit(int32_t spi, SpiStatus *s) {
  s->dat1.CSHOLD = (s->index != 0 || s->cs_hold);
  if (s->mosi_addr != NULL) {
    s->dat1.TXDATA = *(const uint16_t *)s->mosi_addr;
    s->mosi_addr = (const uint16_t *)s->mosi_addr + 1;
  }
  SPI(spi).DAT1 = s->dat1;
}

static bool ReadWord8Bit(int32_t spi, SpiStatus *s) {
  union SPI_BUF buf = SPI(spi).BUF;
  if (!buf.RXEMPTY) {
    if (s->miso_addr != NULL) {
      *(uint8_t *)s->miso_addr = buf.RXDATA;
      s->miso_addr = (uint8_t *)s->miso_addr + 1;
    }
    return true;
  }
  return false;
}

static bool ReadWord16Bit(int32_t spi, SpiStatus *s) {
  union SPI_BUF buf = SPI(spi).BUF;
  if (!buf.RXEMPTY) {
    if (s->miso_addr != NULL) {
      *(uint16_t *)s->miso_addr = buf.RXDATA;
      s->miso_addr = (uint16_t *)s->miso_addr + 1;
    }
    return true;
  }
  return false;
}

static bool LoadNextWord(SpiStatus *s) {
  if (s->index == 0) {
    s->index = s->length - 1;
    --s->count;
  } else {
    --s->index;
  }
  return s->count > 0;
}

static void TransferWord(bool (* const read_func)(int32_t, SpiStatus *),
                         void (* const write_func)(int32_t, SpiStatus *),
                         int32_t spi, SpiStatus *s) {
  if (read_func(spi, s)) {
    if (LoadNextWord(s)) {
      write_func(spi, s);
    } else {
      s->state = kSpiStateIdle;
    }
  }
}

static bool PollTransfer(int32_t spi, SpiStatus *s) {
  switch (s->state) {
    case kSpiStateTransfer8Bit:
      TransferWord(ReadWord8Bit, WriteWord8Bit, spi, s);
      break;
    case kSpiStateTransfer16Bit:
      TransferWord(ReadWord16Bit, WriteWord16Bit, spi, s);
      break;
    case kSpiStateIdle:
      break;
    default:
      assert(false);
      s->state = kSpiStateIdle;
      break;
  }
  return s->state == kSpiStateIdle;
}

void SpiInit(int32_t spi, SpiPinmux spi_pins) {
  assert(SpiBusIsValid(spi));

  // Initialize globals.
  SpiStatus *s = GetSpiStatus(spi);
  memset(s, 0x0, sizeof(*s));

  // Enable pinmux and clocks.
  EnableSpiById(spi, spi_pins);

  // Reset.
  SPI(spi).GCR0.NRESET = 0;
  SPI(spi).GCR0.NRESET = 1;

  // Disable Mib mode.
  SPI(spi).MIBSPIE.MSPIENA = 0;

  // Set master mode.
  SPI(spi).GCR1.CLKMOD = 1;
  SPI(spi).GCR1.MASTER = 1;

  // Enable SOMI, SIMO, and CLK pins.
  SPI(spi).PC0.SOMIFUN0 = ((spi_pins & kSpiPinmuxMiso) == kSpiPinmuxMiso);
  SPI(spi).PC0.SIMOFUN0 = ((spi_pins & kSpiPinmuxMosi) == kSpiPinmuxMosi);
  SPI(spi).PC0.CLKFUN   = 1;

  // Set chip-select to/from transfer delays.
  SPI(spi).DELAY.C2TDELAY = 0;
  SPI(spi).DELAY.T2CDELAY = 0;

  // Enable transfers.
  SPI(spi).GCR1.SPIEN = 1;
}

int32_t SpiAddDevice(int32_t spi, uint8_t cs_mask, int32_t freq_hz,
                     int32_t wdelay_ns, int32_t charlen, bool polarity,
                     bool phase) {
  assert(SpiBusIsValid(spi));
  assert(freq_hz > 0);

  // Enable chip select pins.
  SpiPinConfigureCsMaskForSpi(spi, cs_mask);

  // Specify clock frequency.
  float f_vclk = (float)GetClockFreq(spi);
  int32_t prescale = (int32_t)ceil(f_vclk / freq_hz - 1.0f);

  // Specify word delay in VCLK cycles (required for ADIS16884).
  int32_t wdelay_vclk = ceil(wdelay_ns * f_vclk / 1e9 - 2);
  if (wdelay_vclk < 0) {
    wdelay_vclk = 0;
  }

  // Find next available device number.
  int32_t device;
  for (device = 0; device < SPI_FORMAT_DEVICES; ++device) {
    if (SPI(spi).FMT[device].CHARLEN == 0) {
      break;
    }
  }

  assert(0 <= device && device < SPI_FORMAT_DEVICES);
  assert(0 <= prescale && prescale <= 0xFF);
  assert(0 <= wdelay_vclk && wdelay_vclk <= 0xFF);
  assert(2 <= charlen && charlen <= 16);

  SPI(spi).FMT[device].PRESCALE = prescale;
  SPI(spi).FMT[device].WDELAY = wdelay_vclk;
  SPI(spi).FMT[device].POLARITY = polarity;
  SPI(spi).FMT[device].PHASE = !phase;
  SPI(spi).FMT[device].CHARLEN = charlen;
  SPI(spi).FMT[device].DISCSTIMERS = 1;
  SPI(spi).FMT[device].SHIFTDIR = 0;  // 0 = MSB first.

  // Enable CS pin.
  SPI(spi).PC0.SCSFUN |= cs_mask;

  return device;
}

bool SpiPoll(int32_t spi) {
  SpiStatus *s = GetSpiStatus(spi);
  if (s != NULL) {
    return PollTransfer(spi, s);
  }
  return false;
}

bool SpiTransferUint8Async(int32_t spi, int32_t device, uint8_t cs_mask,
                           bool cs_hold, int32_t count, int32_t length,
                           const uint8_t *mosi, uint8_t *miso) {
  if (SetupTransfer(kSpiStateTransfer8Bit, spi, device, cs_mask, cs_hold,
                    count, length, mosi, miso)) {
    WriteWord8Bit(spi, GetSpiStatus(spi));
    return true;
  }
  return false;
}

bool SpiTransferUint16Async(int32_t spi, int32_t device, uint8_t cs_mask,
                            bool cs_hold, int32_t count, int32_t length,
                            const uint16_t *mosi, uint16_t *miso) {
  if (SetupTransfer(kSpiStateTransfer16Bit, spi, device, cs_mask, cs_hold,
                    count, length, mosi, miso)) {
    WriteWord16Bit(spi, GetSpiStatus(spi));
    return true;
  }
  return false;
}

void SpiTransferUint8(int32_t spi, int32_t device, uint8_t cs_mask,
                      bool cs_hold, int32_t count, int32_t length,
                      const uint8_t *mosi, uint8_t *miso) {
  while (!SpiTransferUint8Async(spi, device, cs_mask, cs_hold, count, length,
                                mosi, miso)) {}
  while (!SpiPoll(spi)) {}
}

void SpiTransferUint16(int32_t spi, int32_t device, uint8_t cs_mask,
                       bool cs_hold, int32_t count, int32_t length,
                       const uint16_t *mosi, uint16_t *miso) {
  while (!SpiTransferUint16Async(spi, device, cs_mask, cs_hold, count, length,
                                 mosi, miso)) {}
  while (!SpiPoll(spi)) {}
}
