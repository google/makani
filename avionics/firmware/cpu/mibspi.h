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

#ifndef AVIONICS_FIRMWARE_CPU_MIBSPI_H_
#define AVIONICS_FIRMWARE_CPU_MIBSPI_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/spi.h"

// See TMS570 datasheet (Rev A): Table 5-31: MIBSPIx Event Trigger Hookup.
typedef enum {
  kMibSpiSourceDisabled    = 0x00,
  kMibSpiSourceGioA0       = 0x01,
  kMibSpiSourceGioA1       = 0x02,
  kMibSpiSourceGioA2       = 0x03,
  kMibSpiSourceGioA3       = 0x04,
  kMibSpiSourceGioA4       = 0x05,
  kMibSpiSourceGioA5       = 0x06,
  kMibSpiSourceGioA6       = 0x07,
  kMibSpiSourceGioA7       = 0x08,
  kMibSpiSourceN2Het1Pin8  = 0x09,
  kMibSpiSourceN2Het1Pin10 = 0x0A,
  kMibSpiSourceN2Het1Pin12 = 0x0B,
  kMibSpiSourceN2Het1Pin14 = 0x0C,
  kMibSpiSourceN2Het1Pin16 = 0x0D,
  kMibSpiSourceN2Het1Pin18 = 0x0E,
  kMibSpiSourceTickCounter = 0x0F
} MibSpiSource;

// See TMS570 reference (Rev A): Table 28-40: TG Control Registers (TGxCTRL).
typedef enum {
  kMibSpiEventNever       = 0x00,
  kMibSpiEventRisingEdge  = 0x01,
  kMibSpiEventFallingEdge = 0x02,
  kMibSpiEventBothEdges   = 0x03,
  // 0x04 reserved.
  kMibSpiEventHighActive  = 0x05,
  kMibSpiEventLowActive   = 0x06,
  kMibSpiEventAlways      = 0x07
} MibSpiEvent;

void MibSPIInit(int32_t spi, SpiPinmux spi_pins);
int32_t MibSPIAddDevice(int32_t spi, uint8_t csmask, int32_t freq_hz,
                        int32_t wdelay_ns, bool polarity, bool phase,
                        int32_t charlen, int32_t num_buffers);
void MibSPISetChipSelect(int32_t spi, int32_t device, uint8_t csmask,
                         bool cshold, int32_t offset, int32_t num_buffers);
bool MibSPITrigger(int32_t spi, int32_t device, MibSpiSource source,
                   MibSpiEvent event, bool one_shot, bool enable);
bool MibSPITriggerBySoftware(int32_t spi, int32_t device);
bool MibSPITriggerDisable(int32_t spi, int32_t device);
bool MibSPIWriteUint8(int32_t spi, int32_t device, int32_t count,
                      int32_t length, const uint8_t *data);
bool MibSPIWriteUint16(int32_t spi, int32_t device, int32_t count,
                       int32_t length, const uint16_t *data);
bool MibSPIWriteConst(int32_t spi, int32_t device, int32_t count,
                      int32_t length, uint16_t ch);
bool MibSPIReadUint8(int32_t spi, int32_t device, int32_t count,
                     int32_t length, uint8_t *data);
bool MibSPIReadUint16(int32_t spi, int32_t device, int32_t count,
                      int32_t length, uint16_t *data);
bool MibSPITransferUint8Async(int32_t spi, int32_t device, int32_t count,
                              int32_t length, const uint8_t *mosi,
                              uint8_t *miso);
void MibSPITransferUint8(int32_t spi, int32_t device, int32_t count,
                         int32_t length, const uint8_t *mosi, uint8_t *miso);
void MibSPITransferUint16(int32_t spi, int32_t device, int32_t count,
                          int32_t length, const uint16_t *mosi, uint16_t *miso);

#endif  // AVIONICS_FIRMWARE_CPU_MIBSPI_H_
