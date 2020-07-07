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

#ifndef AVIONICS_FIRMWARE_CPU_SPI_H_
#define AVIONICS_FIRMWARE_CPU_SPI_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  kSpiPinmuxMiso = 0x01,
  kSpiPinmuxMosi = 0x02,
  kSpiPinmuxAll  = kSpiPinmuxMiso | kSpiPinmuxMosi
} SpiPinmux;

void SpiInit(int32_t spi, SpiPinmux spi_pins);
int32_t SpiAddDevice(int32_t spi, uint8_t cs_mask, int32_t freq_hz,
                     int32_t wdelay_ns, int32_t charlen, bool polarity,
                     bool phase);
bool SpiPoll(int32_t spi);

// Nonblocking functions.
bool SpiTransferUint8Async(int32_t spi, int32_t device, uint8_t cs_mask,
                           bool cs_hold, int32_t count, int32_t length,
                           const uint8_t *mosi, uint8_t *miso);
bool SpiTransferUint16Async(int32_t spi, int32_t device, uint8_t cs_mask,
                            bool cs_hold, int32_t count, int32_t length,
                            const uint16_t *mosi, uint16_t *miso);

// Blocking code for testing. Not for use in flight code.
void SpiTransferUint8(int32_t spi, int32_t device, uint8_t cs_mask,
                      bool cs_hold, int32_t count, int32_t length,
                      const uint8_t *mosi, uint8_t *miso);
void SpiTransferUint16(int32_t spi, int32_t device, uint8_t cs_mask,
                       bool cs_hold, int32_t count, int32_t length,
                       const uint16_t *mosi, uint16_t *miso);

#endif  // AVIONICS_FIRMWARE_CPU_SPI_H_
