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

#ifndef AVIONICS_FIRMWARE_CPU_SCI_H_
#define AVIONICS_FIRMWARE_CPU_SCI_H_

#include <stdbool.h>
#include <stdint.h>

#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/vim.h"

// Applications should create their own buffer for parsing received data.
#define SCI_RECEIVE_SIZE  64
#define SCI_TRANSMIT_SIZE 2048

// TODO: Add DMA mode.
typedef enum {
  kSciModePolled,
  kSciModeInterrupt
} SciMode;

typedef struct {
  uint8_t data[SCI_RECEIVE_SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
  bool is_talking;
  uint32_t timeout;
} SciReceiveBuffer;

typedef struct {
  uint8_t data[SCI_TRANSMIT_SIZE];
  volatile uint32_t head;
  volatile uint32_t tail;
} SciTransmitBuffer;

typedef struct {
  Peripheral peripheral;
  volatile SciRegisters *registers;
  SciReceiveBuffer *receive_buffer;
  IommPinmux receive_pinmux;
  SciTransmitBuffer *transmit_buffer;
  IommPinmux transmit_pinmux;
  VimInterruptFunc interrupt_function;
  VimChannel interrupt_channel;
} SciPeripheral;

// Store device definitions in .const memory space to avoid corruption.
typedef struct {
  // Returns true when receiving data.
  bool (* const poll)(const SciPeripheral *per);
  int32_t (* const read)(const SciPeripheral *per, int32_t length,
                         uint8_t *data);
  int32_t (* const write)(const SciPeripheral *per, int32_t length,
                          const uint8_t *data);

  const SciPeripheral *peripheral;
  SciMode mode;
} SciDevice;

typedef struct {
  const uint8_t *data;
  int32_t length;
} SciData;

extern const SciDevice kSci1Blocking;   // Do not use in flight code.
extern const SciDevice kSci1Buffered;   // Asynchronous.
extern const SciDevice kSci1BufferedHalfDuplex;  // Asynchronous.
extern const SciDevice kSci1Interrupt;  // Asynchronous.

extern const SciDevice kSci2Blocking;   // Do not use in flight code.
extern const SciDevice kSci2Buffered;   // Asynchronous.
extern const SciDevice kSci2BufferedHalfDuplex;  // Asynchronous.
extern const SciDevice kSci2Interrupt;  // Asynchronous.

void SciInit(const SciDevice *dev, int32_t baud);
bool SciPoll(const SciDevice *dev);
int32_t SciRead(const SciDevice *dev, int32_t length, uint8_t *data);
bool SciReadByte(const SciDevice *dev, uint8_t *data);
int32_t SciWrite(const SciDevice *dev, int32_t length, const uint8_t *data);
bool SciWriteByte(const SciDevice *dev, const uint8_t data);
int32_t SciWriteString(const SciDevice *dev, const char *str);
bool SciIsWriteBusy(const SciDevice *dev);
void SciSetBreak(const SciDevice *dev);
void SciClearBreak(const SciDevice *dev);
void SciClearWrite(const SciDevice *dev);

void SciAcquireReceiveData(const SciDevice *dev, SciData *data);
void SciReleaseReceiveData(const SciDevice *dev, SciData *data);

#endif  // AVIONICS_FIRMWARE_CPU_SCI_H_
