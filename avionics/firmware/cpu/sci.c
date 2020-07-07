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

#include "avionics/firmware/cpu/sci.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/firmware/cpu/clock.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/peripherals.h"
#include "avionics/firmware/cpu/registers.h"
#include "avionics/firmware/cpu/vim.h"
#include "common/barrier.h"
#include "common/macros.h"

#define SCI_TALK_TIMEOUT_CYCLES CLOCK32_MSEC_TO_CYCLES(10)

// Buffer sizes divide evenly into 2^32 to ensure that indices remain
// continuous during uint32_t rollovers.
COMPILE_ASSERT(UINT32_MAX % SCI_RECEIVE_SIZE == SCI_RECEIVE_SIZE - 1,
               SCI_RECEIVE_SIZE_must_divide_evenly_into_UINT32);
COMPILE_ASSERT(UINT32_MAX % SCI_TRANSMIT_SIZE == SCI_TRANSMIT_SIZE - 1,
               SCI_TRANSMIT_SIZE_must_divide_evenly_into_UINT32);

static void InterruptHandler1(void);
static void InterruptHandler2(void);

static const SciPeripheral kSciPeripheral1 = {
  .peripheral = kPeripheralSci1,
  .registers = &SCI(1),
  .receive_buffer = (SciReceiveBuffer[1]) {},
  .receive_pinmux = {0, 0},  // Not multiplexed, mark invalid.
  .transmit_buffer = (SciTransmitBuffer[1]) {},
  .transmit_pinmux = {0, 0},  // Not multiplexed, mark invalid.
  .interrupt_function = InterruptHandler1,
  .interrupt_channel = kVimChannelSci1Level0
};

static const SciPeripheral kSciPeripheral2 = {
  .peripheral = kPeripheralSci2,
  .registers = &SCI(2),
  .receive_buffer = (SciReceiveBuffer[1]) {},
  .receive_pinmux = {7, 17},
  .transmit_buffer = (SciTransmitBuffer[1]) {},
  .transmit_pinmux = {8, 1},
  .interrupt_function = InterruptHandler2,
  .interrupt_channel = kVimChannelSci2Level0
};

static void InterruptHandler(volatile SciRegisters *reg, SciReceiveBuffer *rx,
                             SciTransmitBuffer *tx) {
  switch (reg->INTVECT0.raw) {
    case 11:  // Receive interrupt.
      rx->data[rx->head % SCI_RECEIVE_SIZE] = reg->RD.RD;
      ++rx->head;
      break;
    case 12:  // Transmit interrupt.
      if (tx->tail != tx->head) {
        reg->TD.TD = tx->data[tx->tail % SCI_TRANSMIT_SIZE];
        ++tx->tail;
      } else {
        reg->CLEARINT.raw = SCI_CLEARINT_CLR_TX_INT;
      }
      break;
    default:
      break;
  }
}

static void InterruptHandler1(void) {
  InterruptHandler(kSciPeripheral1.registers, kSciPeripheral1.receive_buffer,
                   kSciPeripheral1.transmit_buffer);
}

static void InterruptHandler2(void) {
  InterruptHandler(kSciPeripheral2.registers, kSciPeripheral2.receive_buffer,
                   kSciPeripheral2.transmit_buffer);
}

static void Init(const SciPeripheral *per, int32_t baud, SciMode mode) {
  assert(per != NULL);
  assert(baud > 0);

  // Set pinmux and enables.
  IommSetPinmuxByIndex(&per->receive_pinmux, 1, 0);
  IommSetPinmuxByIndex(&per->transmit_pinmux, 1, 0);
  PeripheralEnable(per->peripheral);

  // Reset.
  volatile SciRegisters *reg = per->registers;
  reg->GCR0.RESET = 0;
  reg->GCR0.RESET = 1;

  // Issue soft reset.
  reg->GCR1.raw = 0;

  // Disable interrupts.
  reg->CLEARINT.raw = 0xFFFFFFFF;

  // Select INT0 interrupt level.
  reg->CLEARINTLVL.raw = 0xFFFFFFFF;

  // Clear pending interrupts.
  reg->FLR.raw = 0xFFFFFFFF;
  reg->FLR.raw = 0;

  // Set receive mode.
  reg->GCR1.RXENA = 1;
  reg->PIO0.RXFUNC = 1;

  // Set transmit mode.
  reg->GCR1.TXENA = 1;
  reg->PIO0.TXFUNC = 1;

  // Set asynchronous timing.
  reg->GCR1.CLOCK = 1;
  reg->GCR1.TIMINGMODE = 1;

  // Set character length to 8 bits.
  reg->FORMAT.CHAR = 7;

  // Set baud rate.
  reg->BRS.BAUD = PeripheralGetClockPrescale(per->peripheral, 16 * baud) - 1;

  // Clear receive buffer.
  SciReceiveBuffer *receive_buffer = per->receive_buffer;
  receive_buffer->head = 0U;
  receive_buffer->tail = 0U;
  receive_buffer->is_talking = false;
  receive_buffer->timeout = 0U;

  // Clear transmit buffer.
  SciTransmitBuffer *transmit_buffer = per->transmit_buffer;
  transmit_buffer->head = 0U;
  transmit_buffer->tail = 0U;

  // Register interrupt handlers.
  if (mode == kSciModeInterrupt) {
    VimRegisterIrq(per->interrupt_channel, per->interrupt_function);
    VimEnableInterrupt(per->interrupt_channel);
  }

  // Enable interrupts.
  if (mode == kSciModeInterrupt) {
    reg->SETINT.raw = SCI_SETINT_SET_RX_INT;
  }

  // Release reset hold.
  reg->GCR1.SWNRST = 1;
}

static bool PollReceive(const SciPeripheral *per) {
  volatile SciRegisters *reg = per->registers;

  // Receive. Operation not permitted while interrupts are enabled.
  if (!reg->SETINT.SET_RX_INT && reg->FLR.RXRDY) {
    SciReceiveBuffer *read = per->receive_buffer;
    uint8_t rd = reg->RD.RD;
    if ((int32_t)(read->head - read->tail) < SCI_RECEIVE_SIZE) {
      read->data[read->head % SCI_RECEIVE_SIZE] = rd;
      ++read->head;
    }
    return true;
  }
  return false;
}

static void PollTransmit(const SciPeripheral *per) {
  volatile SciRegisters *reg = per->registers;

  // Transmit. Operation not permitted while interrupts are enabled.
  SciTransmitBuffer *write = per->transmit_buffer;
  if (!reg->SETINT.SET_TX_INT && reg->FLR.TXRDY
      && write->head != write->tail) {
    reg->TD.TD = write->data[write->tail % SCI_TRANSMIT_SIZE];
    ++write->tail;
  }
}

static bool PollAsync(const SciPeripheral *per) {
  PollTransmit(per);
  return PollReceive(per);
}

// Added for Gill Windmaster. This sensor requires half duplex operation. For
// each transmit byte, we must wait for the device to respond via a single
// character (often an echo of the character sent) or stream of characters. If
// we transmit while receiving data, the device will stop transmitting data.
static bool PollHalfDuplex(const SciPeripheral *per) {
  volatile SciRegisters *reg = per->registers;

  // Receive.
  SciReceiveBuffer *read = per->receive_buffer;
  union SCI_FLR flr = reg->FLR;
  if (flr.RXRDY) {
    uint8_t rd = reg->RD.RD;
    if ((int32_t)(read->head - read->tail) < SCI_RECEIVE_SIZE) {
      read->data[read->head % SCI_RECEIVE_SIZE] = rd;
      ++read->head;
    }
    read->is_talking = true;
    read->timeout = Clock32GetCycles() + SCI_TALK_TIMEOUT_CYCLES;
  } else if (read->is_talking && !flr.RXRDY && !flr.BUSY
             && CLOCK32_GE(Clock32GetCycles(), read->timeout)) {
    read->is_talking = false;
  }

  // Transmit only when clear to talk.
  SciTransmitBuffer *write = per->transmit_buffer;
  if (!read->is_talking && write->head != write->tail && flr.TXRDY) {
    reg->TD.TD = write->data[write->tail % SCI_TRANSMIT_SIZE];
    ++write->tail;

    // Give time for the device to respond.
    read->is_talking = true;
    read->timeout = Clock32GetCycles() + SCI_TALK_TIMEOUT_CYCLES;
  }
  return read->is_talking;
}

static int32_t ReadBlocking(const SciPeripheral *per, int32_t length,
                            uint8_t *data) {
  assert(data != NULL);

  for (int32_t i = 0; i < length; ++i) {
    volatile SciRegisters *reg = per->registers;
    while (!reg->FLR.RXRDY) {}
    data[i] = reg->RD.RD;
  }
  return length;
}

// This function may be interrupted.
static int32_t ReadAsync(const SciPeripheral *per, int32_t length,
                         uint8_t *data) {
  assert(data != NULL);

  SciReceiveBuffer *buf = per->receive_buffer;
  int32_t count = 0;
  for ( ; count < length && buf->head != buf->tail; ++count) {
    data[count] = buf->data[buf->tail % SCI_RECEIVE_SIZE];
    // Since this function may be interrupted, we need to ensure that we
    // load data from the buffer before incrementing tail. If not, the
    // interrupt function could update the data at the tail position before
    // we read.
    MemoryBarrier();
    ++buf->tail;
  }
  return count;
}

static int32_t WriteBlocking(const SciPeripheral *per, int32_t length,
                             const uint8_t *data) {
  assert(data != NULL);

  for (int32_t i = 0; i < length; ++i) {
    volatile SciRegisters *reg = per->registers;
    while (!reg->FLR.TXRDY) {}
    reg->TD.TD = data[i];
  }
  return length;
}

static int32_t WriteBuffered(const SciPeripheral *per, int32_t length,
                             const uint8_t *data) {
  assert(data != NULL);

  SciTransmitBuffer *buf = per->transmit_buffer;
  int32_t count = 0;
  for ( ; (count < length
           && (int32_t)(buf->head - buf->tail) < SCI_TRANSMIT_SIZE); ++count) {
    buf->data[buf->head % SCI_TRANSMIT_SIZE] = data[count];
    ++buf->head;
  }
  return count;
}

// This function may be interrupted.
static int32_t WriteInterrupt(const SciPeripheral *per, int32_t length,
                              const uint8_t *data) {
  assert(data != NULL);

  SciTransmitBuffer *buf = per->transmit_buffer;
  int32_t count = 0;
  for ( ; (count < length
           && (int32_t)(buf->head - buf->tail) < SCI_TRANSMIT_SIZE); ++count) {
    volatile SciRegisters *reg = per->registers;
    if (buf->head == buf->tail && reg->FLR.TXRDY) {
      reg->TD.TD = data[count];
    } else {
      buf->data[buf->head % SCI_TRANSMIT_SIZE] = data[count];
      // Since this function may be interrupted, we need to ensure that we
      // store data into the buffer before incrementing head. If not, the
      // interrupt function could transmit stale data before we update the
      // buffer.
      MemoryBarrier();
      ++buf->head;
    }
    reg->SETINT.raw = SCI_SETINT_SET_TX_INT;
  }
  return count;
}

static bool IsWriteBusy(const SciPeripheral *per) {
  volatile SciRegisters *reg = per->registers;
  const SciTransmitBuffer *buf = per->transmit_buffer;

  union SCI_FLR flr = reg->FLR;
  return !(flr.TXRDY && flr.TXEMPTY
           && !reg->SETINT.SET_TX_INT && buf->head == buf->tail);
}

static void SetBreak(const SciPeripheral *per) {
  volatile SciRegisters *reg = per->registers;

  reg->PIO0.TXFUNC = 0;
  reg->PIO1.TXDIR = 1;
  reg->PIO5.TXCLR = 1;
}

static void ClearBreak(const SciPeripheral *per) {
  volatile SciRegisters *reg = per->registers;

  reg->PIO0.TXFUNC = 1;
}

static void ClearWrite(const SciPeripheral *per) {
  volatile SciRegisters *reg = per->registers;
  SciTransmitBuffer *buf = per->transmit_buffer;

  reg->CLEARINT.raw = SCI_CLEARINT_CLR_TX_INT;
  buf->tail = buf->head;
}

// This function may be interrupted.
static void AcquireReceiveData(const SciPeripheral *per, SciData *data) {
  const SciReceiveBuffer *buf = per->receive_buffer;
  uint32_t tail = buf->tail;
  uint32_t head = buf->head;

  // Since this function may be interrupted, we must ensure consistency of the
  // head position throughout this function.
  MemoryBarrier();

  int32_t tail_wrapped = tail % SCI_RECEIVE_SIZE;
  int32_t head_wrapped = head % SCI_RECEIVE_SIZE;

  // Acquire contiguous data.
  data->data = &buf->data[tail_wrapped];
  if (head == tail) {
    data->length = 0;  // Empty buffer.
  } else if (head_wrapped > tail_wrapped) {
    data->length = head_wrapped - tail_wrapped;  // Tail to head.
  } else {
    data->length = SCI_RECEIVE_SIZE - tail_wrapped;  // Tail to end of buffer.
  }
}

static void ReleaseReceiveData(const SciPeripheral *per, SciData *data) {
  SciReceiveBuffer *buf = per->receive_buffer;

  buf->tail += data->length;
  data->length = 0;
}

// Application functions.

void SciInit(const SciDevice *dev, int32_t baud) {
  assert(dev != NULL);
  Init(dev->peripheral, baud, dev->mode);
}

bool SciPoll(const SciDevice *dev) {
  assert(dev != NULL && dev->poll != NULL);
  return dev->poll(dev->peripheral);
}

int32_t SciRead(const SciDevice *dev, int32_t length, uint8_t *data) {
  assert(dev != NULL && dev->read != NULL);
  return dev->read(dev->peripheral, length, data);
}

bool SciReadByte(const SciDevice *dev, uint8_t *data) {
  assert(dev != NULL && dev->read != NULL);
  return dev->read(dev->peripheral, 1, data) == 1;
}

int32_t SciWrite(const SciDevice *dev, int32_t length, const uint8_t *data) {
  assert(dev != NULL && dev->write != NULL);
  return dev->write(dev->peripheral, length, data);
}

bool SciWriteByte(const SciDevice *dev, const uint8_t data) {
  assert(dev != NULL && dev->write != NULL);
  return dev->write(dev->peripheral, 1, &data) == 1;
}

int32_t SciWriteString(const SciDevice *dev, const char *str) {
  assert(str != NULL);
  int32_t count = 0;
  for ( ; str[count] && SciWriteByte(dev, (const uint8_t)str[count]); ++count) {
  }
  return count;
}

bool SciIsWriteBusy(const SciDevice *dev) {
  assert(dev != NULL);
  return IsWriteBusy(dev->peripheral);
}

void SciSetBreak(const SciDevice *dev) {
  assert(dev != NULL);
  SetBreak(dev->peripheral);
}

void SciClearBreak(const SciDevice *dev) {
  assert(dev != NULL);
  ClearBreak(dev->peripheral);
}

void SciClearWrite(const SciDevice *dev) {
  assert(dev != NULL);
  ClearWrite(dev->peripheral);
}

void SciAcquireReceiveData(const SciDevice *dev, SciData *data) {
  assert(dev != NULL);
  AcquireReceiveData(dev->peripheral, data);
}

void SciReleaseReceiveData(const SciDevice *dev, SciData *data) {
  assert(dev != NULL);
  ReleaseReceiveData(dev->peripheral, data);
}

const SciDevice kSci1Blocking = {
  .poll = PollAsync,
  .read = ReadBlocking,
  .write = WriteBlocking,
  .peripheral = &kSciPeripheral1,
  .mode = kSciModePolled
};

const SciDevice kSci1Buffered = {
  .poll = PollAsync,
  .read = ReadAsync,
  .write = WriteBuffered,
  .peripheral = &kSciPeripheral1,
  .mode = kSciModePolled
};

const SciDevice kSci1BufferedHalfDuplex = {
  .poll = PollHalfDuplex,
  .read = ReadAsync,
  .write = WriteBuffered,
  .peripheral = &kSciPeripheral1,
  .mode = kSciModePolled
};

const SciDevice kSci1Interrupt = {
  .poll = PollAsync,
  .read = ReadAsync,
  .write = WriteInterrupt,
  .peripheral = &kSciPeripheral1,
  .mode = kSciModeInterrupt
};

const SciDevice kSci2Blocking = {
  .poll = PollAsync,
  .read = ReadBlocking,
  .write = WriteBlocking,
  .peripheral = &kSciPeripheral2,
  .mode = kSciModePolled
};

const SciDevice kSci2Buffered = {
  .poll = PollAsync,
  .read = ReadAsync,
  .write = WriteBuffered,
  .peripheral = &kSciPeripheral2,
  .mode = kSciModePolled
};

const SciDevice kSci2BufferedHalfDuplex = {
  .poll = PollHalfDuplex,
  .read = ReadAsync,
  .write = WriteBuffered,
  .peripheral = &kSciPeripheral2,
  .mode = kSciModePolled
};

const SciDevice kSci2Interrupt = {
  .poll = PollAsync,
  .read = ReadAsync,
  .write = WriteInterrupt,
  .peripheral = &kSciPeripheral2,
  .mode = kSciModeInterrupt
};
