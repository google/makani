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

#include "avionics/firmware/cpu/emac.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "avionics/common/network_config.h"
#include "avionics/common/strings.h"
#include "avionics/firmware/cpu/iomm.h"
#include "avionics/firmware/cpu/memcpy.h"
#include "avionics/firmware/cpu/registers.h"
#include "common/macros.h"

// Buffer configuration.
#define BUFFER_LENGTH        256
#define NUM_TRANSMIT_BUFFERS 16
#define NUM_RECEIVE_BUFFERS  16
COMPILE_ASSERT((NUM_TRANSMIT_BUFFERS & (NUM_TRANSMIT_BUFFERS - 1)) == 0,
               NUM_TRANSMIT_BUFFERS_must_be_a_power_of_2);
COMPILE_ASSERT((NUM_RECEIVE_BUFFERS & (NUM_RECEIVE_BUFFERS - 1)) == 0,
               NUM_RECEIVE_BUFFERS_must_be_a_power_of_2);

// Buffer Descriptor flags.
#define SOP   0x8000
#define EOP   0x4000
#define OWNER 0x2000
#define EOQ   0x1000

// Buffer descriptor format.
typedef struct _BufferDescriptor {
  volatile struct _BufferDescriptor *next_descriptor;
  void *buffer;
  uint16_t buffer_offset;
  uint16_t buffer_length;
  uint16_t flags;
  uint16_t packet_length;
} __attribute__((packed)) BufferDescriptor;

// Ring buffer. Size must be a power of 2.
typedef struct {
  volatile BufferDescriptor *base;
  int32_t size;
  uint32_t push_count;
  uint32_t pop_count;
} RingBuffer;

// Initializes a ring buffer.
static void RingBufferInit(RingBuffer *rb,
                           volatile BufferDescriptor *base,
                           int32_t size) {
  rb->base       = base;
  rb->size       = size;
  rb->push_count = 0;
  rb->pop_count  = 0;
}

// Returns the number of used elements in a ring buffer.
static int32_t RingBufferUsed(const RingBuffer *rb) {
  return (int32_t)(rb->push_count - rb->pop_count);
}

// Returns the number of free elements in a ring buffer.
static int32_t RingBufferFree(const RingBuffer *rb) {
  return rb->size - RingBufferUsed(rb);
}

// Returns true if a ring buffer is full.
static bool RingBufferFull(const RingBuffer *rb) {
  return RingBufferFree(rb) == 0;
}

// Returns true if a ring buffer is empty.
static bool RingBufferEmpty(const RingBuffer *rb) {
  return RingBufferUsed(rb) == 0;
}

// Returns the element at a given index in a ring buffer with wraparound.
static volatile BufferDescriptor *RingBufferGet(const RingBuffer *rb,
                                                uint32_t index) {
  return &rb->base[index & (rb->size - 1)];
}

// Pushes an element into a ring buffer and returns a pointer to it, or NULL if
// the ring buffer is full.
static volatile BufferDescriptor *RingBufferPush(RingBuffer *rb) {
  return RingBufferFull(rb) ? NULL : RingBufferGet(rb, rb->push_count++);
}

// Pops an element from a ring buffer and returns a pointer to it, or NULL if
// the ring buffer is empty.
static volatile BufferDescriptor *RingBufferPop(RingBuffer *rb) {
  return RingBufferEmpty(rb) ? NULL : RingBufferGet(rb, rb->pop_count++);
}

// Returns a pointer to the most recently pushed element in a ring buffer, or
// NULL if the ring buffer is empty.
static volatile BufferDescriptor *RingBufferBack(const RingBuffer *rb) {
  return RingBufferEmpty(rb) ? NULL : RingBufferGet(rb, rb->push_count - 1);
}

// Returns a pointer to the most recently popped element in a ring buffer, or
// NULL if the ring buffer is full.
static volatile BufferDescriptor *RingBufferPrev(const RingBuffer *rb) {
  return RingBufferFull(rb) ? NULL : RingBufferGet(rb, rb->pop_count - 1);
}

// Returns a pointer to the next element to be pushed in a ring buffer, or
// NULL if the ring buffer is full.
static volatile BufferDescriptor *RingBufferNext(const RingBuffer *rb) {
  return RingBufferFull(rb) ? NULL : RingBufferGet(rb, rb->push_count);
}

// Transmit ring buffer containing buffer descriptors waiting to be processed by
// the EMAC.
static RingBuffer g_transmit_rb;
static uint8_t g_transmit_buffers[NUM_TRANSMIT_BUFFERS][BUFFER_LENGTH];

// Receive ring buffer containing buffer descriptors waiting to be processed by
// the driver.
static RingBuffer g_receive_rb;
static uint8_t g_receive_buffers[NUM_RECEIVE_BUFFERS][BUFFER_LENGTH];

// Reclaims buffers from the transmit queue, restarting transmits if necessary.
static void PollTransmitQueue(void) {
  if (EMAC.TXINTSTATRAW.TX0PEND) {
    volatile BufferDescriptor *cp
        = (volatile BufferDescriptor *)EMAC.TXCP[0].TXCP;
    if ((cp->flags & EOQ) && (cp->next_descriptor != NULL)) {
      EMAC.TXHDP[0].TXHDP = (uint32_t)cp->next_descriptor;
    }
    while (RingBufferPop(&g_transmit_rb) != cp) {}
    EMAC.TXCP[0].TXCP = (uint32_t)cp;
  }
}

// Reclaims buffers from the receive queue, restarting receives if necessary.
static void PollReceiveQueue(void) {
  if (EMAC.RXINTSTATRAW.RX0PEND) {
    volatile BufferDescriptor *cp
        = (volatile BufferDescriptor *)EMAC.RXCP[0].RXCP;
    if ((cp->flags & EOQ) && (cp->next_descriptor != NULL)) {
      EMAC.RXHDP[0].RXHDP = (uint32_t)cp->next_descriptor;
    }
    while (RingBufferPush(&g_receive_rb) != cp) {}
    EMAC.RXCP[0].RXCP = (uint32_t)cp;
  }
}

void EmacInit(EthernetAddress mac) {
  // Set pinmux.
  IommSetPinmux(10, 1);   // MII_RXER
  IommSetPinmux(11, 26);  // MII_RXD[0]
  IommSetPinmux(12, 1);   // MII_RXD[1]
  IommSetPinmux(12, 18);  // MII_RXD[2]
  IommSetPinmux(12, 26);  // MII_RXD[3]
  IommSetPinmux(13, 2);   // MII_TXD[0]
  IommSetPinmux(13, 10);  // MII_TXD[1]
  IommSetPinmux(13, 18);  // MII_TXEN
  IommSetPinmux(13, 26);  // MII_TXD[2]
  IommSetPinmux(14, 2);   // MII_TXD[3]
  IommSetPinmux(14, 9);   // MII_RXCLK
  IommSetPinmux(17, 1);   // MII_TXCLK
  IommSetPinmux(17, 7);   // MII_CRS
  IommSetPinmux(19, 9);   // MII_RXDV
  IommSetPinmux(20, 18);  // MII_COL

  // Set MII mode.
  IommClearPinmux(29, 24);

  // Reset.
  EMAC.SOFTRESET.SOFTRESET = 1;
  while (EMAC.SOFTRESET.SOFTRESET) {}

  // Clear head descriptor pointers.
  for (int32_t i = 0; i < ARRAYSIZE(EMAC.TXHDP); ++i) {
    EMAC.TXHDP[i].raw = 0x0;
    EMAC.RXHDP[i].raw = 0x0;
  }

  // Set local MAC address.
  EMAC.MACSRCADDRHI.MACSRCADDR5 = mac.a;
  EMAC.MACSRCADDRHI.MACSRCADDR4 = mac.b;
  EMAC.MACSRCADDRHI.MACSRCADDR3 = mac.c;
  EMAC.MACSRCADDRHI.MACSRCADDR2 = mac.d;
  EMAC.MACSRCADDRLO.MACSRCADDR1 = mac.e;
  EMAC.MACSRCADDRLO.MACSRCADDR0 = mac.f;

  // Set MAC address filters to direct matching received packets to channel 0.
  for (int32_t i = 0; i < 8; ++i) {
    EMAC.MACINDEX.MACINDEX   = i;
    EMAC.MACADDRHI.raw       = mac.d << 24 | mac.c << 16 | mac.b << 8 | mac.a;
    EMAC.MACADDRLO.MACADDR1  = mac.e;
    EMAC.MACADDRLO.MACADDR0  = mac.f;
    EMAC.MACADDRLO.VALID     = (i == 0);
    EMAC.MACADDRLO.MATCHFILT = (i == 0);
    EMAC.MACADDRLO.CHANNEL   = 0;
  }

  // Set MAC multicast address hash filters to allow all multicast packets.
  EMAC.MACHASH1.MACHASH1 = 0xFFFFFFFF;
  EMAC.MACHASH2.MACHASH2 = 0xFFFFFFFF;

  // Enable unicast receive on channel 0.
  EMAC.RXUNICASTSET.RXCH0EN = 1;

  // Enable multicast receive on channel 0.
  EMAC.RXMBPENABLE.RXMULTEN = 1;
  EMAC.RXMBPENABLE.RXMULTCH = 0;

  // Enable broadcast receive on channel 0.
  EMAC.RXMBPENABLE.RXBROADEN = 1;
  EMAC.RXMBPENABLE.RXBROADCH = 0;

  // Enable full-duplex.
  EMAC.MACCONTROL.FULLDUPLEX = 1;

  // Set up transmit buffers.
  volatile BufferDescriptor *transmit_base
      = (volatile BufferDescriptor *)0xFC520000;
  RingBufferInit(&g_transmit_rb, transmit_base, NUM_TRANSMIT_BUFFERS);
  for (int32_t i = 0; i < NUM_TRANSMIT_BUFFERS; ++i) {
    volatile BufferDescriptor *bd = &transmit_base[i];
    bd->next_descriptor = NULL;
    bd->buffer          = g_transmit_buffers[i];
    bd->buffer_offset   = 0;
    bd->buffer_length   = 0;
    bd->flags           = 0;
    bd->packet_length   = 0;
  }

  // Set up receive buffers.
  volatile BufferDescriptor *receive_base
      = transmit_base + NUM_TRANSMIT_BUFFERS;
  RingBufferInit(&g_receive_rb, receive_base, NUM_RECEIVE_BUFFERS);
  for (int32_t i = 0; i < NUM_RECEIVE_BUFFERS; ++i) {
    volatile BufferDescriptor *bd = &receive_base[i];
    bd->next_descriptor = (i < NUM_RECEIVE_BUFFERS - 1) ? bd + 1 : NULL;
    bd->buffer          = g_receive_buffers[i];
    bd->buffer_offset   = 0;
    bd->buffer_length   = BUFFER_LENGTH;
    bd->flags           = OWNER;
    bd->packet_length   = 0;
  }
  EMAC.RXHDP[0].RXHDP = (uint32_t)receive_base;

  // Enable transfers.
  EMAC.TXCONTROL.TXEN    = 1;
  EMAC.RXCONTROL.RXEN    = 1;
  EMAC.MACCONTROL.GMIIEN = 1;
}

bool EmacSend(const void *buf, int32_t len) {
  assert(buf != NULL);
  assert(MIN_ETHERNET_LENGTH <= len && len <= MAX_ETHERNET_LENGTH_VLAN);

  // Check for completed transmits.
  PollTransmitQueue();

  // Get number of buffers required.
  int32_t num_buffers = (len + BUFFER_LENGTH - 1) / BUFFER_LENGTH;

  // Not enough buffers.
  if (num_buffers > RingBufferFree(&g_transmit_rb)) {
    return false;
  }

  // Build buffer list and copy data.
  volatile BufferDescriptor *head = RingBufferNext(&g_transmit_rb);
  volatile BufferDescriptor *tail = RingBufferBack(&g_transmit_rb);
  volatile BufferDescriptor *prev = NULL;
  int32_t offset = 0;
  for (int32_t i = 0; i < num_buffers; ++i) {
    bool start = (i == 0);
    bool end   = (i == num_buffers - 1);

    // Get next buffer.
    volatile BufferDescriptor *bd = RingBufferPush(&g_transmit_rb);

    // Get length to copy.
    int32_t copy_length = len < BUFFER_LENGTH ? len : BUFFER_LENGTH;

    // Initialize for transmit.
    bd->next_descriptor = NULL;
    bd->buffer_length   = copy_length;
    bd->flags           = (start ? SOP | OWNER : 0) | (end ? EOP : 0);
    bd->packet_length   = start ? len : 0;

    // Copy and advance data.
    FastCopy(copy_length, (const uint8_t *)buf + offset, bd->buffer);
    offset += copy_length;
    len -= copy_length;

    // Append to previous descriptor.
    if (prev != NULL) {
      prev->next_descriptor = bd;
    }
    prev = bd;
  }

  // Append to transmit queue.
  if (tail == NULL) {
    EMAC.TXHDP[0].TXHDP = (uint32_t)head;
  } else {
    tail->next_descriptor = head;
  }

  // Restart transmits if necessary.
  PollTransmitQueue();

  return true;
}

bool EmacReceive(void *buf, int32_t *len) {
  assert(buf != NULL);
  assert(MIN_ETHERNET_LENGTH <= *len && *len <= MAX_ETHERNET_LENGTH_VLAN);

  // Check for completed receives.
  PollReceiveQueue();

  // No received packets.
  if (RingBufferEmpty(&g_receive_rb)) {
    return false;
  }

  // Consume buffer list and copy data.
  int32_t offset = 0;
  while (true) {
    // Get tail of receive queue.
    volatile BufferDescriptor *tail = RingBufferPrev(&g_receive_rb);

    // Get next buffer.
    volatile BufferDescriptor *bd = RingBufferPop(&g_receive_rb);

    // Get length to copy.
    int32_t copy_length = *len < bd->buffer_length ? *len : bd->buffer_length;

    // Copy and advance data.
    FastCopy(copy_length, bd->buffer, (uint8_t *)buf + offset);
    offset += copy_length;
    *len -= copy_length;

    // Check for end of list.
    bool end = bd->flags & EOP;

    // Initialize for receive.
    bd->next_descriptor = NULL;
    bd->buffer_length   = BUFFER_LENGTH;
    bd->flags           = OWNER;
    bd->packet_length   = 0;

    // Append to receive queue.
    if (tail == NULL) {
      EMAC.RXHDP[0].RXHDP = (uint32_t)bd;
    } else {
      tail->next_descriptor = bd;
    }

    // End of list.
    if (end) {
      break;
    }
  }

  // Restart receives if necessary.
  PollReceiveQueue();

  // Save length.
  *len = offset;

  return true;
}
