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

#ifndef COMMON_RING_H_
#define COMMON_RING_H_

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#include "common/barrier.h"

typedef struct {
  // Ring is empty when the tail iterator equals the new_head iterator.
  volatile uint32_t new_head;  // Iterator defined on [0, UINT32_MAX].
  volatile uint32_t tail;      // Iterator defined on [0, UINT32_MAX].
  uint32_t elements;
} RingBuffer;

static inline void RingInit(uint32_t elements, RingBuffer *ring) {
  // The number of elements must be greater than zero for modulus operations.
  assert(0U < elements && elements < INT32_MAX);
  // The number of elements must divide evenly into 2^32 to avoid a
  // discontinuous index jump on rollover.
  assert(UINT32_MAX % elements == elements - 1U);

  ring->new_head = 0U;
  ring->tail = 0U;
  ring->elements = elements;
}

static inline int32_t RingGetUsed(const RingBuffer *ring) {
  return (int32_t)(ring->new_head - ring->tail);
}

static inline int32_t RingGetAvail(const RingBuffer *ring) {
  return (int32_t)ring->elements - RingGetUsed(ring);
}

static inline bool RingIsEmpty(const RingBuffer *ring) {
  return ring->new_head == ring->tail;
}

static inline bool RingIsFull(const RingBuffer *ring) {
  return RingGetUsed(ring) >= (int32_t)ring->elements;
}

// Expected use case:
//
// if (RingGetNewHeadIndex(ring, &index)) {
//   array[index] = value;
//   RingPushNewHead(ring);
// }
static inline void RingPushNewHead(RingBuffer *ring) {
  assert(!RingIsFull(ring));
  // To avoid concurrency issues, this memory barrier instructs the compiler
  // to generate instructions for array[index] = value, as above, prior to
  // incrementing the new_head iterator.
  MemoryBarrier();
  ++ring->new_head;
}

// Expected use case:
//
// if (RingGetTailIndex(ring, &index)) {
//   value = array[index];
//   RingPopTail(ring);
// }
static inline void RingPopTail(RingBuffer *ring) {
  assert(!RingIsEmpty(ring));
  // To avoid concurrency issues, this memory barrier instructs the compiler
  // to generate instructions for value = array[index], as above, prior to
  // incrementing the tail iterator.
  MemoryBarrier();
  ++ring->tail;
}

static inline void RingAdvanceTail(int32_t count, RingBuffer *ring) {
  assert(0 <= count && count <= RingGetUsed(ring));
  // To avoid concurrency issues, this memory barrier instructs the compiler
  // to generate instructions for value = array[index], as above, prior to
  // incrementing the tail iterator.
  MemoryBarrier();
  ring->tail += (uint32_t)count;
}

// Iterator comparison functions.

static inline bool RingCompareGe(uint32_t a, uint32_t b) {
  return (int32_t)(a - b) >= 0;
}

static inline bool RingCompareGt(uint32_t a, uint32_t b) {
  return (int32_t)(a - b) > 0;
}

static inline bool RingCompareLe(uint32_t a, uint32_t b) {
  return (int32_t)(a - b) <= 0;
}

static inline bool RingCompareLt(uint32_t a, uint32_t b) {
  return (int32_t)(a - b) < 0;
}

// Iterator access functions. These functions return an iterator value that
// spans the entire 32-bit integer space. This value is not suitable for
// addressing an array offset.

static inline uint32_t RingGetHead(const RingBuffer *ring) {
  assert(!RingIsEmpty(ring));
  return ring->new_head - 1U;
}

static inline uint32_t RingGetNewHead(const RingBuffer *ring) {
  assert(!RingIsFull(ring));
  return ring->new_head;
}

static inline uint32_t RingGetTail(const RingBuffer *ring) {
  return ring->tail;
}

// Index access functions. These functions output an index value suitable
// for addressing an array offset. They return true when valid.
//
// Expected use cases:
//
// uint32_t iter = RingGetTail(ring);
// while (RingGetIndex(ring, iter, &index)) {
//   (operate on array[index])
//   ++iter;
// }
//
// for (uint32_t iter = RingGetTail(ring); RingGetIndex(ring, iter, &index);
//      ++iter) {
//   (operate on array[index])
// }
static inline bool RingGetIndex(const RingBuffer *ring, uint32_t iter,
                                int32_t *index) {
  *index = (int32_t)(iter % ring->elements);
  return RingCompareLe(ring->tail, iter) && RingCompareLt(iter, ring->new_head);
}

static inline bool RingGetHeadIndex(const RingBuffer *ring, int32_t *index) {
  return RingGetIndex(ring, ring->new_head - 1U, index);
}

static inline bool RingGetNewHeadIndex(const RingBuffer *ring, int32_t *index) {
  *index = (int32_t)(ring->new_head % ring->elements);
  return !RingIsFull(ring);
}

static inline bool RingGetTailIndex(const RingBuffer *ring, int32_t *index) {
  return RingGetIndex(ring, ring->tail, index);
}

#endif  // COMMON_RING_H_
