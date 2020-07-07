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

#ifndef LIB_UTIL_CIRCULAR_BUFFER_H_
#define LIB_UTIL_CIRCULAR_BUFFER_H_

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>

template <typename Datum>
class BufferReadInterface {
 public:
  enum Errors {
    NO_ERROR = 0,
    BUFFER_EMPTY
  };

  // Returns the number of data currently available to read.
  virtual uint32_t DataReady() const = 0;

  // Attempts to read the next datum from the buffer.
  //
  // Args:
  //   datum: A pointer to a datum to be read into.
  //
  // Returns:
  //   BUFFER_EMPTY if no data is available, otherwise NO_ERROR.
  virtual int32_t Read(Datum *data) = 0;

  virtual ~BufferReadInterface() {}
};

template <typename Datum>
class BufferWriteInterface {
 public:
  enum Errors {
    NO_ERROR = 0,
    BUFFER_FULL
  };
  virtual int32_t Write(const Datum &datum) = 0;

  virtual ~BufferWriteInterface() {}
};

// This class implements a circular buffer shared between two threads.  One
// thread writes data to the buffer one "datum" at a time.  Another thread
// consumes this data.
template <typename Datum>
class CircularBuffer : public BufferReadInterface<Datum>,
                       public BufferWriteInterface<Datum> {
 public:
  explicit CircularBuffer(int32_t size);
  virtual ~CircularBuffer();

  uint32_t DataReady() const {
    // We store these variables locally to avoid race conditions from multiple
    // access.
    uint32_t w = write_idx_;
    uint32_t r = read_idx_;
    // size_ + 1 is used as we keep one slot open.
    return (w < r ? size_ + 1 : 0) + w - r;
  }

  int32_t Write(const Datum &datum);
  int32_t Read(Datum *data);

 private:
  CircularBuffer(const CircularBuffer &src);
  CircularBuffer &operator=(const CircularBuffer &src);

  // These are convenience functions for accessing the data.
  bool IsFull() const { return DataReady() == size_; }
  bool IsEmpty() const { return DataReady() == 0; }
  int32_t NextIndex(int32_t block_num) const {
    return (block_num + 1) % (size_ + 1);
  }

  const uint32_t size_;  // Number of valid data stored.
  Datum *blocks_;  // An array with size_ + 1 slots (one is always kept open).
  uint32_t write_idx_;  // Indicates the block currently being written.
  uint32_t read_idx_;  // Indicates the block currently being written.
};

template <typename Datum>
CircularBuffer<Datum>::CircularBuffer(int32_t size)
    : size_(size), blocks_(NULL), write_idx_(0), read_idx_(0) {
  assert(size_ > 0);
  blocks_ = new Datum[size_ + 1];
}

template <typename Datum>
CircularBuffer<Datum>::~CircularBuffer() {
  if (blocks_ != NULL) delete[] blocks_;
}

template <typename Datum>
int32_t CircularBuffer<Datum>::Write(const Datum &datum) {
  // If we're full, wait for more data to be read.
  if (IsFull()) return BufferWriteInterface<Datum>::BUFFER_FULL;

  blocks_[write_idx_] = datum;
  write_idx_ = NextIndex(write_idx_);

  return BufferWriteInterface<Datum>::NO_ERROR;
}

template <typename Datum>
int32_t CircularBuffer<Datum>::Read(Datum *datum) {
  if (IsEmpty()) return BufferReadInterface<Datum>::BUFFER_EMPTY;

  *datum = blocks_[read_idx_];
  read_idx_ = NextIndex(read_idx_);

  return BufferReadInterface<Datum>::NO_ERROR;
}

#endif  // LIB_UTIL_CIRCULAR_BUFFER_H_
