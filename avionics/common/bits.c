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

#include "avionics/common/bits.h"

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "avionics/common/endian.h"
#include "common/c_math/util.h"

void SetBitByValue16(uint16_t mask, bool value, uint16_t *field) {
  if (value) {
    *field |= mask;
  } else {
    *field &= (uint16_t)~mask;
  }
}

// Write bits to an most significant bit first stream.
void WriteBitsUint32(uint32_t in, int32_t count, int32_t *offset,
                     uint8_t *out) {
  assert(0 <= count);
  assert(offset != NULL && *offset >= 0);
  assert(out != NULL);

  // This function writes the least significant bits first.
  //
  // Example:
  //
  // count = 19, offset = 2
  // xx11 1111 | 1111 1111 | 1111 1xxx
  // (part 2)  | (part 1)  | (part 0)
  //
  // remaining = 21
  // remaining / 8 = 2
  // remaining % 8 = 5
  //
  out += *offset / 8;
  int32_t delta = *offset % 8;

  // Write part 0.
  int32_t remaining = count + delta;
  int32_t shift = remaining % 8;
  out += remaining / 8;
  if (count > 0) {
    uint32_t mask = MaxUnsignedValue(count) << (8 - shift);
    *out &= (uint8_t)~mask;
    *out |= (uint8_t)((in << (8 - shift)) & mask);
  }
  in >>= shift;

  // Write part 1.
  remaining = count - shift;
  while (remaining >= 8) {
    --out;
    *out = (uint8_t)in;
    in >>= 8;
    remaining -= 8;
  }

  // Write part 2.
  if (remaining > 0) {
    uint32_t mask = 0xFFU >> delta;
    --out;
    *out &= (uint8_t)~mask;
    *out |= (uint8_t)(in & mask);
  }

  *offset += count;
}

void WriteBitsInt32(int32_t in, int32_t count, int32_t *offset, uint8_t *out) {
  assert(0 <= count && count <= 32);
  WriteBitsUint32((uint32_t)in, count, offset, out);
}

void WriteBitsSaturateUint32(uint32_t in, int32_t count, int32_t *offset,
                             uint8_t *out) {
  WriteBitsUint32(SaturateUnsigned(in, count), count, offset, out);
}

void WriteBitsSaturateInt32(int32_t in, int32_t count, int32_t *offset,
                            uint8_t *out) {
  WriteBitsInt32(SaturateSigned(in, count), count, offset, out);
}

void WriteBitsRangeFloat(float in, float min, float max, int32_t count,
                         int32_t *offset, uint8_t *out) {
  assert(min < max);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  float f = (in - min) / (max - min);
  uint32_t value;
  if (f <= 0.0f) {
    value = 0U;
  } else if (f >= 1.0f) {
    value = 0xFFFFFFFF;
  } else {
    value = (uint32_t)roundf(f * (float)(count == 0 ? 1 : (1LL << count) - 1));
  }
  WriteBitsUint32(value, count, offset, out);
}

void WriteBitsScaledFloat(float in, float scale, int32_t count, int32_t *offset,
                          uint8_t *out) {
  assert(scale != 0.0f);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  int32_t min = MinSignedValue(count);
  int32_t max = MaxSignedValue(count);
  float f = roundf(in / scale);
  int32_t value;
  if (f <= (float)min) {
    value = min;
  } else if (f >= (float)max) {
    value = max;
  } else {
    value = (int32_t)f;
  }
  WriteBitsUint32((uint32_t)value, count, offset, out);
}

void WriteBitsScaledDouble(double in, double scale, int32_t count,
                           int32_t *offset, uint8_t *out) {
  assert(scale != 0.0);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  int32_t min = MinSignedValue(count);
  int32_t max = MaxSignedValue(count);
  double f = round(in / scale);
  int32_t value;
  if (f <= (double)min) {
    value = min;
  } else if (f >= (double)max) {
    value = max;
  } else {
    value = (int32_t)f;
  }
  WriteBitsUint32((uint32_t)value, count, offset, out);
}

void WriteBitsScaledOffsetFloat(float in, float scale, float zero,
                                int32_t count, int32_t *offset, uint8_t *out) {
  WriteBitsUnsignedFloat(in - zero, scale, count, offset, out);
}

void WriteBitsUnsignedFloat(float in, float scale, int32_t count,
                            int32_t *offset, uint8_t *out) {
  assert(scale > 0.0f);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  uint32_t max = MaxUnsignedValue(count);
  float f = roundf(in / scale);
  uint32_t value;
  if (f <= 0.0f) {
    value = 0U;
  } else if (f >= (float)max) {
    value = max;
  } else {
    value = (uint32_t)f;
  }
  WriteBitsUint32(value, count, offset, out);
}

// Read bits from an MSB bit stream.
uint32_t ReadBitsUint32(const uint8_t *in, int32_t count, int32_t *offset) {
  assert(0 <= count);
  assert(offset != NULL && *offset >= 0);

  in += *offset / 8;
  int32_t delta = *offset % 8;
  uint32_t out = *in & (uint32_t)(0xFF >> delta);

  int32_t remaining = count + delta - 8;
  while (remaining >= 8) {
    ++in;
    out <<= 8;
    out |= *in;
    remaining -= 8;
  }
  if (remaining > 0) {
    ++in;
    out <<= remaining;
    out |= (uint32_t)*in >> (8 - remaining);
  } else {
    out >>= -remaining;
  }

  *offset += count;
  return out;
}

int32_t ReadBitsInt32(const uint8_t *in, int32_t count, int32_t *offset) {
  assert(0 <= count && count <= 32);
  if (count > 0) {
    return SignExtend(ReadBitsUint32(in, count, offset), count);
  } else {
    return 0;
  }
}

float ReadBitsRangeFloat(const uint8_t *in, float min, float max, int32_t count,
                         int32_t *offset) {
  assert(min < max);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  uint32_t u32 = ReadBitsUint32(in, count, offset);
  float f = (float)u32 / (float)(count == 0 ? 1 : (1LL << count) - 1);
  return f * (max - min) + min;
}

float ReadBitsScaledFloat(const uint8_t *in, float scale, int32_t count,
                          int32_t *offset) {
  assert(scale != 0.0f);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  uint32_t u32 = ReadBitsUint32(in, count, offset);
  return (float)SignExtend(u32, count) * scale;
}

double ReadBitsScaledDouble(const uint8_t *in, double scale, int32_t count,
                            int32_t *offset) {
  assert(scale != 0.0);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  uint32_t u32 = ReadBitsUint32(in, count, offset);
  return (double)SignExtend(u32, count) * scale;
}

float ReadBitsScaledOffsetFloat(const uint8_t *in, float scale, float zero,
                                int32_t count, int32_t *offset) {
  return ReadBitsUnsignedFloat(in, scale, count, offset) + zero;
}

float ReadBitsUnsignedFloat(const uint8_t *in, float scale, int32_t count,
                            int32_t *offset) {
  assert(scale > 0.0f);
  assert(0 <= count && count <= 32);
  assert(offset != NULL && *offset >= 0);

  uint32_t u32 = ReadBitsUint32(in, count, offset);
  return (float)u32 * scale;
}

#define BIT_OP_FUNC(FUNC_SUFFIX, READ_SUFFIX, WRITE_SUFFIX, TYPE)             \
  void BitOp##FUNC_SUFFIX(BitOp op, int32_t count, int32_t *offset, TYPE *in, \
                          uint8_t *out) {                                     \
    if (op == kBitOpWrite) {                                                  \
      WriteBits##WRITE_SUFFIX(*in, count, offset, out);                       \
    } else {                                                                  \
      *in = (TYPE)ReadBits##READ_SUFFIX(out, count, offset);                  \
    }                                                                         \
  }

BIT_OP_FUNC(Uint32, Uint32, Uint32, uint32_t)
BIT_OP_FUNC(Uint16, Uint32, Uint32, uint16_t)
BIT_OP_FUNC(Uint8, Uint32, Uint32, uint8_t)
BIT_OP_FUNC(SaturateUint32, Uint32, SaturateUint32, uint32_t)
BIT_OP_FUNC(SaturateUint16, Uint32, SaturateUint32, uint16_t)
BIT_OP_FUNC(SaturateUint8, Uint32, SaturateUint32, uint8_t)
BIT_OP_FUNC(Int32, Int32, Int32, int32_t)
BIT_OP_FUNC(Int16, Int32, Int32, int16_t)
BIT_OP_FUNC(Int8, Int32, Int32, int8_t)
BIT_OP_FUNC(SaturateInt32, Int32, SaturateInt32, int32_t)
BIT_OP_FUNC(SaturateInt16, Int32, SaturateInt32, int16_t)
BIT_OP_FUNC(SaturateInt8, Int32, SaturateInt32, int8_t)

#undef BIT_OP_FUNC

void BitOpScaledOffsetInt32(BitOp op, int32_t scale, int32_t zero,
                            int32_t count, int32_t *offset, int32_t *in,
                            uint8_t *out) {
  assert(scale != 0);
  assert(0 <= count && count < 32);
  assert(offset != NULL && *offset >= 0);

  if (op == kBitOpWrite) {
    int32_t max = (int32_t)MaxUnsignedValue(count);
    int32_t value = SaturateInt32((*in - zero) / scale, 0, max);
    WriteBitsUint32((uint32_t)value, count, offset, out);
  } else {
    *in = scale * (int32_t)ReadBitsUint32(out, count, offset) + zero;
  }
}

void BitOpScaledOffsetInt16(BitOp op, int32_t scale, int32_t zero,
                            int32_t count, int32_t *offset, int16_t *in,
                            uint8_t *out) {
  int32_t in32 = *in;
  BitOpScaledOffsetInt32(op, scale, zero, count, offset, &in32, out);
  *in = (int16_t)in32;
}

void BitOpScaledOffsetInt8(BitOp op, int32_t scale, int32_t zero, int32_t count,
                           int32_t *offset, int8_t *in, uint8_t *out) {
  int32_t in32 = *in;
  BitOpScaledOffsetInt32(op, scale, zero, count, offset, &in32, out);
  *in = (int8_t)in32;
}

void BitOpRangeFloat(BitOp op, float min, float max, int32_t count,
                     int32_t *offset, float *in, uint8_t *out) {
  if (op == kBitOpWrite) {
    WriteBitsRangeFloat(*in, min, max, count, offset, out);
  } else {
    *in = ReadBitsRangeFloat(out, min, max, count, offset);
  }
}

void BitOpScaledFloat(BitOp op, float scale, int32_t count, int32_t *offset,
                      float *in, uint8_t *out) {
  if (op == kBitOpWrite) {
    WriteBitsScaledFloat(*in, scale, count, offset, out);
  } else {
    *in = ReadBitsScaledFloat(out, scale, count, offset);
  }
}

void BitOpScaledDouble(BitOp op, double scale, int32_t count, int32_t *offset,
                       double *in, uint8_t *out) {
  if (op == kBitOpWrite) {
    WriteBitsScaledDouble(*in, scale, count, offset, out);
  } else {
    *in = ReadBitsScaledDouble(out, scale, count, offset);
  }
}

void BitOpScaledOffsetFloat(BitOp op, float scale, float zero, int32_t count,
                            int32_t *offset, float *in, uint8_t *out) {
  if (op == kBitOpWrite) {
    WriteBitsScaledOffsetFloat(*in, scale, zero, count, offset, out);
  } else {
    *in = ReadBitsScaledOffsetFloat(out, scale, zero, count, offset);
  }
}

void BitOpUnsignedFloat(BitOp op, float scale, int32_t count, int32_t *offset,
                        float *in, uint8_t *out) {
  if (op == kBitOpWrite) {
    WriteBitsUnsignedFloat(*in, scale, count, offset, out);
  } else {
    *in = ReadBitsUnsignedFloat(out, scale, count, offset);
  }
}
