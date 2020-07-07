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

#include "avionics/common/endian.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

int32_t ReadUint8Le(const void *in, uint8_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  *value = buf[0];
  return 1;
}

int32_t ReadUint16Le(const void *in, uint16_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint16_t out = buf[1];
  out = (uint16_t)(out << 8);
  out = (uint16_t)(out | buf[0]);
  *value = out;
  return 2;
}

int32_t ReadUint24Le(const void *in, uint32_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint32_t out = buf[2];
  out <<= 8;
  out |= buf[1];
  out <<= 8;
  out |= buf[0];
  *value = out;
  return 3;
}

int32_t ReadUint32Le(const void *in, uint32_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint32_t out = buf[3];
  out <<= 8;
  out |= buf[2];
  out <<= 8;
  out |= buf[1];
  out <<= 8;
  out |= buf[0];
  *value = out;
  return 4;
}

int32_t ReadUint64Le(const void *in, uint64_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint64_t out = buf[7];
  out <<= 8;
  out |= buf[6];
  out <<= 8;
  out |= buf[5];
  out <<= 8;
  out |= buf[4];
  out <<= 8;
  out |= buf[3];
  out <<= 8;
  out |= buf[2];
  out <<= 8;
  out |= buf[1];
  out <<= 8;
  out |= buf[0];
  *value = out;
  return 8;
}

int32_t ReadInt8Le(const void *in, int8_t *value) {
  return ReadUint8Le(in, (uint8_t *)value);
}

int32_t ReadInt16Le(const void *in, int16_t *value) {
  return ReadUint16Le(in, (uint16_t *)value);
}

int32_t ReadInt32Le(const void *in, int32_t *value) {
  return ReadUint32Le(in, (uint32_t *)value);
}

int32_t ReadInt64Le(const void *in, int64_t *value) {
  return ReadUint64Le(in, (uint64_t *)value);
}

int32_t ReadFloatLe(const void *in, float *value) {
  uint32_t u32;
  int32_t length = ReadUint32Le(in, &u32);
  memcpy(value, &u32, sizeof(*value));
  return length;
}

int32_t ReadDoubleLe(const void *in, double *value) {
  uint64_t u64;
  int32_t length = ReadUint64Le(in, &u64);
  memcpy(value, &u64, sizeof(*value));
  return length;
}

int32_t ReadUint8Be(const void *in, uint8_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  *value = buf[0];
  return 1;
}

int32_t ReadUint16Be(const void *in, uint16_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint16_t out = buf[0];
  out = (uint16_t)(out << 8);
  out = (uint16_t)(out | buf[1]);
  *value = out;
  return 2;
}

int32_t ReadUint24Be(const void *in, uint32_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint32_t out = buf[0];
  out <<= 8;
  out |= buf[1];
  out <<= 8;
  out |= buf[2];
  *value = out;
  return 3;
}

int32_t ReadUint32Be(const void *in, uint32_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint32_t out = buf[0];
  out <<= 8;
  out |= buf[1];
  out <<= 8;
  out |= buf[2];
  out <<= 8;
  out |= buf[3];
  *value = out;
  return 4;
}

int32_t ReadUint48Be(const void *in, uint64_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint64_t out = buf[0];
  out <<= 8;
  out |= buf[1];
  out <<= 8;
  out |= buf[2];
  out <<= 8;
  out |= buf[3];
  out <<= 8;
  out |= buf[4];
  out <<= 8;
  out |= buf[5];
  *value = out;
  return 6;
}

int32_t ReadUint64Be(const void *in, uint64_t *value) {
  assert(in != NULL);
  assert(value != NULL);

  const uint8_t *buf = (const uint8_t *)in;
  uint64_t out = buf[0];
  out <<= 8;
  out |= buf[1];
  out <<= 8;
  out |= buf[2];
  out <<= 8;
  out |= buf[3];
  out <<= 8;
  out |= buf[4];
  out <<= 8;
  out |= buf[5];
  out <<= 8;
  out |= buf[6];
  out <<= 8;
  out |= buf[7];
  *value = out;
  return 8;
}

int32_t ReadInt8Be(const void *in, int8_t *value) {
  return ReadUint8Be(in, (uint8_t *)value);
}

int32_t ReadInt16Be(const void *in, int16_t *value) {
  return ReadUint16Be(in, (uint16_t *)value);
}

int32_t ReadInt32Be(const void *in, int32_t *value) {
  return ReadUint32Be(in, (uint32_t *)value);
}

int32_t ReadInt64Be(const void *in, int64_t *value) {
  return ReadUint64Be(in, (uint64_t *)value);
}

int32_t ReadFloatBe(const void *in, float *value) {
  uint32_t u32;
  int32_t length = ReadUint32Be(in, &u32);
  memcpy(value, &u32, sizeof(*value));
  return length;
}

int32_t ReadDoubleBe(const void *in, double *value) {
  uint64_t u64;
  int32_t length = ReadUint64Be(in, &u64);
  memcpy(value, &u64, sizeof(*value));
  return length;
}

int32_t WriteUint8Le(uint8_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = value;
  return 1;
}

int32_t WriteUint16Le(uint16_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 0);
  buf[1] = (uint8_t)(value >> 8);
  return 2;
}

int32_t WriteUint24Le(uint32_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 0);
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)(value >> 16);
  return 3;
}

int32_t WriteUint32Le(uint32_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 0);
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)(value >> 16);
  buf[3] = (uint8_t)(value >> 24);
  return 4;
}

int32_t WriteUint64Le(uint64_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 0);
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)(value >> 16);
  buf[3] = (uint8_t)(value >> 24);
  buf[4] = (uint8_t)(value >> 32);
  buf[5] = (uint8_t)(value >> 40);
  buf[6] = (uint8_t)(value >> 48);
  buf[7] = (uint8_t)(value >> 56);
  return 8;
}

int32_t WriteInt8Le(int8_t value, void *out) {
  return WriteUint8Le((uint8_t)value, out);
}

int32_t WriteInt16Le(int16_t value, void *out) {
  return WriteUint16Le((uint16_t)value, out);
}

int32_t WriteInt24Le(int32_t value, void *out) {
  return WriteUint24Le((uint32_t)value, out);
}

int32_t WriteInt32Le(int32_t value, void *out) {
  return WriteUint32Le((uint32_t)value, out);
}

int32_t WriteInt64Le(int64_t value, void *out) {
  return WriteUint64Le((uint64_t)value, out);
}

int32_t WriteFloatLe(float value, void *out) {
  assert(out != NULL);

  uint32_t u32;
  memcpy(&u32, &value, sizeof(u32));
  return WriteUint32Le(u32, out);
}

int32_t WriteDoubleLe(double value, void *out) {
  assert(out != NULL);

  uint64_t u64;
  memcpy(&u64, &value, sizeof(u64));
  return WriteUint64Le(u64, out);
}

int32_t WriteUint8Be(uint8_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = value;
  return 1;
}

int32_t WriteUint16Be(uint16_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 8);
  buf[1] = (uint8_t)(value >> 0);
  return 2;
}

int32_t WriteUint24Be(uint32_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 16);
  buf[1] = (uint8_t)(value >> 8);
  buf[2] = (uint8_t)(value >> 0);
  return 3;
}

int32_t WriteUint32Be(uint32_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 24);
  buf[1] = (uint8_t)(value >> 16);
  buf[2] = (uint8_t)(value >> 8);
  buf[3] = (uint8_t)(value >> 0);
  return 4;
}

int32_t WriteUint48Be(uint64_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 40);
  buf[1] = (uint8_t)(value >> 32);
  buf[2] = (uint8_t)(value >> 24);
  buf[3] = (uint8_t)(value >> 16);
  buf[4] = (uint8_t)(value >> 8);
  buf[5] = (uint8_t)(value >> 0);
  return 6;
}

int32_t WriteUint64Be(uint64_t value, void *out) {
  assert(out != NULL);

  uint8_t *buf = (uint8_t *)out;
  buf[0] = (uint8_t)(value >> 56);
  buf[1] = (uint8_t)(value >> 48);
  buf[2] = (uint8_t)(value >> 40);
  buf[3] = (uint8_t)(value >> 32);
  buf[4] = (uint8_t)(value >> 24);
  buf[5] = (uint8_t)(value >> 16);
  buf[6] = (uint8_t)(value >> 8);
  buf[7] = (uint8_t)(value >> 0);
  return 8;
}

int32_t WriteInt8Be(int8_t value, void *out) {
  return WriteUint8Be((uint8_t)value, out);
}

int32_t WriteInt16Be(int16_t value, void *out) {
  return WriteUint16Be((uint16_t)value, out);
}

int32_t WriteInt24Be(int32_t value, void *out) {
  return WriteUint24Be((uint32_t)value, out);
}

int32_t WriteInt32Be(int32_t value, void *out) {
  return WriteUint32Be((uint32_t)value, out);
}

int32_t WriteInt64Be(int64_t value, void *out) {
  return WriteUint64Be((uint64_t)value, out);
}

int32_t WriteFloatBe(float value, void *out) {
  assert(out != NULL);

  uint32_t u32;
  memcpy(&u32, &value, sizeof(u32));
  return WriteUint32Be(u32, out);
}

int32_t WriteDoubleBe(double value, void *out) {
  assert(out != NULL);

  uint64_t u64;
  memcpy(&u64, &value, sizeof(u64));
  return WriteUint64Be(u64, out);
}

int32_t SignExtend(uint32_t u32, int32_t bits) {
  assert(0 < bits && bits <= 32);
  if (u32 & (1UL << (bits - 1))) {
    // Since a left shift by 32 bits is undefined, shift by (bits - 1).
    return (int32_t)(u32 | (0xFFFFFFFFU << (bits - 1)));
  } else {
    return (int32_t)u32;
  }
}

// GPS ephemerides are extremely sensitive. The following code scales the LSB
// exactly according to IEEE754 binary32 and binary64 standards for single and
// double precision binary representations.

// Scale binary data to floating point value with a power of 2 LSB.
//
// Args:
//   code: Unsigned binary data.
//   bits: Bit length of binary data (0--24).
//   lsb_pow2: Least significant bit power of two exponent.
//
// Return:
//   Exact floating point representation.
float UnsignedToFloat(uint32_t code, int32_t bits, int32_t lsb_pow2) {
  union {
    float f;
    uint32_t u32;
  } u = {0};

  // Limit range for exact representation.
  assert(0 <= bits && bits <= 24);
  assert(-127 <= lsb_pow2 && lsb_pow2 + bits <= 127);

  // Exponent (bits 30--23).
  int32_t offset = 0;
  for (int32_t i = 0; i < bits; ++i) {
    if (code & (1U << i)) {
      offset = i;
    }
  }
  u.u32 |= ((uint32_t)(127 + lsb_pow2 + offset) << 23) & 0x7F800000;

  // Significand (bits 22--0, left justified).
  for (int32_t i = 0; i < offset; ++i) {
    if (code & (1U << i)) {
      u.u32 |= 1U << (23 - offset + i);
    }
  }
  return u.f;
}

// Scale binary data to floating point value with a power of 2 LSB.
//
// Args:
//   code: 2's complement signed binary data.
//   bits: Bit length of binary data (0--24).
//   lsb_pow2: Least significant bit power of two exponent.
//
// Return:
//   Exact floating point representation.
float SignedToFloat(uint32_t code, int32_t bits, int32_t lsb_pow2) {
  int32_t i32 = SignExtend(code, bits);
  union {
    float f;
    uint32_t u32;
  } u;

  // Sign (bit 31).
  if (i32 < 0) {
    u.f = UnsignedToFloat((uint32_t)(-i32), bits, lsb_pow2);
    u.u32 |= 0x80000000;
  } else {
    u.f = UnsignedToFloat((uint32_t)i32, bits, lsb_pow2);
  }
  return u.f;
}

// Scale binary data to floating point value with a power of 2 LSB.
//
// Args:
//   code: unsigned binary data.
//   bits: Bit length of binary data (0--32).
//   lsb_pow2: Least significant bit power of two exponent.
//
// Return:
//   Exact floating point representation.
double UnsignedToDouble(uint32_t code, int32_t bits, int32_t lsb_pow2) {
  union {
    double d;
    uint64_t u64;
  } u = {0};

  // Limit range for exact representation.
  assert(0 <= bits && bits <= 32);
  assert(-1023 <= lsb_pow2 && lsb_pow2 + bits <= 1023);

  // Exponent (bits 62--52).
  int32_t offset = 0;
  for (int32_t i = 0; i < bits; ++i) {
    if (code & (1U << i)) {
      offset = i;
    }
  }
  u.u64 |= (uint64_t)(1023 + lsb_pow2 + offset) << 52;

  // Significand (bits 51--0, left justified).
  for (int32_t i = 0; i < offset; ++i) {
    if (code & (1U << i)) {
      u.u64 |= 1ULL << (52 - offset + i);
    }
  }
  return u.d;
}

// Scale binary data to floating point value with a power of 2 LSB.
//
// Args:
//   code: 2's complement signed binary data.
//   bits: Bit length of binary data (0--32).
//   lsb_pow2: Least significant bit power of two exponent.
//
// Return:
//   Exact floating point representation.
double SignedToDouble(uint32_t code, int32_t bits, int32_t lsb_pow2) {
  int32_t i32 = SignExtend(code, bits);
  union {
    double d;
    uint64_t u64;
  } u;

  // Sign (bit 63).
  if (i32 < 0) {
    u.d = UnsignedToDouble((uint32_t)(-i32), bits, lsb_pow2);
    u.u64 |= 0x8000000000000000;
  } else {
    u.d = UnsignedToDouble((uint32_t)i32, bits, lsb_pow2);
  }
  return u.d;
}
