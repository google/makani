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

#ifndef AVIONICS_COMMON_BITS_H_
#define AVIONICS_COMMON_BITS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  kBitOpRead,
  kBitOpWrite
} BitOp;

// Set or clear a mask of bits in a 16-bit value.
void SetBitByValue16(uint16_t mask, bool value, uint16_t *field);

// Write bits to an MSB bit stream.
// Args:
//   in: The data to write.
//   count: The number of bits to write.
//   offset: The current bit offset in the bit stream, incremented upon return.
//   out: The bit stream starting address.
void WriteBitsUint32(uint32_t in, int32_t count, int32_t *offset, uint8_t *out);
void WriteBitsInt32(int32_t in, int32_t count, int32_t *offset, uint8_t *out);

// Write bits with saturation at maximum. See WriteBitsUint32.
void WriteBitsSaturateUint32(uint32_t in, int32_t count, int32_t *offset,
                             uint8_t *out);
void WriteBitsSaturateInt32(int32_t in, int32_t count, int32_t *offset,
                            uint8_t *out);

// Write a float using min/max values to an MSB bit stream. See WriteBitsUint32.
void WriteBitsRangeFloat(float in, float min, float max, int32_t count,
                         int32_t *offset, uint8_t *out);

// Write a float using a scale factor to an MSB bit stream. See WriteBitsUint32.
void WriteBitsScaledFloat(float in, float scale, int32_t count, int32_t *offset,
                          uint8_t *out);
void WriteBitsScaledDouble(double in, double scale, int32_t count,
                           int32_t *offset, uint8_t *out);
void WriteBitsScaledOffsetFloat(float in, float scale, float zero,
                                int32_t count, int32_t *offset, uint8_t *out);
void WriteBitsUnsignedFloat(float in, float scale, int32_t count,
                            int32_t *offset, uint8_t *out);

// Read bits from an MSB bit stream.
// Args:
//   in: The bit stream starting address.
//   count: The number of bits to read.
//   offset: The current bit offset in the bit stream, incremented upon return.
//
// Returns:
//   The value.
uint32_t ReadBitsUint32(const uint8_t *in, int32_t count, int32_t *offset);
int32_t ReadBitsInt32(const uint8_t *in, int32_t count, int32_t *offset);

// Read a float using min/max values from an MSB bit stream. See ReadBitsUint32.
float ReadBitsRangeFloat(const uint8_t *in, float min, float max, int32_t count,
                         int32_t *offset);

// Read a float using a scale factor from an MSB bit stream. See ReadBitsUint32.
float ReadBitsScaledFloat(const uint8_t *in, float scale, int32_t count,
                          int32_t *offset);
double ReadBitsScaledDouble(const uint8_t *in, double scale, int32_t count,
                            int32_t *offset);
float ReadBitsScaledOffsetFloat(const uint8_t *in, float scale, float bias,
                                int32_t count, int32_t *offset);
float ReadBitsUnsignedFloat(const uint8_t *in, float scale, int32_t count,
                            int32_t *offset);

void BitOpUint32(BitOp op, int32_t count, int32_t *offset, uint32_t *in,
                 uint8_t *out);
void BitOpUint16(BitOp op, int32_t count, int32_t *offset, uint16_t *in,
                 uint8_t *out);
void BitOpUint8(BitOp op, int32_t count, int32_t *offset, uint8_t *in,
                uint8_t *out);
void BitOpSaturateUint32(BitOp op, int32_t count, int32_t *offset, uint32_t *in,
                         uint8_t *out);
void BitOpSaturateUint16(BitOp op, int32_t count, int32_t *offset, uint16_t *in,
                         uint8_t *out);
void BitOpSaturateUint8(BitOp op, int32_t count, int32_t *offset, uint8_t *in,
                        uint8_t *out);

void BitOpInt32(BitOp op, int32_t count, int32_t *offset, int32_t *in,
                uint8_t *out);
void BitOpInt16(BitOp op, int32_t count, int32_t *offset, int16_t *in,
                uint8_t *out);
void BitOpInt8(BitOp op, int32_t count, int32_t *offset, int8_t *in,
               uint8_t *out);
void BitOpSaturateInt32(BitOp op, int32_t count, int32_t *offset, int32_t *in,
                        uint8_t *out);
void BitOpSaturateInt16(BitOp op, int32_t count, int32_t *offset, int16_t *in,
                        uint8_t *out);
void BitOpSaturateInt8(BitOp op, int32_t count, int32_t *offset, int8_t *in,
                       uint8_t *out);
void BitOpScaledOffsetInt32(BitOp op, int32_t scale, int32_t zero,
                            int32_t count, int32_t *offset, int32_t *in,
                            uint8_t *out);
void BitOpScaledOffsetInt16(BitOp op, int32_t scale, int32_t zero,
                            int32_t count, int32_t *offset, int16_t *in,
                            uint8_t *out);
void BitOpScaledOffsetInt8(BitOp op, int32_t scale, int32_t zero, int32_t count,
                           int32_t *offset, int8_t *in, uint8_t *out);

void BitOpRangeFloat(BitOp op, float min, float max, int32_t count,
                     int32_t *offset, float *in, uint8_t *out);
void BitOpScaledFloat(BitOp op, float scale, int32_t count, int32_t *offset,
                      float *in, uint8_t *out);
void BitOpScaledDouble(BitOp op, double scale, int32_t count, int32_t *offset,
                       double *in, uint8_t *out);
void BitOpScaledOffsetFloat(BitOp op, float scale, float zero, int32_t count,
                            int32_t *offset, float *in, uint8_t *out);
void BitOpUnsignedFloat(BitOp op, float scale, int32_t count, int32_t *offset,
                        float *in, uint8_t *out);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_BITS_H_
