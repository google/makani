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

#ifndef AVIONICS_COMMON_ENDIAN_H_
#define AVIONICS_COMMON_ENDIAN_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t ReadUint8Le(const void *in, uint8_t *value);
int32_t ReadUint16Le(const void *in, uint16_t *value);
int32_t ReadUint24Le(const void *in, uint32_t *value);
int32_t ReadUint32Le(const void *in, uint32_t *value);
int32_t ReadUint64Le(const void *in, uint64_t *value);
int32_t ReadInt8Le(const void *in, int8_t *value);
int32_t ReadInt16Le(const void *in, int16_t *value);
int32_t ReadInt32Le(const void *in, int32_t *value);
int32_t ReadInt64Le(const void *in, int64_t *value);
int32_t ReadFloatLe(const void *in, float *value);
int32_t ReadDoubleLe(const void *in, double *value);

int32_t ReadUint8Be(const void *in, uint8_t *value);
int32_t ReadUint16Be(const void *in, uint16_t *value);
int32_t ReadUint24Be(const void *in, uint32_t *value);
int32_t ReadUint32Be(const void *in, uint32_t *value);
int32_t ReadUint48Be(const void *in, uint64_t *value);
int32_t ReadUint64Be(const void *in, uint64_t *value);
int32_t ReadInt8Be(const void *in, int8_t *value);
int32_t ReadInt16Be(const void *in, int16_t *value);
int32_t ReadInt32Be(const void *in, int32_t *value);
int32_t ReadInt64Be(const void *in, int64_t *value);
int32_t ReadFloatBe(const void *in, float *value);
int32_t ReadDoubleBe(const void *in, double *value);

int32_t WriteUint8Le(uint8_t value, void *out);
int32_t WriteUint16Le(uint16_t value, void *out);
int32_t WriteUint24Le(uint32_t value, void *out);
int32_t WriteUint32Le(uint32_t value, void *out);
int32_t WriteUint64Le(uint64_t value, void *out);
int32_t WriteInt8Le(int8_t value, void *out);
int32_t WriteInt16Le(int16_t value, void *out);
int32_t WriteInt24Le(int32_t value, void *out);
int32_t WriteInt32Le(int32_t value, void *out);
int32_t WriteInt64Le(int64_t value, void *out);
int32_t WriteFloatLe(float value, void *out);
int32_t WriteDoubleLe(double value, void *out);

int32_t WriteUint8Be(uint8_t value, void *out);
int32_t WriteUint16Be(uint16_t value, void *out);
int32_t WriteUint24Be(uint32_t value, void *out);
int32_t WriteUint32Be(uint32_t value, void *out);
int32_t WriteUint48Be(uint64_t value, void *out);
int32_t WriteUint64Be(uint64_t value, void *out);
int32_t WriteInt8Be(int8_t value, void *out);
int32_t WriteInt16Be(int16_t value, void *out);
int32_t WriteInt24Be(int32_t value, void *out);
int32_t WriteInt32Be(int32_t value, void *out);
int32_t WriteInt64Be(int64_t value, void *out);
int32_t WriteFloatBe(float value, void *out);
int32_t WriteDoubleBe(double value, void *out);

int32_t SignExtend(uint32_t u32, int32_t bits);

// These functions only support exact representations.
float UnsignedToFloat(uint32_t code, int32_t bits, int32_t lsb_pow2);
float SignedToFloat(uint32_t code, int32_t bits, int32_t lsb_pow2);
double UnsignedToDouble(uint32_t code, int32_t bits, int32_t lsb_pow2);
double SignedToDouble(uint32_t code, int32_t bits, int32_t lsb_pow2);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // AVIONICS_COMMON_ENDIAN_H_
