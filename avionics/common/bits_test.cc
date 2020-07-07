// Copyright 2020 Makani Technologies LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <float.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "avionics/common/bits.h"
#include "avionics/common/endian.h"
#include "common/c_math/util.h"
#include "common/macros.h"

TEST(ReadBitsUint32, Normal) {
  const uint8_t in[] = {0xC1, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

  for (int32_t offset = 0; offset < 32; ++offset) {
    for (int32_t count = 0; count <= 32; ++count) {
      int32_t new_offset = offset;
      uint32_t actual = ReadBitsUint32(in, count, &new_offset);
      EXPECT_EQ(new_offset, offset + count);

      uint64_t in64 = 0xC123456789ABCDEFUL;
      uint64_t mask = 0xFFFFFFFFUL >> (32 - count);
      uint32_t expected = (uint32_t)((in64 >> (64 - offset - count)) & mask);
      EXPECT_EQ(expected, actual);
    }
  }
}

TEST(ReadBitsUint32, ExtendedRead) {
  const uint8_t in[8] = {0xC1, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};

  // Return value can only store the 32 least significant bits.
  int32_t new_offset = 0;
  EXPECT_EQ(0x89ABCDEF, ReadBitsUint32(in, 64, &new_offset));
}

TEST(ReadBitsInt32, Normal) {
  const uint8_t negative_1[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  const uint8_t negative_325021[4] = {0xFF, 0xFB, 0x0A, 0x63};
  int32_t new_offset = 0;

  // Zero length fields always read zero.
  EXPECT_EQ(0, ReadBitsInt32(negative_1, 0, &new_offset));

  // Make sure function extends the sign bit.
  for (int32_t count = 1; count <= 32; ++count) {
    new_offset = 0;
    EXPECT_EQ(-1, ReadBitsInt32(negative_1, count, &new_offset));
  }
  for (int32_t count = 20; count <= 32; ++count) {
    new_offset = 32 - count;
    EXPECT_EQ(-325021, ReadBitsInt32(negative_325021, count, &new_offset));
  }
}

TEST(WriteBitsUint32, Normal) {
  for (int32_t offset = 0; offset < 32; ++offset) {
    for (int32_t count = 0; count <= 32; ++count) {
      const uint32_t mask = MaxUnsignedValue(count);
      uint8_t out[12] = {0};

      // Test setting bits.
      uint32_t expected = mask;
      int32_t new_offset = offset;
      WriteBitsUint32(expected, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(expected, ReadBitsUint32(out, count, &new_offset));
      new_offset = 0;  // Left canary.
      EXPECT_EQ(0U, ReadBitsUint32(out, offset, &new_offset));
      new_offset = offset + count;  // Right canary.
      EXPECT_EQ(0U, ReadBitsUint32(out, 32, &new_offset));

      // Test zeroing bits.
      memset(out, 0xFF, sizeof(out));
      expected = 0x0;
      new_offset = offset;
      WriteBitsUint32(expected, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(expected, ReadBitsUint32(out, count, &new_offset));
      new_offset = 0;  // Left canary.
      EXPECT_EQ((1U << offset) - 1U, ReadBitsUint32(out, offset, &new_offset));
      new_offset = offset + count;  // Right canary.
      EXPECT_EQ(0xFFFFFFFF, ReadBitsUint32(out, 32, &new_offset));

      // Test arbitrary data.
      const uint32_t data[] = {0x00000000, 0x11111111, 0x22222222, 0x33333333,
                               0x44444444, 0x55555555, 0x66666666, 0x77777777,
                               0x88888888, 0x99999999, 0xAAAAAAAA, 0xBBBBBBBB,
                               0xCCCCCCCC, 0xDDDDDDDD, 0xEEEEEEEE, 0xFFFFFFFF,
                               0x12345678, 0x87654321};
      for (int32_t i = 0; i < ARRAYSIZE(data); ++i) {
        memset(out, 0xFF, sizeof(out));
        expected = data[i] & mask;
        new_offset = offset;
        WriteBitsUint32(expected, count, &new_offset, out);
        EXPECT_EQ(new_offset, offset + count);
        new_offset = offset;
        EXPECT_EQ(expected, ReadBitsUint32(out, count, &new_offset));
        new_offset = 0;  // Left canary.
        EXPECT_EQ((1U << offset) - 1U,
                  ReadBitsUint32(out, offset, &new_offset));
        new_offset = offset + count;  // Right canary.
        EXPECT_EQ(0xFFFFFFFF, ReadBitsUint32(out, 32, &new_offset));
      }
    }
  }
}

TEST(WriteBitsUint32, ExtendedWrite) {
  uint8_t out[8];
  memset(out, 0xFF, sizeof(out));

  uint32_t expected = 0xC1234567;
  int32_t new_offset = 0;
  WriteBitsUint32(expected, 64, &new_offset, out);
  EXPECT_EQ(64, new_offset);
  new_offset = 0;  // Most significant bits should be zero.
  EXPECT_EQ(0x0, ReadBitsUint32(out, 32, &new_offset));
  // Least significant bits should be set to 'expected'.
  EXPECT_EQ(32, new_offset);
  EXPECT_EQ(expected, ReadBitsUint32(out, 32, &new_offset));
}

TEST(WriteBitsSaturateUint32, Normal) {
  uint8_t out[4];
  int32_t new_offset;
  for (int32_t count = 1; count < 32; ++count) {
    uint32_t max = MaxUnsignedValue(count);

    // Write a value that should not saturate.
    new_offset = 0;
    WriteBitsSaturateUint32(max - 1U, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(max - 1U, ReadBitsUint32(out, count, &new_offset));

    // Write maximum value.
    new_offset = 0;
    WriteBitsSaturateUint32(max, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(max, ReadBitsUint32(out, count, &new_offset));

    // Write value greater than maximum; expect saturation.
    new_offset = 0;
    WriteBitsSaturateUint32(max + 1U, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(max, ReadBitsUint32(out, count, &new_offset));
  }

  // We can't saturate at 32-bit.
  new_offset = 0;
  WriteBitsSaturateUint32(0xFFFFFFFF, 32, &new_offset, out);
  new_offset = 0;
  EXPECT_EQ(0xFFFFFFFF, ReadBitsUint32(out, 32, &new_offset));
}

TEST(WriteBitsInt32, Normal) {
  for (int32_t count = 15; count <= 32; ++count) {
    uint8_t out[4];
    const int32_t expected = -13423;
    int32_t new_offset = 0;
    WriteBitsInt32(expected, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(expected, ReadBitsInt32(out, count, &new_offset));
  }
}

TEST(WriteBitsSaturateInt32, Normal) {
  uint8_t out[4];
  int32_t new_offset;
  for (int32_t count = 1; count < 32; ++count) {
    int32_t min = MinSignedValue(count);
    int32_t max = MaxSignedValue(count);

    // Write a value that should not saturate (positive).
    new_offset = 0;
    WriteBitsSaturateInt32(max - 1, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(max - 1, ReadBitsInt32(out, count, &new_offset));

    // Write a value that should not saturate (negative).
    new_offset = 0;
    WriteBitsSaturateInt32(min + 1, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(min + 1, ReadBitsInt32(out, count, &new_offset));

    // Write maximum value.
    new_offset = 0;
    WriteBitsSaturateInt32(max, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(max, ReadBitsInt32(out, count, &new_offset));

    // Write minimum value.
    new_offset = 0;
    WriteBitsSaturateInt32(min, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(min, ReadBitsInt32(out, count, &new_offset));

    // Write value greater than maximum; expect saturation.
    new_offset = 0;
    WriteBitsSaturateInt32(max + 1, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(max, ReadBitsInt32(out, count, &new_offset));

    // Write value less than minimum; expect saturation.
    new_offset = 0;
    WriteBitsSaturateInt32(min - 1, count, &new_offset, out);
    new_offset = 0;
    EXPECT_EQ(min, ReadBitsInt32(out, count, &new_offset));
  }

  // We can't saturate at 32-bit.
  new_offset = 0;
  WriteBitsSaturateInt32(0xFFFFFFFF, 32, &new_offset, out);
  new_offset = 0;
  EXPECT_EQ(0xFFFFFFFF, ReadBitsInt32(out, 32, &new_offset));
}

TEST(WriteReadBitsRangeFloat, Normal) {
  for (int32_t count = 0; count <= 32; ++count) {
    const float min = -100.0f;
    const float max = 200.0f;
    const float tol = fmaxf(0.01f, (max - min) / powf(2.0f, (float)count));
    const int32_t offset = 5;
    uint8_t out[8] = {0};

    // Test in range.
    float expected = 12.3f;
    int32_t new_offset = offset;
    WriteBitsRangeFloat(expected, min, max, count, &new_offset, out);
    EXPECT_EQ(new_offset, offset + count);
    new_offset = offset;
    EXPECT_NEAR(expected,
                ReadBitsRangeFloat(out, min, max, count, &new_offset), tol);

    // Test beyond lower bound.
    new_offset = offset;
    WriteBitsRangeFloat(2.0f * min, min, max, count, &new_offset, out);
    EXPECT_EQ(new_offset, offset + count);
    new_offset = offset;
    EXPECT_EQ(0U, ReadBitsUint32(out, count, &new_offset));
    new_offset = offset;
    EXPECT_NEAR(min, ReadBitsRangeFloat(out, min, max, count, &new_offset),
                FLT_EPSILON);

    // Test beyond upper bound.
    new_offset = offset;
    WriteBitsRangeFloat(2.0f * max, min, max, count, &new_offset, out);
    EXPECT_EQ(new_offset, offset + count);
    if (count >= 32) {
      new_offset = offset;
      EXPECT_EQ(0xFFFFFFFF, ReadBitsUint32(out, count, &new_offset));
    } else {
      new_offset = offset;
      EXPECT_EQ((1U << count) - 1U, ReadBitsUint32(out, count, &new_offset));
    }
    new_offset = offset;
    EXPECT_NEAR(max, ReadBitsRangeFloat(out, min, max, count, &new_offset),
                tol);
  }
}

TEST(WriteReadBitsScaledDoubleFloat, Normal) {
  uint8_t out[8] = {0};
  int32_t new_offset = 5;
  WriteBitsScaledFloat(0.0f, 1.0f, 0, &new_offset, out);
  EXPECT_EQ(5, new_offset);

  new_offset = 5;
  WriteBitsScaledDouble(0.0, 1.0, 0, &new_offset, out);
  EXPECT_EQ(5, new_offset);

  for (int32_t count = 1; count <= 32; ++count) {
    const int32_t raw_min = MinSignedValue(count);
    const int32_t raw_max = MaxSignedValue(count);
    const double scales[] = {-1.5, -0.01, 0.01, 1.5};

    for (int32_t s = 0; s < ARRAYSIZE(scales); ++s) {
      const double scale = scales[s];
      const double tol = fabs(scale);
      const double min = static_cast<double>(raw_min) * scale;
      const double max = static_cast<double>(raw_max) * scale;
      const int32_t offset = 5;

      const float scalef = static_cast<float>(scale);
      const float tolf = static_cast<float>(tol);
      const float minf = static_cast<float>(raw_min) * scalef;
      const float maxf = static_cast<float>(raw_max) * scalef;

      // Test zero value.
      new_offset = offset;
      WriteBitsScaledFloat(0.0f, scalef, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(0U, ReadBitsUint32(out, count, &new_offset));
      new_offset = offset;
      EXPECT_NEAR(0.0f, ReadBitsScaledFloat(out, scalef, count, &new_offset),
                  FLT_EPSILON);

      new_offset = offset;
      WriteBitsScaledDouble(0.0, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(0U, ReadBitsUint32(out, count, &new_offset));
      new_offset = offset;
      EXPECT_NEAR(0.0, ReadBitsScaledDouble(out, scale, count, &new_offset),
                  DBL_EPSILON);

      // Test beyond lower bound.
      new_offset = offset;
      WriteBitsScaledFloat(2.0f * minf, scalef, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(raw_min,
                SignExtend(ReadBitsUint32(out, count, &new_offset), count));
      new_offset = offset;
      EXPECT_NEAR(minf, ReadBitsScaledFloat(out, scalef, count, &new_offset),
                  FLT_EPSILON);

      new_offset = offset;
      WriteBitsScaledDouble(2.0 * min, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(raw_min,
                SignExtend(ReadBitsUint32(out, count, &new_offset), count));
      new_offset = offset;
      EXPECT_NEAR(min, ReadBitsScaledDouble(out, scale, count, &new_offset),
                  DBL_EPSILON);

      // Test beyond upper bound.
      new_offset = offset;
      WriteBitsScaledFloat(2.0f * maxf, scalef, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(raw_max,
                SignExtend(ReadBitsUint32(out, count, &new_offset), count));
      new_offset = offset;
      EXPECT_NEAR(maxf, ReadBitsScaledFloat(out, scalef, count, &new_offset),
                  FLT_EPSILON);

      new_offset = offset;
      WriteBitsScaledDouble(2.0 * max, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(raw_max,
                SignExtend(ReadBitsUint32(out, count, &new_offset), count));
      new_offset = offset;
      EXPECT_NEAR(max, ReadBitsScaledDouble(out, scale, count, &new_offset),
                  DBL_EPSILON);

      // Test in range (negative).
      float expectedf = minf / 2.0f;
      new_offset = offset;
      WriteBitsScaledFloat(expectedf, scalef, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_NEAR(expectedf,
                  ReadBitsScaledFloat(out, scalef, count, &new_offset), tolf);

      double expected = min / 2.0;
      new_offset = offset;
      WriteBitsScaledDouble(expected, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_NEAR(expected,
                  ReadBitsScaledDouble(out, scale, count, &new_offset), tol);

      // Test in range (positive).
      expectedf = maxf / 2.0f;
      new_offset = offset;
      WriteBitsScaledFloat(expectedf, scalef, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_NEAR(expectedf,
                  ReadBitsScaledFloat(out, scalef, count, &new_offset), tolf);

      expected = max / 2.0;
      new_offset = offset;
      WriteBitsScaledDouble(expected, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_NEAR(expected,
                  ReadBitsScaledDouble(out, scale, count, &new_offset), tol);
    }
  }
}

TEST(WriteReadBitsScaledOffsetFloat, Normal) {
  uint8_t out[8] = {0};
  int32_t new_offset = 5;
  WriteBitsScaledOffsetFloat(0.0f, 1.0f, 0.0f, 0, &new_offset, out);
  EXPECT_EQ(5, new_offset);

  // Numerical precision errors may occur for large values. The scale factors,
  // offsets, and values to encode were tweaked to pass this test within the
  // specified tolerance.
  for (int32_t count = 1; count <= 32; ++count) {
    const uint32_t raw_min = 0U;
    const uint32_t raw_max = MaxUnsignedValue(count);
    const float scales[] = {0.01f, 1.2f};

    for (int32_t s = 0; s < ARRAYSIZE(scales); ++s) {
      const float scale = scales[s];
      const float tol = fabsf(scale);
      const float zeros[] = {-2000.0f, 2000.0f};

      for (int32_t z = 0; z < ARRAYSIZE(zeros); ++z) {
        const float zero = zeros[z];
        const int32_t offset = 5;
        const float min = static_cast<float>(raw_min) * scale + zero;
        const float max = static_cast<float>(raw_max) * scale + zero;

        // Test beyond lower bound.
        new_offset = offset;
        WriteBitsScaledOffsetFloat(min - 1.0f, scale, zero, count, &new_offset,
                                   out);
        EXPECT_EQ(new_offset, offset + count);
        new_offset = offset;
        EXPECT_NEAR(min, ReadBitsScaledOffsetFloat(out, scale, zero, count,
                                                   &new_offset), FLT_EPSILON);

        // Test beyond upper bound.
        new_offset = offset;
        WriteBitsScaledOffsetFloat(max + 1.0f, scale, zero, count, &new_offset,
                                   out);
        EXPECT_EQ(new_offset, offset + count);
        new_offset = offset;
        EXPECT_NEAR(max, ReadBitsScaledOffsetFloat(out, scale, zero, count,
                                                   &new_offset), FLT_EPSILON);

        // Test in range.
        const float divides[] = {1.0f, 2.0f, 4.0f, 5.0f};
        for (int d = 0; d < ARRAYSIZE(divides); ++d) {
          float expected = min + (max - min) / divides[d];
          new_offset = offset;
          WriteBitsScaledOffsetFloat(expected, scale, zero, count, &new_offset,
                                     out);
          EXPECT_EQ(new_offset, offset + count);
          new_offset = offset;
          EXPECT_NEAR(expected,
                      ReadBitsScaledOffsetFloat(out, scale, zero, count,
                                                &new_offset), tol);
        }
      }
    }
  }
}

TEST(WriteReadBitsUnsignedFloat, Normal) {
  uint8_t out[8] = {0};
  int32_t new_offset = 5;
  WriteBitsUnsignedFloat(0.0f, 1.0f, 0, &new_offset, out);
  EXPECT_EQ(5, new_offset);

  for (int32_t count = 1; count <= 32; ++count) {
    const uint32_t raw_max = MaxUnsignedValue(count);
    const float scales[] = {0.01f, 1.5f};

    for (int32_t s = 0; s < ARRAYSIZE(scales); ++s) {
      const float scale = scales[s];
      const float tol = fabsf(scale);
      const float max = static_cast<float>(raw_max) * scale;
      const int32_t offset = 5;

      // Test zero value.
      new_offset = offset;
      WriteBitsUnsignedFloat(0.0f, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(0U, ReadBitsUint32(out, count, &new_offset));
      new_offset = offset;
      EXPECT_NEAR(0.0f, ReadBitsUnsignedFloat(out, scale, count, &new_offset),
                  FLT_EPSILON);

      // Test beyond lower bound.
      new_offset = offset;
      WriteBitsUnsignedFloat(-1.0f, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(0U, ReadBitsUint32(out, count, &new_offset));
      new_offset = offset;
      EXPECT_NEAR(0.0f, ReadBitsUnsignedFloat(out, scale, count, &new_offset),
                  FLT_EPSILON);

      // Test beyond upper bound.
      new_offset = offset;
      WriteBitsUnsignedFloat(2.0f * max, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_EQ(raw_max, ReadBitsUint32(out, count, &new_offset));
      new_offset = offset;
      EXPECT_NEAR(max, ReadBitsUnsignedFloat(out, scale, count, &new_offset),
                  FLT_EPSILON);

      // Test in range (positive).
      float expected = max / 2.0f;
      new_offset = offset;
      WriteBitsUnsignedFloat(expected, scale, count, &new_offset, out);
      EXPECT_EQ(new_offset, offset + count);
      new_offset = offset;
      EXPECT_NEAR(expected,
                  ReadBitsUnsignedFloat(out, scale, count, &new_offset), tol);
    }
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
