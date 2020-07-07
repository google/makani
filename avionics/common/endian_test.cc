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

#include <float.h>
#include <math.h>
#include <stdint.h>

#include <gtest/gtest.h>

extern "C" {

#include "avionics/common/endian.h"
#include "common/macros.h"

}  // extern "C"

TEST(SignExtend, Normal) {
  uint32_t u32;
  int32_t i32;

  // Test positive 1-bit number.
  u32 = 0x0;
  i32 = SignExtend(u32, 1);
  EXPECT_EQ(i32, 0x0);

  // Test negative 1-bit number.
  u32 = 0x01;
  i32 = SignExtend(u32, 1);
  EXPECT_EQ(i32, 0xFFFFFFFF);

  // Test positive 20-bit number.
  u32 = 0x00071892;
  i32 = SignExtend(u32, 20);
  EXPECT_EQ(i32, 0x00071892);

  // Test negative 20-bit number.
  u32 = 0x00081892;
  i32 = SignExtend(u32, 20);
  EXPECT_EQ(i32, 0xFFF81892);

  // Test positive 32-bit number.
  u32 = 0x42171892;
  i32 = SignExtend(u32, 32);
  EXPECT_EQ(i32, 0x42171892);

  // Test negative 32-bit number.
  u32 = 0x82171892;
  i32 = SignExtend(u32, 32);
  EXPECT_EQ(i32, 0x82171892);
}

TEST(UnsignedToFloat, Normal) {
  // Exact tests.
  EXPECT_EQ(UnsignedToFloat(0x00, 8, 0), 1.0f);
  EXPECT_EQ(UnsignedToFloat(0x01, 8, 0), 1.0f);
  EXPECT_EQ(UnsignedToFloat(0x05, 8, 0), 5.0f);
  EXPECT_EQ(UnsignedToFloat(0x80, 8, 0), 128.0f);
  EXPECT_EQ(UnsignedToFloat(0x00, 8, 5), 32.0f);
  EXPECT_EQ(UnsignedToFloat(0x00, 8, -5), 0.03125f);
  EXPECT_EQ(UnsignedToFloat(0x05, 8, -5), 0.15625f);
  EXPECT_EQ(UnsignedToFloat(0x80, 8, 5), 4096.0f);
  EXPECT_EQ(UnsignedToFloat(0x80, 8, -5), 4.0f);
  EXPECT_EQ(UnsignedToFloat(0x00800000, 24, -23), 1.0f);

  // Approximate tests.
  uint32_t code;
  int32_t exp;

  code = 0x00FFFFFF;
  exp = -24;
  EXPECT_NEAR(UnsignedToFloat(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);

  code = 0x00932322;
  exp = -12;
  EXPECT_NEAR(UnsignedToFloat(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);

  code = 0x00932322;
  exp = 12;
  EXPECT_NEAR(UnsignedToFloat(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);
}

TEST(SignedToFloat, Normal) {
  // Exact tests.
  EXPECT_EQ(SignedToFloat(0x00, 8, 0), 1.0f);
  EXPECT_EQ(SignedToFloat(0x01, 8, 0), 1.0f);
  EXPECT_EQ(SignedToFloat(0x05, 8, 0), 5.0f);
  EXPECT_EQ(SignedToFloat(0x80, 8, 0), -128.0f);
  EXPECT_EQ(SignedToFloat(0x00, 8, 5), 32.0f);
  EXPECT_EQ(SignedToFloat(0x00, 8, -5), 0.03125f);
  EXPECT_EQ(SignedToFloat(0x05, 8, -5), 0.15625f);
  EXPECT_EQ(SignedToFloat(0x80, 8, 5), -4096.0f);
  EXPECT_EQ(SignedToFloat(0x80, 8, -5), -4.0f);
  EXPECT_EQ(SignedToFloat(0x00800000, 24, -23), -1.0f);

  // Approximate tests.
  uint32_t code;
  int32_t exp;

  code = 0xFFFFFFFF;
  exp = -24;
  EXPECT_NEAR(SignedToFloat(code & 0x00FFFFFF, 24, exp),
              -static_cast<double>(-code) * pow(2.0, exp), DBL_EPSILON);

  code = 0x00732322;
  exp = -12;
  EXPECT_NEAR(SignedToFloat(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);

  code = 0xFF932322;
  exp = 12;
  EXPECT_NEAR(SignedToFloat(code & 0x00FFFFFF, 24, exp),
              -static_cast<double>(-code) * pow(2.0, exp), DBL_EPSILON);
}

TEST(UnsignedToDouble, Normal) {
  // Exact tests.
  EXPECT_EQ(UnsignedToDouble(0x00, 8, 0), 1.0f);
  EXPECT_EQ(UnsignedToDouble(0x01, 8, 0), 1.0f);
  EXPECT_EQ(UnsignedToDouble(0x05, 8, 0), 5.0f);
  EXPECT_EQ(UnsignedToDouble(0x80, 8, 0), 128.0f);
  EXPECT_EQ(UnsignedToDouble(0x00, 8, 5), 32.0f);
  EXPECT_EQ(UnsignedToDouble(0x00, 8, -5), 0.03125f);
  EXPECT_EQ(UnsignedToDouble(0x05, 8, -5), 0.15625f);
  EXPECT_EQ(UnsignedToDouble(0x80, 8, 5), 4096.0f);
  EXPECT_EQ(UnsignedToDouble(0x80, 8, -5), 4.0f);
  EXPECT_EQ(UnsignedToDouble(0x00800000, 24, -23), 1.0f);
  EXPECT_EQ(UnsignedToDouble(0x80000000, 32, -31), 1.0f);

  // Approximate tests.
  uint32_t code;
  int32_t exp;

  code = 0x00FFFFFF;
  exp = -24;
  EXPECT_NEAR(UnsignedToDouble(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);

  code = 0x00932322;
  exp = -12;
  EXPECT_NEAR(UnsignedToDouble(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);

  code = 0x00932322;
  exp = 12;
  EXPECT_NEAR(UnsignedToDouble(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);
}

TEST(SignedToDouble, Normal) {
  // Exact tests.
  EXPECT_EQ(SignedToDouble(0x00, 8, 0), 1.0f);
  EXPECT_EQ(SignedToDouble(0x01, 8, 0), 1.0f);
  EXPECT_EQ(SignedToDouble(0x05, 8, 0), 5.0f);
  EXPECT_EQ(SignedToDouble(0x80, 8, 0), -128.0f);
  EXPECT_EQ(SignedToDouble(0x00, 8, 5), 32.0f);
  EXPECT_EQ(SignedToDouble(0x00, 8, -5), 0.03125f);
  EXPECT_EQ(SignedToDouble(0x05, 8, -5), 0.15625f);
  EXPECT_EQ(SignedToDouble(0x80, 8, 5), -4096.0f);
  EXPECT_EQ(SignedToDouble(0x80, 8, -5), -4.0f);
  EXPECT_EQ(SignedToDouble(0x00800000, 24, -23), -1.0f);
  EXPECT_EQ(SignedToDouble(0x80000000, 32, -31), -1.0f);

  // Approximate tests.
  uint32_t code;
  int32_t exp;

  code = 0xFFFFFFFF;
  exp = -24;
  EXPECT_NEAR(SignedToDouble(code & 0x00FFFFFF, 24, exp),
              -static_cast<double>(-code) * pow(2.0, exp), DBL_EPSILON);

  code = 0x00732322;
  exp = -12;
  EXPECT_NEAR(SignedToDouble(code, 24, exp),
              static_cast<double>(code) * pow(2.0, exp), DBL_EPSILON);

  code = 0xFF932322;
  exp = 12;
  EXPECT_NEAR(SignedToDouble(code & 0x00FFFFFF, 24, exp),
              -static_cast<double>(-code) * pow(2.0, exp), DBL_EPSILON);

  code = 0xFF932322;
  exp = 12;
  EXPECT_NEAR(SignedToDouble(code & 0xFFFFFFFF, 32, exp),
              -static_cast<double>(-code) * pow(2.0, exp), DBL_EPSILON);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
