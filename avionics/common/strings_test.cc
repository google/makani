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
#include <stdint.h>

#include <limits>

extern "C" {

#include "avionics/common/strings.h"

}  // extern "C"

TEST(ReadDecInt32, Normal) {
  int32_t num, length;

  length = ReadDecInt32(reinterpret_cast<const char *>("2222"), 4, &num);
  EXPECT_EQ(4, length);
  EXPECT_EQ(2222, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("2222"), 5, &num);
  EXPECT_EQ(4, length);
  EXPECT_EQ(2222, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("-22"), 3, &num);
  EXPECT_EQ(3, length);
  EXPECT_EQ(-22, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("-0"), 2, &num);
  EXPECT_EQ(2, length);
  EXPECT_EQ(0, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("+12345"), 6, &num);
  EXPECT_EQ(6, length);
  EXPECT_EQ(12345, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("031415"), 6, &num);
  EXPECT_EQ(6, length);
  EXPECT_EQ(31415, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("-031415"), 7, &num);
  EXPECT_EQ(7, length);
  EXPECT_EQ(-31415, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("-123foo"), 7, &num);
  EXPECT_EQ(4, length);
  EXPECT_EQ(-123, num);
}

TEST(ReadDecInt32, ZeroLength) {
  int32_t num, length;

  length = ReadDecInt32(reinterpret_cast<const char *>("-"), 1, &num);
  EXPECT_EQ(0, length);
  EXPECT_EQ(0, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("+"), 1, &num);
  EXPECT_EQ(0, length);
  EXPECT_EQ(0, num);

  length = ReadDecInt32(reinterpret_cast<const char *>(" "), 1, &num);
  EXPECT_EQ(0, length);
  EXPECT_EQ(0, num);

  length = ReadDecInt32(reinterpret_cast<const char *>(""), 0, &num);
  EXPECT_EQ(0, length);
  EXPECT_EQ(0, num);
}

TEST(ReadDecInt32, Extremes) {
  int32_t num, length;

  length = ReadDecInt32(reinterpret_cast<const char *>("2147483647"),
                        10, &num);
  EXPECT_EQ(10, length);
  EXPECT_EQ(2147483647, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("2147483648"),
                        10, &num);
  EXPECT_EQ(10, length);
  EXPECT_EQ(2147483647, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("21474836469"),
                        11, &num);
  EXPECT_EQ(11, length);
  EXPECT_EQ(2147483647, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("-2147483648"),
                        11, &num);
  EXPECT_EQ(11, length);
  EXPECT_EQ(-2147483648, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("-2147483649"),
                        11, &num);
  EXPECT_EQ(11, length);
  EXPECT_EQ(-2147483648, num);

  length = ReadDecInt32(reinterpret_cast<const char *>("-21474836480"),
                        12, &num);
  EXPECT_EQ(12, length);
  EXPECT_EQ(-2147483648, num);
}

TEST(ReadDecFloat, Normal) {
  int32_t length;
  float num;

  length = ReadDecFloat(reinterpret_cast<const char *>("12.345678"), 9, &num);
  EXPECT_EQ(9, length);
  EXPECT_NEAR(12.345678f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("-123456.78"), 10, &num);
  EXPECT_EQ(10, length);
  EXPECT_NEAR(-123456.78f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("123"), 3, &num);
  EXPECT_EQ(3, length);
  EXPECT_NEAR(123.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("123."), 4, &num);
  EXPECT_EQ(4, length);
  EXPECT_NEAR(123.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("-123"), 4, &num);
  EXPECT_EQ(4, length);
  EXPECT_NEAR(-123.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>(".123"), 4, &num);
  EXPECT_EQ(4, length);
  EXPECT_NEAR(0.123f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("-0.123"), 6, &num);
  EXPECT_EQ(6, length);
  EXPECT_NEAR(-0.123f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("-.123"), 5, &num);
  EXPECT_EQ(5, length);
  EXPECT_NEAR(-0.123f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("+.123"), 5, &num);
  EXPECT_EQ(5, length);
  EXPECT_NEAR(0.123f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("0"), 1, &num);
  EXPECT_EQ(1, length);
  EXPECT_EQ(0.0f, num);
}

TEST(ReadDecFloat, ZeroLength) {
  int32_t length;
  float num;

  length = ReadDecFloat(reinterpret_cast<const char *>("-"), 1, &num);
  EXPECT_EQ(0, length);
  EXPECT_NEAR(0.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("+"), 1, &num);
  EXPECT_EQ(0, length);
  EXPECT_NEAR(0.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>(" "), 1, &num);
  EXPECT_EQ(0, length);
  EXPECT_NEAR(0.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>(""), 0, &num);
  EXPECT_EQ(0, length);
  EXPECT_NEAR(0.0f, num, FLT_EPSILON);
}

TEST(ReadDecFloat, Extremes) {
  int32_t length;
  float num;

  length = ReadDecFloat(reinterpret_cast<const char *>("9999999999"),
                        10, &num);
  EXPECT_EQ(10, length);
  EXPECT_NEAR(9999999999.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("-9999999999"),
                        11, &num);
  EXPECT_EQ(11, length);
  EXPECT_NEAR(-9999999999.0f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("0.2147483647"),
                        12, &num);
  EXPECT_EQ(12, length);
  EXPECT_NEAR(0.2147483647f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("0.21474836470"),
                        13, &num);
  EXPECT_EQ(13, length);
  EXPECT_NEAR(0.2147483647f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("0.99999999999"),
                        13, &num);
  EXPECT_EQ(13, length);
  EXPECT_NEAR(0.9999999999f, num, FLT_EPSILON);

  length = ReadDecFloat(reinterpret_cast<const char *>("-0.99999999999"),
                        14, &num);
  EXPECT_EQ(14, length);
  EXPECT_NEAR(-0.9999999999f, num, FLT_EPSILON);
}

TEST(ReadHexUint8, Normal) {
  uint8_t num;
  int32_t length;

  length = ReadHexUint8("12", 2, &num);
  EXPECT_EQ(2, length);
  EXPECT_EQ(0x12, num);

  length = ReadHexUint8("AB", 2, &num);
  EXPECT_EQ(2, length);
  EXPECT_EQ(0xAB, num);

  length = ReadHexUint8("cd", 2, &num);
  EXPECT_EQ(2, length);
  EXPECT_EQ(0xCD, num);

  length = ReadHexUint8("A", 1, &num);
  EXPECT_EQ(1, length);
  EXPECT_EQ(0x0A, num);

  length = ReadHexUint8("1u", 6, &num);
  EXPECT_EQ(1, length);
  EXPECT_EQ(0x01, num);
}

TEST(ReadHexUint8, ZeroLength) {
  uint8_t num;
  int32_t length;

  length = ReadHexUint8("", 0, &num);
  EXPECT_EQ(0, length);

  length = ReadHexUint8(" ", 1, &num);
  EXPECT_EQ(0, length);
}

TEST(ReadHexUint8, Extremes) {
  uint8_t num;
  int32_t length;

  length = ReadHexUint8("FFF", 3, &num);
  EXPECT_EQ(2, length);
  EXPECT_EQ(0xFF, num);

  length = ReadHexUint8("000", 3, &num);
  EXPECT_EQ(2, length);
  EXPECT_EQ(0x0, num);

  length = ReadHexUint8("0", 1, &num);
  EXPECT_EQ(1, length);
  EXPECT_EQ(0x0, num);
}

TEST(ReadHexUint32, Normal) {
  uint32_t num;
  int32_t length;

  length = ReadHexUint32("1234abcd", 8, &num);
  EXPECT_EQ(8, length);
  EXPECT_EQ(0x1234ABCD, num);

  length = ReadHexUint32("1234ABCD", 8, &num);
  EXPECT_EQ(8, length);
  EXPECT_EQ(0x1234ABCD, num);

  length = ReadHexUint32("0A", 2, &num);
  EXPECT_EQ(2, length);
  EXPECT_EQ(0x0A, num);

  length = ReadHexUint32("123foo", 6, &num);
  EXPECT_EQ(4, length);
  EXPECT_EQ(0x123F, num);
}

TEST(ReadHexUint32, ZeroLength) {
  uint32_t num;
  int32_t length;

  length = ReadHexUint32("", 0, &num);
  EXPECT_EQ(0, length);

  length = ReadHexUint32(" ", 1, &num);
  EXPECT_EQ(0, length);
}

TEST(ReadHexUint32, Extremes) {
  uint32_t num;
  int32_t length;

  length = ReadHexUint32("FFFFFFFF", 8, &num);
  EXPECT_EQ(8, length);
  EXPECT_EQ(0xFFFFFFFF, num);

  length = ReadHexUint32("123456789", 9, &num);
  EXPECT_EQ(8, length);
  EXPECT_EQ(0x12345678, num);

  length = ReadHexUint32("00000000", 8, &num);
  EXPECT_EQ(8, length);
  EXPECT_EQ(0x0, num);

  length = ReadHexUint32("0", 1, &num);
  EXPECT_EQ(1, length);
  EXPECT_EQ(0x0, num);
}

TEST(WriteHexUint32, Normal) {
  char str[23];
  int32_t length;

  length = WriteHexUint32(12345U, 2, 5, str);
  EXPECT_STREQ("3039", str);
  EXPECT_EQ(4, length);

  length = WriteHexUint32(12345U, 2, 6, str);
  EXPECT_STREQ("3039", str);
  EXPECT_EQ(4, length);

  length = WriteHexUint32(10U, 2, 22, str);
  EXPECT_STREQ("0A", str);
  EXPECT_EQ(2, length);

  length = WriteHexUint32(11U, 3, 22, str);
  EXPECT_STREQ("00B", str);
  EXPECT_EQ(3, length);

  length = WriteHexUint32(17U, 4, 22, str);
  EXPECT_STREQ("0011", str);
  EXPECT_EQ(4, length);

  length = WriteHexUint32(0U, 5, 22, str);
  EXPECT_STREQ("00000", str);
  EXPECT_EQ(5, length);
}

#if defined(NDEBUG)

TEST(WriteHexUint32, Truncation) {
  char str[4];
  int32_t length;

  length = WriteHexUint32(12345U, 2, 3, str);
  EXPECT_STREQ("303", str);
  EXPECT_EQ(3, length);
}

#else

TEST(WriteHexUint32, Truncation) {
  char str[4];
  EXPECT_DEATH(WriteHexUint32(12345U, 2, 3, str), "");
}

#endif  // defined(NDEBUG)

TEST(WriteDecUint32, Normal) {
  char str[10];
  int32_t length;

  length = WriteDecUint32(0U, 5, str);
  EXPECT_STREQ("0", str);
  EXPECT_EQ(1, length);

  length = WriteDecUint32(12345U, 6, str);
  EXPECT_STREQ("12345", str);
  EXPECT_EQ(5, length);

  length = WriteDecUint32(999999999U, 10, str);
  EXPECT_STREQ("999999999", str);
  EXPECT_EQ(9, length);
}

#if defined(NDEBUG)

TEST(WriteDecUint32, Truncation) {
  char str[4];
  int32_t length;

  length = WriteDecUint32(12345U, 3, str);
  EXPECT_STREQ("123", str);
  EXPECT_EQ(3, length);
}

#else

TEST(WriteDecUint32, Truncation) {
  char str[4];
  EXPECT_DEATH(WriteDecUint32(12345U, 3, str), "");
}

#endif  // defined(NDEBUG)

TEST(WriteDecUint32, Extremes) {
  char str[11];
  int32_t length;

  length = WriteDecUint32(std::numeric_limits<uint32_t>::max(), 11, str);
  EXPECT_STREQ("4294967295", str);
  EXPECT_EQ(10, length);

  length = WriteDecUint32(std::numeric_limits<uint32_t>::max(), 12, str);
  EXPECT_STREQ("4294967295", str);
  EXPECT_EQ(10, length);
}

TEST(WriteDecInt32, Normal) {
  char str[10];
  int32_t length;

  length = WriteDecInt32(0, 5, str);
  EXPECT_STREQ("0", str);
  EXPECT_EQ(1, length);

  length = WriteDecInt32(12345, 6, str);
  EXPECT_STREQ("12345", str);
  EXPECT_EQ(5, length);

  length = WriteDecInt32(-999999999, 11, str);
  EXPECT_STREQ("-999999999", str);
  EXPECT_EQ(10, length);
}

#if defined(NDEBUG)

TEST(WriteDecInt32, Truncation) {
  char str[4];
  int32_t length;

  length = WriteDecInt32(-12345, 3, str);
  EXPECT_STREQ("-12", str);
  EXPECT_EQ(3, length);
}

#else

TEST(WriteDecInt32, Truncation) {
  char str[4];
  EXPECT_DEATH(WriteDecInt32(-12345, 5, str), "");
}

#endif  // defined(NDEBUG)

TEST(WriteDecInt32, Extremes) {
  char str[12];
  int32_t length;

  length = WriteDecInt32(std::numeric_limits<int32_t>::max(), 11, str);
  EXPECT_STREQ("2147483647", str);
  EXPECT_EQ(10, length);

  length = WriteDecInt32(std::numeric_limits<int32_t>::min(), 12, str);
  EXPECT_STREQ("-2147483648", str);
  EXPECT_EQ(11, length);
}

TEST(FindString, Normal) {
  const char *haystack = "The quick brown fox jumps over the lazy dog";
  int32_t haystack_length = (int32_t)strlen(haystack);

  EXPECT_EQ(haystack, FindString(haystack_length, haystack,
                                 haystack_length, haystack));
  EXPECT_EQ(haystack, FindString(haystack_length, haystack, 9, "The quick"));
  EXPECT_EQ(&haystack[40], FindString(haystack_length, haystack, 3, "dog"));
  EXPECT_EQ(nullptr, FindString(haystack_length - 1, haystack, 3, "dog"));
  EXPECT_EQ(nullptr, FindString(3, "dog", haystack_length, haystack));
  EXPECT_EQ(nullptr, FindString(-haystack_length, haystack, 3, "dog"));
  EXPECT_EQ(nullptr, FindString(haystack_length, haystack, -3, "dog"));
  EXPECT_EQ(nullptr, FindString(-haystack_length, haystack, -3, "dog"));
}

TEST(WildCompareString, Normal) {
  const char *ref = "reference";

  EXPECT_TRUE(WildCompareString(ref, ref));
  EXPECT_TRUE(WildCompareString(ref, "*"));
  EXPECT_TRUE(WildCompareString(ref, "?????????"));
  EXPECT_FALSE(WildCompareString(ref, "?"));
  EXPECT_TRUE(WildCompareString(ref, "r*f?*??*e"));
  EXPECT_TRUE(WildCompareString(ref, "r??*??ce"));
  EXPECT_TRUE(WildCompareString(ref, "r??****??ce"));
  EXPECT_FALSE(WildCompareString(ref, "??"));
  EXPECT_TRUE(WildCompareString(ref, "??*?"));
  EXPECT_TRUE(WildCompareString(ref, "ref*"));
  EXPECT_TRUE(WildCompareString(ref, "*erence"));
  EXPECT_TRUE(WildCompareString(ref, "ref*erence"));
  EXPECT_TRUE(WildCompareString(ref, "ref*er?*ce"));
  EXPECT_TRUE(WildCompareString(ref, "ref*nce"));
  EXPECT_TRUE(WildCompareString(ref, "reference**"));
  EXPECT_TRUE(WildCompareString(ref, "**reference"));
  EXPECT_TRUE(WildCompareString(ref, "?efe?enc?"));
}

TEST(IsNumeric, Normal) {
  EXPECT_TRUE(IsNumeric("1234"));
  EXPECT_FALSE(IsNumeric("abcd"));
  EXPECT_FALSE(IsNumeric("a1234"));
  EXPECT_FALSE(IsNumeric("12a34"));
  EXPECT_FALSE(IsNumeric("1234a"));
}

TEST(IsNumeric, Signs) {
  EXPECT_TRUE(IsNumeric("-1234"));
  EXPECT_TRUE(IsNumeric("+1234"));
  EXPECT_FALSE(IsNumeric("1234-"));
  EXPECT_FALSE(IsNumeric("1234+"));
  EXPECT_FALSE(IsNumeric("--1234"));
  EXPECT_FALSE(IsNumeric("++1234"));
  EXPECT_FALSE(IsNumeric("-+1234"));
}

TEST(IsNumeric, Decimals) {
  EXPECT_TRUE(IsNumeric(".1234"));
  EXPECT_TRUE(IsNumeric(",1234"));
  EXPECT_TRUE(IsNumeric("12.34"));
  EXPECT_TRUE(IsNumeric("12,34"));
  EXPECT_FALSE(IsNumeric("12..34"));
  EXPECT_FALSE(IsNumeric("12,,34"));
  EXPECT_FALSE(IsNumeric("..1234"));
  EXPECT_FALSE(IsNumeric(",,1234"));
}

TEST(IsNumeric, DecimalsAndSigns) {
  EXPECT_TRUE(IsNumeric("-12.34"));
  EXPECT_TRUE(IsNumeric("-12,34"));
  EXPECT_TRUE(IsNumeric("+12.34"));
  EXPECT_TRUE(IsNumeric("+12,34"));
  EXPECT_FALSE(IsNumeric("-1.2.34"));
  EXPECT_FALSE(IsNumeric("+1.2.34"));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
