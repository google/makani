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
#include <gtest/gtest.h>
#include <jansson.h>
#include <stdint.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <vector>

#include "common/c_math/quaternion.h"
#include "common/c_math/vec2.h"
#include "common/c_math/vec3.h"
#include "lib/json_load/json_load_basic.h"

class JSONLoadBasicTest : public ::testing::Test {
 protected:
  // String example.
  json_t *json_string_ = json_string("eit");
  // Scalar values for testing limits.
  // Values for Boolean.
  json_t *json_boolean_true_ = json_boolean(true);
  json_t *json_boolean_false_ = json_boolean(false);
  // Values for double.
  json_t *json_double_max_ = json_real(DBL_MAX);
  json_t *json_double_min_ = json_real(DBL_MIN);
  // Values for float.
  json_t *json_float_max_ = json_real(FLT_MAX);
  json_t *json_float_min_ = json_real(FLT_MIN);
  json_t *json_float_too_large_ = json_real((double)FLT_MAX * 2.0);
  json_t *json_float_too_small_ = json_real((double)FLT_MIN / 2.0);
  // Values for uint32_t.
  json_t *json_int_u32_max_ = json_integer(UINT32_MAX);
  json_t *json_int_u32_too_large_ = json_integer((json_int_t)UINT32_MAX + 1);
  // Values for int32_t.
  json_t *json_int_i32_max_ = json_integer(INT32_MAX);
  json_t *json_int_i32_too_large_ = json_integer((json_int_t)INT32_MAX + 1);
  json_t *json_int_i32_min_ = json_integer(INT32_MIN);
  json_t *json_int_i32_too_small_ = json_integer((json_int_t)INT32_MIN - 1);
  // Values for uint16_t.
  json_t *json_int_u16_max_ = json_integer(UINT16_MAX);
  json_t *json_int_u16_too_large_ = json_integer((json_int_t)UINT16_MAX + 1);
  // Values for int16_t.
  json_t *json_int_i16_max_ = json_integer(INT16_MAX);
  json_t *json_int_i16_too_large_ = json_integer((json_int_t)INT16_MAX + 1);
  json_t *json_int_i16_min_ = json_integer(INT16_MIN);
  json_t *json_int_i16_too_small_ = json_integer((json_int_t)INT16_MIN - 1);
  // Values for uint8_t.
  json_t *json_int_u8_max_ = json_integer(UINT8_MAX);
  json_t *json_int_u8_too_large_ = json_integer((json_int_t)UINT8_MAX + 1);
  // Values for int8_t.
  json_t *json_int_i8_max_ = json_integer(INT8_MAX);
  json_t *json_int_i8_too_large_ = json_integer((json_int_t)INT8_MAX + 1);
  json_t *json_int_i8_min_ = json_integer(INT8_MIN);
  json_t *json_int_i8_too_small_ = json_integer((json_int_t)INT8_MIN - 1);
  // General values for all integer types.
  json_t *json_int_positive_ = json_integer(1);
  json_t *json_int_negative_ = json_integer(-1);

  // Array types.
  json_t *json_vec2_ = nullptr;
  json_t *json_vec2_bad_type_ = nullptr;
  json_t *json_vec3_ = nullptr;
  json_t *json_vec3_bad_type_ = nullptr;
  json_t *json_quat_ = nullptr;
  json_t *json_quat_bad_type_ = nullptr;

  // This is the vector of all values (initialized with the non-null
  // values above).
  std::vector<json_t *> values_ = {json_string_,
                                   json_boolean_true_, json_boolean_false_,
                                   json_double_max_, json_double_min_,
                                   json_float_max_, json_float_min_,
                                   json_float_too_large_, json_float_too_small_,
                                   json_int_u32_max_, json_int_u32_too_large_,
                                   json_int_i32_max_, json_int_i32_too_large_,
                                   json_int_i32_min_, json_int_i32_too_small_,
                                   json_int_u16_max_, json_int_u16_too_large_,
                                   json_int_i16_max_, json_int_i16_too_large_,
                                   json_int_i16_min_, json_int_i16_too_small_,
                                   json_int_u8_max_, json_int_u8_too_large_,
                                   json_int_i8_max_, json_int_i8_too_large_,
                                   json_int_i8_min_, json_int_i8_too_small_,
                                   json_int_positive_, json_int_negative_};

  JSONLoadBasicTest() {
    // Initialize remaining values and add them to values_.
    json_vec2_ = json_array();
    json_array_append_new(json_vec2_, json_real(1.0));
    json_array_append_new(json_vec2_, json_real(2.0));
    values_.push_back(json_vec2_);

    json_vec2_bad_type_ = json_array();
    json_array_append_new(json_vec2_bad_type_, json_real(1.0));
    json_array_append_new(json_vec2_bad_type_, json_integer(2));
    values_.push_back(json_vec2_bad_type_);

    json_vec3_ = json_array();
    json_array_append_new(json_vec3_, json_real(1.0));
    json_array_append_new(json_vec3_, json_real(2.0));
    json_array_append_new(json_vec3_, json_real(3.0));
    values_.push_back(json_vec3_);

    json_vec3_bad_type_ = json_array();
    json_array_append_new(json_vec3_bad_type_, json_real(1.0));
    json_array_append_new(json_vec3_bad_type_, json_real(2.0));
    json_array_append_new(json_vec3_bad_type_, json_integer(3));
    values_.push_back(json_vec3_bad_type_);

    json_quat_ = json_array();
    json_array_append_new(json_quat_, json_real(1.0));
    json_array_append_new(json_quat_, json_real(2.0));
    json_array_append_new(json_quat_, json_real(3.0));
    json_array_append_new(json_quat_, json_real(4.0));
    values_.push_back(json_quat_);

    json_quat_bad_type_ = json_array();
    json_array_append_new(json_quat_bad_type_, json_real(1.0));
    json_array_append_new(json_quat_bad_type_, json_real(2.0));
    json_array_append_new(json_quat_bad_type_, json_real(3.0));
    json_array_append_new(json_quat_bad_type_, json_string("foo"));
    values_.push_back(json_quat_bad_type_);
  }

  virtual ~JSONLoadBasicTest() {
    for (json_t *value : values_) {
      json_decref(value);
    }
  }
};

// Test behavior for nullptr arguments.
TEST_F(JSONLoadBasicTest, NullArgumentTest) {
  int32_t status = 22;
  char c;

  EXPECT_EQ(-1, JSONLoadString(nullptr, 0U, &c));

  EXPECT_FALSE(JSONLoadBoolean(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0.0, JSONLoadDouble(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0.0f, JSONLoadFloat(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0U, JSONLoadUInt32(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0, JSONLoadInt32(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0U, JSONLoadUInt16(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0, JSONLoadInt16(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0U, JSONLoadUInt8(nullptr, &status));
  EXPECT_EQ(-1, status);

  status = 22;
  EXPECT_EQ(0, JSONLoadInt8(nullptr, &status));
  EXPECT_EQ(-1, status);

  Vec2 v;
  EXPECT_EQ(-1, JSONLoadStruct_Vec2(nullptr, &v));

  Vec3 w;
  EXPECT_EQ(-1, JSONLoadStruct_Vec3(nullptr, &w));

  Quat q;
  EXPECT_EQ(-1, JSONLoadStruct_Quat(nullptr, &q));
}

TEST_F(JSONLoadBasicTest, StructArgumentTest) {
  // Struct example.
  json_t *s = json_object();
  json_object_set_new(s, "foo", json_integer(1U));
  json_object_set_new(s, "bar", json_real(2.0));

  EXPECT_EQ(1U, json_integer_value(JSONLoadStructGet(s, "foo")));
  EXPECT_EQ(2.0, json_real_value(JSONLoadStructGet(s, "bar")));
  EXPECT_EQ(nullptr, JSONLoadStructGet(s, "baz"));

  EXPECT_EQ(0, JSONLoadStructCheck(s, 2U));
  // Incorrect number of fields.
  EXPECT_EQ(-1, JSONLoadStructCheck(s, 3U));
  EXPECT_EQ(-1, JSONLoadStructCheck(s, 1U));
  // Object pointer is nullptr.
  EXPECT_EQ(-1, JSONLoadStructCheck(nullptr, 2U));
  // Not an object.
  EXPECT_EQ(-1, JSONLoadStructCheck(json_string_, 2U));

  json_decref(s);
}

TEST_F(JSONLoadBasicTest, StringArgumentTest) {
  char s[4];
  const char *s_value = json_string_value(json_string_);
  EXPECT_EQ(0, JSONLoadString(json_string_, 3, s));
  EXPECT_STREQ(s, s_value);

  // String value too long.
  EXPECT_EQ(-1, JSONLoadString(json_string_, 2, s));
  // Object is a nullptr.
  EXPECT_EQ(-1, JSONLoadString(nullptr, 3, s));
  // String length is too long.
  EXPECT_EQ(-1, JSONLoadString(json_string_, UINT32_MAX, s));
  // Incorrect type.
  EXPECT_EQ(-1, JSONLoadString(json_quat_, 3, s));
}

// Test loading 1D arrays.
TEST_F(JSONLoadBasicTest, Array1DDoubleArgumentTest) {
  std::vector<const json_t *> legal = {json_vec2_, json_vec3_, json_quat_};

  for (const json_t *value : legal) {
    uint32_t len = static_cast<uint32_t>(json_array_size(value));
    std::vector<double> v(len + 1);
    EXPECT_EQ(-1, JSONLoadArray1D_Double(value, len - 1, v.data()));
    EXPECT_EQ(-1, JSONLoadArray1D_Double(value, len + 1, v.data()));
    EXPECT_EQ(0, JSONLoadArray1D_Double(value, len, v.data()));
    for (uint32_t j = 0U; j < len; ++j) {
      EXPECT_EQ(v[j], json_real_value(json_array_get(value, j)));
    }
  }

  for (const json_t *value : values_) {
    if (std::find(legal.begin(), legal.end(), value) == legal.end()) {
      double v[4] = {1.0, 2.0, 3.0, 4.0};
      EXPECT_EQ(-1, JSONLoadArray1D_Double(value, 4, v));
    }
  }
}

// Test loading structures that are based on array notation.
TEST_F(JSONLoadBasicTest, Vec2ArgumentTest) {
  Vec2 v = {22.0, 23.0};
  EXPECT_EQ(JSONLoadStruct_Vec2(json_vec2_, &v), 0);
  EXPECT_EQ(json_real_value(json_array_get(json_vec2_, 0)), v.x);
  EXPECT_EQ(json_real_value(json_array_get(json_vec2_, 1)), v.y);

  for (const json_t *value : values_) {
    if (value != json_vec2_) {
      EXPECT_EQ(-1, JSONLoadStruct_Vec2(value, &v));
    }
  }
}

TEST_F(JSONLoadBasicTest, Vec3ArgumentTest) {
  Vec3 v = {22.0, 23.0, 24.0};
  EXPECT_EQ(0, JSONLoadStruct_Vec3(json_vec3_, &v));
  EXPECT_EQ(json_real_value(json_array_get(json_vec3_, 0)), v.x);
  EXPECT_EQ(json_real_value(json_array_get(json_vec3_, 1)), v.y);
  EXPECT_EQ(json_real_value(json_array_get(json_vec3_, 2)), v.z);

  for (const json_t *value : values_) {
    if (value != json_vec3_) {
      EXPECT_EQ(JSONLoadStruct_Vec3(value, &v), -1);
    }
  }
}

TEST_F(JSONLoadBasicTest, QuatArgumentTest) {
  Quat q = {22.0, 23.0, 24.0, 25.0};
  EXPECT_EQ(JSONLoadStruct_Quat(json_quat_, &q), 0);
  EXPECT_EQ(json_real_value(json_array_get(json_quat_, 0)), q.q0);
  EXPECT_EQ(json_real_value(json_array_get(json_quat_, 1)), q.q1);
  EXPECT_EQ(json_real_value(json_array_get(json_quat_, 2)), q.q2);
  EXPECT_EQ(json_real_value(json_array_get(json_quat_, 3)), q.q3);

  for (const json_t *value : values_) {
    if (value != json_quat_) {
      EXPECT_EQ(-1, JSONLoadStruct_Quat(value, &q));
    }
  }
}

// Death tests.
#if !defined(NDEBUG)

typedef JSONLoadBasicTest JSONLoadBasicDeathTest;

TEST_F(JSONLoadBasicDeathTest, NullArgumentTest) {
  EXPECT_DEATH(JSONLoadString(nullptr, 0, nullptr), "");
  EXPECT_DEATH(JSONLoadBoolean(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadDouble(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadFloat(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadUInt32(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadInt32(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadUInt16(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadInt16(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadStruct_Vec2(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadStruct_Vec3(nullptr, nullptr), "");
  EXPECT_DEATH(JSONLoadStruct_Quat(nullptr, nullptr), "");
}
#endif  // !defined(NDEBUG)

TEST_F(JSONLoadBasicTest, JSONLoadBooleanTest) {
  int32_t status;

  for (const json_t *value : values_) {
    status = 22;
    if (value == json_boolean_true_) {
      EXPECT_TRUE(JSONLoadBoolean(value, &status));
      EXPECT_EQ(0, status);
    } else if (value == json_boolean_false_) {
      EXPECT_FALSE(JSONLoadBoolean(value, &status));
      EXPECT_EQ(0, status);
    } else {
      EXPECT_FALSE(JSONLoadBoolean(value, &status));
      EXPECT_EQ(-1, status);
    }
  }
}

// Floating point tests.
class JSONLoadBasicRealTest : public JSONLoadBasicTest {
 protected:
  template <typename T>
  void TestRealArguments(
      std::function<T(const json_t *obj, int32_t *status)> load_func,
      const std::vector<const json_t *> &legal) {
    for (const json_t *value : legal) {
      int32_t status = 22;
      EXPECT_EQ(json_real_value(value), load_func(value, &status));
      EXPECT_EQ(0, status);
    }

    for (const json_t *value : values_) {
      if (std::find(legal.begin(), legal.end(), value) == legal.end()) {
        int32_t status = 22;
        EXPECT_EQ(0.0, load_func(value, &status));
        EXPECT_EQ(-1, status);
      }
    }
  }
};

TEST_F(JSONLoadBasicRealTest, DoubleArgumentTest) {
  TestRealArguments<double>(
      &JSONLoadDouble, {json_double_max_, json_double_min_, json_float_max_,
            json_float_min_, json_float_too_large_,
            json_float_too_small_});
}

TEST_F(JSONLoadBasicRealTest, FloatArgumentTest) {
  TestRealArguments<float>(
      &JSONLoadFloat, {json_float_max_, json_float_min_});
}

// Integer value tests.
class JSONLoadBasicIntegerTest : public JSONLoadBasicTest {
 protected:
  template <typename T>
  void TestIntegerArguments(
      std::function<T(const json_t *obj, int32_t *status)> load_func,
      const std::vector<const json_t *> &legal) {
    for (const json_t *value : legal) {
      int32_t status = 22;
      EXPECT_EQ(json_integer_value(value), load_func(value, &status));
      EXPECT_EQ(0, status);
    }

    for (const json_t *value : values_) {
      if (std::find(legal.begin(), legal.end(), value) == legal.end()) {
        int32_t status = 22;
        EXPECT_EQ(0, load_func(value, &status));
        EXPECT_EQ(-1, status);
      }
    }
  }
};

TEST_F(JSONLoadBasicIntegerTest, UInt32ArgumentTest) {
  TestIntegerArguments<uint32_t>(
      &JSONLoadUInt32, {json_int_u32_max_, json_int_i32_max_,
            json_int_i32_too_large_, json_int_u16_max_,
            json_int_u16_too_large_, json_int_i16_max_,
            json_int_i16_too_large_,
            json_int_u8_max_, json_int_u8_too_large_,
            json_int_i8_max_, json_int_i8_too_large_,
            json_int_positive_});
}

TEST_F(JSONLoadBasicIntegerTest, Int32ArgumentTest) {
  TestIntegerArguments<int32_t>(
      &JSONLoadInt32, {json_int_i32_max_, json_int_i32_min_,
            json_int_u16_max_, json_int_u16_too_large_,
            json_int_i16_max_, json_int_i16_too_large_,
            json_int_i16_min_, json_int_i16_too_small_,
            json_int_u8_max_, json_int_u8_too_large_,
            json_int_i8_max_, json_int_i8_too_large_,
            json_int_i8_min_, json_int_i8_too_small_,
            json_int_positive_, json_int_negative_});
}

TEST_F(JSONLoadBasicIntegerTest, UInt16ArgumentTest) {
  TestIntegerArguments<uint16_t>(
      &JSONLoadUInt16, {json_int_u16_max_, json_int_i16_max_,
            json_int_i16_too_large_, json_int_u8_max_,
            json_int_u8_too_large_, json_int_i8_max_,
            json_int_i8_too_large_, json_int_positive_});
}

TEST_F(JSONLoadBasicIntegerTest, Int16ArgumentTest) {
  TestIntegerArguments<int16_t>(
      &JSONLoadInt16, {json_int_i16_max_, json_int_i16_min_,
            json_int_u8_max_, json_int_u8_too_large_,
            json_int_i8_max_, json_int_i8_too_large_,
            json_int_i8_min_, json_int_i8_too_small_,
            json_int_positive_, json_int_negative_});
}

TEST_F(JSONLoadBasicIntegerTest, UInt8ArgumentTest) {
  TestIntegerArguments<uint8_t>(
      &JSONLoadUInt8, {json_int_u8_max_, json_int_i8_max_,
            json_int_i8_too_large_, json_int_positive_});
}

TEST_F(JSONLoadBasicIntegerTest, Int8ArgumentTest) {
  TestIntegerArguments<int8_t>(
      &JSONLoadInt8, {json_int_i8_max_, json_int_i8_min_, json_int_positive_,
            json_int_negative_});
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
