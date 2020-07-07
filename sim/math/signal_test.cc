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

#include <functional>
#include <limits>
#include <vector>

#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"
#include "sim/math/signal.h"

namespace {

void ExpectVec3VectorEq(const std::vector<double> &vec, const Vec3 &v) {
  EXPECT_EQ(3U, vec.size());
  EXPECT_EQ(v.x, vec[0]);
  EXPECT_EQ(v.y, vec[1]);
  EXPECT_EQ(v.z, vec[2]);
}

}  // namespace

namespace sim {
namespace signal {

TEST(signal_double, Quantize) {
  double x = 3.9;
  EXPECT_NEAR(3.3, Quantize(x, 1.1, &x),
              3.9 * std::numeric_limits<double>::epsilon());
  x = 3.1415;
  EXPECT_EQ(3.1415, Quantize(x, 0.0, &x));
}

TEST(signal_Vec3, Quantize) {
  Vec3 v = {1.8, 2.4, 3.9};
  EXPECT_NEAR_VEC3(Vec3({1.1, 2.2, 3.3}),
                   Quantize(v, Vec3({1.1, 1.1, 1.1}), &v),
                   3.9 * std::numeric_limits<double>::epsilon());
  Vec3 v_copy = v;
  EXPECT_EQ_VEC3(v_copy, Quantize(v, kVec3Zero, &v));
}

TEST(signal_Vec3, CompareToDouble) {
  NamedRandomNumberGenerator rng("CompareToDouble");

  Vec3 u, v, w;
  GetRandNormal<Vec3>(&rng, &u);
  GetRandNormal<Vec3>(&rng, &v);
  GetRandNormal<Vec3>(&rng, &w);

  std::vector<double> vec_u, vec_v, vec_w;
  ToVector<Vec3>(u, &vec_u);
  ToVector<Vec3>(v, &vec_v);
  ToVector<Vec3>(w, &vec_w);

  ExpectVec3VectorEq(vec_u, u);
  ExpectVec3VectorEq(vec_v, v);
  ExpectVec3VectorEq(vec_w, w);

  Add<Vec3>(u, v, &w);
  for (int32_t i = 0; i < 3; ++i) {
    Add<double>(vec_u[i], vec_v[i], &vec_w[i]);
  }
  ExpectVec3VectorEq(vec_w, w);

  Mult<Vec3>(u, v, &w);
  for (int32_t i = 0; i < 3; ++i) {
    Mult<double>(vec_u[i], vec_v[i], &vec_w[i]);
  }
  ExpectVec3VectorEq(vec_w, w);

  w = GetAddIdentity<Vec3>();
  for (int32_t i = 0; i < 3; ++i) {
    vec_w[i] = GetAddIdentity<double>();
  }
  ExpectVec3VectorEq(vec_w, w);

  w = GetMultIdentity<Vec3>();
  for (int32_t i = 0; i < 3; ++i) {
    vec_w[i] = GetMultIdentity<double>();
  }
  ExpectVec3VectorEq(vec_w, w);

  vec_u = {2.0, -1.0, 0.0};
  vec_v = {3.0, 0.0, 2.0};
  vec_w = {1.0, 1.0, 1.0};
  FromVector<Vec3>(vec_u, &u);
  FromVector<Vec3>(vec_v, &v);
  FromVector<Vec3>(vec_w, &w);

  Saturate<Vec3>(w, u, v, &w);
  for (int32_t i = 0; i < 3; ++i) {
    Saturate<double>(vec_w[i], vec_u[i], vec_v[i], &vec_w[i]);
  }
  ExpectVec3VectorEq(vec_w, w);
}

TEST(signal_Vec3, RandNormal) {
  NamedRandomNumberGenerator rng("RandNormal");

  Vec3 v = kVec3Zero, w = kVec3Zero;
  GetRandNormal<Vec3>(&rng, &v);
  EXPECT_NE(w.x, v.x);
  EXPECT_NE(w.y, v.y);
  EXPECT_NE(w.z, v.z);
  GetRandNormal<Vec3>(&rng, &w);
  EXPECT_NE(v.x, w.x);
  EXPECT_NE(v.y, w.y);
  EXPECT_NE(v.z, w.z);
}

TEST(signal_Vec3, FromVector_ToVector) {
  Vec3 v = {4.0, 5.0, 6.0};
  std::vector<double> vec = {1.0, 2.0, 3.0};
  std::vector<double> result = {4.0, 5.0, 6.0};

  FromVector<Vec3>(vec, &v);
  EXPECT_EQ(vec[0], v.x);
  EXPECT_EQ(vec[1], v.y);
  EXPECT_EQ(vec[2], v.z);
  ToVector<Vec3>(v, &result);
  for (int32_t i = 0; i < 3; ++i) EXPECT_EQ(vec[i], result[i]);
}

}  // namespace signal
}  // namespace sim

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
