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

#ifndef LIB_UTIL_TEST_UTIL_H_
#define LIB_UTIL_TEST_UTIL_H_

#include <stdint.h>

#include <string>

#include "common/c_math/vec3.h"
#include "common/macros.h"

#define EXPECT_NEAR_RELATIVE(a, b, tol)         \
  do {                                          \
    EXPECT_TRUE(fabs(a) > 0.0);                 \
    EXPECT_NEAR(a, b, fabs(a) * tol);           \
  } while (0)

#define EXPECT_NEAR_RELATIVE_VEC3(a, b, tol)    \
  do {                                          \
    EXPECT_NEAR_RELATIVE((a).x, (b).x, tol);    \
    EXPECT_NEAR_RELATIVE((a).y, (b).y, tol);    \
    EXPECT_NEAR_RELATIVE((a).z, (b).z, tol);    \
  } while (0)

#define EXPECT_NEAR_VEC3(a, b, tol)             \
  do {                                          \
    EXPECT_NEAR((a).x, (b).x, tol);             \
    EXPECT_NEAR((a).y, (b).y, tol);             \
    EXPECT_NEAR((a).z, (b).z, tol);             \
  } while (0)

#define EXPECT_EQ_VEC3(a, b)                    \
  do {                                          \
    EXPECT_EQ((a).x, (b).x);                    \
    EXPECT_EQ((a).y, (b).y);                    \
    EXPECT_EQ((a).z, (b).z);                    \
  } while (0)

#define EXPECT_GT_VEC3(a, b)                    \
  do {                                          \
    EXPECT_GT((a).x, (b).x);                    \
    EXPECT_GT((a).y, (b).y);                    \
    EXPECT_GT((a).z, (b).z);                    \
  } while (0)

#define EXPECT_LT_VEC3(a, b)                    \
  do {                                          \
    EXPECT_LT((a).x, (b).x);                    \
    EXPECT_LT((a).y, (b).y);                    \
    EXPECT_LT((a).z, (b).z);                    \
  } while (0)

#define EXPECT_GE_VEC3(a, b)                    \
  do {                                          \
    EXPECT_GE((a).x, (b).x);                    \
    EXPECT_GE((a).y, (b).y);                    \
    EXPECT_GE((a).z, (b).z);                    \
  } while (0)

#define EXPECT_LE_VEC3(a, b)                    \
  do {                                          \
    EXPECT_LE((a).x, (b).x);                    \
    EXPECT_LE((a).y, (b).y);                    \
    EXPECT_LE((a).z, (b).z);                    \
  } while (0)

#define EXPECT_NEAR_VEC(a, b, tol)                              \
  do {                                                          \
    for (int32_t _i = 0; _i < (a).length; ++_i) {               \
      EXPECT_NEAR(VecGet(&(a), _i), VecGet(&(b), _i), tol);     \
    }                                                           \
  } while (0)

#define EXPECT_NEAR_MAT2(a, b, tol)                     \
  do {                                                  \
    for (int32_t _i = 0; _i < 2; ++_i) {                \
      for (int32_t _j = 0; _j < 2; ++_j) {              \
        EXPECT_NEAR((a).d[_i][_j], (b).d[_i][_j], tol); \
      }                                                 \
    }                                                   \
  } while (0)

#define EXPECT_NEAR_MAT3(a, b, tol)                     \
  do {                                                  \
    for (int32_t _i = 0; _i < 3; ++_i) {                \
      for (int32_t _j = 0; _j < 3; ++_j) {              \
        EXPECT_NEAR((a).d[_i][_j], (b).d[_i][_j], tol); \
      }                                                 \
    }                                                   \
  } while (0)

#define EXPECT_NEAR_MAT(a, b, tol)                                      \
  do {                                                                  \
    for (int32_t _i = 0; _i < (a).nr; ++_i) {                           \
      for (int32_t _j = 0; _j < (a).nc; ++_j) {                         \
        EXPECT_NEAR(MatGet(&(a), _i, _j), MatGet(&(b), _i, _j), tol);   \
      }                                                                 \
    }                                                                   \
  } while (0)

#define EXPECT_EQ_MAT(a, b)                                     \
  do {                                                          \
    for (int32_t _i = 0; _i < (a).nr; ++_i) {                   \
      for (int32_t _j = 0; _j < (a).nc; ++_j) {                 \
        EXPECT_EQ(MatGet(&(a), _i, _j), MatGet(&(b), _i, _j));  \
      }                                                         \
    }                                                           \
  } while (0)

#define EXPECT_NEAR_QUAT(a, b, tol)             \
  do {                                          \
    EXPECT_NEAR((a).q0, (b).q0, tol);           \
    EXPECT_NEAR((a).q1, (b).q1, tol);           \
    EXPECT_NEAR((a).q2, (b).q2, tol);           \
    EXPECT_NEAR((a).q3, (b).q3, tol);           \
  } while (0)

namespace test_util {

const int32_t kNumTests = 1000;

// Compares the input to Lpf (see common/c_math/filter.h) to its step response
// after a certain number of iterations.
//
// Computes the expected step response of Lpf given fixed filter coefficient
// time step and number of iterations.
//
// Args:
//   fc: Cutoff frequency [Hz] passed to Lpf.
//   ts: Timestep used in filter discretization.
//   n: Number of iterations Lpf was called.
double LpfIterationTolerance(double fc, double ts, int32_t n);

double Rand();
double Rand(double min, double max);
double RandNormal();
float RandNormalf();
Vec3 RandNormalVec3();

// Returns the runfiles directory for a unit test.
std::string TestRunfilesDir();

// Sets the runfiles directory as seen by non-test code to the value of
// TestRunfilesDir().
void SetRunfilesDir();

// Object that will override a value at a specific location, restoring the
// original value once the object goes out of scope. Useful for overriding
// global values for tests.
template <typename T>
class Overrider {
 public:
  Overrider(T *ref, T value) : ref_(ref), original_value_(*ref) {
    *ref = value;
  }

  virtual ~Overrider() { *ref_ = original_value_; }

 private:
  T *ref_;
  const T original_value_;

  DISALLOW_COPY_AND_ASSIGN(Overrider);
};

}  // namespace test_util

#endif  // LIB_UTIL_TEST_UTIL_H_
