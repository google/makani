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
#include <stdlib.h>

#include "sim/math/util.h"

TEST(NamedRandomNumberGeneratorTest, SameName) {
  NamedRandomNumberGenerator g1("foo");
  NamedRandomNumberGenerator g2("foo");
  for (int32_t i = 0; i < 10; ++i) {
    EXPECT_EQ(g1.GetUniformReal(), g2.GetUniformReal());
    EXPECT_EQ(g1.GetUniformInt32(), g2.GetUniformInt32());
    EXPECT_EQ(g1.GetNormal(), g2.GetNormal());
  }
}

TEST(NamedRandomNumberGeneratorTest, SameNames_DifferentOffsets) {
  NamedRandomNumberGenerator::SetSeedOffset(0U);
  NamedRandomNumberGenerator g1("foo");
  NamedRandomNumberGenerator::SetSeedOffset(1U);
  NamedRandomNumberGenerator g2("foo");
  for (int32_t i = 0; i < 10; ++i) {
    EXPECT_NE(g1.GetUniformReal(), g2.GetUniformReal());
    EXPECT_NE(g1.GetUniformInt32(), g2.GetUniformInt32());
    EXPECT_NE(g1.GetNormal(), g2.GetNormal());
  }
}

TEST(NamedRandomNumberGeneratorTest, DifferentNames) {
  NamedRandomNumberGenerator g1("foo");
  NamedRandomNumberGenerator g2("bar");
  for (int32_t i = 0; i < 10; ++i) {
    EXPECT_NE(g1.GetUniformReal(), g2.GetUniformReal());
    EXPECT_NE(g1.GetUniformInt32(), g2.GetUniformInt32());
    EXPECT_NE(g1.GetNormal(), g2.GetNormal());
  }
}

TEST(IntersectCircles, Equilateral) {
  double x, y;
  EXPECT_TRUE(IntersectCircles(1.0, 1.0, 1.0, &x, &y));
  EXPECT_NEAR(0.5, x, DBL_EPSILON);
  EXPECT_NEAR(sqrt(3.0) / 2.0, y, DBL_EPSILON);
}

TEST(IntersectCircles, ThreeFourFive) {
  double x, y;
  EXPECT_TRUE(IntersectCircles(5.0, 3.0, 4.0, &x, &y));
  EXPECT_NEAR(3.0 / 5.0 * 3.0, x, 2.0 * DBL_EPSILON);
  EXPECT_NEAR(4.0 / 5.0 * 3.0, y, 2.0 * DBL_EPSILON);

  EXPECT_TRUE(IntersectCircles(5.0, 4.0, 3.0, &x, &y));
  EXPECT_NEAR(5.0 - 3.0 / 5.0 * 3.0, x, 2.0 * DBL_EPSILON);
  EXPECT_NEAR(4.0 / 5.0 * 3.0, y, 2.0 * DBL_EPSILON);
}

TEST(IntersectCircles, NoIntersection) {
  double x, y;
  EXPECT_FALSE(IntersectCircles(3.0, 1.0, 1.0, &x, &y));
}

TEST(IntersectCircles, NoIntersectionSameCenter) {
  double x, y;
  EXPECT_FALSE(IntersectCircles(0.0, 1.0, 2.0, &x, &y));
}

TEST(IntersectCircles, SameCenterSameRadius) {
  double x, y;
  EXPECT_TRUE(IntersectCircles(0.0, 1.0, 1.0, &x, &y));
  EXPECT_NEAR(1.0, x, DBL_EPSILON);
  EXPECT_NEAR(0.0, y, DBL_EPSILON);
}

TEST(IntersectCircles, PointIntersection) {
  double x, y;
  EXPECT_TRUE(IntersectCircles(2.0, 1.0, 1.0, &x, &y));
  EXPECT_NEAR(1.0, x, DBL_EPSILON);
  EXPECT_NEAR(0.0, y, DBL_EPSILON);
}

TEST(gsl_interp1Test, Interp_0) {
  const uint32_t veclen = 22;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen);
  gsl_vector *const vy = gsl_vector_alloc(veclen);
  for (uint32_t i = 0; i < veclen; i++) {
    gsl_vector_set(vx, i, i);
    gsl_vector_set(vy, i, i * i);
  }
  // interpolation tests
  for (uint32_t i = 0; i < veclen; i++) {
    a = gsl_interp1(vx, vy, i, kInterpOptionDefault);
    EXPECT_EQ(a, i * i);
  }
  gsl_vector_free(vx);
  gsl_vector_free(vy);
}

TEST(gsl_interp1Test, Extrap_0) {
  const uint32_t veclen = 22;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen);
  gsl_vector *const vy = gsl_vector_alloc(veclen);
  for (uint32_t i = 0; i < veclen; i++) {
    gsl_vector_set(vx, i, i);
    gsl_vector_set(vy, i, i * i);
  }
  // extrapolation tests (linear)
  a = gsl_interp1(vx, vy, -1, kInterpOptionDefault);
  EXPECT_EQ(a, -1);
  a = gsl_interp1(vx, vy, veclen, kInterpOptionDefault);
  EXPECT_EQ(a, ((veclen - 1) * (veclen - 1) - (veclen - 2) * (veclen - 2) +
                (veclen - 1) * (veclen - 1)));
  gsl_vector_free(vx);
  gsl_vector_free(vy);
}

TEST(gsl_interp1Test, Extrap_1) {
  const uint32_t veclen = 22;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen);
  gsl_vector *const vy = gsl_vector_alloc(veclen);
  for (uint32_t i = 0; i < veclen; i++) {
    gsl_vector_set(vx, i, i);
    gsl_vector_set(vy, i, i * i);
  }
  // extrapolation tests (saturated)
  a = gsl_interp1(vx, vy, -1, kInterpOptionSaturate);
  EXPECT_EQ(a, 0);
  a = gsl_interp1(vx, vy, veclen, kInterpOptionSaturate);
  EXPECT_EQ(a, (veclen - 1) * (veclen - 1));
  gsl_vector_free(vx);
  gsl_vector_free(vy);
}

TEST(gsl_interp2Test, Interp_0) {
  const uint32_t veclen = 22;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen);
  gsl_vector *const vy = gsl_vector_alloc(veclen);
  gsl_matrix *const z = gsl_matrix_alloc(veclen, veclen);
  for (uint32_t i = 0; i < veclen; i++) {
    gsl_vector_set(vx, i, i);
    gsl_vector_set(vy, i, i);
  }
  for (uint32_t i = 0; i < veclen; i++)
    for (uint32_t j = 0; j < veclen; j++)
      gsl_matrix_set(z, i, j, (i - 22.2) * (j + 2.2));

  // interpolation tests
  for (uint32_t i = 0; i < veclen; i++)
    for (uint32_t j = 0; j < veclen; j++) {
      a = gsl_interp2(vx, vy, z, i, j, kInterpOptionDefault);
      EXPECT_EQ(a, (i - 22.2) * (j + 2.2));
    }
  a = gsl_interp2(vx, vy, z, 1.2, 3.4, kInterpOptionDefault);
  // Calculated with MATLAB: interp2(vx, vy, z, 3.4, 1.2, 'linear')
  // Remember that the fields are orderd col, row in the MATLAB function
  EXPECT_NEAR(a, -117.6, 1.0e-9);
  gsl_vector_free(vx);
  gsl_vector_free(vy);
  gsl_matrix_free(z);
}

TEST(gsl_interp2Test, Extrap_0) {
  const uint32_t veclen = 22;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen);
  gsl_vector *const vy = gsl_vector_alloc(veclen);
  gsl_matrix *const z = gsl_matrix_alloc(veclen, veclen);
  for (uint32_t i = 0; i < veclen; i++)
    for (uint32_t j = 0; j < veclen; j++) {
      gsl_vector_set(vx, i, i);
      gsl_vector_set(vy, j, j);
      gsl_matrix_set(z, i, j, i * j);
    }
  // Extrapolation tests
  a = gsl_interp2(vx, vy, z, -1, 0, kInterpOptionDefault);
  EXPECT_EQ(a, 0);
  a = gsl_interp2(vx, vy, z, 0, -1, kInterpOptionDefault);
  EXPECT_EQ(a, 0);
  a = gsl_interp2(vx, vy, z, -1, -1, kInterpOptionDefault);
  EXPECT_EQ(a, 1);

  // hard-coded values looked up in Octave
  a = gsl_interp2(vx, vy, z, veclen, veclen - 1, kInterpOptionDefault);
  EXPECT_EQ(a, 462);
  a = gsl_interp2(vx, vy, z, veclen - 1, veclen, kInterpOptionDefault);
  EXPECT_EQ(a, 462);
  a = gsl_interp2(vx, vy, z, veclen, veclen, kInterpOptionDefault);
  EXPECT_EQ(a, 484);
  gsl_vector_free(vx);
  gsl_vector_free(vy);
  gsl_matrix_free(z);
}

TEST(gsl_interp2Test, Extrap_1) {
  const uint32_t veclen = 22;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen);
  gsl_vector *const vy = gsl_vector_alloc(veclen);
  gsl_matrix *const z = gsl_matrix_alloc(veclen, veclen);
  for (uint32_t i = 0; i < veclen; i++)
    for (uint32_t j = 0; j < veclen; j++) {
      gsl_vector_set(vx, i, i);
      gsl_vector_set(vy, j, j);
      gsl_matrix_set(z, i, j, i * j);
    }
  // Extrapolation tests (saturated)
  a = gsl_interp2(vx, vy, z, -1, 0, kInterpOptionSaturate);
  EXPECT_EQ(a, 0);
  a = gsl_interp2(vx, vy, z, 0, -1, kInterpOptionSaturate);
  EXPECT_EQ(a, 0);
  a = gsl_interp2(vx, vy, z, -1, -1, kInterpOptionSaturate);
  EXPECT_EQ(a, 0);

  a = gsl_interp2(vx, vy, z, veclen, veclen - 1, kInterpOptionSaturate);
  EXPECT_EQ(a, (veclen - 1) * (veclen - 1));
  a = gsl_interp2(vx, vy, z, veclen - 1, veclen, kInterpOptionSaturate);
  EXPECT_EQ(a, (veclen - 1) * (veclen - 1));
  a = gsl_interp2(vx, vy, z, veclen, veclen, kInterpOptionSaturate);
  EXPECT_EQ(a, (veclen - 1) * (veclen - 1));
  gsl_vector_free(vx);
  gsl_vector_free(vy);
  gsl_matrix_free(z);
}

TEST(gsl_interp3Test, Interp_0) {
  const uint32_t veclen_x = 22;
  const uint32_t veclen_y = 23;
  const uint32_t veclen_z = 24;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen_x);
  gsl_vector *const vy = gsl_vector_alloc(veclen_y);
  gsl_vector *const vz = gsl_vector_alloc(veclen_z);
  gsl_vector *const mat = gsl_vector_alloc(veclen_x * veclen_y * veclen_z);
  for (uint32_t i = 0; i < veclen_x; i++) gsl_vector_set(vx, i, i);
  for (uint32_t i = 0; i < veclen_y; i++) gsl_vector_set(vy, i, i);
  for (uint32_t i = 0; i < veclen_z; i++) gsl_vector_set(vz, i, i);

  for (uint32_t i = 0; i < veclen_x; i++)
    for (uint32_t j = 0; j < veclen_y; j++)
      for (uint32_t k = 0; k < veclen_z; k++)
        gsl_vector_set(mat, i * veclen_y * veclen_z + j * veclen_z + k,
                       (i - 22.2) * (j * 2.3) * (k + 2.2));
  // interpolation tests
  for (uint32_t i = 0; i < veclen_x; i++)
    for (uint32_t j = 0; j < veclen_y; j++)
      for (uint32_t k = 0; k < veclen_z; k++) {
        a = gsl_interp3(vx, vy, vz, mat, i, j, k, kInterpOptionDefault);
        EXPECT_NEAR(a, (i - 22.2) * (j * 2.3) * (k + 2.2), 1.0e-9);
      }
  a = gsl_interp3(vx, vy, vz, mat, 1.2, 3.4, 5.6, kInterpOptionDefault);
  // Calculated with MATLAB: interp3(vx, vy, vz, mat, 3.4, 1.2, 5.6)
  // Remember that the fields are orderd col, row in the MATLAB function
  EXPECT_NEAR(a, -1280.916, 1.0e-9);
  gsl_vector_free(vx);
  gsl_vector_free(vy);
  gsl_vector_free(vz);
  gsl_vector_free(mat);
}

TEST(gsl_interp3Test, Extrap_0) {
  const uint32_t veclen_x = 22;
  const uint32_t veclen_y = 23;
  const uint32_t veclen_z = 24;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen_x);
  gsl_vector *const vy = gsl_vector_alloc(veclen_y);
  gsl_vector *const vz = gsl_vector_alloc(veclen_z);
  gsl_vector *const mat = gsl_vector_alloc(veclen_x * veclen_y * veclen_z);
  for (uint32_t i = 0; i < veclen_x; i++) gsl_vector_set(vx, i, i);
  for (uint32_t i = 0; i < veclen_y; i++) gsl_vector_set(vy, i, i);
  for (uint32_t i = 0; i < veclen_z; i++) gsl_vector_set(vz, i, i);

  for (uint32_t i = 0; i < veclen_x; i++)
    for (uint32_t j = 0; j < veclen_y; j++)
      for (uint32_t k = 0; k < veclen_z; k++)
        gsl_vector_set(mat, i * veclen_y * veclen_z + j * veclen_z + k,
                       (i - 22.2) * (j * 2.3) * (k + 2.2));
  // Extrapolation tests
  // Looked up in MATLAB: interp3(vy, vx, vz, mat, yi, xi, zi)
  a = gsl_interp3(vx, vy, vz, mat, -1, 0, 0, kInterpOptionDefault);
  EXPECT_NEAR(a, 0.0, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, 0, -1, 0, kInterpOptionDefault);
  EXPECT_NEAR(a, 112.332, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, 0, 0, -1, kInterpOptionDefault);
  EXPECT_NEAR(a, 0.0, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, -1, -1, -1, kInterpOptionDefault);
  EXPECT_NEAR(a, 64.032, 1.0e-9);

  a = gsl_interp3(vx, vy, vz, mat, veclen_x, veclen_y - 1, veclen_z - 1,
                  kInterpOptionDefault);
  EXPECT_NEAR(a, -255.024, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, veclen_x - 1, veclen_y, veclen_z - 1,
                  kInterpOptionDefault);
  EXPECT_NEAR(a, -1599.696, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, veclen_x - 1, veclen_y - 1, veclen_z,
                  kInterpOptionDefault);
  EXPECT_NEAR(a, -1590.864, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, veclen_x, veclen_y, veclen_z,
                  kInterpOptionDefault);
  EXPECT_NEAR(a, -277.196, 1.0e-9);
  gsl_vector_free(vx);
  gsl_vector_free(vy);
  gsl_vector_free(vz);
  gsl_vector_free(mat);
}

TEST(gsl_interp3Test, Extrap_1) {
  const uint32_t veclen_x = 22;
  const uint32_t veclen_y = 23;
  const uint32_t veclen_z = 24;
  double a;
  gsl_vector *const vx = gsl_vector_alloc(veclen_x);
  gsl_vector *const vy = gsl_vector_alloc(veclen_y);
  gsl_vector *const vz = gsl_vector_alloc(veclen_z);
  gsl_vector *const mat = gsl_vector_alloc(veclen_x * veclen_y * veclen_z);
  for (uint32_t i = 0; i < veclen_x; i++) gsl_vector_set(vx, i, i);
  for (uint32_t i = 0; i < veclen_y; i++) gsl_vector_set(vy, i, i);
  for (uint32_t i = 0; i < veclen_z; i++) gsl_vector_set(vz, i, i);

  for (uint32_t i = 0; i < veclen_x; i++)
    for (uint32_t j = 0; j < veclen_y; j++)
      for (uint32_t k = 0; k < veclen_z; k++)
        gsl_vector_set(mat, i * veclen_y * veclen_z + j * veclen_z + k,
                       (i - 22.2) * (j * 2.3) * (k + 2.2));
  // Extrapolation tests
  // Looked up in MATLAB: interp3(vy, vx, vz, mat, yi, xi, zi)
  a = gsl_interp3(vx, vy, vz, mat, -1, 0, 0, kInterpOptionSaturate);
  EXPECT_NEAR(a, 0.0, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, 0, -1, 0, kInterpOptionSaturate);
  EXPECT_NEAR(a, 0.0, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, 0, 0, -1, kInterpOptionSaturate);
  EXPECT_NEAR(a, 0.0, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, -1, -1, -1, kInterpOptionSaturate);
  EXPECT_NEAR(a, 0.0, 1.0e-9);

  a = gsl_interp3(vx, vy, vz, mat, veclen_x, veclen_y - 1, veclen_z - 1,
                  kInterpOptionSaturate);
  EXPECT_NEAR(a, -1530.144, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, veclen_x - 1, veclen_y, veclen_z - 1,
                  kInterpOptionSaturate);
  EXPECT_NEAR(a, -1530.144, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, veclen_x - 1, veclen_y - 1, veclen_z,
                  kInterpOptionSaturate);
  EXPECT_NEAR(a, -1530.144, 1.0e-9);
  a = gsl_interp3(vx, vy, vz, mat, veclen_x, veclen_y, veclen_z,
                  kInterpOptionSaturate);
  EXPECT_NEAR(a, -1530.144, 1.0e-9);
  gsl_vector_free(vx);
  gsl_vector_free(vy);
  gsl_vector_free(vz);
  gsl_vector_free(mat);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
