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

#include "lib/util/test_util.h"

#include <math.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <random>

#include "common/runfiles_dir.h"

DECLARE_string(runfiles_dir);

namespace test_util {

std::default_random_engine g_rng;
std::uniform_real_distribution<double> g_uniform_rand(0.0, 1.0);
std::normal_distribution<double> g_normal_rand(0.0, 1.0);
std::normal_distribution<float> g_normal_randf(0.0f, 1.0f);

double LpfIterationTolerance(double fc, double ts, int32_t n) {
  return 1.0 - pow(1.0 - ts / 2.0 / M_PI /fc, n);
}

double Rand() {
  return g_uniform_rand(g_rng);
}

double Rand(double min, double max) {
  std::uniform_real_distribution<double> uniform_rand(min, max);
  return uniform_rand(g_rng);
}

double RandNormal() {
  return g_normal_rand(g_rng);
}

float RandNormalf() {
  return g_normal_randf(g_rng);
}

Vec3 RandNormalVec3() {
  return {RandNormal(), RandNormal(), RandNormal()};
}

std::string TestRunfilesDir() {
  char *test_srcdir = getenv("TEST_SRCDIR");
  char *test_workspace = getenv("TEST_WORKSPACE");
  if (test_srcdir == nullptr) {
    LOG(FATAL) << "Environment variable TEST_SRCDIR is not set. Run this test "
               << "via 'bazel test', or set TEST_SRCDIR to the runfiles "
               << "directory for the test binary.";
  }
  if (test_workspace == nullptr) {
    LOG(FATAL) << "Environment variable TEST_WORKSPACE is not set. Run this "
               << "test via 'bazel test', or set TEST_WORKSPACE to the "
               << "workspace name";
  }
  return std::string(test_srcdir) + std::string("/") +
      std::string(test_workspace);
}

void SetRunfilesDir() {
  FLAGS_runfiles_dir = TestRunfilesDir();
}

}  // namespace test_util
