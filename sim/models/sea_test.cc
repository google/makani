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

#include "sim/models/sea.h"
#include <gtest/gtest.h>
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "sim/models/environment.h"
#include "sim/sim_params.h"

using ::test_util::RandNormal;
using ::test_util::Rand;

class SeaTest : public ::testing::Test {
 protected:
  const SystemParams &system_params_;
  const SimParams &sim_params_;
  std::unique_ptr<Environment> environment_;
  std::unique_ptr<GroundFrame> ground_frame_;

  std::unique_ptr<Sea> sea_;

  SeaTest();
  ~SeaTest() {}
  double *GetSampledFrequencies() { return sea_->w_i; }
  double *GetSampledAmplitudes() { return sea_->H_i; }
  double *GetSampledWaveNumbers() { return sea_->c_i; }
};

SeaTest::SeaTest()
    : system_params_(*GetSystemParams()),
      sim_params_(*GetSimParams()),
      environment_(nullptr),
      ground_frame_(nullptr),
      sea_(nullptr) {
  environment_.reset(new Environment(
      sim_params_.iec_sim, sim_params_.phys_sim, system_params_.phys,
      system_params_.wind_sensor, system_params_.ground_frame));
  ground_frame_.reset(new GroundFrame(environment_->ned_frame(),
                                      system_params_.ground_frame,
                                      sim_params_.ground_frame_sim));
  sea_.reset(new Sea(*environment_, *ground_frame_, sim_params_.sea_sim,
                     sim_params_.buoy_sim.msl_pos_z_g));
}

// Test that the initial and final sampled frequencies of the spectrum are
// close to the requested values.
TEST_F(SeaTest, InitialAndFinalFrequencyTest) {
  int N = sim_params_.sea_sim.number_of_waves;
  double *w = GetSampledFrequencies();
  EXPECT_NEAR(w[0], sim_params_.sea_sim.initial_frequency, 0.01);
  EXPECT_NEAR(w[N - 1], sim_params_.sea_sim.cutoff_frequency, 0.01);
}

// Test that the majority of the spectrum is being sampled between
// initial_frequency and cutoff_frequency. For this, we assume the initial and
// final amplitudes are at a maximum 1/50 of the significant height.
TEST_F(SeaTest, InitialAndFinalAmplitudeTest) {
  int N = sim_params_.sea_sim.number_of_waves;
  double *H = GetSampledAmplitudes();
  EXPECT_LT(H[0], sim_params_.sea_sim.significant_height / 50.0);
  EXPECT_LT(H[N - 1], sim_params_.sea_sim.significant_height / 50.0);
}

// Test the infinite water depth approximation. For details, see
// docs.google.com/document/d/10--gfN41OCA1k3tjxwAdSkv7BPQm0VIaORuc5npkUWk
TEST_F(SeaTest, InfiniteWaterDepthTest) {
  double *w = GetSampledFrequencies();
  double water_depth = PI * environment_->g() / Square(w[0]);
  EXPECT_LT(water_depth, sim_params_.sea_sim.water_depth);
}

// Test that the wave height is not too high. Significant height is defined as
// the mean of the highest 1/3 of the wave heights. Wave height is measured
// crest to trough, so wave height is twice the wave amplitude. We check here
// that the amplitude is less than the significant height plus 10% margin.
TEST_F(SeaTest, SeaElevationVsSignificantHeightTest) {
  double wave_elev_g, wave_height;
  for (int i = 0; i < 50; i++) {
    wave_elev_g = sea_->GetSeaElevation(Rand(0., 60.), 50. * RandNormal(),
                                        50. * RandNormal());
    wave_height = sea_->GetMeanSeaLevelPosZ() - wave_elev_g;
    EXPECT_LT(fabs(wave_height), 1.1 * sim_params_.sea_sim.significant_height);
  }
}

// Test that the wave particle velocity and acceleration above the wave are set
// to zero.
TEST_F(SeaTest, WaveKinematicsAboveWaveTest) {
  Vec3 pos_g, vel_g, accel_g;
  double wave_elev_g;
  double time;
  for (int i = 0; i < 50; i++) {
    time = Rand(0., 60.);
    pos_g.x = 50. * RandNormal();
    pos_g.y = 50. * RandNormal();
    pos_g.z = 50. * RandNormal();
    sea_->GetWaveKinematics(time, &pos_g, &vel_g, &accel_g);
    wave_elev_g = sea_->GetSeaElevation(time, pos_g.x, pos_g.y);
    if (wave_elev_g > pos_g.z) {
      EXPECT_EQ_VEC3(vel_g, kVec3Zero);
      EXPECT_EQ_VEC3(accel_g, kVec3Zero);
    }
  }
}

// Test that the wave equation is correct:
//
//   w^2 = g*c*tanh(c*wd)
//
// Where:
//   w is the wave component frequency [rad/s].
//   g is the gravitational acceleration [m/s^2].
//   c is the wave component wavenumber [1/m].
//   wd is the water depth [m].
//
// We test w^2 >= g*|c|, which works for both the finite and infinite water
// depth models.
TEST_F(SeaTest, WaveNumberEquationTest) {
  int N = sim_params_.sea_sim.number_of_waves;
  double *c = GetSampledWaveNumbers();
  double *w = GetSampledFrequencies();
  for (int i = 0; i < N; i++) {
    EXPECT_GE(Square(w[i]), environment_->g() * fabs(c[i]) - 1e-6);
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
