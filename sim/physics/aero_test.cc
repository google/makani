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

#include <glog/logging.h>
#include <math.h>
#include <memory>

#include "common/c_math/linalg.h"
#include "common/c_math/util.h"
#include "common/c_math/vec3.h"
#include "control/system_params.h"
#include "lib/util/test_util.h"
#include "sim/physics/aero.h"
#include "sim/physics/aero_frame.h"
#include "sim/sim_params.h"
#include "sim/sim_types.h"

using ::test_util::RandNormalVec3;

class AeroTest : public ::testing::Test {
 public:
  AeroTest()
      : params_(GetSimParams()->aero_sim),
        aero_(),
        reynolds_number_(),
        alphas_(),
        betas_(),
        omega_hat_0_() {
    test_util::SetRunfilesDir();

    OverrideAeroParams(false, &params_);
    aero_.reset(new Aero(params_));
    reynolds_number_ = aero_->small_deflection_databases_[0]->reynolds_number();
    alphas_.resize(aero_->small_deflection_databases_[0]->alphads_->size);
    for (uint32_t i = 0U; i < alphas_.size(); ++i) {
      alphas_[i] =
          gsl_vector_get(aero_->small_deflection_databases_[0]->alphads_, i) *
          PI / 180.0;
    }
    betas_.resize(aero_->small_deflection_databases_[0]->betads_->size);
    for (uint32_t i = 0U; i < betas_.size(); ++i) {
      betas_[i] =
          gsl_vector_get(aero_->small_deflection_databases_[0]->betads_, i) *
          PI / 180.0;
    }
    omega_hat_0_ = aero_->small_deflection_databases_[0]->omega_hat_0_;
  }

 protected:
  AeroSimParams params_;

  std::unique_ptr<Aero> aero_;
  double reynolds_number_;
  std::vector<double> alphas_, betas_;
  Vec3 omega_hat_0_;

  void OverrideAeroParams(bool use_spoilers, AeroSimParams *params) const;
  void CheckSpoilerPhysics() const;
  void CheckSpoilerSymmetry() const;
  void CheckZeroSpoilers() const;
  void CheckFullSpoilers() const;
  void CheckInterpSpoilers() const;
  void CheckCoeffOffsets() const;
  void CheckFlapOffsets() const;
  void CheckBasicScaling() const;
  void CheckRateAndFlapScaling() const;
};

void AeroTest::OverrideAeroParams(bool use_spoilers,
                                  AeroSimParams *params) const {
  params->use_spoilers = use_spoilers;
  // Increase stall angle limits so we are just testing the low
  // incidence angle database.
  params->low_alpha_stall_angle = -M_PI / 2.0;
  params->high_alpha_stall_angle = M_PI / 2.0;
  params->low_beta_stall_angle = -M_PI / 2.0;
  params->high_beta_stall_angle = M_PI / 2.0;
}

void AeroTest::CheckZeroSpoilers() const {
  AeroSimParams modified_params = GetSimParams()->aero_sim;
  OverrideAeroParams(true, &modified_params);
  Aero modified_aero(modified_params);
  VEC_INIT(kNumFlaps, zero_flaps, {0});
  zero_flaps.d[2] = zero_flaps.d[3] = 0.0 / 180.0 * M_PI;
  double re = 1e6;
  double thrust_coeff = 0.0;
  for (double alpha : alphas_) {
    for (double beta : betas_) {
      ForceMoment cfm;
      ForceMoment modified_cfm;

      aero_->CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps, re, &cfm,
                                  thrust_coeff);
      modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps, re,
                                         &modified_cfm, thrust_coeff);
      EXPECT_DOUBLE_EQ(cfm.force.x, modified_cfm.force.x);
    }
  }
}

/* Check that the spoilers do not add propulsive drag */
/* Check that the spoilers do not add 25% more lift */
void AeroTest::CheckSpoilerPhysics() const {
  AeroSimParams modified_params = GetSimParams()->aero_sim;
  OverrideAeroParams(true, &modified_params);
  Aero modified_aero(modified_params);

  VEC_INIT(kNumFlaps, flaps, {0.0});
  double re = 1e6;
  double thrust_coeff = 0.0;
  for (double beta : betas_) {
    for (double alpha : alphas_) {
      for (double delta = -20.0; delta >= -100.0; delta -= 20.0) {
        double delta_rad = delta * M_PI / 180.0;
        flaps.d[kFlapA4] = flaps.d[kFlapA5] = delta_rad;

        ForceMoment cfm_b, modified_cfm_b;
        Vec3 force_w, modified_force_w;
        aero_->CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps, re, &cfm_b,
                                    -thrust_coeff);
        RotBToW(&cfm_b.force, alpha, beta, &force_w);
        modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps, re,
                                           &modified_cfm_b, thrust_coeff);
        RotBToW(&modified_cfm_b.force, alpha, beta, &modified_force_w);

        // Make sure the spoilers add more drag.
        EXPECT_LE(-force_w.x, -modified_force_w.x);
        // Make sure the spoilers do not introduce too much lift.
        EXPECT_GE(-force_w.z * 1.25, -modified_force_w.z);
      }
    }
  }
}

/* Check that the spoilers do not add side force, roll, and yaw moments */
void AeroTest::CheckSpoilerSymmetry() const {
  AeroSimParams modified_params = GetSimParams()->aero_sim;
  OverrideAeroParams(true, &modified_params);
  Aero modified_aero(modified_params);
  VEC_INIT(kNumFlaps, flaps, {0.0});
  flaps.d[2] = flaps.d[3] = -60.0 / 180.0 * M_PI;
  double re = 1e6;
  double thrust_coeff = 0.0;
  for (double alpha : alphas_) {
    for (double beta : betas_) {
      ForceMoment cfm;
      ForceMoment modified_cfm;

      aero_->CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps, re, &cfm,
                                  thrust_coeff);
      modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps, re,
                                         &modified_cfm, thrust_coeff);
      EXPECT_DOUBLE_EQ(cfm.force.y, modified_cfm.force.y);
      EXPECT_DOUBLE_EQ(cfm.moment.x, modified_cfm.moment.x);
      EXPECT_DOUBLE_EQ(cfm.moment.z, modified_cfm.moment.z);
    }
  }
}

/* Check that there is more drag when spoilers are deployed */
void AeroTest::CheckFullSpoilers() const {
  AeroSimParams modified_params = GetSimParams()->aero_sim;
  OverrideAeroParams(true, &modified_params);
  Aero modified_aero(modified_params);
  const double full_delta = -100.0 / 180.0 * M_PI;
  VEC_INIT(kNumFlaps, full_flaps,
           {full_delta, full_delta, full_delta, full_delta, full_delta,
            full_delta, full_delta, full_delta});

  double re = 1e6;
  double thrust_coeff = 0.0;
  double alpha = 0.0;
  for (double beta : betas_) {
    ForceMoment cfm;
    ForceMoment modified_cfm;

    aero_->CalcForceMomentCoeff(alpha, beta, kVec3Zero, full_flaps, re, &cfm,
                                thrust_coeff);
    modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, full_flaps, re,
                                       &modified_cfm, thrust_coeff);

    Vec3 force_w, modified_force_w;
    RotBToW(&cfm.force, alpha, beta, &force_w);
    RotBToW(&modified_cfm.force, alpha, beta, &modified_force_w);

    EXPECT_GE(force_w.x, modified_force_w.x);
  }
}

void AeroTest::CheckInterpSpoilers() const {
  AeroSimParams modified_params = GetSimParams()->aero_sim;
  OverrideAeroParams(true, &modified_params);
  Aero modified_aero(modified_params);

  double re = 1e6;
  double thrust_coeff = 0.0;
  double beta = 0.0;
  double da = 0.01;
  double deg2rad = M_PI / 180.0;
  const double full_delta = -80.0 * deg2rad;

  // Test the database's interpolation capability.
  // Test interpolation across various alpha.
  for (double alpha = -10.0 * deg2rad; alpha <= 10.0 * deg2rad;
       alpha += 2.0 * deg2rad) {
    double flap = -10.0 * deg2rad;
    ForceMoment cfm_z1, cfm, cfm_n1;
    VEC_INIT(kNumFlaps, flaps,
             {flap, flap, flap, flap, flap, flap, flap, flap});
    modified_aero.CalcForceMomentCoeff(alpha - da, beta, kVec3Zero, flaps, re,
                                       &cfm_z1, thrust_coeff);

    modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps, re, &cfm,
                                       thrust_coeff);

    modified_aero.CalcForceMomentCoeff(alpha + da, beta, kVec3Zero, flaps, re,
                                       &cfm_n1, thrust_coeff);

    EXPECT_NE(cfm.force.x, cfm_z1.force.x);
    EXPECT_NE(cfm.force.x, cfm_n1.force.x);
  }

  // Test interpolation across various delta.
  for (double flap = full_delta; flap <= -10.0 * deg2rad;
       flap += 20.0 * deg2rad) {
    double alpha = 3.0 * deg2rad;
    double flap_z1 = flap - deg2rad;
    double flap_n1 = flap + deg2rad;
    VEC_INIT(kNumFlaps, flaps_z1,
             {flap, flap, flap_z1, flap_z1, flap, flap, flap, flap});
    VEC_INIT(kNumFlaps, flaps,
             {flap, flap, flap, flap, flap, flap, flap, flap});
    VEC_INIT(kNumFlaps, flaps_n1,
             {flap, flap, flap_n1, flap_n1, flap, flap, flap, flap});

    ForceMoment cfm_z1, cfm, cfm_n1;
    modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps_z1, re,
                                       &cfm_z1, thrust_coeff);
    modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps, re, &cfm,
                                       thrust_coeff);
    modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps_n1, re,
                                       &cfm_n1, thrust_coeff);

    EXPECT_LT((cfm.force.x - cfm_z1.force.x) * (cfm.force.x - cfm_n1.force.x),
              0.0);
  }
}

void AeroTest::CheckCoeffOffsets() const {
  Vec3 force_coeff_offsets = {0.1, 0.12, 0.27};
  Vec3 moment_coeff_offsets = {0.01, 0.02, 0.03};
  Vec3 slopes = {0.3, 0.1, 0.7};
  AeroSimParams modified_params = GetSimParams()->aero_sim;
  OverrideAeroParams(false, &modified_params);

  modified_params.coeff_offsets.CD -= force_coeff_offsets.x;
  modified_params.coeff_offsets.CC -= force_coeff_offsets.y;
  modified_params.coeff_offsets.CL -= force_coeff_offsets.z;
  modified_params.coeff_offsets.Cl += moment_coeff_offsets.x;
  modified_params.coeff_offsets.Cm += moment_coeff_offsets.y;
  modified_params.coeff_offsets.Cn += moment_coeff_offsets.z;
  modified_params.coeff_offsets.dCldbeta += slopes.x;
  modified_params.coeff_offsets.dCmdalpha += slopes.y;
  modified_params.coeff_offsets.dCndbeta += slopes.z;
  Aero modified_aero(modified_params);

  VEC_INIT(kNumFlaps, zero_flaps, {0});
  double re = 1e6;
  double thrust_coeff = 0.0;
  for (double alpha : alphas_) {
    for (double beta : betas_) {
      ForceMoment cfm;
      aero_->CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps, re, &cfm,
                                  thrust_coeff);

      ForceMoment modified_cfm;
      modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps, re,
                                         &modified_cfm, thrust_coeff);

      Vec3 force_w, modified_force_w;
      RotBToW(&cfm.force, alpha, beta, &force_w);
      RotBToW(&modified_cfm.force, alpha, beta, &modified_force_w);

      EXPECT_NEAR(cfm.moment.x + slopes.x * beta + moment_coeff_offsets.x,
                  modified_cfm.moment.x, DBL_TOL);
      EXPECT_NEAR(cfm.moment.y + slopes.y * alpha + moment_coeff_offsets.y,
                  modified_cfm.moment.y, DBL_TOL);
      EXPECT_NEAR(cfm.moment.z + slopes.z * beta + moment_coeff_offsets.z,
                  modified_cfm.moment.z, DBL_TOL);
      EXPECT_NEAR(force_w.x + force_coeff_offsets.x, modified_force_w.x,
                  DBL_TOL);
      EXPECT_NEAR(force_w.y + force_coeff_offsets.y, modified_force_w.y,
                  DBL_TOL);
      EXPECT_NEAR(force_w.z + force_coeff_offsets.z, modified_force_w.z,
                  DBL_TOL);
    }
  }
}

void AeroTest::CheckFlapOffsets() const {
  AeroSimParams modified_params = GetSimParams()->aero_sim;

  double flap_offsets[] = {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08};
  EXPECT_EQ(ARRAYSIZE(flap_offsets), kNumFlaps);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    modified_params.flap_offsets[i] += flap_offsets[i];
  }
  OverrideAeroParams(false, &modified_params);

  Aero modified_aero(modified_params);

  VEC_INIT(kNumFlaps, zero_flaps, {0});
  VEC_CLONE(kNumFlaps, offset_flaps, &flap_offsets[0]);
  double re = 1e6;
  double thrust_coeff = 0.0;
  for (double alpha : alphas_) {
    for (double beta : betas_) {
      ForceMoment cfm;
      aero_->CalcForceMomentCoeff(alpha, beta, kVec3Zero, offset_flaps, re,
                                  &cfm, thrust_coeff);

      ForceMoment modified_cfm;
      modified_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps, re,
                                         &modified_cfm, thrust_coeff);

      EXPECT_NEAR_VEC3(cfm.force, modified_cfm.force, DBL_TOL);
      EXPECT_NEAR_VEC3(cfm.moment, modified_cfm.moment, DBL_TOL);
    }
  }
}

namespace {

void RotateScalingMatrix(const Vec3 &scale, const Mat3 &dcm, Mat3 *mat) {
  *mat = kMat3Zero;
  mat->d[0][0] = scale.x;
  mat->d[1][1] = scale.y;
  mat->d[2][2] = scale.z;

  Mat3Mult(&dcm, kNoTrans, Mat3Mult(mat, kNoTrans, &dcm, kTrans, mat), kNoTrans,
           mat);
}

}  // namespace

void AeroTest::CheckBasicScaling() const {
  AeroSimParams unscaled_params = GetSimParams()->aero_sim;
  OverrideAeroParams(false, &unscaled_params);

  // Zero other coefficients to avoid flap and rate offsets.
  unscaled_params.coeff_offsets = AeroCoeffOffsets();
  unscaled_params.force_coeff_w_scale_factors = AeroCoeffs();
  unscaled_params.moment_coeff_b_scale_factors = AeroCoeffs();
  unscaled_params.force_coeff_w_scale_factors.coeff = kVec3Ones;
  unscaled_params.moment_coeff_b_scale_factors.coeff = kVec3Ones;
  Aero unscaled_aero(unscaled_params);

  AeroSimParams scaled_params = unscaled_params;
  scaled_params.force_coeff_w_scale_factors.coeff = RandNormalVec3();
  scaled_params.moment_coeff_b_scale_factors.coeff = RandNormalVec3();
  Aero scaled_aero(scaled_params);

  VEC_INIT(kNumFlaps, zero_flaps, {0});
  double re = 1e6;
  double thrust_coeff = 0.0;
  for (double alpha : alphas_) {
    for (double beta : betas_) {
      ForceMoment unscaled_cfm;
      ForceMoment scaled_cfm;
      unscaled_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps, re,
                                         &unscaled_cfm, thrust_coeff);
      scaled_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps, re,
                                       &scaled_cfm, thrust_coeff);

      // Check force and moment scaling.
      Mat3 dcm_w2b;
      Mat3 scale_b;
      CalcDcmWToB(alpha, beta, &dcm_w2b);
      RotateScalingMatrix(scaled_params.force_coeff_w_scale_factors.coeff,
                          dcm_w2b, &scale_b);
      Vec3 answer;
      Mat3Vec3Mult(&scale_b, &unscaled_cfm.force, &answer);
      EXPECT_NEAR_VEC3(answer, scaled_cfm.force, DBL_TOL);

      Vec3Mult(&scaled_params.moment_coeff_b_scale_factors.coeff,
               &unscaled_cfm.moment, &answer);
      EXPECT_NEAR_VEC3(answer, scaled_cfm.moment, DBL_TOL);
    }
  }
}

namespace {

void CompareRotatedScaledDifference(
    const Mat3 &dcm_w2b, const ForceMoment &cfm_0, const ForceMoment &cfm,
    const ForceMoment &scaled_cfm_0, const ForceMoment &scaled_cfm,
    const Vec3 &force_scale_w, const Vec3 &moment_scale_b) {
  Vec3 answer;
  Vec3Sub(&cfm.force, &cfm_0.force, &answer);
  Mat3TransVec3Mult(&dcm_w2b, &answer, &answer);
  Vec3Mult(&answer, &force_scale_w, &answer);

  Vec3 comparison;
  Vec3Sub(&scaled_cfm.force, &scaled_cfm_0.force, &comparison);
  Mat3TransVec3Mult(&dcm_w2b, &comparison, &comparison);
  EXPECT_NEAR_VEC3(answer, comparison, DBL_TOL);

  // Check moments.
  Vec3Sub(&cfm.moment, &cfm_0.moment, &answer);
  Vec3Mult(&answer, &moment_scale_b, &answer);
  Vec3Sub(&scaled_cfm.moment, &scaled_cfm_0.moment, &comparison);
  EXPECT_NEAR_VEC3(answer, comparison, DBL_TOL);
}

}  // namespace

void AeroTest::CheckRateAndFlapScaling() const {
  AeroSimParams unscaled_params = GetSimParams()->aero_sim;
  OverrideAeroParams(false, &unscaled_params);

  AeroSimParams scaled_params = unscaled_params;

  // Set all the unscaled parameter scale factors to unity.
  unscaled_params.force_coeff_w_scale_factors.rate_derivatives.p = kVec3Ones;
  unscaled_params.force_coeff_w_scale_factors.rate_derivatives.q = kVec3Ones;
  unscaled_params.force_coeff_w_scale_factors.rate_derivatives.r = kVec3Ones;
  unscaled_params.moment_coeff_b_scale_factors.rate_derivatives.p = kVec3Ones;
  unscaled_params.moment_coeff_b_scale_factors.rate_derivatives.q = kVec3Ones;
  unscaled_params.moment_coeff_b_scale_factors.rate_derivatives.r = kVec3Ones;
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    unscaled_params.force_coeff_w_scale_factors.flap_derivatives[i] = kVec3Ones;
    unscaled_params.moment_coeff_b_scale_factors.flap_derivatives[i] =
        kVec3Ones;
  }

  // Randomize the scaled_params scale factors.
  scaled_params.force_coeff_w_scale_factors.rate_derivatives.p =
      RandNormalVec3();
  scaled_params.force_coeff_w_scale_factors.rate_derivatives.q =
      RandNormalVec3();
  scaled_params.force_coeff_w_scale_factors.rate_derivatives.r =
      RandNormalVec3();
  scaled_params.moment_coeff_b_scale_factors.rate_derivatives.p =
      RandNormalVec3();
  scaled_params.moment_coeff_b_scale_factors.rate_derivatives.q =
      RandNormalVec3();
  scaled_params.moment_coeff_b_scale_factors.rate_derivatives.r =
      RandNormalVec3();
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    scaled_params.force_coeff_w_scale_factors.flap_derivatives[i] =
        RandNormalVec3();
    scaled_params.moment_coeff_b_scale_factors.flap_derivatives[i] =
        RandNormalVec3();
  }

  Aero unscaled_aero(unscaled_params);
  Aero scaled_aero(scaled_params);

  VEC_INIT(kNumFlaps, zero_flaps, {0});
  double reynolds_number = 1e6;
  double thrust_coeff = 0.0;
  for (double alpha : alphas_) {
    for (double beta : betas_) {
      Mat3 dcm_w2b;
      CalcDcmWToB(alpha, beta, &dcm_w2b);

      ForceMoment cfm_0;
      ForceMoment scaled_cfm_0;
      aero_->CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps,
                                  reynolds_number, &cfm_0, thrust_coeff);
      scaled_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, zero_flaps,
                                       reynolds_number, &scaled_cfm_0,
                                       thrust_coeff);

      constexpr double h = 1e-4;
      // Check rate steps.
      Vec3 force_scales[] = {
          scaled_params.force_coeff_w_scale_factors.rate_derivatives.p,
          scaled_params.force_coeff_w_scale_factors.rate_derivatives.q,
          scaled_params.force_coeff_w_scale_factors.rate_derivatives.r};
      Vec3 moment_scales[] = {
          scaled_params.moment_coeff_b_scale_factors.rate_derivatives.p,
          scaled_params.moment_coeff_b_scale_factors.rate_derivatives.q,
          scaled_params.moment_coeff_b_scale_factors.rate_derivatives.r};
      Vec3 omega_hats[] = {{h, 0.0, 0.0}, {0.0, h, 0.0}, {0.0, 0.0, h}};
      for (int32_t i = 0; i < ARRAYSIZE(omega_hats); ++i) {
        ForceMoment cfm;
        aero_->CalcForceMomentCoeff(alpha, beta, omega_hats[i], zero_flaps,
                                    reynolds_number, &cfm, thrust_coeff);
        ForceMoment scaled_cfm;
        scaled_aero.CalcForceMomentCoeff(alpha, beta, omega_hats[i], zero_flaps,
                                         reynolds_number, &scaled_cfm,
                                         thrust_coeff);
        CompareRotatedScaledDifference(dcm_w2b, cfm_0, cfm, scaled_cfm_0,
                                       scaled_cfm, force_scales[i],
                                       moment_scales[i]);
      }

      // Check flap steps.
      for (int32_t i = 0; i < kNumFlaps; ++i) {
        VEC_INIT(kNumFlaps, flaps_step, {0});
        *VecPtr(&flaps_step, i) = h;

        ForceMoment cfm;
        unscaled_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps_step,
                                           reynolds_number, &cfm, thrust_coeff);
        ForceMoment scaled_cfm;
        scaled_aero.CalcForceMomentCoeff(alpha, beta, kVec3Zero, flaps_step,
                                         reynolds_number, &scaled_cfm,
                                         thrust_coeff);
        CompareRotatedScaledDifference(
            dcm_w2b, cfm_0, cfm, scaled_cfm_0, scaled_cfm,
            scaled_params.force_coeff_w_scale_factors.flap_derivatives[i],
            scaled_params.moment_coeff_b_scale_factors.flap_derivatives[i]);
      }
    }
  }
}

TEST_F(AeroTest, CheckCoeffOffsets) { CheckCoeffOffsets(); }

TEST_F(AeroTest, CheckFlapOffsets) { CheckFlapOffsets(); }

TEST_F(AeroTest, CheckScaling) {
  CheckBasicScaling();
  CheckRateAndFlapScaling();
}

TEST_F(AeroTest, CheckSpoilerSymmetry) { CheckSpoilerSymmetry(); }

TEST_F(AeroTest, CheckSpoilerPhysics) { CheckSpoilerPhysics(); }

TEST_F(AeroTest, CheckZeroSpoilers) { CheckZeroSpoilers(); }

TEST_F(AeroTest, CheckFullSpoilers) { CheckFullSpoilers(); }

TEST_F(AeroTest, CheckInterpSpoilers) { CheckInterpSpoilers(); }

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
