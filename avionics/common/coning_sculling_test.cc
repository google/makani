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

#include <math.h>
#include <stdint.h>

#include "avionics/common/coning_sculling.h"
#include "common/c_math/geometry.h"
#include "common/c_math/quaternion.h"
#include "common/c_math/util.h"
#include "common/c_math/mat3.h"
#include "common/c_math/vec3f.h"
#include "common/c_math/vec3.h"
#include "lib/util/test_util.h"

extern "C" {

float DcmTaylorSeriesTrig(float x2, int32_t f0);

}  // extern "C"

namespace {

double sinc(double a) {
  if (fabs(a) <= 0.0) return 1.0;
  return sin(a) / a;
}

}  // namespace

TEST(DcmTaylorSeriesTrig, Test2) {
  for (int32_t i = 0; i < 1000; ++i) {
    float a = (float)(-PI / 4.0 + (PI / 2.0)*(i / 1000.0));
    double d = sinc(a / 2.0)/2.0;
    EXPECT_NEAR(DcmTaylorSeriesTrig(a * a, 2), 2.0 * d * d, 1e-6);
  }
}

TEST(DcmTaylorSeriesTrig, Test3) {
  EXPECT_NEAR(DcmTaylorSeriesTrig(0.0f, 3), 1.0 / 6.0, 1e-6);
  for (int32_t i = 0; i < 1000; ++i) {
    float a = (float)(-PI + (2.0 * PI)*(i / 1000.0));
    if (fabs(a) > 0.0) {
      EXPECT_NEAR(DcmTaylorSeriesTrig(a * a, 3),
                  (1.0 - sinc(a)) / (a * a), 1e-6);
    }
  }
}

// Spin-cone model described in:
//
// [1] Savage, P.G. "Performance Analysis of Strapdown Systems."  In
//     RTO-EN-SET-116-2009: Low-Cost Navigation Sensors and
//     Integration Technology. March 2009: NATO Science and Technology
//     Organization.
//
// Simulates a rotation of a rigid body that gives rise to "coning"
// errors when angular rates are decimated by naive integration.
class SpinConeModel {
 public:
  SpinConeModel(double omega_s, double omega_c, double beta, double phi_0)
      : omega_s_(omega_s), omega_c_(omega_c), beta_(beta), phi_0_(phi_0) {}

 public:
  // Get the exact DCM rotating body coordinates to navigation coordinates.
  void GetDcmBToN(double t, Mat3 *dcm_b2n) const;
  // Get the increments to be handed to the coning and sculling algorithm.
  void GetDalpha(double t_1, double t, Vec3 *dalpha_l) const;

 protected:
  // Kinematic description of motion, see [1] equation (1).
  double GetPhi(double t) const {
    return phi_0_ + (omega_s_ - omega_c_ * cos(beta_)) * t;
  }

  double GetTheta() const {
    return PI / 2.0 - beta_;
  }

  double GetPsi(double t) const {
    return - omega_c_ * t;
  }

  // Calculates the running integral of the body angular rates.
  void GetIOmegaIB(double t, Vec3 *int_omega_ib) const;

  // Simulation parameters.
  // TODO: Add dcm_b2r.
  double omega_s_;  // Spin angular rate [rad/s].
  double omega_c_;  // Precession angular rate [rad/s].
  double beta_;  // Angle [rad] of precession axis to spin axis.
  double phi_0_;  // Initial spin angle.
};

void SpinConeModel::GetDcmBToN(double t, Mat3 *dcm_b2n) const {
  // See [1] equation (4).
  double phi = GetPhi(t);
  double theta = GetTheta();
  double psi = GetPsi(t);

  dcm_b2n->d[0][0] = cos(theta) * cos(psi);
  dcm_b2n->d[0][1] = - cos(phi) * sin(psi) + sin(phi) * sin(theta) * cos(psi);
  dcm_b2n->d[0][2] = sin(phi) * sin(psi) + cos(phi) * sin(theta) * cos(psi);

  dcm_b2n->d[1][0] = cos(theta) * sin(psi);
  dcm_b2n->d[1][1] = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
  dcm_b2n->d[1][2] = - sin(phi) * cos(psi) + cos(phi) * sin(theta) * sin(psi);

  dcm_b2n->d[2][0] = - sin(theta);
  dcm_b2n->d[2][1] = sin(phi) * cos(theta);
  dcm_b2n->d[2][2] = cos(phi) * cos(theta);
}

void SpinConeModel::GetDalpha(double t_1, double t, Vec3 *dalpha_l) const {
  // See [1] equation (3).
  Vec3 tmp;
  GetIOmegaIB(t_1, &tmp);
  GetIOmegaIB(t, dalpha_l);
  Vec3Sub(dalpha_l, &tmp, dalpha_l);
}

void SpinConeModel::GetIOmegaIB(double t, Vec3 *int_omega_ib) const {
  // See [1] equation (2).
  double phi = GetPhi(t);
  double mult = omega_c_ * sin(beta_) / (omega_s_ - omega_c_ * cos(beta_));
  int_omega_ib->x = omega_s_ * t;
  int_omega_ib->y = mult * (cos(phi) - cos(phi_0_));
  int_omega_ib->z = -mult * (sin(phi) - sin(phi_0_));
}

bool IntegrateInc(const SpinConeModel &spin_cone, double t_1, double t,
                  double dt, int32_t dec) {
  Vec3 dalpha_l;
  spin_cone.GetDalpha(t_1, t, &dalpha_l);

  Vec3f dalpha_lf = {(float)dalpha_l.x, (float)dalpha_l.y, (float)dalpha_l.z};
  Vec3f dnu_lf = kVec3fZero;
  return ConingScullingUpdateInc(dec, (float)dt, &dalpha_lf, &dnu_lf);
}

bool IntegrateRaw(const SpinConeModel &spin_cone, double t_1, double t,
                  double dt, int32_t dec) {
  Vec3 dalpha_l;
  spin_cone.GetDalpha(t_1, t, &dalpha_l);

  Vec3f omega_ib_b = {(float)(dalpha_l.x / dt),
                      (float)(dalpha_l.y / dt),
                      (float)(dalpha_l.z / dt)};
  Vec3f a_sf_b = kVec3fZero;
  return ConingScullingUpdateRaw(dec, (float)dt, &omega_ib_b, &a_sf_b);
}

// Test ConingScullingUpdateInc using the spin cone model.
//
// Accumulates the output of ConingScullingSample into a DCM which is compared
// to the result expected from the SpinConeModel.
void TestConingScullingUpdateSpinCone(
    const SpinConeModel &spin_cone,
    bool (* const int_func)(const SpinConeModel &spin_cone, double t_1,
                            double t, double dt, int32_t dec),
    double dt, int32_t dec, int32_t N, double tol) {
  ConingScullingInit();

  bool init = false;
  Mat3 dcm_bhat2n;
  double t_1 = 0.0;
  for (int32_t i = 1; i < N + 1; ++i) {
    float t = (float)(i * dt);
    bool ready = int_func(spin_cone, t_1, t, dt, dec);
    EXPECT_EQ(i % dec == 0, ready);
    if (ready) {
      float dt_m;
      Vec3f phi_m, dvsf_m, alpha_m, nu_m;
      ConingScullingSample(&dt_m, &phi_m, &dvsf_m, &alpha_m, &nu_m);
      EXPECT_NEAR(dt_m, dec * dt, dec * dt * 1e-4);

      if (!init) {
        // On the first iteration, simply copy out the resulting DCM.
        spin_cone.GetDcmBToN(t, &dcm_bhat2n);
        init = true;
      } else {
        Vec3 phi_m_v = {-phi_m.x, -phi_m.y, -phi_m.z};
        Quat q;
        Mat3 tmp;
        Mat3Mat3Mult(&dcm_bhat2n, QuatToDcm(AxisToQuat(&phi_m_v, &q), &tmp),
                     &dcm_bhat2n);
        Mat3 dcm_b2n;
        spin_cone.GetDcmBToN(t, &dcm_b2n);
        Mat3 dcm_b2bhat;
        Mat3Mult(&dcm_bhat2n, kTrans, &dcm_b2n, kNoTrans, &dcm_b2bhat);
        EXPECT_NEAR(0.0, acos((Mat3Trace(&dcm_b2bhat)-1.0)/2.0), tol * t);
      }
    }
    t_1 = t;
  }
}

TEST(ConingScullingUpdateInc, SpinConeTenHertz) {
  // Spin cone model parameters.
  double omega_s = 2.0 * PI;
  double omega_c = 2.0 * PI * 10.0;
  double beta = PI / 4.0;
  SpinConeModel spin_cone(omega_s, omega_c, beta, 0.0);

  double dt = 1.0 / 2400.0;  // High frequency sampling period.
  int32_t dec = 24;  // Decimation rate.
  int32_t N = 24000;  // Number of samples to simulate.
  double tol = 1e-2;  // Tolerable drift in [rad / sec].
  TestConingScullingUpdateSpinCone(spin_cone, IntegrateInc, dt, dec, N, tol);
}

TEST(ConingScullingUpdateRaw, SpinConeTenHertz) {
  // Spin cone model parameters.
  double omega_s = 2.0 * PI;
  double omega_c = 2.0 * PI * 10.0;
  double beta = PI / 4.0;
  SpinConeModel spin_cone(omega_s, omega_c, beta, 0.0);

  double dt = 1.0 / 2400.0;  // High frequency sampling period.
  int32_t dec = 24;  // Decimation rate.
  int32_t N = 24000;  // Number of samples to simulate.
  double tol = 1e-0;  // Tolerable drift in [rad / sec].
  TestConingScullingUpdateSpinCone(spin_cone, IntegrateRaw, dt, dec, N, tol);
}

// Spin-rock model described in:
//
// [1] Savage, P.G. "Performance Analysis of Strapdown Systems."  In
//     RTO-EN-SET-116-2009: Low-Cost Navigation Sensors and
//     Integration Technology. March 2009: NATO Science and Technology
//     Organization.
//
// Simulates a rotation of a rigid body with angular velocity oscillations.
class SpinRockModel {
 public:
  SpinRockModel(const Vec3 &u_gamma_b, const Vec3 &l_0_b,
                double a, double b, double omega)
      : u_gamma_b_(u_gamma_b), l_0_b_(l_0_b), a_(a), b_(b), omega_(omega) {}

  // See [1] equations (7) and (8).
  void GetDalphaDnu(double t_1, double t, Vec3 *dalpha_l, Vec3 *dnu_l) const;
  // Calculates the kinmatic state of the model.
  void GetState(double t, Mat3 *dcm, Vec3 *v_n, Vec3 *r_n) const;


 private:
  // Kinematic description of the model, see [1] equation (6).
  double GetGamma(double t) const {
    return a_ * t + b_ * sin(omega_ * t);
  }
  double GetGammaDot(double t) const {
    return a_  + b_ * omega_ * cos(omega_ * t);
  }
  // Factors for calculating dnu_l, see [1] equation (9).
  double GetFa(double t) const {
    return b_ * omega_ * cos(omega_ * t);
  }
  double GetFb(double t) const {
    return (a_ * a_ + 0.5 * b_ * b_ * omega_ * omega_) * t
        + 2.0 * a_ * b_ * sin(omega_ * t)
        + 0.5 * b_ * b_ * omega_ * sin(omega_ * t) * cos(omega_ * t);
  }

  Vec3 u_gamma_b_;  // Rotation axis (unit norm).
  Vec3 l_0_b_;  // Position of sensor on body [m].
  double a_;  // Mean angular rate [rad/s].
  double b_;  // Angular rate oscillation amplitude [rad/s].
  double omega_;  // Angular rate oscillation frequency [rad/s].
};

void SpinRockModel::GetDalphaDnu(double t_1, double t,
                                 Vec3 *dalpha_l, Vec3 *dnu_l) const {
  Vec3Scale(&u_gamma_b_, GetGamma(t) - GetGamma(t_1), dalpha_l);

  // Assumes the sensitive axes of the accelerometers are aligned with
  // the axes of the body frame.
  Vec3 tmp;
  Vec3Cross(&u_gamma_b_, &l_0_b_, &tmp);
  Vec3Cross(&u_gamma_b_, &tmp, dnu_l);
  Vec3Scale(dnu_l, GetFb(t) - GetFb(t_1), dnu_l);
  Vec3Scale(&tmp, GetFa(t) - GetFa(t_1), &tmp);
  Vec3Add(&tmp, dnu_l, dnu_l);
}

void SpinRockModel::GetState(double t, Mat3 *dcm, Vec3 *v_n, Vec3 *r_n) const {
  // Assumes C_B^N(0) is the identity, see [1] equation (10).
  Mat3 tmp;
  Mat3Cross(&u_gamma_b_, &tmp);
  double gamma = GetGamma(t);
  Mat3Scale(Mat3Mat3Mult(&tmp, &tmp, dcm), 1.0 - cos(gamma),  dcm);
  Mat3Add(Mat3Scale(&tmp, sin(gamma), &tmp), kNoTrans,
          dcm, kNoTrans, dcm);
  Mat3Add(&kMat3Identity, kNoTrans,
          dcm, kNoTrans, dcm);

  // Calculate velocity, see [1] equation (11).
  Vec3Scale(Mat3Vec3Mult(dcm,
                         Vec3Cross(&u_gamma_b_, &l_0_b_, v_n),
                         v_n), GetGammaDot(t), v_n);
  // Calculate position, see [1] equation (12).
  Mat3Vec3Mult(dcm, &l_0_b_, r_n);
}

bool IntegrateInc(const SpinRockModel &spin_rock, double t_1, double t,
                  double dt, int32_t dec) {
  Vec3 dalpha_l;
  Vec3 dnu_l;
  spin_rock.GetDalphaDnu(t_1, t, &dalpha_l, &dnu_l);

  Vec3f dalpha_lf = {(float)dalpha_l.x, (float)dalpha_l.y, (float)dalpha_l.z};
  Vec3f dnu_lf = {(float)dnu_l.x, (float)dnu_l.y, (float)dnu_l.z};
  return ConingScullingUpdateInc(dec, (float)dt, &dalpha_lf, &dnu_lf);
}

bool IntegrateRaw(const SpinRockModel &spin_rock, double t_1, double t,
                  double dt, int32_t dec) {
  Vec3 dalpha_l;
  Vec3 dnu_l;
  spin_rock.GetDalphaDnu(t_1, t, &dalpha_l, &dnu_l);

  Vec3f omega_ib_b = {(float)(dalpha_l.x / dt),
                      (float)(dalpha_l.y / dt),
                      (float)(dalpha_l.z / dt)};
  Vec3f a_sf_b = {(float)(dnu_l.x / dt),
                  (float)(dnu_l.y / dt),
                  (float)(dnu_l.z / dt)};
  return ConingScullingUpdateRaw(dec, (float)dt, &omega_ib_b, &a_sf_b);
}

// Test ConingScullingUpdateInc using the spin rock model.
//
// Accumulates the output of ConingScullingSample into a DCM and
// velocity vector which are compared to the results expected from
// SpinRockModel.
void TestConingScullingUpdateSpinRock(
    const SpinRockModel &spin_rock,
    bool (* const int_func)(const SpinRockModel &spin_rock, double t_1,
                            double t, double dt, int32_t dec),
    double dt, int32_t dec, int32_t N, double angle_tol, double velocity_tol) {
  ConingScullingInit();

  bool init = false;
  Mat3 dcm_bhat2n;
  Vec3 v_hat;
  double t_1 = 0.0;
  for (int32_t i = 1; i < N + 1; ++i) {
    float t = (float)(i * dt);
    bool ready = int_func(spin_rock, t_1, t, dt, dec);
    EXPECT_EQ(i % dec == 0, ready);
    if (ready) {
      float dt_m;
      Vec3f phi_m, dvsf_m, alpha_m, nu_m;
      ConingScullingSample(&dt_m, &phi_m, &dvsf_m, &alpha_m, &nu_m);
      EXPECT_NEAR(dt_m, dec * dt, dec * dt * 1e-4);

      if (!init) {
        Vec3 r_hat;  // Unused.
        spin_rock.GetState(t, &dcm_bhat2n, &v_hat, &r_hat);
        init = true;
      } else {
        Mat3 dcm_b2n;
        Vec3 v_n, r_n;
        spin_rock.GetState(t, &dcm_b2n, &v_n, &r_n);

        // Update DCM.
        Mat3 tmp_mat;
        Quat q;
        Vec3 phi_m_v = {-phi_m.x, -phi_m.y, -phi_m.z};
        Mat3Mat3Mult(&dcm_bhat2n, QuatToDcm(AxisToQuat(&phi_m_v, &q), &tmp_mat),
                     &dcm_bhat2n);
        Mat3 dcm_b2bhat;
        Mat3Mult(&dcm_bhat2n, kTrans, &dcm_b2n, kNoTrans, &dcm_b2bhat);
        EXPECT_NEAR(3.0, Mat3Trace(&dcm_b2bhat), angle_tol * angle_tol * t * t);

        // Update velocity.
        Vec3 dvsf_m_v = {dvsf_m.x, dvsf_m.y, dvsf_m.z};
        Vec3 tmp;
        Vec3Axpy(1.0, Mat3Vec3Mult(&dcm_bhat2n, &dvsf_m_v, &tmp), &v_hat);
        EXPECT_NEAR_VEC3(v_n, v_hat, velocity_tol * t);
      }
    }
    t_1 = t;
  }
}

TEST(ConingScullingUpdateInc, SpinRockTwoHertz) {
  // Spin-Rock model parameters.
  double a = 1.0;
  double b = 0.15;
  double omega = 2.0 * PI * 2.0;
  const Vec3 u_gamma_b = {1.0/sqrt(2.0), 1.0/sqrt(2.0), 0.0};
  const Vec3 l_0_b = {0.0, 1.0/sqrt(2.0), 1.0/sqrt(2.0)};
  SpinRockModel spin_rock(u_gamma_b, l_0_b, a, b, omega);

  double dt = 1.0 / 2400.0;  // High frequency sampling period.
  int32_t dec = 24;  // Decimation rate.
  int32_t N = 2400;  // Number of samples to simulate.
  double angle_tol = 1e-2;  // Acceptable angle drift [rad / sec].
  double velocity_tol = 2e-1;  // Acceptable velocity drift [m / s^2].
  TestConingScullingUpdateSpinRock(
      spin_rock, IntegrateInc, dt, dec, N, angle_tol, velocity_tol);
}

TEST(ConingScullingUpdateRaw, SpinRockTwoHertz) {
  // Spin-Rock model parameters.
  double a = 1.0;
  double b = 0.15;
  double omega = 2.0 * PI * 2.0;
  const Vec3 u_gamma_b = {1.0/sqrt(2.0), 1.0/sqrt(2.0), 0.0};
  const Vec3 l_0_b = {0.0, 1.0/sqrt(2.0), 1.0/sqrt(2.0)};
  SpinRockModel spin_rock(u_gamma_b, l_0_b, a, b, omega);

  double dt = 1.0 / 2400.0;  // High frequency sampling period.
  int32_t dec = 24;  // Decimation rate.
  int32_t N = 2400;  // Number of samples to simulate.
  double angle_tol = 1e-2;  // Acceptable angle drift [rad / sec].
  double velocity_tol = 2e-1;  // Acceptable velocity drift [m / s^2].
  TestConingScullingUpdateSpinRock(
      spin_rock, IntegrateRaw, dt, dec, N, angle_tol, velocity_tol);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
