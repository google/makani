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

#include "sim/physics/avl_aero_database.h"

#include <glog/logging.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <math.h>
#include <stdint.h>

#include <string>

#include "common/c_math/vec3.h"
#include "lib/json_load/json_array_loader.h"
#include "lib/json_load/json_load_or_die.h"
#include "sim/math/util.h"
#include "sim/physics/aero_frame.h"
#include "system/labels.h"

namespace {

class AvlJsonArrayLoader : public json_load::JsonArrayLoader {
 public:
  AvlJsonArrayLoader(int32_t num_alphas, int32_t num_betas, json_t *root)
      : json_load::JsonArrayLoader(num_alphas, num_betas, root) {}
  virtual ~AvlJsonArrayLoader() {}

  void LoadAlphaBetaMatrix(const std::string &field_name, gsl_matrix *dest) {
    CHECK_NOTNULL(dest);

    json_t *field = json_load::LoadFieldOrDie(root_, field_name);
    CHECK(json_is_array(field));

    // We only support a single delta value at the moment.
    CHECK_EQ(1, json_array_size(field));
    field = json_array_get(field, 0);

    LoadMatrixFromField(field, dest);
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(AvlJsonArrayLoader);
};

}  // namespace

AvlAeroDatabase::AvlAeroDatabase(const std::string &filename,
                                 const AeroSimParams &aero_sim_params)
    : aero_sim_params_(aero_sim_params),
      reynolds_number_(),
      num_flaps_(-1),
      alphads_(),
      betads_(),
      eleds_(),
      omega_hat_0_(kVec3Zero),
      CX_(),
      CY_(),
      CZ_(),
      CL_(),
      CM_(),
      CN_(),
      CXp_(),
      CYp_(),
      CZp_(),
      CLp_(),
      CMp_(),
      CNp_(),
      CXq_(),
      CYq_(),
      CZq_(),
      CLq_(),
      CMq_(),
      CNq_(),
      CXr_(),
      CYr_(),
      CZr_(),
      CLr_(),
      CMr_(),
      CNr_(),
      CXd_(),
      CYd_(),
      CZd_(),
      CLd_(),
      CMd_(),
      CNd_() {
  json_t *avl_json = json_load::LoadFileOrDie(filename.c_str());

  int32_t num_alphads = json_load::LoadInt32OrDie(avl_json, "num_alphas");
  int32_t num_betads = json_load::LoadInt32OrDie(avl_json, "num_betas");
  json_t *flap_list = json_load::LoadFieldOrDie(avl_json, "flap_list");
  CHECK(json_is_array(flap_list));
  num_flaps_ = static_cast<int32_t>(json_array_size(flap_list));

  CHECK_EQ(num_flaps_, kNumFlaps)
      << "Detected the wrong number of flaps in database.  The AVL database"
      << "currently only supports " << static_cast<int32_t>(kNumFlaps)
      << " flaps.";

  int32_t json_num_deltas = json_load::LoadInt32OrDie(avl_json, "num_deltas");
  CHECK_EQ(1, json_num_deltas);

  alphads_ = gsl_vector_alloc(num_alphads);
  betads_ = gsl_vector_alloc(num_betads);
  eleds_ = gsl_matrix_alloc(num_alphads, num_betads);

  CX_ = gsl_matrix_alloc(num_alphads, num_betads);
  CY_ = gsl_matrix_alloc(num_alphads, num_betads);
  CZ_ = gsl_matrix_alloc(num_alphads, num_betads);
  CL_ = gsl_matrix_alloc(num_alphads, num_betads);
  CM_ = gsl_matrix_alloc(num_alphads, num_betads);
  CN_ = gsl_matrix_alloc(num_alphads, num_betads);

  CXp_ = gsl_matrix_alloc(num_alphads, num_betads);
  CYp_ = gsl_matrix_alloc(num_alphads, num_betads);
  CZp_ = gsl_matrix_alloc(num_alphads, num_betads);
  CLp_ = gsl_matrix_alloc(num_alphads, num_betads);
  CMp_ = gsl_matrix_alloc(num_alphads, num_betads);
  CNp_ = gsl_matrix_alloc(num_alphads, num_betads);

  CXq_ = gsl_matrix_alloc(num_alphads, num_betads);
  CYq_ = gsl_matrix_alloc(num_alphads, num_betads);
  CZq_ = gsl_matrix_alloc(num_alphads, num_betads);
  CLq_ = gsl_matrix_alloc(num_alphads, num_betads);
  CMq_ = gsl_matrix_alloc(num_alphads, num_betads);
  CNq_ = gsl_matrix_alloc(num_alphads, num_betads);

  CXr_ = gsl_matrix_alloc(num_alphads, num_betads);
  CYr_ = gsl_matrix_alloc(num_alphads, num_betads);
  CZr_ = gsl_matrix_alloc(num_alphads, num_betads);
  CLr_ = gsl_matrix_alloc(num_alphads, num_betads);
  CMr_ = gsl_matrix_alloc(num_alphads, num_betads);
  CNr_ = gsl_matrix_alloc(num_alphads, num_betads);

  if (!alphads_ || !betads_ || !eleds_ || !CX_ || !CY_ || !CZ_ || !CL_ ||
      !CM_ || !CN_ || !CXp_ || !CYp_ || !CZp_ || !CLp_ || !CMp_ || !CNp_ ||
      !CXq_ || !CYq_ || !CZq_ || !CLq_ || !CMq_ || !CNq_ || !CXr_ || !CYr_ ||
      !CZr_ || !CLr_ || !CMr_ || !CNr_) {
    CHECK(false) << "Could not allocate memory for AVL aero database.";
  }

  for (int32_t i = 0; i < num_flaps_; ++i) {
    CXd_[i] = gsl_matrix_alloc(num_alphads, num_betads);
    CYd_[i] = gsl_matrix_alloc(num_alphads, num_betads);
    CZd_[i] = gsl_matrix_alloc(num_alphads, num_betads);
    CLd_[i] = gsl_matrix_alloc(num_alphads, num_betads);
    CMd_[i] = gsl_matrix_alloc(num_alphads, num_betads);
    CNd_[i] = gsl_matrix_alloc(num_alphads, num_betads);

    if (!CXd_[i] || !CYd_[i] || !CZd_[i] || !CLd_[i] || !CMd_[i] || !CNd_[i]) {
      CHECK(false) << "Could not allocate memory for AVL aero database.";
    }
  }

  // Read Reynolds number.
  reynolds_number_ = json_load::LoadDoubleOrDie(avl_json, "reynolds_number");
  CHECK_GT(reynolds_number_, 0.0) << "Reynolds number must be positive.";

  // Read alpha and beta values.
  AvlJsonArrayLoader loader(num_alphads, num_betads, avl_json);
  loader.LoadVec3("omega_hat", &omega_hat_0_);
  loader.LoadVector("alphads", alphads_);
  loader.LoadVector("betads", betads_);

  // Read arrays defined over alpha and beta.
  loader.LoadAlphaBetaMatrix("de1", eleds_);

  loader.LoadAlphaBetaMatrix("CXtot", CX_);
  loader.LoadAlphaBetaMatrix("CYtot", CY_);
  loader.LoadAlphaBetaMatrix("CZtot", CZ_);
  loader.LoadAlphaBetaMatrix("Cltot", CL_);
  loader.LoadAlphaBetaMatrix("Cmtot", CM_);
  loader.LoadAlphaBetaMatrix("Cntot", CN_);

  loader.LoadAlphaBetaMatrix("CXp", CXp_);
  loader.LoadAlphaBetaMatrix("CYp", CYp_);
  loader.LoadAlphaBetaMatrix("CZp", CZp_);
  loader.LoadAlphaBetaMatrix("Clp", CLp_);
  loader.LoadAlphaBetaMatrix("Cmp", CMp_);
  loader.LoadAlphaBetaMatrix("Cnp", CNp_);

  loader.LoadAlphaBetaMatrix("CXq", CXq_);
  loader.LoadAlphaBetaMatrix("CYq", CYq_);
  loader.LoadAlphaBetaMatrix("CZq", CZq_);
  loader.LoadAlphaBetaMatrix("Clq", CLq_);
  loader.LoadAlphaBetaMatrix("Cmq", CMq_);
  loader.LoadAlphaBetaMatrix("Cnq", CNq_);

  loader.LoadAlphaBetaMatrix("CXr", CXr_);
  loader.LoadAlphaBetaMatrix("CYr", CYr_);
  loader.LoadAlphaBetaMatrix("CZr", CZr_);
  loader.LoadAlphaBetaMatrix("Clr", CLr_);
  loader.LoadAlphaBetaMatrix("Cmr", CMr_);
  loader.LoadAlphaBetaMatrix("Cnr", CNr_);

  for (int32_t i = 0; i < num_flaps_; ++i) {
    std::string flap_num = std::to_string(i + 1);
    loader.LoadAlphaBetaMatrix("CXd" + flap_num, CXd_[i]);
    loader.LoadAlphaBetaMatrix("CYd" + flap_num, CYd_[i]);
    loader.LoadAlphaBetaMatrix("CZd" + flap_num, CZd_[i]);
    loader.LoadAlphaBetaMatrix("Cld" + flap_num, CLd_[i]);
    loader.LoadAlphaBetaMatrix("Cmd" + flap_num, CMd_[i]);
    loader.LoadAlphaBetaMatrix("Cnd" + flap_num, CNd_[i]);
  }

  json_decref(avl_json);

  CHECK(IsValid()) << "Invalid AVL database (" << filename << ").";
}

AvlAeroDatabase::~AvlAeroDatabase() {
  gsl_vector_free(alphads_);
  gsl_vector_free(betads_);

  gsl_matrix_free(eleds_);

  gsl_matrix_free(CX_);
  gsl_matrix_free(CY_);
  gsl_matrix_free(CZ_);
  gsl_matrix_free(CL_);
  gsl_matrix_free(CM_);
  gsl_matrix_free(CN_);

  gsl_matrix_free(CXp_);
  gsl_matrix_free(CYp_);
  gsl_matrix_free(CZp_);
  gsl_matrix_free(CLp_);
  gsl_matrix_free(CMp_);
  gsl_matrix_free(CNp_);

  gsl_matrix_free(CXq_);
  gsl_matrix_free(CYq_);
  gsl_matrix_free(CZq_);
  gsl_matrix_free(CLq_);
  gsl_matrix_free(CMq_);
  gsl_matrix_free(CNq_);

  gsl_matrix_free(CXr_);
  gsl_matrix_free(CYr_);
  gsl_matrix_free(CZr_);
  gsl_matrix_free(CLr_);
  gsl_matrix_free(CMr_);
  gsl_matrix_free(CNr_);

  for (int32_t i = 0; i < num_flaps_; ++i) {
    gsl_matrix_free(CXd_[i]);
    gsl_matrix_free(CYd_[i]);
    gsl_matrix_free(CZd_[i]);
    gsl_matrix_free(CLd_[i]);
    gsl_matrix_free(CMd_[i]);
    gsl_matrix_free(CNd_[i]);
  }
}

// Returns true if the AVL aerodynamic database passes the checks:
//
// - The force coefficient responses to increased alpha and beta have
//   the correct sign: an increase in alpha should result in a
//   decrease in C_Z; an increase in beta should result in a decrease
//   in C_Y.
//
// - Control derivatives have the correct sign: a positive port
//   aileron (kFlapA1/2) deflection results in a positive roll
//   moment; a positive starboard aileron (kFlapA7/8) deflection
//   results in a negative roll moment; a positive elevator
//   (kFlapEle) deflection results in a negative pitch moment; and a
//   positive rudder (kFlapRud) deflection results in a negative yaw
//   moment.
bool AvlAeroDatabase::IsValid() const {
  AvlAeroCoeffs coeffs, pos_alpha_coeffs, pos_beta_coeffs;
  CalcAvlAeroCoeffs(0.0, 0.0, &coeffs);
  CalcAvlAeroCoeffs(0.1, 0.0, &pos_alpha_coeffs);
  CalcAvlAeroCoeffs(0.0, 0.1, &pos_beta_coeffs);

  bool check_alpha_sign = pos_alpha_coeffs.CZ < coeffs.CZ;
  bool check_beta_sign = pos_beta_coeffs.CY < coeffs.CY;

  bool check_control_deriv_signs =
      coeffs.CLd[kFlapA1] >= 0.0 && coeffs.CLd[kFlapA2] >= 0.0 &&
      coeffs.CLd[kFlapA7] <= 0.0 && coeffs.CLd[kFlapA8] <= 0.0 &&
      coeffs.CMd[kFlapEle] <= 0.0 && coeffs.CNd[kFlapRud] <= 0.0;

  return check_alpha_sign && check_beta_sign && check_control_deriv_signs;
}

double AvlAeroDatabase::GetNominalElevatorDeflection(double alpha,
                                                     double beta) const {
  double alphad = 180.0 / M_PI * alpha;
  double betad = 180.0 / M_PI * beta;
  double eled = gsl_interp2(alphads_, betads_, eleds_, alphad, betad,
                            kInterpOptionSaturate);
  return eled * M_PI / 180.0;
}

void AvlAeroDatabase::CalcForceMomentCoeff(double alpha, double beta,
                                           const Vec3 &omega_hat,
                                           const Vec &flaps, ForceMoment *cfm,
                                           double thrust_coeff,
                                           AvlAeroCoeffs *aero_coeffs) const {
  CHECK_EQ(flaps.length, num_flaps_)
      << "The AVL aero database is expecting " << num_flaps_ << " flaps, "
      << "but was passed " << flaps.length << " flaps.";
  double alphad = 180.0 / M_PI * alpha;
  double betad = 180.0 / M_PI * beta;
  double flaps_mod_data[kNumFlaps], flapsd_data[kNumFlaps];
  Vec flaps_mod = {num_flaps_, flaps_mod_data, 0, 0};
  Vec flapsd = {num_flaps_, flapsd_data, 0, 0};

  VecCopy(&flaps, &flaps_mod);
  if (aero_sim_params_.use_nonlinear_flaps) {
    AdjustNonlinearFlaps(flaps_mod, &flaps_mod);
  }

  // Saturate all flaps.
  for (int32_t i = 0; i < flaps_mod.length; ++i) {
    flaps_mod.d[i] =
        Saturate(flaps_mod.d[i], aero_sim_params_.min_avl_flap_angles[i],
                 aero_sim_params_.max_avl_flap_angles[i]);
  }
  VecScale(&flaps_mod, 180.0 / M_PI, &flapsd);

  AvlAeroCoeffs coeffs;
  CalcAvlAeroCoeffs(alphad, betad, &coeffs);

  double eled = gsl_interp2(alphads_, betads_, eleds_, alphad, betad,
                            kInterpOptionSaturate);
  flapsd.d[kFlapEle] -= eled;

  Vec3 domega_hat;
  Vec3Sub(&omega_hat, &omega_hat_0_, &domega_hat);

  cfm->force.x = coeffs.CX + coeffs.CXp * domega_hat.x +
                 coeffs.CXq * domega_hat.y + coeffs.CXr * domega_hat.z;
  cfm->force.y = coeffs.CY + coeffs.CYp * domega_hat.x +
                 coeffs.CYq * domega_hat.y + coeffs.CYr * domega_hat.z;
  cfm->force.z = coeffs.CZ + coeffs.CZp * domega_hat.x +
                 coeffs.CZq * domega_hat.y + coeffs.CZr * domega_hat.z;
  cfm->moment.x = coeffs.CL + coeffs.CLp * domega_hat.x +
                  coeffs.CLq * domega_hat.y + coeffs.CLr * domega_hat.z;
  cfm->moment.y = coeffs.CM + coeffs.CMp * domega_hat.x +
                  coeffs.CMq * domega_hat.y + coeffs.CMr * domega_hat.z;
  cfm->moment.z = coeffs.CN + coeffs.CNp * domega_hat.x +
                  coeffs.CNq * domega_hat.y + coeffs.CNr * domega_hat.z;

  for (int32_t i = 0; i < num_flaps_; ++i) {
    if (i == 6 || i == 7) {
      // The dynamic pressure fractional loss (or gain) from the rotor wake is
      // proportional to the thrust coefficient (when using the wind turbine
      // form of the coefficient). See b/78358369 comment#11. This correction
      // is only applied to the tail control surfaces (elevator and rudder).
      double scale_factor;
      scale_factor = (1.0 + thrust_coeff);
      coeffs.CXd[i] *= scale_factor;
      coeffs.CYd[i] *= scale_factor;
      coeffs.CZd[i] *= scale_factor;
      coeffs.CLd[i] *= scale_factor;
      coeffs.CMd[i] *= scale_factor;
      coeffs.CNd[i] *= scale_factor;
    }
    cfm->force.x += coeffs.CXd[i] * flapsd.d[i];
    cfm->force.y += coeffs.CYd[i] * flapsd.d[i];
    cfm->force.z += coeffs.CZd[i] * flapsd.d[i];
    cfm->moment.x += coeffs.CLd[i] * flapsd.d[i];
    cfm->moment.y += coeffs.CMd[i] * flapsd.d[i];
    cfm->moment.z += coeffs.CNd[i] * flapsd.d[i];
  }

  if (aero_coeffs != nullptr) {
    *aero_coeffs = coeffs;
  }
}

void AvlAeroDatabase::CalcAvlAeroCoeffs(double alphad, double betad,
                                        AvlAeroCoeffs *coeffs) const {
  double s = gsl_interp_index(alphads_, alphad);
  double t = gsl_interp_index(betads_, betad);

  coeffs->CX = gsl_interp2_scaled(s, t, CX_, kInterpOptionSaturate);
  coeffs->CY = gsl_interp2_scaled(s, t, CY_, kInterpOptionSaturate);
  coeffs->CZ = gsl_interp2_scaled(s, t, CZ_, kInterpOptionSaturate);
  coeffs->CL = gsl_interp2_scaled(s, t, CL_, kInterpOptionSaturate);
  coeffs->CM = gsl_interp2_scaled(s, t, CM_, kInterpOptionSaturate);
  coeffs->CN = gsl_interp2_scaled(s, t, CN_, kInterpOptionSaturate);

  coeffs->CXp = gsl_interp2_scaled(s, t, CXp_, kInterpOptionSaturate);
  coeffs->CYp = gsl_interp2_scaled(s, t, CYp_, kInterpOptionSaturate);
  coeffs->CZp = gsl_interp2_scaled(s, t, CZp_, kInterpOptionSaturate);
  coeffs->CLp = gsl_interp2_scaled(s, t, CLp_, kInterpOptionSaturate);
  coeffs->CMp = gsl_interp2_scaled(s, t, CMp_, kInterpOptionSaturate);
  coeffs->CNp = gsl_interp2_scaled(s, t, CNp_, kInterpOptionSaturate);

  coeffs->CXq = gsl_interp2_scaled(s, t, CXq_, kInterpOptionSaturate);
  coeffs->CYq = gsl_interp2_scaled(s, t, CYq_, kInterpOptionSaturate);
  coeffs->CZq = gsl_interp2_scaled(s, t, CZq_, kInterpOptionSaturate);
  coeffs->CLq = gsl_interp2_scaled(s, t, CLq_, kInterpOptionSaturate);
  coeffs->CMq = gsl_interp2_scaled(s, t, CMq_, kInterpOptionSaturate);
  coeffs->CNq = gsl_interp2_scaled(s, t, CNq_, kInterpOptionSaturate);

  coeffs->CXr = gsl_interp2_scaled(s, t, CXr_, kInterpOptionSaturate);
  coeffs->CYr = gsl_interp2_scaled(s, t, CYr_, kInterpOptionSaturate);
  coeffs->CZr = gsl_interp2_scaled(s, t, CZr_, kInterpOptionSaturate);
  coeffs->CLr = gsl_interp2_scaled(s, t, CLr_, kInterpOptionSaturate);
  coeffs->CMr = gsl_interp2_scaled(s, t, CMr_, kInterpOptionSaturate);
  coeffs->CNr = gsl_interp2_scaled(s, t, CNr_, kInterpOptionSaturate);

  for (int32_t i = 0; i < num_flaps_; ++i) {
    coeffs->CXd[i] = gsl_interp2_scaled(s, t, CXd_[i], kInterpOptionSaturate);
    coeffs->CYd[i] = gsl_interp2_scaled(s, t, CYd_[i], kInterpOptionSaturate);
    coeffs->CZd[i] = gsl_interp2_scaled(s, t, CZd_[i], kInterpOptionSaturate);
    coeffs->CLd[i] = gsl_interp2_scaled(s, t, CLd_[i], kInterpOptionSaturate);
    coeffs->CMd[i] = gsl_interp2_scaled(s, t, CMd_[i], kInterpOptionSaturate);
    coeffs->CNd[i] = gsl_interp2_scaled(s, t, CNd_[i], kInterpOptionSaturate);
  }
}

// Returns a Vec of effective flap deflections assuming that the
// physical flaps exhibit a decrease in effectiveness at large
// deflections, while the AVL/DVL aerodynamic models are linear.
// The nonlinear model assumes:
//   - 0% aileron and elevator degradations (due to lack of data).
//   - A bi-linear rudder effectiveness correction based on CFD data
//     (see http://b/80444327 for details).
void AvlAeroDatabase::AdjustNonlinearFlaps(const Vec &flaps,
                                           Vec *flaps_nonlin) const {
  DCHECK_EQ(flaps.length, flaps_nonlin->length)
      << "Input and output flap vectors are not the same length.";

  double delta_pos =
      flaps.d[kFlapRud] -
      aero_sim_params_.positive_rudder_deflection_scaling_threshold;
  if (delta_pos > 0.0) {
    flaps_nonlin->d[kFlapRud] =
        (aero_sim_params_.positive_rudder_deflection_scaling_threshold +
         aero_sim_params_.positive_rudder_deflection_scaling * delta_pos);
  }
  double delta_neg =
      flaps.d[kFlapRud] -
      aero_sim_params_.negative_rudder_deflection_scaling_threshold;
  if (delta_neg < 0.0) {
    flaps_nonlin->d[kFlapRud] =
        (aero_sim_params_.negative_rudder_deflection_scaling_threshold +
         aero_sim_params_.negative_rudder_deflection_scaling * delta_neg);
  }
}

namespace {

void ApplyRotatedScaleAndOffsetToCoeffB(const Mat3 &dcm_o2b,
                                        const Vec3 &offset_o,
                                        const Vec3 &scale_o, size_t i, size_t j,
                                        gsl_matrix *coeff_b_x,
                                        gsl_matrix *coeff_b_y,
                                        gsl_matrix *coeff_b_z) {
  Vec3 coeff = {gsl_matrix_get(coeff_b_x, i, j),
                gsl_matrix_get(coeff_b_y, i, j),
                gsl_matrix_get(coeff_b_z, i, j)};

  // Rotate coefficients to offset coordinates.
  Mat3TransVec3Mult(&dcm_o2b, &coeff, &coeff);
  // Scale and offset coefficients.
  Vec3Mult(&scale_o, &coeff, &coeff);
  Vec3Add(&offset_o, &coeff, &coeff);
  // Rotate back.
  Mat3Vec3Mult(&dcm_o2b, &coeff, &coeff);

  gsl_matrix_set(coeff_b_x, i, j, coeff.x);
  gsl_matrix_set(coeff_b_y, i, j, coeff.y);
  gsl_matrix_set(coeff_b_z, i, j, coeff.z);
}

}  // namespace

void AvlAeroDatabase::ApplyCoeffAdjustments(
    const AeroCoeffOffsets &coeff_offsets,
    const AeroCoeffs &force_coeff_w_scale_factors,
    const AeroCoeffs &moment_coeff_b_scale_factors) {
  for (size_t i = 0U; i < alphads_->size; ++i) {
    for (size_t j = 0U; j < betads_->size; ++j) {
      double alpha = gsl_vector_get(alphads_, i) * PI / 180.0;
      double beta = gsl_vector_get(betads_, j) * PI / 180.0;
      Mat3 dcm_w2b;
      CalcDcmWToB(alpha, beta, &dcm_w2b);

      Vec3 offset_w = {-coeff_offsets.CD, -coeff_offsets.CC, -coeff_offsets.CL};
      ApplyRotatedScaleAndOffsetToCoeffB(dcm_w2b, offset_w,
                                         force_coeff_w_scale_factors.coeff, i,
                                         j, CX_, CY_, CZ_);
      ApplyRotatedScaleAndOffsetToCoeffB(
          dcm_w2b, kVec3Zero, force_coeff_w_scale_factors.rate_derivatives.p, i,
          j, CXp_, CYp_, CZp_);
      ApplyRotatedScaleAndOffsetToCoeffB(
          dcm_w2b, kVec3Zero, force_coeff_w_scale_factors.rate_derivatives.q, i,
          j, CXq_, CYq_, CZq_);
      ApplyRotatedScaleAndOffsetToCoeffB(
          dcm_w2b, kVec3Zero, force_coeff_w_scale_factors.rate_derivatives.r, i,
          j, CXr_, CYr_, CZr_);

      Vec3 offset_b = {coeff_offsets.dCldbeta * beta + coeff_offsets.Cl,
                       coeff_offsets.dCmdalpha * alpha + coeff_offsets.Cm,
                       coeff_offsets.dCndbeta * beta + coeff_offsets.Cn};
      ApplyRotatedScaleAndOffsetToCoeffB(kMat3Identity, offset_b,
                                         moment_coeff_b_scale_factors.coeff, i,
                                         j, CL_, CM_, CN_);
      ApplyRotatedScaleAndOffsetToCoeffB(
          kMat3Identity, kVec3Zero,
          moment_coeff_b_scale_factors.rate_derivatives.p, i, j, CLp_, CMp_,
          CNp_);
      ApplyRotatedScaleAndOffsetToCoeffB(
          kMat3Identity, kVec3Zero,
          moment_coeff_b_scale_factors.rate_derivatives.q, i, j, CLq_, CMq_,
          CNq_);
      ApplyRotatedScaleAndOffsetToCoeffB(
          kMat3Identity, kVec3Zero,
          moment_coeff_b_scale_factors.rate_derivatives.r, i, j, CLr_, CMr_,
          CNr_);

      for (int32_t k = 0; k < num_flaps_; ++k) {
        ApplyRotatedScaleAndOffsetToCoeffB(
            dcm_w2b, kVec3Zero, force_coeff_w_scale_factors.flap_derivatives[k],
            i, j, CXd_[k], CYd_[k], CZd_[k]);

        ApplyRotatedScaleAndOffsetToCoeffB(
            kMat3Identity, kVec3Zero,
            moment_coeff_b_scale_factors.flap_derivatives[k], i, j, CLd_[k],
            CMd_[k], CNd_[k]);
      }
    }
  }
}
