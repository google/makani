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

#include "sim/physics/dvl_aero_database.h"

#include <glog/logging.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include <string>
#include <vector>

#include "common/c_math/force_moment.h"
#include "common/c_math/linalg.h"
#include "common/c_math/util.h"
#include "control/system_types.h"
#include "lib/json_load/json_array_loader.h"
#include "lib/json_load/json_load_or_die.h"
#include "sim/math/util.h"
#include "sim/physics/aero_frame.h"
#include "system/labels.h"

DvlAeroDatabase::DvlAeroDatabase(const std::string &filename)
    : num_flaps_(),
      reynolds_number_(),
      alphas_(),
      betas_(),
      deltas_(kNumFlaps, nullptr),
      cfm_(),
      dcfm_dp_(),
      dcfm_dq_(),
      dcfm_dr_(),
      flap_cfms_(kNumFlaps, nullptr),
      flap_dcfm_dps_(kNumFlaps, nullptr),
      flap_dcfm_dqs_(kNumFlaps, nullptr),
      flap_dcfm_drs_(kNumFlaps, nullptr) {
  json_t *dvl_json = json_load::LoadFileOrDie(filename.c_str());

  int32_t num_alphas = json_load::LoadInt32OrDie(dvl_json, "num_alphas");
  int32_t num_betas = json_load::LoadInt32OrDie(dvl_json, "num_betas");

  std::vector<int32_t> num_deltas(kNumFlaps);
  json_t *num_deltas_list = json_load::LoadFieldOrDie(dvl_json, "num_deltas");
  CHECK(json_is_array(num_deltas_list));
  num_flaps_ = static_cast<int32_t>(json_array_size(num_deltas_list));
  CHECK_EQ(num_flaps_, kNumFlaps)
      << "Detected the wrong number of flaps in database.  The DVL database"
      << "currently only supports " << static_cast<int32_t>(kNumFlaps)
      << " flaps.";
  json_load::LoadArray1D_Int32OrDie(num_deltas_list, num_flaps_, &num_deltas);

  reynolds_number_ = json_load::LoadDoubleOrDie(dvl_json, "reynolds_number");
  CHECK_GT(reynolds_number_, 0.0) << "Reynolds number must be positive.";

  alphas_ = gsl_vector_alloc(num_alphas);
  CHECK_NOTNULL(alphas_);

  betas_ = gsl_vector_alloc(num_betas);
  CHECK_NOTNULL(betas_);

  cfm_ = gsl_vector_alloc(6 * num_alphas * num_betas);
  CHECK_NOTNULL(cfm_);

  dcfm_dp_ = gsl_vector_alloc(6 * num_alphas * num_betas);
  CHECK_NOTNULL(dcfm_dp_);

  dcfm_dq_ = gsl_vector_alloc(6 * num_alphas * num_betas);
  CHECK_NOTNULL(dcfm_dq_);

  dcfm_dr_ = gsl_vector_alloc(6 * num_alphas * num_betas);
  CHECK_NOTNULL(dcfm_dr_);

  for (int32_t i = 0; i < num_flaps_; ++i) {
    deltas_[i] = gsl_vector_alloc(num_deltas[i]);
    CHECK_NOTNULL(deltas_[i]);

    flap_cfms_[i] =
        gsl_vector_alloc(6 * num_alphas * num_betas * num_deltas[i]);
    CHECK_NOTNULL(flap_cfms_[i]);

    flap_dcfm_dps_[i] =
        gsl_vector_alloc(6 * num_alphas * num_betas * num_deltas[i]);
    CHECK_NOTNULL(flap_dcfm_dps_[i]);

    flap_dcfm_dqs_[i] =
        gsl_vector_alloc(6 * num_alphas * num_betas * num_deltas[i]);
    CHECK_NOTNULL(flap_dcfm_dqs_[i]);

    flap_dcfm_drs_[i] =
        gsl_vector_alloc(6 * num_alphas * num_betas * num_deltas[i]);
    CHECK_NOTNULL(flap_dcfm_drs_[i]);
  }

  json_load::JsonArrayLoader loader(num_alphas, num_betas, dvl_json);
  loader.LoadVector("alphas", alphas_);
  loader.LoadVector("betas", betas_);
  loader.LoadVector<3>("cfm", {{6, num_alphas, num_betas}}, cfm_);
  loader.LoadVector<3>("dcfm_dp", {{6, num_alphas, num_betas}}, dcfm_dp_);
  loader.LoadVector<3>("dcfm_dq", {{6, num_alphas, num_betas}}, dcfm_dq_);
  loader.LoadVector<3>("dcfm_dr", {{6, num_alphas, num_betas}}, dcfm_dr_);

  for (int32_t i = 0; i < num_flaps_; ++i) {
    loader.LoadVector("delta" + std::to_string(i + 1) + "s", deltas_[i]);
    loader.LoadVector<4>("dcfm" + std::to_string(i + 1),
                         {{6, num_alphas, num_betas, num_deltas[i]}},
                         flap_cfms_[i]);
    loader.LoadVector<4>("dcfm" + std::to_string(i + 1) + "_dp",
                         {{6, num_alphas, num_betas, num_deltas[i]}},
                         flap_dcfm_dps_[i]);
    loader.LoadVector<4>("dcfm" + std::to_string(i + 1) + "_dq",
                         {{6, num_alphas, num_betas, num_deltas[i]}},
                         flap_dcfm_dqs_[i]);
    loader.LoadVector<4>("dcfm" + std::to_string(i + 1) + "_dr",
                         {{6, num_alphas, num_betas, num_deltas[i]}},
                         flap_dcfm_drs_[i]);
  }

  json_decref(dvl_json);

  CHECK(IsValid()) << "Invalid DVL database (" << filename << ").";
}

DvlAeroDatabase::~DvlAeroDatabase() {
  gsl_vector_free(alphas_);
  gsl_vector_free(betas_);
  gsl_vector_free(cfm_);
  gsl_vector_free(dcfm_dp_);
  gsl_vector_free(dcfm_dq_);
  gsl_vector_free(dcfm_dr_);

  for (int32_t i = 0; i < num_flaps_; ++i) {
    gsl_vector_free(deltas_[i]);
    gsl_vector_free(flap_cfms_[i]);
    gsl_vector_free(flap_dcfm_dps_[i]);
    gsl_vector_free(flap_dcfm_dqs_[i]);
    gsl_vector_free(flap_dcfm_drs_[i]);
  }
}

// Returns true if the DVL aerodynamic database passes the checks:
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
bool DvlAeroDatabase::IsValid() const {
  std::vector<double> zeros;
  std::vector<double> pos_ail, neg_ail, pos_ele, neg_ele, pos_rud, neg_rud;
  zeros.assign({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  pos_ail.assign({-0.1, -0.1, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0});
  neg_ail.assign({0.1, 0.1, 0.0, 0.0, -0.1, -0.1, 0.0, 0.0});
  pos_ele.assign({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0});
  neg_ele.assign({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.0});
  pos_rud.assign({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1});
  neg_rud.assign({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1});

  // Check alpha sign.
  //
  // NOTE: We only check the sign of dCz/dalpha on high
  // Reynolds, i.e. high airspeed, databases because this value can be
  // negative due to effects from the rotors at low airspeeds.
  Vec pos_flaps = {num_flaps_, pos_ail.data(), 0, 0};
  Vec neg_flaps = {num_flaps_, neg_ail.data(), 0, 0};
  Vec zero_flaps = {num_flaps_, zeros.data(), 0, 0};
  DvlAeroCoeffs pos_coeffs;
  DvlAeroCoeffs neg_coeffs;
  CalcDvlAeroCoeffs(0.1, 0.0, zero_flaps, &pos_coeffs);
  CalcDvlAeroCoeffs(-0.1, 0.0, zero_flaps, &neg_coeffs);
  if (reynolds_number() > 5e5 &&
      pos_coeffs.cfm.force.z > neg_coeffs.cfm.force.z) {
    return false;
  }

  // Check beta sign.
  //
  // NOTE: Historically, DVL databases had signs on their
  // beta and rudder values opposite of convention. This failure might
  // indicate that a new DVL database has been generated without
  // correcting these signs.
  //
  // NOTE: We only check the sign of dCy/dbeta on high
  // Reynolds, i.e. high airspeed, databases because this value can be
  // positive due to effects from the rotors at low airspeeds.
  // This has been modified to allow OktoberKite databases to clear the Reynolds
  // number check limit because of the increase in chord and tail. (b/145955686)
  CalcDvlAeroCoeffs(0.0, 0.1, zero_flaps, &pos_coeffs);
  CalcDvlAeroCoeffs(0.0, -0.1, zero_flaps, &neg_coeffs);
  if (reynolds_number() > 1.0E6 &&
      pos_coeffs.cfm.force.y > neg_coeffs.cfm.force.y) {
    return false;
  }

  // Check aileron sign.
  pos_flaps.d = pos_ail.data();
  neg_flaps.d = neg_ail.data();
  CalcDvlAeroCoeffs(0.0, 0.0, pos_flaps, &pos_coeffs);
  CalcDvlAeroCoeffs(0.0, 0.0, neg_flaps, &neg_coeffs);
  if (pos_coeffs.flap_cfms[kFlapA1].moment.x >
          neg_coeffs.flap_cfms[kFlapA1].moment.x ||
      pos_coeffs.flap_cfms[kFlapA2].moment.x >
          neg_coeffs.flap_cfms[kFlapA2].moment.x ||
      pos_coeffs.flap_cfms[kFlapA7].moment.x >
          neg_coeffs.flap_cfms[kFlapA7].moment.x ||
      pos_coeffs.flap_cfms[kFlapA8].moment.x >
          neg_coeffs.flap_cfms[kFlapA8].moment.x) {
    return false;
  }

  // Check elevator sign.
  pos_flaps.d = pos_ele.data();
  neg_flaps.d = neg_ele.data();
  CalcDvlAeroCoeffs(0.0, 0.0, pos_flaps, &pos_coeffs);
  CalcDvlAeroCoeffs(0.0, 0.0, neg_flaps, &neg_coeffs);
  if (pos_coeffs.flap_cfms[kFlapEle].moment.y >
      neg_coeffs.flap_cfms[kFlapEle].moment.y) {
    return false;
  }

  // Check rudder sign.
  //
  // NOTE: See comment on beta, above.
  pos_flaps.d = pos_rud.data();
  neg_flaps.d = neg_rud.data();
  CalcDvlAeroCoeffs(0.0, 0.0, pos_flaps, &pos_coeffs);
  CalcDvlAeroCoeffs(0.0, 0.0, neg_flaps, &neg_coeffs);
  if (pos_coeffs.flap_cfms[kFlapRud].moment.z >
      neg_coeffs.flap_cfms[kFlapRud].moment.z) {
    return false;
  }

  return true;
}

void DvlAeroDatabase::CalcForceMomentCoeff(double alpha, double beta,
                                           const Vec3 &omega_hat,
                                           const Vec &flaps, ForceMoment *cfm,
                                           double /*thrust_coeff*/,
                                           DvlAeroCoeffs *aero_coeffs) const {
  DvlAeroCoeffs coeffs;
  CalcDvlAeroCoeffs(alpha, beta, flaps, &coeffs);

  // TODO: This is a hack that limits the large positive
  // roll rate derivatives you get at high angles-of-attack once a
  // significant roll rate is achieved.  Eventually, we should
  // implement a nonlinear angular rate derivative lookup.
  coeffs.dcfm_dp.moment.x =
      Crossfade(coeffs.dcfm_dp.moment.x, fmin(coeffs.dcfm_dp.moment.x, -0.2),
                fabs(omega_hat.x), 0.1, 0.15);

  *cfm = coeffs.cfm;
  ForceMoment cfm_p = coeffs.dcfm_dp;
  ForceMoment cfm_q = coeffs.dcfm_dq;
  ForceMoment cfm_r = coeffs.dcfm_dr;
  for (int32_t i = 0; i < num_flaps_; ++i) {
    ForceMomentAdd(cfm, &coeffs.flap_cfms[i], cfm);
    ForceMomentAdd(&cfm_p, &coeffs.flap_dcfm_dps[i], &cfm_p);
    ForceMomentAdd(&cfm_q, &coeffs.flap_dcfm_dqs[i], &cfm_q);
    ForceMomentAdd(&cfm_r, &coeffs.flap_dcfm_drs[i], &cfm_r);
  }
  ForceMomentScale(&cfm_p, omega_hat.x, &cfm_p);
  ForceMomentScale(&cfm_q, omega_hat.y, &cfm_q);
  ForceMomentScale(&cfm_r, omega_hat.z, &cfm_r);

  ForceMomentAdd(cfm, &cfm_p, cfm);
  ForceMomentAdd(cfm, &cfm_q, cfm);
  ForceMomentAdd(cfm, &cfm_r, cfm);

  if (aero_coeffs != nullptr) {
    *aero_coeffs = coeffs;
  }
}

void DvlAeroDatabase::CalcDvlAeroCoeffs(double alpha, double beta,
                                        const Vec &flaps,
                                        DvlAeroCoeffs *coeffs) const {
  CHECK_EQ(flaps.length, num_flaps_) << "flaps has the wrong number of flaps.";

  double u = gsl_interp_index(alphas_, alpha);
  double v = gsl_interp_index(betas_, beta);
  std::vector<double> w(kNumFlaps);
  for (int32_t i = 0; i < num_flaps_; ++i) {
    w[i] = gsl_interp_index(deltas_[i], flaps.d[i]);
  }

  int32_t num_alphas = static_cast<int32_t>(alphas_->size);
  int32_t num_betas = static_cast<int32_t>(betas_->size);

  int32_t s0, t0;
  double ds, dt;
  gsl_interp2_indices(u, v, num_alphas, num_betas, kInterpOptionSaturate, &s0,
                      &t0, &ds, &dt);
  LookupForceMomentCoeff2D(*cfm_, num_alphas, num_betas, s0, t0, ds, dt,
                           &coeffs->cfm);
  LookupForceMomentCoeff2D(*dcfm_dp_, num_alphas, num_betas, s0, t0, ds, dt,
                           &coeffs->dcfm_dp);
  LookupForceMomentCoeff2D(*dcfm_dq_, num_alphas, num_betas, s0, t0, ds, dt,
                           &coeffs->dcfm_dq);
  LookupForceMomentCoeff2D(*dcfm_dr_, num_alphas, num_betas, s0, t0, ds, dt,
                           &coeffs->dcfm_dr);

  for (int32_t i = 0; i < num_flaps_; ++i) {
    int32_t indices[8];
    double du, dv, dw;
    gsl_interp3_indices(u, v, w[i], num_alphas, num_betas,
                        static_cast<int32_t>(deltas_[i]->size),
                        kInterpOptionSaturate, indices, &du, &dv, &dw);
    LookupForceMomentCoeff3D(*flap_cfms_[i], indices, du, dv, dw,
                             &coeffs->flap_cfms[i]);
    LookupForceMomentCoeff3D(*flap_dcfm_dps_[i], indices, du, dv, dw,
                             &coeffs->flap_dcfm_dps[i]);
    LookupForceMomentCoeff3D(*flap_dcfm_dqs_[i], indices, du, dv, dw,
                             &coeffs->flap_dcfm_dqs[i]);
    LookupForceMomentCoeff3D(*flap_dcfm_drs_[i], indices, du, dv, dw,
                             &coeffs->flap_dcfm_drs[i]);
  }
}

void DvlAeroDatabase::LookupForceMomentCoeff2D(const gsl_vector &cfm_data,
                                               size_t size1, size_t size2,
                                               int32_t s0, int32_t t0,
                                               double ds, double dt,
                                               ForceMoment *cfm) const {
  size_t num_data = size1 * size2;
  gsl_vector_const_view cx = gsl_vector_const_subvector(&cfm_data, 0, num_data);
  gsl_vector_const_view cy =
      gsl_vector_const_subvector(&cfm_data, num_data, num_data);
  gsl_vector_const_view cz =
      gsl_vector_const_subvector(&cfm_data, 2 * num_data, num_data);
  gsl_vector_const_view cl =
      gsl_vector_const_subvector(&cfm_data, 3 * num_data, num_data);
  gsl_vector_const_view cm =
      gsl_vector_const_subvector(&cfm_data, 4 * num_data, num_data);
  gsl_vector_const_view cn =
      gsl_vector_const_subvector(&cfm_data, 5 * num_data, num_data);

  gsl_matrix_const_view cx_mat =
      gsl_matrix_const_view_vector(&cx.vector, size1, size2);
  gsl_matrix_const_view cy_mat =
      gsl_matrix_const_view_vector(&cy.vector, size1, size2);
  gsl_matrix_const_view cz_mat =
      gsl_matrix_const_view_vector(&cz.vector, size1, size2);
  gsl_matrix_const_view cl_mat =
      gsl_matrix_const_view_vector(&cl.vector, size1, size2);
  gsl_matrix_const_view cm_mat =
      gsl_matrix_const_view_vector(&cm.vector, size1, size2);
  gsl_matrix_const_view cn_mat =
      gsl_matrix_const_view_vector(&cn.vector, size1, size2);

  cfm->force.x = gsl_interp2_scaled_helper(&cx_mat.matrix, s0, t0, ds, dt);
  cfm->force.y = gsl_interp2_scaled_helper(&cy_mat.matrix, s0, t0, ds, dt);
  cfm->force.z = gsl_interp2_scaled_helper(&cz_mat.matrix, s0, t0, ds, dt);
  cfm->moment.x = gsl_interp2_scaled_helper(&cl_mat.matrix, s0, t0, ds, dt);
  cfm->moment.y = gsl_interp2_scaled_helper(&cm_mat.matrix, s0, t0, ds, dt);
  cfm->moment.z = gsl_interp2_scaled_helper(&cn_mat.matrix, s0, t0, ds, dt);
}

void DvlAeroDatabase::LookupForceMomentCoeff3D(const gsl_vector &cfm_data,
                                               int32_t indices[], double du,
                                               double dv, double dw,
                                               ForceMoment *cfm) const {
  size_t num_data = cfm_data.size / 6U;
  gsl_vector_const_view cx = gsl_vector_const_subvector(&cfm_data, 0, num_data);
  gsl_vector_const_view cy =
      gsl_vector_const_subvector(&cfm_data, num_data, num_data);
  gsl_vector_const_view cz =
      gsl_vector_const_subvector(&cfm_data, 2 * num_data, num_data);
  gsl_vector_const_view cl =
      gsl_vector_const_subvector(&cfm_data, 3 * num_data, num_data);
  gsl_vector_const_view cm =
      gsl_vector_const_subvector(&cfm_data, 4 * num_data, num_data);
  gsl_vector_const_view cn =
      gsl_vector_const_subvector(&cfm_data, 5 * num_data, num_data);

  cfm->force.x = gsl_interp3_scaled_helper(&cx.vector, indices, du, dv, dw);
  cfm->force.y = gsl_interp3_scaled_helper(&cy.vector, indices, du, dv, dw);
  cfm->force.z = gsl_interp3_scaled_helper(&cz.vector, indices, du, dv, dw);
  cfm->moment.x = gsl_interp3_scaled_helper(&cl.vector, indices, du, dv, dw);
  cfm->moment.y = gsl_interp3_scaled_helper(&cm.vector, indices, du, dv, dw);
  cfm->moment.z = gsl_interp3_scaled_helper(&cn.vector, indices, du, dv, dw);
}
