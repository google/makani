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

#include "sim/physics/delta_coeff_aero_database.h"

#include <glog/logging.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include <math.h>
#include <stdint.h>

#include <string>
#include <vector>

#include "common/c_math/vec3.h"
#include "lib/json_load/json_array_loader.h"
#include "lib/json_load/json_load_or_die.h"
#include "sim/math/util.h"
#include "sim/physics/aero_frame.h"
#include "system/labels.h"

namespace {

class DeltaCoeffJsonArrayLoader : public json_load::JsonArrayLoader {
 public:
  DeltaCoeffJsonArrayLoader(int32_t num_alphas, int32_t num_deltas,
                            json_t *root)
      : json_load::JsonArrayLoader(num_alphas, num_deltas, root) {}
  virtual ~DeltaCoeffJsonArrayLoader() {}

  void LoadAlphaDeltaMatrix(const std::string &field_name,
                            bool dest_mask[kNumFlaps],
                            gsl_matrix *dest[kNumFlaps]) {
    CHECK_NOTNULL(dest);

    json_t *field = json_load::LoadFieldOrDie(root_, field_name);
    CHECK(json_is_array(field));

    int num_tables = 0;
    for (int flap = 0; flap < kNumFlaps; ++flap) {
      if (dest_mask[flap]) {
        ++num_tables;
      }
    }

    CHECK_EQ(num_tables, json_array_size(field));

    int table = 0;
    for (int flap = 0; flap < kNumFlaps; ++flap) {
      if (dest_mask[flap]) {
        json_t *child = json_array_get(field, table++);
        CHECK(json_is_array(child));
        LoadMatrixFromField(child, dest[flap]);
      }
    }
  }

 private:
  DISALLOW_COPY_AND_ASSIGN(DeltaCoeffJsonArrayLoader);
};

}  // namespace

DeltaCoeffAeroDatabase::DeltaCoeffAeroDatabase(const std::string &filename)
    : reynolds_number_(),
      alphads_(),
      deltads_(),
      CX_(),
      CY_(),
      CZ_(),
      CL_(),
      CM_(),
      CN_() {
  json_t *delta_coeff_json = json_load::LoadFileOrDie(filename.c_str());
  int32_t num_alphads =
      json_load::LoadInt32OrDie(delta_coeff_json, "num_alphas");
  int32_t num_deltads =
      json_load::LoadInt32OrDie(delta_coeff_json, "num_deltas");
  json_t *flap_list = json_load::LoadFieldOrDie(delta_coeff_json, "flap_list");
  CHECK(json_is_array(flap_list));
  CHECK(static_cast<int32_t>(json_array_size(flap_list)) == kNumFlaps)
      << "Detected the wrong number of flaps in database. "
      << "The delta coeff aero database is expecting "
      << static_cast<int32_t>(kNumFlaps) << " flaps.";

  std::vector<int32_t> flaps_enabled;
  json_load::LoadArray1D_Int32OrDie(flap_list, kNumFlaps, &flaps_enabled);
  for (int16_t i = 0; i < kNumFlaps; ++i) {
    applicable_flaps_[i] = (flaps_enabled[i] != 0);
  }

  alphads_ = gsl_vector_alloc(num_alphads);
  deltads_ = gsl_vector_alloc(num_deltads);

  int table_count = 0;
  for (int flap = 0; flap < kNumFlaps; ++flap) {
    if (applicable_flaps_[flap]) {
      CX_[flap] = gsl_matrix_alloc(num_alphads, num_deltads);
      CY_[flap] = gsl_matrix_alloc(num_alphads, num_deltads);
      CZ_[flap] = gsl_matrix_alloc(num_alphads, num_deltads);
      CL_[flap] = gsl_matrix_alloc(num_alphads, num_deltads);
      CM_[flap] = gsl_matrix_alloc(num_alphads, num_deltads);
      CN_[flap] = gsl_matrix_alloc(num_alphads, num_deltads);
      table_count++;

      if (!CX_[flap] || !CY_[flap] || !CZ_[flap] || !CL_[flap] || !CM_[flap] ||
          !CN_[flap]) {
        CHECK(false)
            << "Could not allocate memory for delta coeff aero database.";
      }
    }
  }

  // Read alpha and delta values.
  DeltaCoeffJsonArrayLoader loader(num_alphads, num_deltads, delta_coeff_json);
  loader.LoadVector("alphads", alphads_);
  loader.LoadVector("deltads", deltads_);

  loader.LoadAlphaDeltaMatrix("CX_offset", applicable_flaps_, CX_);
  loader.LoadAlphaDeltaMatrix("CY_offset", applicable_flaps_, CY_);
  loader.LoadAlphaDeltaMatrix("CZ_offset", applicable_flaps_, CZ_);
  loader.LoadAlphaDeltaMatrix("Cl_offset", applicable_flaps_, CL_);
  loader.LoadAlphaDeltaMatrix("Cm_offset", applicable_flaps_, CM_);
  loader.LoadAlphaDeltaMatrix("Cn_offset", applicable_flaps_, CN_);

  json_decref(delta_coeff_json);
  json_decref(flap_list);

  CHECK(IsValid()) << "Invalid delta coeff aero database (" << filename << ").";
}

DeltaCoeffAeroDatabase::~DeltaCoeffAeroDatabase() {
  gsl_vector_free(alphads_);
  gsl_vector_free(deltads_);

  for (int flap = 0; flap < kNumFlaps; ++flap) {
    if (applicable_flaps_[flap]) {
      gsl_matrix_free(CX_[flap]);
      gsl_matrix_free(CY_[flap]);
      gsl_matrix_free(CZ_[flap]);
      gsl_matrix_free(CL_[flap]);
      gsl_matrix_free(CM_[flap]);
      gsl_matrix_free(CN_[flap]);
    }
  }
}

// Returns true if the delta coeff aerodynamic database passes the checks:
//
// - Currently there are no checks.

bool DeltaCoeffAeroDatabase::IsValid() const { return 1; }

void DeltaCoeffAeroDatabase::AdjustForceMomentCoeff(
    double alpha, const Vec &flaps, ForceMoment *force_moment_coeff) const {
  CHECK_EQ(flaps.length, kNumFlaps)
      << "The delta coeff aero database is expecting "
      << static_cast<int>(kNumFlaps) << " flaps, "
      << "but was passed " << flaps.length << " flaps.";
  double alphad = 180.0 / M_PI * alpha;

  // Note that the alphads for delta coeffs have a different range than
  // that of the overall database. In the case that the range is narrower,
  // the alpha value saturates.
  double s = gsl_interp_index(alphads_, alphad);
  for (int f = 0; f < kNumFlaps; ++f) {
    if (applicable_flaps_[f]) {
      double flapd = flaps.d[f] / M_PI * 180.0;
      double t = gsl_interp_index(deltads_, flapd);

      force_moment_coeff->force.x +=
          gsl_interp2_scaled(s, t, CX_[f], kInterpOptionSaturate);
      force_moment_coeff->force.y +=
          gsl_interp2_scaled(s, t, CY_[f], kInterpOptionSaturate);
      force_moment_coeff->force.z +=
          gsl_interp2_scaled(s, t, CZ_[f], kInterpOptionSaturate);
      force_moment_coeff->moment.x +=
          gsl_interp2_scaled(s, t, CL_[f], kInterpOptionSaturate);
      force_moment_coeff->moment.y +=
          gsl_interp2_scaled(s, t, CM_[f], kInterpOptionSaturate);
      force_moment_coeff->moment.z +=
          gsl_interp2_scaled(s, t, CN_[f], kInterpOptionSaturate);
    }
  }
}
