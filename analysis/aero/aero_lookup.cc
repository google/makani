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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "common/c_math/force_moment.h"
#include "common/c_math/linalg.h"
#include "common/c_math/vec3.h"
#include "common/macros.h"
#include "common/runfiles_dir.h"
#include "control/system_params.h"
#include "lib/json_load/load_params.h"
#include "sim/physics/aero.h"
#include "sim/physics/aero_frame.h"
#include "sim/sim_params.h"

DEFINE_string(small_deflection_database, "",
              "Name of the small deflection database to use.");
DEFINE_string(large_deflection_database, "",
              "Name of the large deflection database to use.");
DEFINE_string(alpha, "0.0",
              "Angle-of-attack in radians (add 'd' suffix for degrees).");
DEFINE_string(beta, "0.0",
              "Sideslip angle in radians (add 'd' suffix for degrees).");
DEFINE_string(omega_hat, "0.0, 0.0, 0.0", "Dimensionless angular rates.");
DEFINE_string(flaps, "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0",
              "Flap deflections in radians (add 'd' suffix for degrees).");
DEFINE_double(reynolds_number, 5.4e6, "Reynolds number.");
DEFINE_double(thrust_coeff, 0.0, "Thrust coefficient.");
DEFINE_bool(avl, true, "Display AVL coefficients.");
DEFINE_bool(dvl, true, "Display DVL coefficients.");
DEFINE_bool(output, true, "Display output coefficients.");

namespace {

void PrintAvlCoeffs(const AvlAeroCoeffs &coeffs) {
  printf(
      "\nCx   :%10.6f    Cy   :%10.6f    Cz   :%10.6f  "
      "  Cl   :%10.6f    Cm   :%10.6f    Cn   :%10.6f  "
      "\nCxp  :%10.6f    Cyp  :%10.6f    Czp  :%10.6f  "
      "  Clp  :%10.6f    Cmp  :%10.6f    Cnp  :%10.6f  "
      "\nCxq  :%10.6f    Cyq  :%10.6f    Czq  :%10.6f  "
      "  Clq  :%10.6f    Cmq  :%10.6f    Cnq  :%10.6f  "
      "\nCxr  :%10.6f    Cyr  :%10.6f    Czr  :%10.6f  "
      "  Clr  :%10.6f    Cmr  :%10.6f    Cnr  :%10.6f  ",
      coeffs.CX, coeffs.CY, coeffs.CZ, coeffs.CL, coeffs.CM, coeffs.CN,
      coeffs.CXp, coeffs.CYp, coeffs.CZp, coeffs.CLp, coeffs.CMp, coeffs.CNp,
      coeffs.CXq, coeffs.CYq, coeffs.CZq, coeffs.CLq, coeffs.CMq, coeffs.CNq,
      coeffs.CXr, coeffs.CYr, coeffs.CZr, coeffs.CLr, coeffs.CMr, coeffs.CNr);
  for (int32_t i = 0; i < ARRAYSIZE(coeffs.CXd); ++i) {
    printf(
        "\nCxd%d :%10.6f    Cyd%d :%10.6f    Czd%d :%10.6f  "
        "  Cld%d :%10.6f    Cmd%d :%10.6f    Cnd%d :%10.6f  ",
        i + 1, coeffs.CXd[i], i + 1, coeffs.CYd[i], i + 1, coeffs.CZd[i], i + 1,
        coeffs.CLd[i], i + 1, coeffs.CMd[i], i + 1, coeffs.CNd[i]);
  }
  printf("\n");
}

void PrintForceMoment(const ForceMoment &coeffs, const std::string &suffix) {
  printf(
      "\nCx%s:%10.6f    Cy%s:%10.6f    Cz%s:%10.6f  "
      "  Cl%s:%10.6f    Cm%s:%10.6f    Cn%s:%10.6f  ",
      suffix.c_str(), coeffs.force.x, suffix.c_str(), coeffs.force.y,
      suffix.c_str(), coeffs.force.z, suffix.c_str(), coeffs.moment.x,
      suffix.c_str(), coeffs.moment.y, suffix.c_str(), coeffs.moment.z);
}

void MixAvlCoeffs(const AvlAeroCoeffs &lower_coeffs,
                  const AvlAeroCoeffs &upper_coeffs, double lower_weight,
                  AvlAeroCoeffs *mixed_coeffs) {
  mixed_coeffs->CX =
      lower_weight * lower_coeffs.CX + (1.0 - lower_weight) * upper_coeffs.CX;
  mixed_coeffs->CY =
      lower_weight * lower_coeffs.CY + (1.0 - lower_weight) * upper_coeffs.CY;
  mixed_coeffs->CZ =
      lower_weight * lower_coeffs.CZ + (1.0 - lower_weight) * upper_coeffs.CZ;
  mixed_coeffs->CL =
      lower_weight * lower_coeffs.CL + (1.0 - lower_weight) * upper_coeffs.CL;
  mixed_coeffs->CM =
      lower_weight * lower_coeffs.CM + (1.0 - lower_weight) * upper_coeffs.CM;
  mixed_coeffs->CN =
      lower_weight * lower_coeffs.CN + (1.0 - lower_weight) * upper_coeffs.CN;
  mixed_coeffs->CXp =
      lower_weight * lower_coeffs.CXp + (1.0 - lower_weight) * upper_coeffs.CXp;
  mixed_coeffs->CYp =
      lower_weight * lower_coeffs.CYp + (1.0 - lower_weight) * upper_coeffs.CYp;
  mixed_coeffs->CZp =
      lower_weight * lower_coeffs.CZp + (1.0 - lower_weight) * upper_coeffs.CZp;
  mixed_coeffs->CLp =
      lower_weight * lower_coeffs.CLp + (1.0 - lower_weight) * upper_coeffs.CLp;
  mixed_coeffs->CMp =
      lower_weight * lower_coeffs.CMp + (1.0 - lower_weight) * upper_coeffs.CMp;
  mixed_coeffs->CNp =
      lower_weight * lower_coeffs.CNp + (1.0 - lower_weight) * upper_coeffs.CNp;
  mixed_coeffs->CXq =
      lower_weight * lower_coeffs.CXq + (1.0 - lower_weight) * upper_coeffs.CXq;
  mixed_coeffs->CYq =
      lower_weight * lower_coeffs.CYq + (1.0 - lower_weight) * upper_coeffs.CYq;
  mixed_coeffs->CZq =
      lower_weight * lower_coeffs.CZq + (1.0 - lower_weight) * upper_coeffs.CZq;
  mixed_coeffs->CLq =
      lower_weight * lower_coeffs.CLq + (1.0 - lower_weight) * upper_coeffs.CLq;
  mixed_coeffs->CMq =
      lower_weight * lower_coeffs.CMq + (1.0 - lower_weight) * upper_coeffs.CMq;
  mixed_coeffs->CNq =
      lower_weight * lower_coeffs.CNq + (1.0 - lower_weight) * upper_coeffs.CNq;
  mixed_coeffs->CXr =
      lower_weight * lower_coeffs.CXr + (1.0 - lower_weight) * upper_coeffs.CXr;
  mixed_coeffs->CYr =
      lower_weight * lower_coeffs.CYr + (1.0 - lower_weight) * upper_coeffs.CYr;
  mixed_coeffs->CZr =
      lower_weight * lower_coeffs.CZr + (1.0 - lower_weight) * upper_coeffs.CZr;
  mixed_coeffs->CLr =
      lower_weight * lower_coeffs.CLr + (1.0 - lower_weight) * upper_coeffs.CLr;
  mixed_coeffs->CMr =
      lower_weight * lower_coeffs.CMr + (1.0 - lower_weight) * upper_coeffs.CMr;
  mixed_coeffs->CNr =
      lower_weight * lower_coeffs.CNr + (1.0 - lower_weight) * upper_coeffs.CNr;
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    mixed_coeffs->CXd[i] = lower_weight * lower_coeffs.CXd[i] +
                           (1.0 - lower_weight) * upper_coeffs.CXd[i];
    mixed_coeffs->CYd[i] = lower_weight * lower_coeffs.CYd[i] +
                           (1.0 - lower_weight) * upper_coeffs.CYd[i];
    mixed_coeffs->CZd[i] = lower_weight * lower_coeffs.CZd[i] +
                           (1.0 - lower_weight) * upper_coeffs.CZd[i];
    mixed_coeffs->CLd[i] = lower_weight * lower_coeffs.CLd[i] +
                           (1.0 - lower_weight) * upper_coeffs.CLd[i];
    mixed_coeffs->CMd[i] = lower_weight * lower_coeffs.CMd[i] +
                           (1.0 - lower_weight) * upper_coeffs.CMd[i];
    mixed_coeffs->CNd[i] = lower_weight * lower_coeffs.CNd[i] +
                           (1.0 - lower_weight) * upper_coeffs.CNd[i];
  }
}

void MixDvlCoeffs(const DvlAeroCoeffs &lower_coeffs,
                  const DvlAeroCoeffs &upper_coeffs, double lower_weight,
                  DvlAeroCoeffs *mixed_coeffs) {
  ForceMomentLinComb(lower_weight, &lower_coeffs.cfm, 1.0 - lower_weight,
                     &upper_coeffs.cfm, &mixed_coeffs->cfm);
  ForceMomentLinComb(lower_weight, &lower_coeffs.dcfm_dp, 1.0 - lower_weight,
                     &upper_coeffs.dcfm_dp, &mixed_coeffs->dcfm_dp);
  ForceMomentLinComb(lower_weight, &lower_coeffs.dcfm_dq, 1.0 - lower_weight,
                     &upper_coeffs.dcfm_dq, &mixed_coeffs->dcfm_dq);
  ForceMomentLinComb(lower_weight, &lower_coeffs.dcfm_dr, 1.0 - lower_weight,
                     &upper_coeffs.dcfm_dr, &mixed_coeffs->dcfm_dr);
  for (int32_t i = 0; i < kNumFlaps; ++i) {
    ForceMomentLinComb(lower_weight, &lower_coeffs.flap_cfms[i],
                       1.0 - lower_weight, &upper_coeffs.flap_cfms[i],
                       &mixed_coeffs->flap_cfms[i]);
    ForceMomentLinComb(lower_weight, &lower_coeffs.flap_dcfm_dps[i],
                       1.0 - lower_weight, &upper_coeffs.flap_dcfm_dps[i],
                       &mixed_coeffs->flap_dcfm_dps[i]);
    ForceMomentLinComb(lower_weight, &lower_coeffs.flap_dcfm_dqs[i],
                       1.0 - lower_weight, &upper_coeffs.flap_dcfm_dqs[i],
                       &mixed_coeffs->flap_dcfm_dqs[i]);
    ForceMomentLinComb(lower_weight, &lower_coeffs.flap_dcfm_drs[i],
                       1.0 - lower_weight, &upper_coeffs.flap_dcfm_drs[i],
                       &mixed_coeffs->flap_dcfm_drs[i]);
  }
}

void PrintDvlCoeffs(const DvlAeroCoeffs &coeffs) {
  PrintForceMoment(coeffs.cfm, "   ");
  PrintForceMoment(coeffs.dcfm_dp, "dp ");
  PrintForceMoment(coeffs.dcfm_dq, "dq ");
  PrintForceMoment(coeffs.dcfm_dr, "dr ");
  for (int32_t i = 0; i < ARRAYSIZE(coeffs.flap_cfms); ++i) {
    PrintForceMoment(coeffs.flap_cfms[i], std::to_string(i + 1) + "  ");
    PrintForceMoment(coeffs.flap_dcfm_dps[i], std::to_string(i + 1) + "dp");
    PrintForceMoment(coeffs.flap_dcfm_dqs[i], std::to_string(i + 1) + "dq");
    PrintForceMoment(coeffs.flap_dcfm_drs[i], std::to_string(i + 1) + "dr");
  }
  printf("\n");
}

void PrintAeroSimParams(const AeroSimParams &params) {
  printf("\nsmall_deflection_databases:");
  for (int32_t i = 0; i < MAX_SMALL_DEFLECTION_DATABASES; ++i) {
    if (*params.small_deflection_databases[i].name) {
      printf("\n  %s", params.small_deflection_databases[i].name);
    }
  }
  printf("\nlarge_deflection_databases:");
  for (int32_t i = 0; i < MAX_LARGE_DEFLECTION_DATABASES; ++i) {
    if (*params.large_deflection_databases[i].name) {
      printf("\n  %s", params.large_deflection_databases[i].name);
    }
  }
  printf("\nextra_drag_coeff: %f", params.coeff_offsets.CD);
  printf("\nextra_side_force_coeff: %f", params.coeff_offsets.CC);
  printf("\nextra_lift_coeff: %f", params.coeff_offsets.CL);
  printf("\nextra_roll_moment_coeff: %f", params.coeff_offsets.Cl);
  printf("\nextra_pitch_moment_coeff: %f", params.coeff_offsets.Cm);
  printf("\nextra_yaw_moment_coeff: %f", params.coeff_offsets.Cn);
  printf("\nextra_dcl_dbeta: %f", params.coeff_offsets.dCldbeta);
  printf("\nextra_dcm_dalpha: %f", params.coeff_offsets.dCmdalpha);
  printf("\nextra_dcn_dbeta: %f", params.coeff_offsets.dCndbeta);
  printf("\n");
}

void PrintAeroInputs(double reynolds_number, double alpha, double beta,
                     const Vec3 &omega_hat, const Vec &flaps,
                     double thrust_coeff) {
  printf("\nreynolds number: %f", reynolds_number);
  printf(
      "\nalpha: % f rad  (%f deg)"
      "\nbeta : % f rad  (%f deg)",
      alpha, 180.0 / M_PI * alpha, beta, 180.0 / M_PI * beta);
  printf("\np_hat: % f\nq_hat: % f\nr_hat: % f", omega_hat.x, omega_hat.y,
         omega_hat.z);
  for (int32_t i = 0; i < flaps.length; ++i) {
    printf("\nd%d   : % f rad  (%f deg)", i + 1, VecGet(&flaps, i),
           180.0 / M_PI * VecGet(&flaps, i));
  }
  printf("\nthrust coeff: %f", thrust_coeff);
  printf("\n");
}

double AngleStringToDouble(const std::string &s) {
  if (s.back() == 'd') {
    return M_PI / 180.0 * std::stof(s.substr(0, s.length() - 1));
  } else {
    return std::stof(s);
  }
}

std::string GetToken(const std::string &s, const std::string &delimiter,
                     size_t start, size_t *next) {
  size_t delimiter_pos = s.find(delimiter, start);
  if (delimiter_pos == std::string::npos || delimiter_pos == s.length() - 1) {
    *next = std::string::npos;
    return s.substr(start, std::string::npos);
  } else {
    *next = delimiter_pos + 1;
    return s.substr(start, delimiter_pos - start);
  }
}

void ParseInputs(double *reynolds_number, double *alpha, double *beta,
                 Vec3 *omega_hat, Vec *flaps, double *thrust_coeff) {
  *reynolds_number = FLAGS_reynolds_number;
  CHECK_LT(0.0, *reynolds_number) << "The Reynolds number must be positive.";

  *alpha = AngleStringToDouble(FLAGS_alpha);
  *beta = AngleStringToDouble(FLAGS_beta);

  size_t pos = 0U;
  omega_hat->x = std::stof(GetToken(FLAGS_omega_hat, ",", pos, &pos));
  omega_hat->y = std::stof(GetToken(FLAGS_omega_hat, ",", pos, &pos));
  omega_hat->z = std::stof(GetToken(FLAGS_omega_hat, ",", pos, &pos));

  pos = 0U;
  int32_t num_flaps = 0;
  while (num_flaps < flaps->max_length) {
    std::string token = GetToken(FLAGS_flaps, ",", pos, &pos);
    flaps->d[num_flaps] = AngleStringToDouble(token);
    ++num_flaps;
    if (pos == std::string::npos) break;
  }
  flaps->length = num_flaps;
  *thrust_coeff = FLAGS_thrust_coeff;

  CHECK_EQ(kNumFlaps, num_flaps) << "You must input exactly "
                                 << static_cast<int32_t>(kNumFlaps)
                                 << " flap deflections.";
}

}  // namespace

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // Specify runfiles dir for database access.
  SetRunfilesDirFromBinaryPath(argv[0]);

  // Load runtime parameters.
  json_load::LoadSystemParams(GetSystemParamsUnsafe());
  json_load::LoadSimParams(GetSimParamsUnsafe());

  // Select aerodynamic databases.  Only use single database when
  // setting them from the command line.
  if (!FLAGS_small_deflection_database.empty()) {
    strncpy(GetSimParamsUnsafe()->aero_sim.small_deflection_databases[0].name,
            FLAGS_small_deflection_database.c_str(),
            FLAGS_small_deflection_database.length() + 1);
    if (MAX_SMALL_DEFLECTION_DATABASES > 1) {
      GetSimParamsUnsafe()->aero_sim.small_deflection_databases[1].name[0] = 0;
    }
  }
  if (!FLAGS_large_deflection_database.empty()) {
    strncpy(GetSimParamsUnsafe()->aero_sim.large_deflection_databases[0].name,
            FLAGS_large_deflection_database.c_str(),
            FLAGS_large_deflection_database.length() + 1);
    if (MAX_LARGE_DEFLECTION_DATABASES > 1) {
      GetSimParamsUnsafe()->aero_sim.large_deflection_databases[1].name[0] = 0;
    }
  }

  // Always calculate both the AVL and DVL database values.
  GetSimParamsUnsafe()->aero_sim.force_use_both_databases = true;

  Aero aero(GetSimParams()->aero_sim);

  double alpha, beta, reynolds_number, thrust_coeff;
  Vec3 omega_hat;
  VEC_INIT(kNumFlaps, flaps, {0});
  ParseInputs(&reynolds_number, &alpha, &beta, &omega_hat, &flaps,
              &thrust_coeff);

  ForceMoment force_moment_coeff;
  RawAeroCoeffs raw_aero_coeffs;
  aero.CalcForceMomentCoeff(alpha, beta, omega_hat, flaps, reynolds_number,
                            &force_moment_coeff, thrust_coeff,
                            &raw_aero_coeffs);

  printf("\nSimulation parameters:");
  PrintAeroSimParams(GetSimParams()->aero_sim);

  printf("\nInputs:");
  PrintAeroInputs(reynolds_number, alpha, beta, omega_hat, flaps, thrust_coeff);

  if (FLAGS_avl) {
    printf("\nSmall deflection coefficients:");
    AvlAeroCoeffs mixed_small_deflections_coeffs;
    MixAvlCoeffs(raw_aero_coeffs.lower_reynolds_small_deflections_aero_coeffs,
                 raw_aero_coeffs.upper_reynolds_small_deflections_aero_coeffs,
                 raw_aero_coeffs.lower_reynolds_small_deflections_weight,
                 &mixed_small_deflections_coeffs);

    PrintAvlCoeffs(mixed_small_deflections_coeffs);
  }

  if (FLAGS_dvl) {
    printf("\nDVL coefficients:");
    DvlAeroCoeffs mixed_dvl_coeffs;
    MixDvlCoeffs(raw_aero_coeffs.lower_reynolds_dvl_aero_coeffs,
                 raw_aero_coeffs.upper_reynolds_dvl_aero_coeffs,
                 raw_aero_coeffs.lower_reynolds_dvl_weight, &mixed_dvl_coeffs);
    PrintDvlCoeffs(mixed_dvl_coeffs);
  }

  if (FLAGS_output) {
    printf("\nOutput coefficients:");
    PrintForceMoment(force_moment_coeff, "   ");
    Vec3 force_coeff_w;
    RotBToW(&force_moment_coeff.force, alpha, beta, &force_coeff_w);
    printf("\nCD   :%10.6f    CL   :%10.6f", -force_coeff_w.x,
           -force_coeff_w.z);
  }

  printf("\n");

  return EXIT_SUCCESS;
}
