#!/bin/bash
# Copyright 2020 Makani Technologies LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# TODO: This job could be converted to use bazel run.

readonly BAZEL_PY_BIN="${MAKANI_HOME}/bazel-out/k8-py2-fastbuild/bin"
readonly AERO_DIR="${MAKANI_HOME}/analysis/aero"
readonly AIRFOIL_BIN="${BAZEL_BIN}/analysis/aero/xfoil/generate_airfoil_database"

bazel build //analysis/aero/xfoil:generate_airfoil_database

# Generate main wing foil.

# TODO: Currently this is done with the
# airfoils/convert_ref.m MATLAB script.  Eventually, we will uses MSES
# from within this repository.

# TODO: Make the Reynolds number for each foil depend on
# its chord length.

# Generate pylon foil.
"${BAZEL_PY_BIN}/analysis/aero/xfoil/generate_airfoil_database" \
  --airfoil_file="${AERO_DIR}/airfoils/mp6d.dat" \
  --reynolds_number=1000000.0 \
  --mach_number=0.044 \
  --n_crit=5 \
  --alphas_deg=-15,15,101 \
  --json_output_file="${AERO_DIR}/hover_model/airfoils/mp6d.json"

# # Generate horizontal stabilizer.
"${BAZEL_PY_BIN}/analysis/aero/xfoil/generate_airfoil_database" \
  --airfoil_file="${AERO_DIR}/airfoils/f8.dat" \
  --reynolds_number=1000000 \
  --mach_number=0.044 \
  --flap_deflections_deg=0,0,1 \
  --alphas_deg=-25,25,51 \
  --json_output_file="${AERO_DIR}/hover_model/airfoils/f8.json"

# Generate vertical stabilizer.
"${BAZEL_PY_BIN}/analysis/aero/xfoil/generate_airfoil_database" \
  --airfoil_file="${AERO_DIR}/airfoils/APPROXblade.rj8" \
  --reynolds_number=1000000 \
  --mach_number=0.044 \
  --hinge_position=0.67,0 \
  --flap_deflections_deg=-20,20,5 \
  --alphas_deg=-25,25,51 \
  --json_output_file="${AERO_DIR}/hover_model/airfoils/approx_blade_rj8.json"

# Generate high angle-of-attack foil for blending.
"${BAZEL_PY_BIN}/analysis/aero/xfoil/generate_airfoil_database" \
  --naca=0012 \
  --reynolds_number=1000000 \
  --mach_number=0.044 \
  --hinge_position=0.859,0 \
  --flap_deflections_deg=-25,25,5 \
  --alphas_deg=-180,180,401 \
  --stall_angles_deg=-15,15 \
  --json_output_file="${AERO_DIR}/airfoils/naca0012_high_aoa.json"
