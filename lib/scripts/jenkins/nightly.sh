#!/bin/bash -xu
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

#
# Run nightly batch simulation tests.
#
# The "-xe" flags above make Bash behave similarly to Jenkin's default
# behavior; it runs Bash in debug mode and errors out on the first
# error.  The "-u" option makes it an error to try to expand an unset
# variable (for example, if WORKSPACE isn't set).

if [[ $# < 1 ]]; then
  readonly JOB_NAME='nightly'
else
  readonly JOB_NAME="$1"
fi
readonly GCS_DIR="gs://makani/dashboard/${JOB_NAME}"

export MAKANI_HOME="${WORKSPACE}"
source "${MAKANI_HOME}/lib/scripts/jenkins/test_command.sh"

declare -i result=0
output_dir_prefix="${MAKANI_HOME}/logs/batch_sim_reports"
git clean -dxf

# Run onshore crosswind sweeps with Monte Carlo variation of system parameters
# and class C Turbsim wind databases.
RUN_NAME='onshore_cw_sweeps_monte_carlo'
( set -e
  output_dir="${output_dir_prefix}/${RUN_NAME}"
  # TODO: Consider converting all of these tasks to use 'bazel run'.
  bazel_build analysis/crosswind_batch_sims:crosswind_sweeps_batch_sim
  ./bazel-out/k8-py2-fastbuild/bin/analysis/crosswind_batch_sims/crosswind_sweeps_batch_sim \
    --sim_name="${JOB_NAME}_${RUN_NAME}" \
    --max_jobs_per_worker=5 \
    --output_dir="${output_dir}" \
    --keep_h5_logs \
    --monte_carlo \
    --turbsim_folder 20181010-007-Initial_ParkerRanch_turbC_refht21m \
    --parameter_seed 12345

  # Export HTML to Cloud Storage for easy access.  We delete the
  # directory twice because the first rm just deletes the contents of
  # the directory, while the second rm deletes the folder itself.
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m cp -R "${output_dir}" "${GCS_DIR}/${RUN_NAME}"
)
(( result |= $? ))

# Run onshore crosswind sweeps with deterministic variation of system parameters
# and Dryden wind model.
RUN_NAME='onshore_cw_sweeps'
( set -e
output_dir="${output_dir_prefix}/${RUN_NAME}"
  bazel_build analysis/crosswind_batch_sims:crosswind_sweeps_batch_sim
  ./bazel-out/k8-py2-fastbuild/bin/analysis/crosswind_batch_sims/crosswind_sweeps_batch_sim \
    --sim_name="${JOB_NAME}_${RUN_NAME}" \
    --max_jobs_per_worker=5 \
    --output_dir="${output_dir}" \
    --keep_h5_logs

  # Export HTML to Cloud Storage for easy access.  See comment above
  # regarding the multiple rm statements.
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m cp -R "${output_dir}" "${GCS_DIR}/${RUN_NAME}"
)
(( result |= $? ))

# Run offshore crosswind sweeps with Monte Carlo variation of system parameters
# and class C Turbsim wind databases.
RUN_NAME='offshore_cw_sweeps_monte_carlo'
( set -e
  output_dir="${output_dir_prefix}/${RUN_NAME}"
  bazel_build analysis/crosswind_batch_sims:crosswind_sweeps_batch_sim
  ./bazel-out/k8-py2-fastbuild/bin/analysis/crosswind_batch_sims/crosswind_sweeps_batch_sim \
    --sim_name="${JOB_NAME}_${RUN_NAME}" \
    --max_jobs_per_worker=5 \
    --output_dir="${output_dir}" \
    --keep_h5_logs \
    --monte_carlo \
    --offshore \
    --turbsim_folder 20190619-012-Offshore_turb10_refht32m \
    --parameter_seed 12345

  # Export HTML to Cloud Storage for easy access.  We delete the
  # directory twice because the first rm just deletes the contents of
  # the directory, while the second rm deletes the folder itself.
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m cp -R "${output_dir}" "${GCS_DIR}/${RUN_NAME}"
)
(( result |= $? ))

# Run hover disturbances.
# NOTE: Consider expanding the wind envelope by increasing  the
# range of wind speed variation to negative values. Develop software to support
# negative wind speeds.
RUN_NAME='hover_disturbances'
( set -e
  output_dir="${output_dir_prefix}/${RUN_NAME}"
  bazel_build analysis:hover_disturbances_batch_sim
  ./bazel-out/k8-py2-fastbuild/bin/analysis/hover_disturbances_batch_sim \
    --sim_name="${JOB_NAME}_${RUN_NAME}" \
    --wind_speeds='0.0, 15.0, 7' \
    --max_jobs_per_worker=5 \
    --output_dir="${output_dir}" \
    --keep_h5_logs

  # Export HTML to Cloud Storage for easy access.  See comment above
  # regarding the multiple rm statements.
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m cp -R "${output_dir}" "${GCS_DIR}/${RUN_NAME}"
)
(( result |= $? ))

# Run power curve.
RUN_NAME='power_curve'
( set -e
  output_dir="${output_dir_prefix}/${RUN_NAME}"
  bazel_build analysis:power_curve_batch_sim
  ./bazel-out/k8-py2-fastbuild/bin/analysis/power_curve_batch_sim \
    --sim_name="${JOB_NAME}_${RUN_NAME}" \
    --wind_speeds='3.0, 20.0, 20' \
    --joystick_throttles='0.5, 1.0, 5' \
    --max_jobs_per_worker=5 \
    --output_dir="${output_dir}" \
    --keep_h5_logs

  # Export HTML to Cloud Storage for easy access.  See comment above
  # regarding the multiple rm statements.
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m rm -R "${GCS_DIR}/${RUN_NAME}" || true
  gsutil -m cp -R "${output_dir}" "${GCS_DIR}/${RUN_NAME}"
)
(( result |= $? ))

# Generate coverage report for unit tests.
# TODO: enable this test once we figure out how to get it working
# on stretch.
# ( set -e
#   ./lib/scripts/run_lcov.py --run_tests --norun_sim --output_dir logs/lcov_test
#   # Export HTML to Cloud Storage for easy access.  See comment above
#   # regarding the multiple rm statements.
#   gsutil -m rm -R "${GCS_DIR}/lcov_test" || true
#   gsutil -m rm -R "${GCS_DIR}/lcov_test" || true
#   gsutil -m cp -R logs/lcov_test "${GCS_DIR}/lcov_test"
# )
# (( result |= $? ))

# Generate coverage report for simulations.
# TODO(b/137200989): Enable once we figure out a solution to get it working on
# a stretch worker without crashing.
# ( set -e
#   ./lib/scripts/run_lcov.py --norun_tests --run_sim --sim_time 1500 \
#     --output_dir logs/lcov_sim
#   # Export HTML to Cloud Storage for easy access.  See comment above
#   # regarding the multiple rm statements.
#   gsutil -m rm -R "${GCS_DIR}/lcov_sim" || true
#   gsutil -m rm -R "${GCS_DIR}/lcov_sim" || true
#   gsutil -m cp -R logs/lcov_sim "${GCS_DIR}/lcov_sim"
# )
# (( result |= $? ))

exit ${result}
