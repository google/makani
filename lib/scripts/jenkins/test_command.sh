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

#
# This library contains common functions for testing commands on Jenkins and
# submitting reviews to Gerrit.  It makes use of several environment variables
# that are set by Jenkins; see
# https://wiki.jenkins.io/display/JENKINS/Building+a+software+project.

# Note: If you change this it also needs to be updated in the configuration
# of build-flow projects on Jenkins.
readonly CREDS="${HOME}/.makanicreds"

# Clean up bazel output for Jenkins:
#   --color no: Disable color control codes.
#   --noshow_progress: Disable interactive progress output.
#   --verbose_failures: Get more detailed failure output.
#   --test_output=errors: Show test stderr in the log.
readonly BAZEL_OPTS="--color no --noshow_progress --verbose_failures --test_output=errors"

# Helpers to use bazel build and bazel run with BAZEL_OPTS enabled.
function bazel_build() {
  bazel build "${BAZEL_OPTS}" "$@"
}

function bazel_run() {
  bazel run "${BAZEL_OPTS}" "$@"
}

# Obtain a URL for the build. Use go link to preserve link integrity when we
# move the Jenkins host.
function build_url() {
  echo "http://FIXME/makanijenkins/job/${JOB_NAME}/${BUILD_NUMBER}"
}

# Obtain a URL for the build console.
function console_url() {
  echo "$(build_url)/console#footer"
}

# Post a failed commit review to Gerrit.
function fail_commit() {
  post-review -creds="${CREDS}" -verified=-1 "${COMMIT}" \
    "Presubmit failed: $*; Jenkins URL: $(console_url)" && exit 1
}

# Post a successful commit review to Gerrit.
function pass_commit() {
  post-review -creds="${CREDS}" -verified=+1 "${COMMIT}" 'Presubmit passed.'
}

# Reset the current repository to the commit given by the global
# variable COMMIT, aggressively clean the repository, and execute a
# command given by the function arguments.  Fail the commit of the
# command reports errors.
function test_command() {
  git reset "${COMMIT}"
  git clean -dxf
  git checkout .
  # You must use the || operator here, rather than a second line with an
  # if statement, because otherwise, eval $@ will just exit this script
  # if $@ exits.
  eval "$@" || fail_commit "$@"
}

# Post a failed embedded commit review to Gerrit.
function fail_commit_embedded() {
  post-review -creds="${CREDS}" -embedded=-1 "${COMMIT}" \
    "Embedded presubmit failed: $*; Jenkins URL: $(console_url)"
  exit 1
}

# Post a successful embedded commit review to Gerrit.
function pass_commit_embedded() {
  post-review -creds="${CREDS}" -embedded=+1 "${COMMIT}" \
    "Embedded presubmit passed; Jenkins URL: $(console_url)"
}

# Reset the current repository to the commit given by the global
# variable COMMIT, aggressively clean the repository, and execute a
# command given by the function arguments.  Fail the commit of the
# command reports errors.
function test_command_embedded() {
  git reset "${COMMIT}"
  git clean -dxf
  git checkout .
  # You must use the || operator here, rather than a second line with an
  # if statement, because otherwise, eval $@ will just exit this script
  # if $@ exits.
  eval "$@" || fail_commit_embedded "$@"
  pass_commit_embedded
}

# Post a Gerrit review saying the regression test failed.
function fail_commit_regression() {
  post-review -creds="${CREDS}" -regression=-1 "${COMMIT}" \
    "Regression test failed: $*; Jenkins URL: $(console_url)" \
    && exit 1
}

# Post a Gerrit review saying log files do not match.
function warn_commit_regression() {
  # NOTE: Line-breaking as below is the only way I could get
  # line breaks to appear on Gerrit.
  post-review -creds="${CREDS}" -regression=+1 "${COMMIT}" \
    "Logs do not match last commit: $*
Jenkins URL: $(console_url)
Plots: $(build_url)/Plots"
}

# Post a Gerrit review saying log files match.
function pass_commit_regression() {
  post-review -creds="${CREDS}" -regression=+2 "${COMMIT}" \
    "Logs match last commit; Jenkins URL: $(console_url)"
}

# Executes a command for the previous and current commit.  Expects to
# find a log file in logs/last.h5 after each command and compares the
# two generated logs.  Exits with a failure if either simulation fails
# or the commits do not match.
function test_command_regression() {
  # Checkout the commit directly preceding this one, run a simulation,
  # and copy the resulting log file into prev.h5.
  git checkout "${COMMIT}"
  git checkout "$(git log -n 1 --skip 1 --pretty='%H')"
  git clean -dxf
  git checkout .
  # You must use the || operator here -- see presubmit_verify.sh.
  eval "$@" || fail_commit_regression "$@"
  cp logs/last.h5 logs/prev.h5

  # Run this commit, then compare the logs.
  git checkout "${COMMIT}"
  git clean -dxf -e logs
  git checkout .
  eval "$@" || fail_commit_regression "$@"

  bazel_build //analysis/util:h5comp
  if ! bazel-out/k8-py2-fastbuild/bin/analysis/util/h5comp \
    --config analysis/util/h5comp_default_config.json \
    --truncate logs/last.h5 logs/prev.h5 > logs/diff.txt; then
    fail_commit_regression "$@"
    exit 1
  fi

  bazel_build analysis/util:plot_regression_output
  if ! bazel-out/k8-py2-fastbuild/bin/analysis/util/plot_regression_output \
    --old_log logs/prev.h5 \
    --new_log logs/last.h5 \
    --output_dir logs/regression; then
    fail_commit_regression "$@"
    exit 1
  fi

  bazel_build analysis/util:validate_flight
  if ! bazel-out/k8-py2-fastbuild/bin/analysis/util/validate_flight logs/last.h5; then
    fail_commit_regression "$@"
    exit 1
  fi

  # Run suite of crosswind batch sim scoring functions on last.h5.  Strip color
  # from command output.
  # TODO: Compare scores between last and prev.
  bazel_build analysis/crosswind_batch_sims:score_log_file
  bazel-out/k8-py2-fastbuild/bin/analysis/crosswind_batch_sims/score_log_file \
      --benchmark logs/last.h5 \
      | sed -r "s/\x1B\[([0-9]{1,2}(;[0-9]{1,2})?)?[m|K]//g"
  if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    fail_commit_regression "$@"
    exit 1
  fi

  # Run scoring restricted to using controller telemetry only. This validates
  # scoring functions will not break for flight logs.
  bazel-out/k8-py2-fastbuild/bin/analysis/crosswind_batch_sims/score_log_file \
      --control_telem_only=True logs/last.h5 \
      | sed -r "s/\x1B\[([0-9]{1,2}(;[0-9]{1,2})?)?[m|K]//g"
  if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    fail_commit_regression "$@"
    exit 1
  fi

  if [[ -s logs/diff.txt ]]; then
    cat logs/diff.txt
    warn_commit_regression "$@"
  else
    pass_commit_regression
  fi
}
