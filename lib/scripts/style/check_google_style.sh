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
# Checks for some common style errors that are not caught by
# cpplint.py, such as proper indentation and spaces around binary
# operators.

# Removes comments and quotes from the code.
function remove_comments_and_quotes() {
  sed -e 's/\/\/\(.*\)$//' -e 's/"\(.*\)"/""/' -e 's/<\(.*\)>/<>/' $1
}

# Removes a specific construct in our C code (i.e. macros with brace
# initializers) because there's a bug in emacs' batch mode that makes
# it indent some of the macros improperly.
function remove_init_macros() {
  sed -e '/_INIT(/,/);/ s/.*//' -e '/_INIT(/,/);/ s/.*//g'
}

# Checks a regexp in the code after removing comments and quotes.
function check_style_regex() {
  remove_comments_and_quotes $1 | grep --color --line-number $2
}

# Source shflags.
source /opt/shflags-1.0.3/src/shflags

# Define flags.
DEFINE_boolean 'verbose' true 'Print the name of each file being checked.'

FLAGS "$@" || exit "$?"
eval set -- "${FLAGS_ARGV}"

bazel build --color no --noshow_progress //lib/scripts/style:autoformat
readonly AUTOFORMAT="${MAKANI_HOME}/bazel-out/k8-py2-fastbuild/bin/lib/scripts/style/autoformat"

num_errors=0
for f in "$@"; do
  if [[ -e "${f}" ]]; then
    if [[ "${FLAGS_verbose}" -eq "${FLAGS_TRUE}" ]]; then
      echo "Checking style in ${f}"
    fi

    # Check indentation.

    # If clang-format-3.8.1 exists, check if the files matches its
    # output.
    #
    # Using a pipe instead of a temporary file for clang-format's output
    # frequently produces the error "LLVM ERROR: IO failure on output stream."
    # when running as a Python subprocess.
    clang_output="$(mktemp)"
    hash clang-format-3.8.1 2> /dev/null \
      && clang-format-3.8.1 -style=google "${f}" > "${clang_output}" \
      && diff -sq "${clang_output}" "${f}" > /dev/null
    clang_status="$?"
    rm "${clang_output}"

    if [[ "${clang_status}" != 0 ]]; then
      # Any file supported by autoformat.py must be clang-formatted.
      if "${AUTOFORMAT}" --check_if_file_supported "${f}"; then
        echo "!-- ${f} should be clang-formatted. Run 'autoformat'"
        let num_errors++
        continue
      fi

      # If it does not match the output of clang-format, check if it
      # matches the output of Emacs.
      xemacs --batch "${f}" \
        -l "${MAKANI_HOME}/lib/scripts/style/apply-google-style.el" \
        -f apply-google-style 2> /dev/null \
        | sed 1d | remove_init_macros | diff <(cat "${f}" \
        | remove_init_macros) - | colordiff
      # Note that '3' below indicates the position of diff in the series
      # of pipes above.
      if [[ "${PIPESTATUS[3]}" != "0" ]]; then
        let num_errors++
      fi

      # Look for binary + or - without space around it.
      check_style_regex "${f}" '[a-df-zA-Z0-9]-[^->]' - && let num_errors++
      check_style_regex "${f}" "[a-zA-Z0-9_]+[^+']" - && let num_errors++
      check_style_regex "${f}" "[^+']+[-a-zA-Z0-9_\(]" - && let num_errors++
    fi
  fi
done

exit "${num_errors}"
