#!/usr/bin/python
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

"""Checks that Bazel test targets are defined for all appropriate files."""

import os
import sys

from makani.lib.bazel import bazel_util


def main(unused_argv):
  # Build a list of test files.
  test_files = set()
  workspace_root = bazel_util.GetWorkspaceRoot()
  for dirpath, _, filenames in os.walk(workspace_root):
    for filename in filenames:
      if filename.endswith('_test.cc') or filename.endswith('_test.py'):
        workspace_filepath = os.path.relpath(os.path.join(dirpath, filename),
                                             workspace_root)
        test_files.add(workspace_filepath)

  # Query Bazel for names of actual test targets.
  file_list = bazel_util.Query('labels(srcs, tests(...))')
  tested_files = set()
  for line in file_list:
    tested_files.add(line[2:].replace(':', '/'))

  unused_files = (test_files - tested_files)
  if unused_files:
    print 'The following test files are not included in any Bazel targets:'
    for f in sorted(unused_files):
      print '  ' + f
    sys.exit(1)


if __name__ == '__main__':
  main(sys.argv)
