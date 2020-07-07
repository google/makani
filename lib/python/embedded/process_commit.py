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

"""Build and execute embedded software test functions for a given commit."""

import sys

import gflags
from makani.lib.python.embedded import build
from makani.lib.python.embedded import test_runner


FLAGS = gflags.FLAGS

gflags.DEFINE_string('workpath', None,
                     'Full path to source tree.')


def main(argv):
  try:
    argv = FLAGS(test_runner.ParseFlags(argv))
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  tests = test_runner.LoadTestsFromYaml()
  q7_targets = test_runner.GetQ7Targets(tests)
  tms570_targets = test_runner.GetTms570Targets(tests)  # Returns firmware elfs.

  # Build.
  builder = build.Builder()
  if not builder.Build(workpath=FLAGS.workpath,
                       q7_targets=q7_targets,
                       tms570_targets=tms570_targets):
    sys.exit(1)

  # Test.
  sys.exit(not test_runner.Run(tests))


if __name__ == '__main__':
  main(sys.argv)
