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

"""Coordinate embedded software builders and testers."""

import sys
import time

import gflags
from makani.lib.python.embedded import build
from makani.lib.python.embedded import test_fixture


FLAGS = gflags.FLAGS


class TestServer(object):
  """Coordinate embedded software builders and testers."""

  def __init__(self):
    self.fixture = test_fixture.TestFixture(
        database=FLAGS.database)
    self.fixture.LoadHardware(yaml_file=FLAGS.config)
    self.build = build.BuildQueue()

  def Iterate(self):
    self.fixture.Iterate()
    self.build.Iterate()

  def Run(self):
    print 'Test server initialized.'
    while True:
      self.Iterate()
      time.sleep(1.0)


def main(argv):
  try:
    argv = FLAGS(test_fixture.ParseFlags(argv))
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  server = TestServer()
  server.Run()


if __name__ == '__main__':
  main(sys.argv)
