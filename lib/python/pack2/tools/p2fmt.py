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

"""Pack2 formatting utility."""

import sys

import gflags
from makani.lib.python.pack2 import parser

FLAGS = gflags.FLAGS


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  source_string = sys.stdin.read()

  p = parser.Parser()
  p.Parse(source_string)
  print p.GetFormattedSource(),


if __name__ == '__main__':
  main(sys.argv)
