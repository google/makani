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

"""Pack2 code generation utility."""

import os
import sys

import gflags
from makani.lib.python.pack2 import backend_c
from makani.lib.python.pack2 import backend_py
from makani.lib.python.pack2 import generator
from makani.lib.python.pack2 import parser

gflags.DEFINE_string('input', None, 'Pack2 input file.')
gflags.DEFINE_string('output_py', None, 'Python output file name.')
gflags.DEFINE_string('base_dir', None, '')
gflags.DEFINE_string('output_c_header', None, 'C header output file name.')
gflags.DEFINE_string('output_c_source', None, 'C source output file name.')
FLAGS = gflags.FLAGS
gflags.MarkFlagAsRequired('input')

FLAGS = gflags.FLAGS


def _GetCHeaderPath():
  c_header_path = FLAGS.output_c_header
  if c_header_path.startswith(FLAGS.base_dir):
    c_header_path = os.path.relpath(c_header_path, FLAGS.base_dir)
  return c_header_path


def _GeneratePy(metadata):
  backend = backend_py.BackendPy(c_header_path=_GetCHeaderPath())
  gen = generator.Generator(backend)
  gen.Generate(metadata)

  gen.WriteSourceFile('source', FLAGS.output_py)


def _GenerateC(metadata):
  backend = backend_c.BackendC(header_path=_GetCHeaderPath())
  gen = generator.Generator(backend)
  gen.Generate(metadata)

  gen.WriteSourceFile('source', FLAGS.output_c_source)
  gen.WriteSourceFile('header', FLAGS.output_c_header)


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  with open(FLAGS.input, 'r') as source_file:
    source_string = source_file.read()

  p = parser.Parser()
  metadata = p.Parse(source_string)

  _GeneratePy(metadata)
  _GenerateC(metadata)

if __name__ == '__main__':
  main(sys.argv)
