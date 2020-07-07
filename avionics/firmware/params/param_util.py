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

"""Utility for working with parameter data."""

import importlib
import os
import re
import sys

import gflags
from makani.avionics.firmware.params import client
from makani.avionics.firmware.params import codec
from makani.avionics.network import aio_node
from makani.lib.python import c_helpers
from makani.lib.python import string_util

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)

_FILE_FORMATS = ['bin', 'yaml']
_SECTION_MAP = {
    'config': client.SECTION_CONFIG,
    'calib': client.SECTION_CALIB,
    'serial': client.SECTION_SERIAL,
    'carrier_serial': client.SECTION_CARRIER_SERIAL,
}

gflags.DEFINE_multistring('include', [], 'Python files to include.')
gflags.DEFINE_string('input', None, 'File containing values to convert.'
                     'If the value is "net", data will be read from the network'
                     'using the --node and --section options.')
gflags.DEFINE_enum('input_format', None, _FILE_FORMATS, 'Input file format.')
gflags.DEFINE_string('output', None, 'File containing values to convert.')
gflags.DEFINE_enum('output_format', None, _FILE_FORMATS, 'Output file format.')

gflags.DEFINE_string('node', None,
                     'Aio node from which to read params.')
gflags.DEFINE_enum('section', None,
                   _SECTION_MAP.keys(),
                   'Param section to read from network.')

gflags.DEFINE_string('yaml_key', None,
                     'If specified, yaml file is treated as a dict.  The param '
                     'of this key is read/written')

gflags.DEFINE_multistring('set_value', [],
                          'If specified, sets a field to the specified value '
                          'before writing.  Format: field.sub_field:value')

gflags.DEFINE_boolean('read_source', False,
                      'Read pack2 source from the end of a binary file instead '
                      'of using the paramdb to find the object format.')
gflags.DEFINE_boolean('write_source', True,
                      'Append bzip encoded pack2 source of the object to '
                      'binary files.')

gflags.MarkFlagAsRequired('input')
gflags.MarkFlagAsRequired('output')

FLAGS = gflags.FLAGS


def _LoadParamFromNet(node, section):
  if string_util.IsSnakeCase(node):
    node = string_util.SnakeToCamel(node)
  if node not in aio_node_helper:
    ValueError('%s is not a valid aio node.' % node)
  c = client.Client()
  return c.GetSection(aio_node_helper.Value(node), _SECTION_MAP[section])


def _ReadInputData(filename):
  if filename == '-':
    return sys.stdin.read()
  else:
    with open(filename, 'r') as f:
      return f.read()


def _InferFileFormat(filename):
  if filename == 'net':
    return 'bin'

  (_, ext) = os.path.splitext(filename)
  # Strip '.' from extension.
  ext = ext[1:]
  if ext not in _FILE_FORMATS:
    raise ValueError("Can't infer file format for %s" % filename)
  return ext


def _LoadParam(input_data, file_format):
  if file_format == 'bin':
    return codec.DecodeBin(input_data, FLAGS.read_source)
  elif file_format == 'yaml':
    return codec.DecodeYaml(input_data, FLAGS.yaml_key)


def _StoreBinParam(param, output_file):
  data = codec.EncodeBin(param, FLAGS.write_source)
  output_file.write(data)


def _StoreYamlParam(param, output_file):
  yaml_string = codec.EncodeYaml(param, FLAGS.yaml_key)
  output_file.write(yaml_string)


def _StoreParam(param, output_file, file_format):
  if file_format == 'bin':
    return _StoreBinParam(param, output_file)
  elif file_format == 'yaml':
    return _StoreYamlParam(param, output_file)


def _SetValue(param, field, value):
  """Sets field in param to value."""
  attr = None
  attr_name = ''
  for attr_name in field.split('.'):
    if attr:
      param = attr

    if not hasattr(param, attr_name):
      raise ValueError("Can't find field %s." % field)
    attr = getattr(param, attr_name)
  param.SetField(attr_name, value)


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  include_modules = {}
  for include in FLAGS.include:
    include_modpath = include.rsplit(os.path.extsep, 1)[0]
    include_modpath = 'makani.' + re.sub('[/]', '.', include_modpath)
    include_modules[include_modpath] = importlib.import_module(include_modpath)

  input_format = FLAGS.input_format
  if not input_format:
    input_format = _InferFileFormat(FLAGS.input)

  output_format = FLAGS.output_format
  if not output_format:
    output_format = _InferFileFormat(FLAGS.output)

  if FLAGS.input == 'net':
    param = _LoadParamFromNet(FLAGS.node, FLAGS.section)
  else:
    input_data = _ReadInputData(FLAGS.input)
    param = _LoadParam(input_data, input_format)

  for cmd in FLAGS.set_value:
    (field, value) = cmd.split(':', 1)
    _SetValue(param, field, value)

  if FLAGS.output == '-':
    _StoreParam(param, sys.stdout, output_format)
  else:
    with open(FLAGS.output, 'w') as f:
      _StoreParam(param, f, output_format)


if __name__ == '__main__':
  main(sys.argv)
