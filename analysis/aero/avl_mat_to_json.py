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

"""Conversion utility for transforming .mat aerodynamics databases to .json."""
import collections
import json
import sys

import gflags
from makani.control import system_types
from makani.lib.python import dict_util
import numpy
from scipy import io

FLAGS = gflags.FLAGS

gflags.DEFINE_string('input_file', None, 'MATLAB .mat file to read in.')
gflags.DEFINE_string('output_file', None, 'JSON file to write to.')


def _ConvertMatlabStructure(data):
  """Convert a database loaded from a .mat file to be written to a JSON file."""
  if not hasattr(data, 'dtype'):
    raise ValueError('Argument must be an numpy array.')

  if hasattr(data.dtype, 'fields') and data.dtype.fields:
    result = {}
    for key in data.dtype.fields.keys():
      result[key] = _ConvertMatlabStructure(data[key])
    return result
  elif data.dtype == numpy.dtype('O'):
    if data.size != 1:
      raise ValueError('Structures must be scalar.')
    return _ConvertMatlabStructure(data[0])
  elif data.shape and data.shape[-1] == 1:
    return _ConvertMatlabStructure(data[..., 0])
  else:
    return data.tolist()


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  data = io.loadmat(FLAGS.input_file)['database']
  # Parameters and grid defintiion.
  keys = [
      'reynolds_number',
      'num_alphas', 'num_betas', 'num_deltas',
      'alphads', 'betads', 'deltads',
      'flap_list', 'omega_hat', 'Sref',
      'Cref', 'Bref', 'mach_number'
  ]

  # Arrays of data that are of shape (num_deltas, num_alphas, num_betas)
  # in shape.
  coefficients = [
      'CLtot', 'CDtot', 'de1',
      'CXtot', 'CYtot', 'CZtot',
      'Cltot', 'Cmtot', 'Cntot',
      'CXp', 'CXq', 'CXr',
      'Clp', 'Clq', 'Clr',
      'CYp', 'CYq', 'CYr',
      'Cmp', 'Cmq', 'Cmr',
      'CZp', 'CZq', 'CZr',
      'Cnp', 'Cnq', 'Cnr',
      'CXd1', 'CYd1', 'CZd1',
      'Cld1', 'Cmd1', 'Cnd1',
      'CXd2', 'CYd2', 'CZd2',
      'Cld2', 'Cmd2', 'Cnd2',
      'CXd3', 'CYd3', 'CZd3',
      'Cld3', 'Cmd3', 'Cnd3',
      'CXd4', 'CYd4', 'CZd4',
      'Cld4', 'Cmd4', 'Cnd4',
      'CXd5', 'CYd5', 'CZd5',
      'Cld5', 'Cmd5', 'Cnd5',
      'CXd6', 'CYd6', 'CZd6',
      'Cld6', 'Cmd6', 'Cnd6',
      'CXd7', 'CYd7', 'CZd7',
      'Cld7', 'Cmd7', 'Cnd7',
      'CXd8', 'CYd8', 'CZd8',
      'Cld8', 'Cmd8', 'Cnd8'
  ]
  output_dict = _ConvertMatlabStructure(data)
  output_dict = collections.OrderedDict(
      [(key, output_dict[key]) for key in keys]
      + [(key, output_dict[key]) for key in coefficients]
      + [('params', dict_util.OrderDict(output_dict['params']))]
  )

  # Force shapes to be correct.
  output_dict['alphads'] = numpy.reshape(output_dict['alphads'],
                                         (output_dict['num_alphas'],)).tolist()
  output_dict['betads'] = numpy.reshape(output_dict['betads'],
                                        (output_dict['num_betas'],)).tolist()
  output_dict['deltads'] = numpy.reshape(output_dict['deltads'],
                                         (output_dict['num_deltas'],)).tolist()
  output_dict['flap_list'] = numpy.reshape(
      output_dict['flap_list'], (system_types.kNumFlaps,)).tolist()
  output_dict['omega_hat'] = numpy.reshape(
      output_dict['omega_hat'], (3,)).tolist()

  for coeff in coefficients:
    output_dict[coeff] = numpy.reshape(
        output_dict[coeff],
        (output_dict['num_deltas'], output_dict['num_alphas'],
         output_dict['num_betas'])).tolist()

  output_string = json.dumps(output_dict, separators=(', ', ':\n  '))
  output_string = (output_string
                   .replace(', \"', ',\n\"')
                   .replace('], [', '],\n   [')
                   .replace(' [[', '[[')
                   .replace('{', '{\n')
                   .replace('}', '\n}')) + '\n'
  with open(FLAGS.output_file, 'w') as f:
    f.write(output_string)


if __name__ == '__main__':
  gflags.MarkFlagAsRequired('input_file')
  gflags.MarkFlagAsRequired('output_file')
  main(sys.argv)
