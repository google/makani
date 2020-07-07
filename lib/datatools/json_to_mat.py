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

"""Converts a JSON file to a MATLAB .mat file.

Usage: json_to_mat.py foo.json
"""

import collections
import json
import os
import sys
import scipy.io


def _Sanitize(data):
  """Converts data to a format acceptable by scipy.io.savemat.

  The scipy.io.savemat function cannot handle Booleans, NoneTypes, or
  unicode strings.

  Args:
    data: Dictionary returned by json.load.

  Returns:
    Sanitized dictionary that is compatible with scipy.io.savemat.
  """
  if isinstance(data, collections.OrderedDict):
    return collections.OrderedDict([(str(k), _Sanitize(v))
                                    for k, v in data.items()])
  if isinstance(data, dict):
    return {str(k): _Sanitize(v) for k, v in data.items()}
  elif isinstance(data, list):
    return [_Sanitize(x) for x in data]
  elif data is None:
    return []
  elif isinstance(data, bool):
    return 1 if data else 0
  else:
    return data


def _PrintUsage():
  print
  print 'Usage: json_to_mat.py foo.json'
  print


def main(argv):
  if len(argv) != 2:
    print 'Error: Wrong number of arguments.'
    _PrintUsage()
    sys.exit(1)

  if not os.path.isfile(argv[1]):
    print 'Error: File does not exist.'
    _PrintUsage()
    sys.exit(1)

  with open(argv[1], 'r') as f:
    data = _Sanitize(json.load(f, object_pairs_hook=collections.OrderedDict))

  filename, _ = os.path.splitext(argv[1])
  scipy.io.savemat(filename + '.mat', data, long_field_names=True)


if __name__ == '__main__':
  main(sys.argv)
