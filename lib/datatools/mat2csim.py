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


import string
import os

import numpy
from scipy import io as spio

# This code is from:
# http://stackoverflow.com/questions/7008608/
# scipy-io-loadmat-nested-structures-i-e-dictionaries
def loadmat(filename):
    '''This function should be called instead of direct spio.loadmat
    as it cures the problem of not properly recovering python
    dictionaries from mat files. It calls the function check keys to
    cure all entries which are still mat-objects.'''
    data = spio.loadmat(filename, struct_as_record=False, squeeze_me=True)
    return _check_keys(data)


def _check_keys(d):
    '''Checks if entries in dictionary are mat-objects. If yes todict
    is called to change them to nested dictionaries.'''
    for key in d:
        if isinstance(d[key], spio.matlab.mio5_params.mat_struct):
            d[key] = _todict(d[key])
    return d


def _todict(matobj):
    '''A recursive function which constructs from matobjects nested
    dictionaries.'''
    dict = {}
    for strg in matobj._fieldnames:
        elem = matobj.__dict__[strg]
        if isinstance(elem, spio.matlab.mio5_params.mat_struct):
            dict[strg] = _todict(elem)
        elif isinstance(elem, numpy.ndarray):
            dict[strg] = elem.tolist()
        else:
            dict[strg] = elem
    return dict


if __name__ == "__main__":
    mat = loadmat(os.environ['MAKANI_HOME']
                  + '/database/m600/M600_20130810.mat')
    mat['mat'] = mat['sampleWing']

    f = open(os.environ['MAKANI_HOME'] + '/lib/datatools/sys_params.txt', 'r')
    sys_params_template = f.read()
    f.close()

    f = open(os.environ['MAKANI_HOME'] + '/base/m600_sys_params.py', 'w')
    f.write(sys_params_template.format(**mat))
    f.close()
