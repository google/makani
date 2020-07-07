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

"""Contains the wing model flag."""

import os

import gflags

from makani.control import system_types
from makani.lib.python import c_helpers

_WING_MODEL_HELPER = c_helpers.EnumHelper('WingModel', system_types,
                                          prefix='kWingModel')

# TODO(b/147502899): The empty string option allows us
# to call run_sim with the all_params flag, by also
# passing a wing_model flag with the empty string to explicitly
# force that the wing_model flag will be ignored. We should
# check that our code is robust to a wing_model flag with an empty
# sting in other contexts, or find a better solution.
WING_MODELS = ['m600', 'oktoberkite', '']

gflags.DEFINE_enum('wing_model',
                   os.environ.get('PYTHON_DEFAULT_WING_MODEL', 'm600'),
                   WING_MODELS,
                   'Wing model.',
                   short_name='w')

FLAGS = gflags.FLAGS


def FlagToWingModelName(flag):
  if flag == 'm600':
    return 'kWingModelYm600'
  elif flag == 'oktoberkite':
    return 'kWingModelOktoberKite'
  else:
    raise ValueError('Wing model "%s" is not recognized.' % flag)


def WingModelToConfigName(enum_value):
  if not isinstance(enum_value, int):
    enum_value = _WING_MODEL_HELPER.Value(enum_value)

  if enum_value in [
      system_types.kWingModelYm600,
      system_types.kWingModelM600a]:
    return 'm600'
  else:
    return 'oktoberkite'


def AppeaseLintWhenImportingFlagOnly():
  """Because you can't "pylint: disable" and explain why on the same line."""
  pass
