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

"""Canonical parameter ranges for parameter sweep simulations."""

from makani.control import system_types
from makani.lib.python import c_helpers
from makani.lib.python.batch_sim import parameter_tables
from makani.system import labels as system_labels

_AIL_FLAP_BASIS = [
    1.0 if flap_idx in [system_types.kFlapA1, system_types.kFlapA2,
                        system_types.kFlapA4, system_types.kFlapA5,
                        system_types.kFlapA7, system_types.kFlapA8] else 0.0
    for flap_idx in range(system_types.kNumFlaps)
]

_CENTER_FLAP_BASIS = [
    1.0 if flap_idx in [system_types.kFlapA4, system_types.kFlapA5] else 0.0
    for flap_idx in range(system_types.kNumFlaps)
]

_ELE_FLAP_BASIS = [
    1.0 if flap_idx == system_types.kFlapEle else 0.0
    for flap_idx in range(system_types.kNumFlaps)
]

_RUD_FLAP_BASIS = [
    1.0 if flap_idx == system_types.kFlapRud else 0.0
    for flap_idx in range(system_types.kNumFlaps)
]

_FLAP_LABEL_LIST = c_helpers.EnumHelper(
    'FlapLabel', system_labels, prefix='kFlap').ShortNames()
_FLAP_LABELS = {i: _FLAP_LABEL_LIST[i] for i in range(len(_FLAP_LABEL_LIST))}


def GetFlapOffsetParameterRanges():
  return [
      # Assuming maximum 1 deg (0.02 rad) error in control surface angles for
      # Monte Carlo sweeps, ~3 deg (0.05 rad) variation in crosswind sweeps.
      parameter_tables.AeroSimFlapOffsetParameterRange(
          'Center Flap Offset [rad]', _CENTER_FLAP_BASIS, [-0.05, 0.05],
          distribution={'mean': 0.0, 'sigma': 0.01, 'bound': 2.0,
                        'type': 'normal'}),
      parameter_tables.AeroSimFlapOffsetParameterRange(
          'Elevator Offset [rad]', _ELE_FLAP_BASIS, [-0.05, 0.05],
          distribution={'mean': 0.0, 'sigma': 0.01, 'bound': 2.0,
                        'type': 'normal'}),
      parameter_tables.AeroSimFlapOffsetParameterRange(
          'Rudder Offset [rad]', _RUD_FLAP_BASIS, [-0.05, 0.05],
          distribution={'mean': 0.0, 'sigma': 0.01, 'bound': 2.0,
                        'type': 'normal'}),
  ]


def GetAeroDerivativeParameterRanges():
  """Returns a list of parameter ranges."""

  params_list = [
      # Offsets below are set based on variation between ASWING aero database
      # and full kite CFD data.
      parameter_tables.AeroSimOffsetParameterRange(
          'dCldbeta', [-0.2, 0.2], distribution={
              'mean': 0.0, 'sigma': 0.05, 'bound': 2.0, 'type': 'normal'}),
      parameter_tables.AeroSimOffsetParameterRange(
          'dCmdalpha', [-1.0, 1.0], distribution={
              'mean': 0.0, 'sigma': 0.35, 'bound': 2.0, 'type': 'normal'}),
      parameter_tables.AeroSimOffsetParameterRange(
          'dCndbeta', [-0.04, 0.04], distribution={
              'mean': 0.0, 'sigma': 0.01, 'bound': 2.0, 'type': 'normal'}),
  ]

  # Aero force and moment variations with body rates.
  # Use maximum 50% error in Monte Carlo and crosswind sweeps for all aero
  # derivatives.
  for ii in [0, 1, 2]:
    for jj in ['p', 'q', 'r']:
      params_list += [
          parameter_tables.AeroSimForceBRateScalingParameterRange(
              ii, jj, [-0.5, 0.5], distribution={
                  'mean': 0.0, 'sigma': 0.25, 'bound': 2.0, 'type': 'normal'})]
      params_list += [
          parameter_tables.AeroSimMomentBRateScalingParameterRange(
              ii, jj, [-0.5, 0.5], distribution={
                  'mean': 0.0, 'sigma': 0.25, 'bound': 2.0, 'type': 'normal'})]

  # Aero force variations with aileron derivatives.
  force_coeffs = ['CX', 'CY', 'CZ']
  moment_coeffs = ['Cl', 'Cm', 'Cn']
  flap_coeffs = ['da', 'dr', 'de']

  for ii in [0, 1, 2]:
    label = '%s%s Scaling Factor Offset [#]' %(force_coeffs[ii],
                                               flap_coeffs[0])
    params_list += [
        parameter_tables.AeroSimForceBFlapScalingParameterRange(
            label.replace('da', 'd{0}'.format(_FLAP_LABELS[ail_idx])), ii,
            [1. if i == ail_idx else 0. for i in range(system_types.kNumFlaps)],
            [-0.5, 0.5], distribution={'mean': 0.0,
                                       'sigma': 0.25,
                                       'bound': 2.0,
                                       'type': 'normal'})
        for ail_idx in range(system_types.kNumFlaps)
        if _AIL_FLAP_BASIS[ail_idx] > 0.0]

    label = '%s%s Scaling Factor Offset [#]' %(force_coeffs[ii], flap_coeffs[1])
    params_list += [
        parameter_tables.AeroSimForceBFlapScalingParameterRange(
            label, ii, _RUD_FLAP_BASIS, [-0.5, 0.5], distribution=
            {'mean': 0.0, 'sigma': 0.25, 'bound': 2.0, 'type': 'normal'})]

    label = '%s%s Scaling Factor Offset [#]' %(force_coeffs[ii], flap_coeffs[2])
    params_list += [
        parameter_tables.AeroSimForceBFlapScalingParameterRange(
            label, ii, _ELE_FLAP_BASIS, [-0.5, 0.5], distribution=
            {'mean': 0.0, 'sigma': 0.25, 'bound': 2.0, 'type': 'normal'})]

  for ii in [0, 1, 2]:
    label = '%s%s Scaling Factor Offset [#]' %(moment_coeffs[ii],
                                               flap_coeffs[0])
    params_list += [
        parameter_tables.AeroSimMomentBFlapScalingParameterRange(
            label.replace('da', 'd{0}'.format(_FLAP_LABELS[ail_idx])), ii,
            [1. if i == ail_idx else 0. for i in range(system_types.kNumFlaps)],
            [-0.5, 0.5], distribution={'mean': 0.0,
                                       'sigma': 0.25,
                                       'bound': 2.0,
                                       'type': 'normal'})
        for ail_idx in range(system_types.kNumFlaps)
        if _AIL_FLAP_BASIS[ail_idx] > 0.0]

    label = '%s%s Scaling Factor Offset [#]' %(moment_coeffs[ii],
                                               flap_coeffs[1])
    params_list += [
        parameter_tables.AeroSimMomentBFlapScalingParameterRange(
            label, ii, _RUD_FLAP_BASIS, [-0.5, 0.5], distribution=
            {'mean': 0.0, 'sigma': 0.25, 'bound': 2.0, 'type': 'normal'})]

    label = '%s%s Scaling Factor Offset [#]' %(moment_coeffs[ii],
                                               flap_coeffs[2])
    params_list += [
        parameter_tables.AeroSimMomentBFlapScalingParameterRange(
            label, ii, _ELE_FLAP_BASIS, [-0.5, 0.5], distribution=
            {'mean': 0.0, 'sigma': 0.25, 'bound': 2.0, 'type': 'normal'})]

  return params_list
