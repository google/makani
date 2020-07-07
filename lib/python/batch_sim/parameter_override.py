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

"""Generators for simple parameter overrides in batch sims."""

import collections
import copy
import itertools

from makani.config import mconfig
from makani.sim import sim_types as m


class OverrideDef(object):

  def __init__(self, name, path, clean, preprocess_fn=None):
    self.name = name
    self.path = path
    self.clean = clean
    self.preprocess_fn = preprocess_fn

Override = collections.namedtuple('Override', ['name_def', 'values'])


_OVERRIDES = {o.name: o for o in [
    OverrideDef('min_airspeed', [
        'control', 'crosswind', 'power', 'min_airspeed'], True),
    OverrideDef('max_airspeed', [
        'control', 'crosswind', 'power', 'max_airspeed'], True),
    OverrideDef('crosswind_throttle', [
        'sim', 'joystick_sim', 'throttle_settings',
        m.kSimJoystickThrottleCrosswindNormal], True),
    OverrideDef('wind_speed', [
        'sim', 'phys_sim', 'wind_speed'], True),
    OverrideDef('bridle_y_offset', [
        'system', 'wing', 'bridle_y_offset'], False),
    OverrideDef('cd_offset', [
        'sim', 'aero_sim', 'coeff_offsets', 'CD'], False),
    OverrideDef('cl_offset', [
        'sim', 'aero_sim', 'coeff_offsets', 'CL'], False),
    OverrideDef('airspeed_phase', [
        'control', 'crosswind', 'power',
        'airspeed_variation_loop_angle_offset'], True),
    OverrideDef('airspeed_amp_0', [
        'control', 'crosswind', 'power', 'airspeed_variation_0'], True),
    OverrideDef('airspeed_amp_slope', [
        'control', 'crosswind', 'power', 'airspeed_variation_slope'], True),
    OverrideDef('mass', ['system', 'wing', 'm'], False),
    OverrideDef('power_limit', [
        'control', 'crosswind', 'inner', 'max_airspeed_control_power'], True),
    OverrideDef('alpha_min', [
        'control', 'crosswind', 'curvature', 'alpha_min'], True),
    OverrideDef('alpha_min_airspeed', [
        'control', 'crosswind', 'curvature', 'alpha_min_airspeed'], True),
    OverrideDef('dalpha_dairspeed', [
        'control', 'crosswind', 'curvature', 'dalpha_dairspeed'], True),
    OverrideDef('thrust_weight', [
        'control', 'crosswind', 'output', 'thrust_moment_weights', 'thrust'],
                True),
    OverrideDef('moment_x_weight', [
        'control', 'crosswind', 'output', 'thrust_moment_weights', 'moment',
        0], True),
    OverrideDef('moment_y_weight', [
        'control', 'crosswind', 'output', 'thrust_moment_weights', 'moment',
        1], True),
    OverrideDef('moment_z_weight', [
        'control', 'crosswind', 'output', 'thrust_moment_weights', 'moment',
        2], True),
    # Crosswind inner terms.
    OverrideDef('elevator_flap_ratio', [
        'control', 'crosswind', 'inner', 'elevator_flap_ratio'],
                True),
    OverrideDef('delevator_dalpha', [
        'control', 'crosswind', 'inner', 'delevator_dalpha'],
                True),
    OverrideDef('kp_flap', [
        'control', 'crosswind', 'inner', 'kp_flap'],
                True),
    OverrideDef('kp_airspeed', [
        'control', 'crosswind', 'inner', 'airspeed_pid', 'kp'],
                True),
    OverrideDef('ki_airspeed', [
        'control', 'crosswind', 'inner', 'airspeed_pid', 'ki'],
                True),
    OverrideDef('kd_airspeed', [
        'control', 'crosswind', 'inner', 'airspeed_pid', 'kd'],
                True),
    OverrideDef('max_airspeed_control_power', [
        'control', 'crosswind', 'inner', 'max_airspeed_control_power_gen'],
                True),
    # Crosswind path terms.
    OverrideDef('ki_crosstrack', [
        'control', 'crosswind', 'path', 'crosstrack_pid', 'ki'],
                True),
    OverrideDef('ki_crosstrack_max', [
        'control', 'crosswind', 'path', 'crosstrack_pid', 'int_output_max'],
                True),
    OverrideDef('ki_crosstrack_min', [
        'control', 'crosswind', 'path', 'crosstrack_pid', 'int_output_min'],
                True),
]}


def GetOverrideDef(path, name=None, clean=False):
  path = path.split('.')
  for i in range(len(path)):
    try:
      path[i] = int(path[i])
    except ValueError:
      pass   # Not an integer, continue.
  return OverrideDef(name if name else path[-1], path, clean)


def ApplyOverride(base, override, value, allow_extend=False):
  """Generate a single config with a set of overrides."""
  elem = base
  for i, key in enumerate(override.path):
    if i < len(override.path) - 1:
      if isinstance(elem, list):
        if len(elem) <= key:
          raise ValueError(
              'Unable to override path with missing array index: %s'
              % str(override.path))
      elif key not in elem:
        if isinstance(override.path[i + 1], str):
          if allow_extend:
            elem[key] = dict()
          else:
            raise ValueError(
                'Unable to override path with missing dict: %s'
                % str(override.path))
        else:
          raise ValueError(
              'Unable to override path with missing array: %s'
              % str(override.path))
      elem = elem[key]
    else:
      if isinstance(elem, list):
        if len(elem) <= key:
          raise ValueError(
              'Unable to override path with missing array index: %s'
              % str(override.path))
      elif key not in elem and not allow_extend:
        raise ValueError(
            'Unable to override path with missing value: %s'
            % str(override.path))
      if override.preprocess_fn:
        elem[key] = override.preprocess_fn(value)
      else:
        elem[key] = value


def _PreprocessOverrideList(override_list):
  """Clean up override list, convert dict args, and create Overrides."""
  if isinstance(override_list, dict):
    override_list = override_list.items()
  overrides = []
  for item in override_list:
    if not isinstance(item, Override):
      item = Override(*item)
    if isinstance(item.name_def, OverrideDef):
      overrides.append(item)
    elif item.name_def in _OVERRIDES:
      overrides.append(Override(_OVERRIDES[item.name_def], item.values))
    else:
      overrides.append(Override(GetOverrideDef(item.name_def), item.values))
  return overrides


def GenerateConfig(base, override_list):
  override_list = [(name, [value]) for name, value in override_list]
  return GenerateConfigList(base, override_list).next()


def GenerateConfigList(base, override_list):
  """Generate a series of configs based on lists of overrides."""
  override_list = _PreprocessOverrideList(override_list)
  clean_overrides = [o for o in override_list if o.name_def.clean]
  unclean_overrides = [o for o in override_list if not o.name_def.clean]
  if not unclean_overrides:
    base_config = mconfig.MakeParams('common.all_params', copy.deepcopy(base),
                                     override_method='derived')
  for i in range(len(override_list[0].values)):
    if unclean_overrides:
      base_override = copy.deepcopy(base)
      for override in unclean_overrides:
        ApplyOverride(base_override, override.name_def, override.values[i],
                      True)
      config = mconfig.MakeParams('common.all_params', base_override,
                                  override_method='derived')
    else:
      config = copy.deepcopy(base_config)
    for override in clean_overrides:
      ApplyOverride(config, override.name_def, override.values[i])
    config['adjusted_parameters'] = {
        o.name_def.name: o.values[i] for o in override_list}
    yield config


def GenerateConfigPermutations(base, override_params):
  """Generate a series of configs based on permutations of overrides."""
  override_params = _PreprocessOverrideList(override_params)
  clean_overrides = [o for o in override_params if o.name_def.clean]
  unclean_overrides = [o for o in override_params if not o.name_def.clean]
  for unclean_values in itertools.product(
      *[o.values for o in unclean_overrides]):
    base_override = copy.deepcopy(base)
    for override, value in zip(unclean_overrides, unclean_values):
      ApplyOverride(base_override, override.name_def, value, True)
    base_config = mconfig.MakeParams('common.all_params', base_override,
                                     override_method='derived')
    for clean_values in itertools.product(*[o.values for o in clean_overrides]):
      config = copy.deepcopy(base_config)
      for override, value in zip(clean_overrides, clean_values):
        ApplyOverride(config, override.name_def, value)
      adjusted_params = (zip(unclean_overrides, unclean_values)
                         + zip(clean_overrides, clean_values))
      config['adjusted_parameters'] = {
          o.name_def.name: v for o, v in adjusted_params}
      yield config
