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

# Python imports
from __future__ import absolute_import
from __future__ import print_function
import copy
import math
from matplotlib import pyplot as plt
import numpy as np
import os

# mx_modeling imports
from lib import fun
from lib import utils
from power_calcs import kite_pose

from . import rotor_model_util
from .fbl_load_csim_database import FittedDatabase


# cD offsets for various components
# Effective tether drag referenced to kite wing area.
_cD_eff_tether_stranded_base_m600 = 0.11306  # only valid for S=32.9
_cD_eff_tether_fluted_m600 = 0.06756  # only valid for S=32.9
_cD_eff_tether_half_fair_m600 = 0.0131
_cD_eff_tether_faired_m600 = 0.0064

# Drag estimates for other components.
# Baseline from bottoms up drag buildup minus gear
_cD_parasitic_no_fair = 0.0653 - 0.0093

_delta_cD_fair_bridle = -0.0151  # only valid for S=32.9
_delta_cD_fair_landing_gear = -0.0061  # only valid for S=32.9

_cD_gear_m600 = 0.0093  # revised 2017-11 from Ben's estimate
_cD_gear_faired_m600 = 0.0047

# Glide estimates from RPX07 indicated that config had a total drag of ~0.075
# This means that:
# _cD_parasitic_no_fair + _delta_cD_fair_bridle
# + _cD_gear_faired_m600 + _cD_addnl_offset = 0.075
# This fudge factor fills in the difference between these estimate.
_cD_addnl_offset = 0.0294  # fit to make match overall offset from CSim
_cL_offset = -0.125

# Approximate mass deltas for various components
_m_gear_fairing_delta = 8.
_m_gear_removed_delta = -40.
# Based on mass difference between SN04 and SN05 in crosswind.
# Note that this mass difference only includes mass increase from installing
# the slats. The mass increase from reinforced structure and slat bracket
# mounts is already included in the base mass.
# https://codesearch.corp.google.com/makani/config/m600/wing.py?l=62
_m_slats_installed = 40.

# tether and ground station args
_l_tether = 432. + 7.2  # tether + bridle
# Parker Ranch tether attachment point.
# https://codesearch.corp.google.com/makani/config/m600/ground_frame.py?l=29
_gs_position = [0., 0., 6.122]

# Effective center of rotor thrust (assuming all rotors working equally) is the
# average of all rotor locations.
# Source: https://codesearch.corp.google.com/makani/config/m600/rotors.py?l=130
_rotor_locations = {}
_rotor_locations['Sbo'] = [1.613, 3.639, 1.597]
_rotor_locations['Sbi'] = [1.613, 1.213, 1.597]
_rotor_locations['Pbi'] = [1.613, -1.213, 1.597]
_rotor_locations['Pbo'] = [1.613, -3.639, 1.597]
_rotor_locations['Pto'] = [1.960, -3.639, -1.216]
_rotor_locations['Pti'] = [1.960, -1.213, -1.216]
_rotor_locations['Sti'] = [1.960, 1.213, -1.216]
_rotor_locations['Sto'] = [1.960, 3.639, -1.216]
_rotor_thrust_center = np.mean(list(_rotor_locations.values()), axis=0)
# Unit vector of rotor thrust in body frame.
# Direction indicates positive thrust.
# TODO: Update with actual direction. Actual is very nearly along X.
_rotor_thrust_axis = np.array([1., 0., 0.])

# Center of gravity (CG) and rotational inertia of the kite.
# Source: https://codesearch.corp.google.com/makani/config/m600/wing.py?l=54
_CG_sn4 = np.array([-0.085, 0.037, 0.108])
# Source: SN05 crosswind mass:
# https://codesearch.corp.google.com/makani/config/m600/wing.py?l=94
_m_kite = 1629.6 + 51.5 + 10.6 + 0.6
# Inertia is scaled according to actual mass over reference mass.
_inertia_sn4 = np.array(
    [[31401.2,  47.3,    22.3],
     [47.3,   8228.9,    22.3],
     [22.3,     22.3, 36864.5]]) * _m_kite / 1606.98

# Wing reference areas and lengths.
# Source: https://codesearch.corp.google.com/makani/config/m600/wing.py?l=129
_S = 32.9 # Refers to wing area, called A in source.
_b = 25.66 # Wing span
_c = 1.28 # Wing chord

# Bridle parameters.
# Source: https://codesearch.corp.google.com/makani/config/m600/wing.py?l=175
# Bridle hardpoint locations (center of spherical bearing).
_bridle_pos = np.array(
    [[-0.1494, -5.8843, 0.13035],
     [-0.1494,  5.8661, 0.13035]])
# Bridle radius (distance from axis between bridle anchors to bridle knot).
_bridle_rad= 4.7860
# Bridle offset. Y offset of bridle knot along bridle axis from center.
_bridle_y_offset= -0.5

# Make the bridle model.
_m600_bridle = utils.BridleMoments(_bridle_pos, _bridle_rad, _bridle_y_offset)

# Roll limits.
# Source: https://codesearch.corp.google.com/makani/config/m600/control/crosswind.py?l=421
_max_roll_excursion = 0.48
_nom_tether_roll = np.arctan2(-_bridle_y_offset, _bridle_rad + _bridle_pos[1][2])
_tether_roll_min = _nom_tether_roll - _max_roll_excursion
_tether_roll_max = _nom_tether_roll + _max_roll_excursion

# Aero thrust limits.
# The M600 has aero thrust power limits, max_airspeed_control_power_gen/motor,
# to ensure the controller is not expecting unreasonable thrusts.
# Source: https://codesearch.corp.google.com/makani/config/m600/control/crosswind.py?type=cs&q=max_airspeed_control_power_gen&g=0&l=467
# Notation is - for gen and + for thrust.
# This limit is not inherent to the physics of the FBL or physical kite
# limits, but may be useful when trying to compare results to csim.
_aero_thrust_p_max = 650e3
_aero_thrust_p_min = -1000e3

# Model aero device.
# For the M600, the aero device is the inner ailerons, deflected to spoil the
# wing.
# Source: https://docs.google.com/spreadsheets/d/1lQTqghsm7R2yY-OLu4VsKKtOrJo5OHYYcWZgqPmQS98/edit
# CFD of d4 deflections is used to find average relative change in cL and cD
# with full flap deployment. Largest deflection (100deg) is used,
# so small drag flap deflections are poorly modeled.
# A linear fit of d_cm = f(alpha) is used for cm.
# All values are multiplied by 2 as d4 flap will be used in pair with d3.
# D3 effectiveness assumed equal.
# Changes to cl, cn, and cy are ignored, as these moments should be largely
# balanced by similar balancing moments from d3.
def _drag_flaps(flap_norm, state):
  cD_offset = (1.3 * state['cD'] - state['cD']) * 2. * flap_norm
  cL_offset = (0.853 * state['cL'] - state['cL']) * 2. * flap_norm
  cm_offset = (-0.00769 * state['alpha'] - 0.10391) * 2. * flap_norm

  state['aero_device_cD_offset'] = cD_offset
  state['aero_device_cL_offset'] = cL_offset
  state['aero_device_cm_offset'] = cm_offset

  return {
      'cD': cD_offset,
      'cL': cL_offset,
      'cm': cm_offset}

# Modify the usual 'eta_shaft_to_padmount' so the model
# reports at the desired power point.
_power_points = {
    'shaft': lambda p: 1.0,
    'motor_elec': lambda p: 0.94,  # Assumes motor is 94% efficient
    'padmount': (
        lambda p: 0.94 * 0.96 * (1. - (abs(p) * 1.0)/(3400.**2)) * 0.975)}


def _csim_db_fitter(csimdb, alphad, betad, omega_hat=None):
  body_coeff = {}
  if omega_hat is None:
    omega_hat = csimdb._bomega_hat_0
  body_coeff.update(csimdb.CalcForceCoeffs(alphad, betad, omega_hat))
  body_coeff.update(csimdb.CalcMomentCoeffs(alphad, betad, omega_hat))

  return body_coeff

# Create aero databases from files.
# Includes rudder extension.
_aswing_baseline_csim_db = (
    FittedDatabase(os.path.dirname(__file__)
                   + '/aero_databases/m600_aswing_baseline.json'))
_aswing_baseline_zero_angular_csim_db = (
    FittedDatabase(
        os.path.dirname(__file__)
        + '/aero_databases/m600_aswing_baseline_zero_angular_rate.json'))
# Does NOT include rudder extension.
_aswing_stage3slats_csim_db = (
    FittedDatabase(os.path.dirname(__file__)
                   + '/aero_databases/m600_aero_database_stage_3_slats.json'))

def _aswing_body_coeff_baseline(alpha, beta, omega_hat=None):
  # If omega_hat is not provided, use the nominal omega_hat from the database.
  if omega_hat is None:
    omega_hat = _aswing_baseline_csim_db._bomega_hat_0
  return _csim_db_fitter(_aswing_baseline_csim_db, alpha, beta, omega_hat)


def _aswing_body_coeff_stage_3_slat(alpha, beta, omega_hat=None):
  # If omega_hat is not provided, use the nominal omega_hat from the database.
  if omega_hat is None:
    omega_hat = _aswing_stage3slats_csim_db._bomega_hat_0
  return _csim_db_fitter(_aswing_stage3slats_csim_db, alpha, beta, omega_hat)


m600_base = {
    'c': _c,
    'b': _b,
    's': _S,
    'CG': _CG_sn4,
    'inertia': _inertia_sn4,
    'rotor_thrust_center': _rotor_thrust_center,
    'rotor_thrust_axis': _rotor_thrust_axis,
    'aero_device': _drag_flaps,
    'v_a_max': 70.0,
    'aero_thrust_p_max': _aero_thrust_p_max,
    'aero_thrust_p_min': _aero_thrust_p_min,
    'tether_roll_min': _tether_roll_min,
    'tether_roll_max': _tether_roll_max,
    'bridle_moment_from_tether_pitch_roll': (
      _m600_bridle.CalculateMomentFromPitchRoll),
    # TODO: Add source for control moment residual limits.
    'cl_residual_max': 0.11,
    'cl_residual_min': -0.1,
    'cm_residual_max': 0.4,
    'cm_residual_min': -0.6,
    'cn_residual_max': 0.02,
    'cn_residual_min': -0.02,
    'gs_position': _gs_position,
    'h_min': 90.,
    'incl_max': 1.0,
    'l_tether': _l_tether,
    'm_kite': _m_kite,
    'm_tether': 390.45,
    'tension_max': 260000.,
    'v_a_min': 35.,
    'shaft_power_from_drag_power': (
        rotor_model_util.Gen4RotorConfigBySizeFixedPitch(
            0.0, n_rotors=8, r_rotor=1.15)),
    'power_shaft_max': 900000.,
    'torque_shaft_max': 1000., # Nm, from 08/12 pencil spec, https://docs.google.com/spreadsheets/d/1R3nDoGct9COB6donjDL9S-K9XdtTe3shFhP_GG3ElpI/edit#gid=0
    'rotor_mach_limit': 0.8,
    'eta_shaft_to_pad': _power_points['padmount']}

m600_configs = {
    'stage3_slats_faired_fluted': {
        'body_coeff_from_alpha_beta': _aswing_body_coeff_stage_3_slat,
        'cD_offset': (_cD_parasitic_no_fair
                       + _delta_cD_fair_bridle
                       + _cD_gear_faired_m600
                       + _cD_addnl_offset),
        'alpha_min': 0.,
        'alpha_max': 8.,
        'beta_min': -5.,
        'beta_max': 5.,
        'm_kite': m600_base['m_kite'] + _m_slats_installed,
        'cD_eff_tether': _cD_eff_tether_fluted_m600,
        'description': ('SN03, stage3 aswing aero, faired gear, ' +
                        'partial fair bridle, ' +
                        'fluted stranded tether, ' +
                        'alpha 8deg')},

    'stage3_slats_fluted_conservative': {
        'body_coeff_from_alpha_beta': _aswing_body_coeff_stage_3_slat,
        'cD_offset': (_cD_parasitic_no_fair
                       + _delta_cD_fair_bridle
                       + _cD_gear_faired_m600
                       + _cD_addnl_offset),
        'tether_roll_max': m600_base['tether_roll_max'] - math.radians(10.),
        'tether_roll_min': m600_base['tether_roll_min'] + math.radians(10.),
        'alpha_min': 0.,
        'alpha_max': 5.,
        'beta_min': -4.,
        'beta_max': 4.,
        'm_kite': m600_base['m_kite'] + _m_slats_installed,
        'cD_eff_tether': _cD_eff_tether_fluted_m600,
        'description': ('SN03, stage3 aswing aero, faired gear, ' +
                        'partial fair bridle, ' +
                        'fluted stranded tether, ' +
                        'alpha 5deg')},

    'baseline_faired_fluted': {  # Note: Unfaired bridles
        'body_coeff_from_alpha_beta': _aswing_body_coeff_baseline,
        'cD_offset': (_cD_parasitic_no_fair
                       + _cD_gear_faired_m600
                       + _cD_addnl_offset),
        'cL_offset': _cL_offset,
        'alpha_min': -8.,
        'alpha_max': 5.,
        'beta_min': -5.,
        'beta_max': 5.,
        'cD_eff_tether': _cD_eff_tether_fluted_m600,
        'description': ('SN05, baseline aswing aero, faired gear, ' +
                        'unfaired bridle, ' +
                        'fluted stranded tether, ' +
                        'alpha max 5deg')},
    }

for key, config in m600_configs.items():
  base = copy.deepcopy(m600_base)
  base.update(config)
  m600_configs[key] = base

# Test configs are approximate.
test_to_config = {
    'RPX07': 'stage3_slats_faired_fluted', # GS Position not updated for CL
    'RPX08': 'stage3_slats_faired_fluted',
    'RPX09': 'stage3_slats_faired_fluted',
    'CW01': 'baseline_faired_fluted',
    'CW02': 'baseline_faired_fluted'}

_base_config = m600_configs['baseline_faired_fluted']
_base_slats_config = m600_configs['stage3_slats_faired_fluted']


def UpdateKitesAndModelsFromOverride(
    override, kites, aero_models, bridle_models):
  """Updates kites, aero_models, and bridle_models with new kite.

  Args:
    override: Dict of overrides to define a kite config.
    kites: Dict of kites that is updated with new kites from override. Any kites
      with the same name as override will NOT be overwritten.
    aero_models: Dict of aero database model that is updated with aero model
      that is created from aero database specified in override. Key is 'name'
      specified in override.
    bridle_models: Dict of bridle models that is updated with model that is
      created from override. Key is 'name' specified in override.
  """
  # Setup kites
  assert override['name'] not in kites, (
      'Name already exists in kites and was not updated. Check that new '
      + 'name is correct. \n'
      + 'If name is correct, delete existing entry and rerun to revise.')
  kite = override['name']
  kites[kite] = MakeKiteFromOverride(override)

  # Setup Bridles
  assert override['name'] not in bridle_models, (
      'Name already exists in bridle_models and was not updated. Check that new'
      + ' name is correct. \n'
      + 'If name is correct, delete existing entry and rerun to revise.')

  # Make the bridle model.
  bridle_left = np.array(override['tether_hardpoint'])
  bridle_right = np.array(override['tether_hardpoint'])
  # Define axis along y
  bridle_left[1] -= 0.5
  bridle_right[1] += 0.5
  bridle_pos = np.array([bridle_left, bridle_right])
  # Bridle radius (distance from axis between bridle anchors to bridle knot).
  bridle_rad = override['bridle_radial_length']
  # Bridle offset. Y offset of bridle knot along bridle axis from center.
  bridle_y_offset = override['bridle_y_offset']
  bridle_models[kite] = utils.BridleMoments(
      bridle_pos, bridle_rad, bridle_y_offset)

  # Setup aero model
  assert override['name'] not in aero_models, (
      'Name already exists in aero_models and was not updated. Check that new '
      + 'name is correct. \n'
      + 'If name is correct, delete existing entry and rerun to revise.')
  aero_models[kite] = (
      FittedDatabase(
          fun.GetFullPath('tools/aero_databases/')
          + override['aero_db_file']))

def MakeKiteFromOverride(override, aero_devices=None):
  """Makes a kite config from override dict.

  Args:
    override: Dict of overrides to define a kite config.

  Returns:
    Dict of kite config.
  """

  kite = {}

  # Setup aero device
  # Pick from the dict of provided aero devices - if not provided, make a dict
  # of ones in the manager.
  if aero_devices is None:
    aero_devices = {'m600_drag_flaps': _drag_flaps}

  if 'aero_device_name' in override:
    kite['aero_device'] = aero_devices[override['aero_device_name']]

  # Setup Bridles
  # Get the tether attach location. Must be defined as an axis, which we assume
  # is along kite y. This allows us to only specify a single point, and stretch
  # it along y to define the axis.
  bridle_left = np.array(override['tether_hardpoint'])
  bridle_right = np.array(override['tether_hardpoint'])
  # Define axis along y
  bridle_left[1] -= 0.5
  bridle_right[1] += 0.5
  bridle_pos = np.array([bridle_left, bridle_right])
  # Bridle radius (distance from axis between bridle anchors to bridle knot).
  bridle_rad = override['bridle_radial_length']
  # Bridle offset. Y offset of bridle knot along bridle axis from center.
  bridle_y_offset = override['bridle_y_offset']
  bridle_model = utils.BridleMoments(
      bridle_pos, bridle_rad, bridle_y_offset)
  kite['bridle_moment_from_tether_pitch_roll'] = (
      bridle_model.CalculateMomentFromPitchRoll)

  # Make aero model
  aero_model = (
      FittedDatabase(
          fun.GetFullPath('tools/aero_databases/')
          + override['aero_db_file']))

  kite['body_coeff_from_alpha_beta'] = (
      lambda a, b, o=None: _csim_db_fitter(aero_model, a, b, o))

  # Setup shaft power function for rotors
  kite['shaft_power_from_drag_power'] = (
      rotor_model_util.Gen4RotorConfigBySizeFixedPitch(
          override['rotor_pitch'],
          a_rotors=override['a_rotors'],
          n_rotors=override['n_rotors'],
          rotor_mach_limit=override['rotor_mach_limit']))

  # Setup eta_shaft_to_padmount function
  kite['eta_shaft_to_pad'] = (
      lambda p: (
          override['eta_motors'] * override['eta_motor_ctrls']
          * (1. - (abs(p) * override['ohms_per_m_tether'] * override['l_tether']
                   /(override['v_tether']**2)))
          * override['eta_pad_trans']))

  # Setup tether effective drag
  # Assumes tether drag and thickness is constant over length of tether
  kite['cD_eff_tether'] = (
      override['cD_tether'] * override['l_tether'] * override['t_tether']
      / (4. * override['s']))

  # Note: Some values (name, bridle settings, rotor settings, etc.)
  # from the override are only used here, and not by the config dict needed for
  # FBL. We put all of them in one place so the full definition is saved.
  kite.update(override)

  return kite


def ChangeKiteMass(config, mass_delta):
  config['m_kite'] += mass_delta


def ChangeTetherMass(config, mass_delta):
  config['m_tether'] += mass_delta


def ChangePowerPoint(config, power_point):
  if power_point not in _power_points:
    print(('Power point must be: \''
          + '\', or \''.join(list(_power_points.keys()))
          + '\''))
  else:
    config['eta_shaft_to_pad'] = _power_points[power_point]


def ModifyC_Doffset(config, cD_delta):
  config['cD_offset'] += cD_delta


def GetConfigByName(name='baseline_faired_fluted', power_point='padmount'):
  if name not in m600_configs:
    print('Name must be: \'' + '\', or \''.join(list(m600_configs.keys())) + '\'')
    m600_configs[name]
  else:
    config = copy.deepcopy(m600_configs[name])
    ChangePowerPoint(config, power_point)
    return config


def GetConfigByTest(test='CW02', power_point='motor_elec'):
  if test not in test_to_config:
    print('Test must be: \'' + '\', or \''.join(list(test_to_config.keys())) + '\'')
    test_to_config[test]
  else:
    name = test_to_config[test]
    config = copy.deepcopy(m600_configs[name])
    ChangePowerPoint(config, power_point)
    return config


def GetPathArgsByR_LoopAndMinHeight(config, h_min, r_loop, azim=0.):
  """
  Returns a dictionary of arguments that can be passed to initialize a
  KitePath object. Calcs inclination required to meet minimum height h_min.
  """

  if 'gs_position' in config:
    gs_position = config['gs_position']
  else:
    gs_position = [0., 0., 0.]

  angle_half_cone = math.asin(r_loop/config['l_tether'])
  angle_h_min = math.asin((h_min - gs_position[2])/config['l_tether'])
  incl = angle_half_cone + angle_h_min

  args = {'shape_params': {'r_loop': r_loop,
                           'type': 'circle'},
          'location_params': {'incl': incl,
                              'azim': azim}}
  return args

m600_configs['stage3_slats_no_gear'] = copy.deepcopy(_base_slats_config)
m600_configs['stage3_slats_no_gear']['m_kite'] += _m_gear_removed_delta
ModifyC_Doffset(m600_configs['stage3_slats_no_gear'], -_cD_gear_faired_m600)


m600_configs['stage3_slats_no_gear_faired_tether'] = copy.deepcopy(
    m600_configs['stage3_slats_no_gear'])
ModifyC_Doffset(m600_configs['stage3_slats_no_gear_faired_tether'],
                -_cD_eff_tether_fluted_m600 + _cD_eff_tether_faired_m600)

m600_configs['stage3_slats_faired_tether'] = copy.deepcopy(_base_slats_config)
ModifyC_Doffset(m600_configs['stage3_slats_faired_tether'],
                -_cD_eff_tether_fluted_m600 + _cD_eff_tether_faired_m600)


def PlotKiteAero(config, **kwargs):
  """Plots the aero database for the given config. Returns a dict of plot
  objects.

  Kwargs:
    alpha_linspace: Tuple of linspace values for alphas.
    beta_linspace: Tuple of linspace values for betas.
    omega_hat: Reduced angular rates for aero lookup.
    plots: A dict where the keys are the variable that's plotted and values are
      plot objects. Enables user to append to plots.
    levels: For contour plots, sets number of levels.
    colormap: For contour plots, sets colormap.
    color: For line plots, sets line color.
    label: For line plots, sets line label for legend.
    figsize: Matplotlib figsize kwarg.
    keys: Variables to plot.

  If one of alphas or betas linspace is a single value, plot changes to a line
  plot instead of the default contour plot."""

  alphas = np.linspace(*kwargs.get('alpha_linspace', (-5., 15., 40)))
  betas = np.linspace(*kwargs.get('beta_linspace', (-10., 10., 40)))
  omega_hat = kwargs.get('omega_hat', None)
  plots = kwargs.get('plots', {})
  levels = kwargs.get('levels', 30)
  colormap = kwargs.get('colormap', 'viridis')
  color = kwargs.get('color', 'C0')
  label = kwargs.get('label', None)
  figsize = kwargs.get('figsize', (9,7))
  linestyle = kwargs.get('linestyle', '-')

  keys = kwargs.get('keys', ['cL', 'cD', 'cY', 'zeta', 'L1.5/D'])

  o = {}
  for k in keys:
    o[k] = []
    for beta in betas:
      row = []
      for alpha in alphas:
        state = {}
        state['alpha'] = alpha
        state['beta'] = beta
        state.update(
            config['body_coeff_from_alpha_beta'](alpha, beta, omega_hat))
        kite_pose.KitePose._aero_coeff_from_body_coeff(state)
        kite_pose.KitePose._apply_aero_offsets(state, config)
        if k == 'zeta':
          row.append(
              (4./27.) * state['cL']**3
               / (state['cD'] + config['cD_eff_tether'])**2)
        elif k == 'L1.5/D':
          row.append(
              state['cL']**1.5
               / (state['cD'] + config['cD_eff_tether']))
        elif k == 'L/D':
          row.append(
              state['cL']
               / (state['cD'] + config['cD_eff_tether']))
        else:
          row.append(state[k])
      o[k].append(row)

  for k in keys:
    if k in plots:
      fig = plots[k]
    else:
      plots[k] = plt.figure(figsize=figsize)
    ax = plots[k].gca()
    if len(alphas) == 1:
      ax.plot(betas, [v[0] for v in o[k]], color=color, label=label,
              linestyle=linestyle)
      ax.set_title(k + ' as function of beta @ alpha %0.1f deg' % alphas[0])
      ax.set_xlabel('beta [deg]')
      ax.set_ylabel(k)
    elif len(betas) == 1:
      ax.plot(alphas, o[k][0], color=color, label=label, linestyle=linestyle)
      ax.set_title(k + ' as function of alpha @ beta %0.1f deg' % betas[0])
      ax.set_xlabel('alpha [deg]')
      ax.set_ylabel(k)
    else:
      CS = ax.contour(
          alphas, betas, o[k], levels, cmap=colormap)
      ax.clabel(CS, inline=1, fontsize=11)
      if omega_hat is None:
        title = k + ' as function of alpha and beta @ default omega_hat'
      else:
        title = (k + ' as function of alpha and beta @ omega_hat '
          + '[%0.2f, %0.2f, %0.2f]' % tuple(o for o in omega_hat))
      ax.set_title(title)
      ax.set_xlabel('alpha [deg]')
      ax.set_ylabel('beta [deg]')
    if label is not None:
      ax.legend()
    ax.grid(linestyle='--', linewidth=0.5)
    plt.tight_layout()
  return plots
