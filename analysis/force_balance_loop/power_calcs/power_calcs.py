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

from __future__ import print_function
from __future__ import absolute_import

import numpy as np
from scipy import interp
from scipy import stats
from scipy import trapz
from scipy.optimize import minimize
import json
import copy
import itertools
import time
import pprint
import pandas as pd

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from cycler import cycler

from lib import utils
from power_calcs import kite_path
from power_calcs import kite_pose
from power_calcs import kite_loop
from tools import rotor_model_util
from six.moves import range
from six.moves import zip

i = 0
j = 1
k = 2


def calc_loyd(kite, alpha, beta, apply_offsets=True, model_tether_drag=True):
  """Returns zeta, cL, and cL/cD for given alpha and beta.

  Aero coefficient offsets and tether drag in the config can be turned on or off via the
  'apply_offsets' and "model_tether_drag' booleans, respectively."""
  state = {}
  state['alpha'] = alpha
  state['beta'] = beta

  # Get body coeff and calc aero coeff.
  state.update(kite['body_coeff_from_alpha_beta'](alpha, beta))
  kite_pose.KitePose._aero_coeff_from_body_coeff(state)

  if apply_offsets:
    kite_pose.KitePose._apply_aero_offsets(state, kite)
  if model_tether_drag:
    total_drag = state['cD'] + kite['cD_eff_tether']
  else:
    total_drag = state['cD']

  return (
      (4./27.) * state['cL']**3 / total_drag**2,
      state['cL'],
      state['cL'] /  total_drag)


def calc_loyd_optimums(
    kite, apply_offsets=True, model_tether_drag=True, alpha_range=(None, None),
    beta_range=(None, None)):
    """Finds the Loyd optimum operating point for a given kite config.

    Only works for an aero database in body_coeff_from_alpha_beta form."""

    bounds = [alpha_range, beta_range]

    alpha, beta = minimize(
        lambda x: -calc_loyd(
          kite, *x, apply_offsets=apply_offsets, model_tether_drag=model_tether_drag)[0],
        [0., 0.], bounds=bounds)['x']

    # calc loyd zetas and cL/cD for use in plots later
    zeta, cL, cL_over_cD = calc_loyd(kite, alpha, beta, apply_offsets, model_tether_drag)
    print('Max zeta %0.2f for cL of %0.2f, cL/cD is %0.2f at alpha %0.2f and '
          'beta %0.2f.' % (zeta, cL, cL_over_cD, alpha, beta))
    return zeta, cL, cL_over_cD, alpha, beta

class KitePowerCurve(object):
  """
  Creates a power curve object for a given flight path, resource, and kite config.
  Essentially a container and solver for KiteLoop objects.

  Poses can be explicitly specified or optimized for. See KitePose for
  available options and methodology.

  Args:
    resource: a dictionary that specifies a resource
      Required items:
        v_w_at_height:
          a function of (height, wind_speed_at_ref) that returns a wind speed
          at a given height and wind speed at ref
        rho: air density
      Optional items:
        v_w_avg_h_ref:
          a default wind speed that is used as wind speed at ref when none
          is provided
    config: a dictionary that specifies a kite configuration
      Required items:
        <aero_model>:
          An aero model for the kite (no tether) must be provided.
          options are:
            aero_coeff_from_alpha_beta:
              as above, but includes aero side lift coefficient, cY
            body_coeff_from_alpha_beta:
              as above, but body coefficients including cy.
              note that cY is aero coefficient, and cy is body.
        cD_eff_tether:
          Effective drag coefficient of the tether at the kite, referenced to
          wing area.
        eta_shaft_to_pad:
          a function of (power_shaft) that returns a total efficiency for
          the system from shaft to padmount
        m_kite:
          mass of the kite in kg
        m_tether':
          mass of the tether in kg
        power_shaft_max:
          maximum net (all motors) shaft power limit, in watts
        s:
          Kite reference wing area, in m^2
        c:
          Kite reference chord (Mean Aerodynamic Chord), in m.
        b:
          Kite reference span, in m.
        l_tether:
          length of the tether from GS attach point to the wing
          (including bridle length)
        shaft_power_from_drag_power:
          A function of (rho, c_sound, v_a, rotor_force) that returns a
          dictionary that must at least contain 'power_shaft'.
          Negative force is generation.

          Required returned items:
            power_shaft:
              Shaft power, in W.
          Optional items:
            Anything desired to be appended to state, such as performance
            coefficients, status messages, additional constraints (see KitePose
            code for format).
            Things appended to state are accessible by all plotting tools, etc

            Important notes:
              If constraints are provided it's assumed the function is doing its
              own stall and/or Betz checks. If none are provided, a Betz limit
              constraint is applied to the pose, and 'rotor_area' (total) is
              required to be in the config.
      Optional items:
        alpha_min:
          Minimum alpha, applied as a constraint.
        alpha_max:
          Maximum alpha, applied as a constraint.
        cL_max:
          Maximum coefficient of lift, applied as a constraint.
        cL_offset:
          Additional aero coefficient offset to be applied to the kite
        cY_offset:
          Additional lateral lift coefficient offset to be applied to the kite
        cD_offset:
          Additional drag coefficient offset to be applied to the kite
        min_turn_r:
          Instantaneous minimum turning radius, applied as a constraint.
        tension_max:
          Maximum operating tension, in Newtons. Applied as a constraint.
        tension_min:
          Min operating tension, in Newtons. Applied as a constraint.
        incl_max:
          Maximum inclination for any individual pose. Applied as a constraint.
        h_min:
          Minimum height AGL for any individual pose. Applied as a constraint.
        roll_min:
          Minimum roll angle, tether to kite.
          Positive roll is in to nominal clockwise looking downwind circle.
          Ie: rolling to kite's left.
        roll_max:
          Maximum roll angle, tether to kite.
          See roll_min for details.
        v_a_min:
          Minimum allowable airspeed for the kite, in m/s.
        v_a_max:
          Maximum allowable airspeed for the kite, in m/s
        aero_thrust_p_max/min
          The M600 has aero thrust power limits,
          max_airspeed_control_power_gen/motor, to ensure the controller is not
          expecting unreasonable thrusts.
          Source: https://codesearch.corp.google.com/makani/config/m600/control/crosswind.py?type=cs&q=max_airspeed_control_power_gen&g=0&l=467
          Notation is - for gen and + for thrust.
          This limit is not inherent to the physics of the FBL or physical kite
          limits, but may be useful when trying to compare results to csim.

        gs_position:
          Array like position vector specifying the tether attach point in
          ground coordinates. If not specified, will default to [0,0,0]
        v_w_h_hub_cut_out:
          Wind speed at virtual hub height for cut out.
          Will truncate v_w range in KitePowerCurve if exceeded, and is
          used to clean up power curve in KitePowerCurve object
          and provide ref lines in plotting tools.
        v_w_h_hub_cut_in:
          Wind speed at virtual hub height for cut in.
          Used to clean up power curve in KitePowerCurve object and
          provide ref lines in plotting tools.
        v_a_max:
          maximum kite airspeed, in m/s
        rotor_area:
          Total rotor area (all rotors) in m^2.
          Required if 'shaft_power_from_drag_power' does not provide
          constraints, in which case local Betz limit is calculated and applied
          as a simple constraint.
          See 'shaft_power_from_drag_power' item for details.

          Entirely ignored if 'shaft_power_from_drag_power' function in config
          provides constraints, as it's assumed the rotor function is doing
          its own, more detailed modeling.

  Kwargs:
    pose_states:
      List of pose_states for each wind speed. See KiteLoop for format of each
      item.
    pose_states_param:
      List of parameterized pose states for each wind speed. See KiteLoop for
      format of each item.
    path_shape_params:
      dictionary that specifies path shape.
      see KitePath for usage
    path_location_params:
      dictionary that specifies path_location
      see KitePath for usage
    vars_to_opt:
      dict of variables to optimize over, and method of parameterization
      see KiteLoop for usage
    v_w_at_h_ref_range:
      range of wind speeds at h_ref to sweep through
    v_w_step:
      step size of wind speeds to sweep through
    v_w_hat:
      wind direction. nominal direction is [1, 0, 0]
    grav_mult:
      multiplier to apply to gravity - nominal is 1.
    verbose:
      Boolean - adds additional print statements about calculation.
    opt_params:
      optimization params that are passed to the KiteLoop objects
      see KiteLoop for details
    power_lim_simple:
      Boolean - if true, will only calculate power curve until first rated
      power point, then assumes rated power can be met until end of
      v_w range or cut out

  Typical Usage:
    Create a KitePowerCurve with a resource, and kite config,
    specifying any desired optimization parameters or fixed values.

    Use KitePowerCurve.solve() to fill object with results.
    Use plotting tools to inspect results, or directly pull desired data out.

    Summary loop data is contained in a pandas Dataframe object in:
      self.data_loops
    KiteLoop objects for each wind speed are contained in:
      self.loops
    Data and plotting functions for each loop can be accessed via this list

    A filtered and truncated power curve is provided with powers_final.

    If seeds are provided in vars_to_opt, it is only used on the first
    wind speed. All other loops are seeded with the previous loops results.

  """
  def __init__(self, resource, config, v_w_at_h_ref_range=(2.,21.),
               v_w_step=1., v_w_hat=None, grav_mult=1.0, verbose=False,
               opt_params={}, power_lim_simple=False, **kwargs):

    self.resource = resource
    self.config = config
    self.verbose = verbose
    self.v_w_hat = v_w_hat
    self.grav_mult = grav_mult
    self.opt_params = dict(opt_params)
    self.power_lim_simple = power_lim_simple
    self.pose_states = kwargs.pop('pose_states', {})
    self.pose_states_param = kwargs.pop('pose_states_param', {})
    self.solver = kwargs.get('solver', {})

    self.path_location_params = kwargs.pop('path_location_params', {})
    self.path_shape_params = kwargs.pop('path_shape_params', {})
    self.kwargs = utils.deepcopy_lite(kwargs)

    self.v_ws_at_h_ref = np.arange(v_w_at_h_ref_range[0],
                                   v_w_at_h_ref_range[1],
                                   step=v_w_step).tolist()

    # Define cut in and cut out points if defined, else set to +/- infinity.
    self.v_w_h_hub_cut_in = config.get('v_w_h_hub_cut_in', float('-inf'))
    self.v_w_h_hub_cut_out = config.get('v_w_h_hub_cut_out', float('inf'))

    self.powers = []
    self.loops = []
    self.valids = []
    self.v_ws_at_h_hub = []
    self.v_w_h_hub_first_rated = None
    self.v_w_h_ref_first_rated = None

    self.p_rated = (
        self.config['power_shaft_max']
        * self.config['eta_shaft_to_pad'](self.config['power_shaft_max']))

  def __getitem__(self, key):
    return getattr(self, key)

  def solve(self):
    """Fills KitePowerCurve with results data."""

    start_t = time.time()
    prev_opt_vars = self.kwargs.pop('vars_to_opt', {})

    self.solve_summary = ('\'V\' for Valid, \'I\' for Invalid\n')
    column_format = '{: ^13}' * len(self.v_ws_at_h_ref)
    self.solve_summary += ((column_format.format(
        *[str(v_w) + 'm/s' for v_w in self.v_ws_at_h_ref])) + '\n' +
        ('-' * 13*len(self.v_ws_at_h_ref)) + '\n')
    if not self.verbose:
      # Print a minimal status even if not verbose.
      print('Solving for v_ws at h_ref: ')
      print(self.solve_summary)
    self.solve_summary = 'Solved for v_ws at h_ref: \n' + self.solve_summary

    for ii, v_w in enumerate(self.v_ws_at_h_ref):
      if self.verbose:
        print()
        print(utils.TextFormat.BOLD + 'Solving for v_w at ref = %0.1f m/s'
              % v_w + utils.TextFormat.END)
      else:
        status = (
            ['V' if v else 'I' for v in self.valids])
        power = (
            ['%d kW' % (p / 1000.0) for p in self.powers])
        # CoLab clears the line when a carriage return is printed, so we must
        # return before the new print to overwrite the old one rather than
        # carriage return at the end of the old print.
        messages = ['\r'] + [s + ': ' + p for s, p in zip(status, power)]
        messages.append('Solving...')
        column_format = '{}' + '{: ^13}' * (len(messages) - 1)
        # End is blank to prevent default carriage return and new line.
        # This allows overwriting the previous print.
        print(column_format.format(*messages), end='')

      # If pose or path states are specified, hand them to loop objects.
      input_kwargs = dict(self.kwargs)
      input_args =  ['pose_states', 'pose_states_param', 'path_location_params',
                     'path_shape_params']
      for input_arg_key, input_arg in zip(
          input_args, [self[key] for key in input_args]):
        if input_arg:
          if type(input_arg) is list:
            input_kwargs[input_arg_key] = input_arg[ii]
          else:
            input_kwargs[input_arg_key] = input_arg


      loop = kite_loop.KiteLoop(
          self.resource, self.config, v_w_at_h_ref=v_w, v_w_hat=self.v_w_hat,
          grav_mult=self.grav_mult, verbose=self.verbose,
          opt_params=self.opt_params, vars_to_opt=prev_opt_vars,
          solver=self.solver,
          **input_kwargs)

      self.v_ws_at_h_hub.append(loop.v_w_at_h_hub)

      loop.solve()

      # Seed the next loop with the previous loop optimization results.
      for key, value in loop.vars_to_opt.items():
        prev_opt_vars[key] = copy.copy(value)
        prev_opt_vars[key]['values'] = (
            loop._vars_to_opt_param_dict[key].get_param_values())

      self.powers.append(loop.power)
      self.loops.append(loop)
      self.valids.append(loop.valid)

      # Check if we are at rated power
      if loop.power/self.p_rated > 0.99:
        if self.v_w_h_hub_first_rated is None:
          self.v_w_h_hub_first_rated = loop.v_w_at_h_hub
          self.v_w_h_ref_first_rated = loop.v_w_at_h_ref
        # If simple power limit, assume we can hold rated power and fill in
        # rest of power curve.
        if self.power_lim_simple:
          extend_len = len(self.v_ws_at_h_ref) - len(self.powers)
          self.powers.extend([self.p_rated] * extend_len)
          self.valids.extend([True] * extend_len)
          v_ws_remaining = np.array([v for v in self.v_ws_at_h_ref if v > v_w])
          self.v_ws_at_h_hub.extend(
              self.resource['v_w_at_height'](
                  loop.path.h_hub,
                  v_ws_remaining).tolist())
          break

    # Fill in the last column of solve status messages.
    status = (
        ['V' if v else 'I' for v in self.valids])
    power = (
        ['%d kW' % (p / 1000.0) for p in self.powers])
    messages = ['\r'] + [s + ': ' + p for s, p in zip(status, power)]
    column_format = '{}' + '{: ^13}' * (len(messages) - 1)
    result = column_format.format(*messages) + '\n'
    self.solve_summary += result
    if not self.verbose:
      print(result)
    print()

    self._extract_loop_data()
    self._calc_pc_data()
    self._clean_power_curve()

    end_t = time.time()
    self.solve_time = end_t - start_t
    print('Time to solve power curve is %0.2fs' % self.solve_time)

  def _calc_pc_data(self):

    self.zeta_max_actual = max(self.data_loops['zeta_padmount_avg_time'])
    if self.v_w_h_hub_first_rated:
      self.zeta_first_p_rated = (
        self.data_loops['zeta_padmount_avg_time'][self.v_w_h_ref_first_rated])
    else:
      self.zeta_first_p_rated = None

  def _extract_loop_data(self):
    if self.verbose:
      print('Extracting loop data into DataFrame.')

    data = []
    index = []

    self.pose_states_output = []
    self.pose_states_param_output = []
    self.path_location_params_output = []
    self.path_shape_params_output = []

    for ii, loop in enumerate(self.loops):
      index.append(loop.data_loop['v_w_at_h_ref'])
      data.append(loop.data_loop)
      self.pose_states_output.append(loop.pose_states)
      self.pose_states_param_output.append(loop.pose_states_param)
      self.path_location_params_output.append(loop.path_location_params)
      self.path_shape_params_output.append(loop.path_shape_params)

    self.data_loops = pd.DataFrame(data, index=index)

  def plot_paths(self, **kwargs):
    figsize = kwargs.get('figsize', (6,6))
    fig = kwargs.get('fig', None)
    every_nth = kwargs.get('every_nth', 1)
    path_colormap = kwargs.get('path_colormap', 'Greys')
    if not fig:
      fig, ax = plt.subplots(figsize=figsize)
      kwargs['fig'] = fig
    ax = fig.get_axes()[0]

    path_colors = [plt.cm.get_cmap(path_colormap)(x) for x in
        np.linspace(0.2, 1., len(self.loops))]
    for ii, (loop, path_color) in enumerate(zip(self.loops, path_colors)):
      if ii % every_nth == 0:
        fig = loop.plot_path_positions(path_color=path_color, **kwargs)
    ax.legend()
    ax.set_title('Crosswind Plane Paths & Poses for Each Wind Speed')
    return fig

  def plot_constraints(self, **kwargs):
    """Plots constraints for each loop.

    Kwargs:
      figsize_path: Figure size for plotting path.
      figsize_constraints: Figure size for plotting constraints.
    """
    figs = {}

    # Extract data for all constraints for all loops.
    out = {}
    for loop in self.loops:
      constraints = {}
      for c in loop.constraints:
        if c['name'] not in constraints:
            constraints[c['name']] = []
        constraints[c['name']].append(c['margin_norm'])
      for k, v in constraints.items():
        if k not in out:
          out[k] = []
        out[k].append(constraints[k])

    constraint_keys = sorted(
        [c['name'] for c in self.loops[0].poses[0].state['constraints']])
    num_constraints = len(constraint_keys)

    # Plot crosswind paths to show color scheme for positions.
    figsize_path = kwargs.get('figsize_path', (7, 7))
    figs['paths'] = self.plot_paths(figsize=figsize_path)
    figs['paths'].suptitle('Color Key of Poses for Constraints')
    figs['paths'].get_axes()[0].set_title(
        'Crosswind Plane Paths & Poses for Each Wind Speed')

    # Plot constraints
    figsize_constraints = (
        kwargs.get('figsize_constraints_h', 11),
        kwargs.get('figsize_constraints_v', 6) * num_constraints)
    figs['constraints'] = plt.figure(figsize=figsize_constraints)
    plt.subplots_adjust(hspace=0.4, top=0.99, bottom=0.01)

    for ii, k in enumerate(constraint_keys):
      plt.subplot(num_constraints, 1, ii+1)
      utils.set_rainbow_plot_cycler(n=len(out[k][0]), ax=plt.gca())
      plt.xlabel('Wind Speed at h_ref [m/s]')
      plt.title('Constraint: ' + k)
      ps = plt.plot(self.v_ws_at_h_ref, out[k])
      plt.ylim((min(-0.1, min(min(out[k]))), None))
      plt.legend(ps, labels=list(range(len(ps))), bbox_to_anchor=(1.01, 1.0), loc=2)
      plt.grid(linewidth=1, linestyle=':')

    # Restore the default plotting style.
    plt.show()
    return figs

  def plot_convergence_data(self, **kwargs):
    """Plots loop convergence data for each wind speed.

    Warning: Plotting is slow, and consumes a great deal of memory.

    Kwargs:
      figsize_values_h: Horizontal figure size for plotting values.
      figsize_values_v: Per value vertical figure size.
      figsize_path: Figure size for plotting path.
      figsize_constraints_h: Horizontal figure size for plotting constraints.
      figsize_constraints_v: Per constraint vertical figure size.
    """
    #TODO: Add more plotting options. Speed up data extraction.

    out = {'obj_out': []}
    winds = [(0., None)]
    for ii, (wind, loop) in enumerate(zip(self.v_ws_at_h_ref, self.loops)):
      winds.append((winds[ii][0] + len(loop.convergence_data), wind))
      for ii, d in enumerate(loop.convergence_data):
        for k, v in d['params'].items():
          if k not in out:
              out[k] = []
          out[k].append(v)
        out['obj_out'].append((d['obj_out'],))
        constraints = {}
        for c in d['constraints']:
          if c['name'] not in constraints:
              constraints[c['name']] = []
          constraints[c['name']].append(c['margin_norm'])
        for k, v in constraints.items():
          if k not in out:
            out[k] = []
          out[k].append(constraints[k])

    constraint_keys = sorted(
        [c['name'] for c in self.loops[0].poses[0].state['constraints']])
    value_keys = [x for x in sorted(out.keys()) if x not in constraint_keys]

    num_values = len(value_keys)
    num_constraints = len(constraint_keys)
    num_poses = self.loops[0].num_poses

    figs = {}

    # Plot crosswind paths to show color scheme for positions.
    figsize_path = kwargs.get('figsize_path', (7, 7))
    figs['paths'] = self.plot_paths(figsize=figsize_path)
    figs['paths'].suptitle('Color Key of Poses for Constraints')
    figs['paths'].get_axes()[0].set_title(
        'Crosswind Plane Paths & Poses for Each Wind Speed')

    # Plot values.
    figsize_values = (
      kwargs.get('figsize_values_h', 11),
      kwargs.get('figsize_values_v', 6) * num_values)
    figs['values'] = plt.figure(figsize=figsize_values)
    plt.subplots_adjust(hspace=0.4, top=0.99, bottom=0.01)

    for ii, k in enumerate(value_keys):
      plt.subplot(num_values, 1, ii+1)
      utils.set_rainbow_plot_cycler(n=len(out[k][0]), ax=plt.gca())
      plt.xlabel('Iteration number')
      plt.title('Optimization Values: ' + k)
      ps = plt.plot(out[k])
      y_b, y_t = plt.gca().get_ylim()
      for i, wind in winds[1:]:
        plt.axvline(i, color='black', linestyle='--')
        plt.text(i, 0.9 * (y_t - y_b) + y_b, 'v_w: %0.1f'% wind,
                 horizontalalignment='right', rotation=90, fontsize=13)
      plt.legend(ps, labels=list(range(len(ps))), bbox_to_anchor=(1.01, 1.0), loc=2)
      plt.grid(linewidth=1, linestyle=':')

    # Plot constraints
    figsize_constraints = (
        kwargs.get('figsize_constraints_h', 11),
        kwargs.get('figsize_constraints_v', 6) * num_constraints)
    figs['constraints'] = plt.figure(figsize=figsize_constraints)
    plt.subplots_adjust(hspace=0.4, top=0.99, bottom=0.01)

    for ii, k in enumerate(constraint_keys):
      plt.subplot(num_constraints, 1, ii+1)
      utils.set_rainbow_plot_cycler(n=len(out[k][0]), ax=plt.gca())
      plt.xlabel('Iteration number')
      plt.title('Constraint: ' + k)
      ps = plt.plot(out[k])
      plt.ylim((min(-0.1, min(min(out[k]))), None))
      y_b, y_t = plt.gca().get_ylim()
      for i, wind in winds[1:]:
        plt.axvline(i, color='black', linestyle='--')
        plt.text(i, 0.9 * (y_t - y_b) + y_b, 'v_w: %0.1f'% wind,
                 horizontalalignment='right', rotation=90, fontsize=13)
      plt.legend(ps, labels=list(range(len(ps))), bbox_to_anchor=(1.01, 1.0), loc=2)
      plt.grid(linewidth=1, linestyle=':')
    plt.show()
    return figs

  def plot_pose_data(self, x='dist_norm', ys=['power', 'v_k', 'tension', 'cL'],
                     ylims=None, figsize=(7,5), every_nth=1, **kwargs):
    """Plots pose data for each loop in self.loops.

    Kwargs:
      figsize: Matplotlib figsize tuple.
      legend_kwargs: Kwargs passed to matplotlib legend call.
      wrap_x: Value to use for wrapped x. If not provided, plots do not wrap
        back to first pose. Need a value because we're unable to assume even
        pose spacing.
    """
    legend_kwargs = {'bbox_to_anchor': (1.01, 1.), 'prop':{'size':8}}
    legend_kwargs.update(kwargs.get('legend_kwargs', {}))
    wrap_x = kwargs.get('wrap_x', None)
    dpi = kwargs.get('dpi', None)
    if ys == 'all':
      ys = []
      data = self.loops[0].data_poses.to_dict()
      for key in data.keys():
        ys.append(key)

    plot_kwargs = kwargs.get('plot_kwargs', {})
    base_label = kwargs.get('base_label', '')

    if ylims == None:
      ylims = [None] * len(ys)

    fig = kwargs.get('fig', None)
    if fig is None:
      fig, axes = plt.subplots(figsize=(figsize[0],figsize[1] * len(ys)),
                               nrows=len(ys), ncols=1, dpi=dpi)
    else:
      axes = fig.get_axes()

    num_colors = len(
        [n for n in range(len(self.v_ws_at_h_hub)) if n % every_nth==0])
    if not hasattr(axes, '__iter__'):
      # Axes must be iterable to be later zipped with the ys.
      axes = [axes]

    for ax in axes:
      utils.set_rainbow_plot_cycler(n=num_colors, ax=ax)

    for key, ylim, ax in zip(ys, ylims, axes):
      try:
        ax.set_title('%s vs %s for different v_ws at hub height' % (key, x))
        if ylim is not None:
          ax.set_ylim(ylim)
        for ii, (loop, v_w) in enumerate(zip(self.loops, self.v_ws_at_h_hub)):
          if ii % every_nth is 0:
            if wrap_x is not None:
              xs = np.array(loop.data_poses[x].tolist() + [wrap_x])
              ys = np.array(
                  loop.data_poses[key].tolist() + [loop.data_poses[key][0]])
            else:
              xs = loop.data_poses[x]
              ys = loop.data_poses[key]
            sort_idx = np.argsort(xs)
            ax.plot(xs[sort_idx], ys[sort_idx],
                    label=(base_label + '%0.1f m/s'%v_w), **plot_kwargs)
        ax.set_xlabel(x)
        ax.legend(**legend_kwargs)
      except:
        print('Unable to plot %s variable.' % key)
        print('Values are:')
        print([loop.data_poses[key] for loop in self.loops])
    return fig

  def plot_pose_data_as_surf(self,
      x='dist_norm', keys=['power', 'v_k', 'v_a', 'tension', 'cL', 'valid'],
      ylims=None, figsize=(7,5), plot_args={}):
    """
    Plots loop pose data with provided x_axis and z_axis (keys) for each wind speed (y_axis).

    Anything plottable in KitePose.state can be provided for keys or x_axis.
    """

    x_base = self.loops[0].data_poses[x]

    if keys == 'all':
      keys = []
      data = self.loops[0].data_poses.to_dict()
      for key in data.keys():
        keys.append(key)

    plot_args_final = {'cmap': cm.coolwarm,
                       'linewidth': 0.5,
                       'rstride':1,
                       'cstride':1,}

    plot_args_final.update(plot_args)

    if ylims is None:
      ylims = [None] * len(keys)

    for key, ylim in zip(keys, ylims):
      zs = []
      v_ws = []
      for ii, (loop, v_w) in enumerate(zip(self.loops, self.v_ws_at_h_hub)):
        v_ws.append(v_w)
        zs.append(loop.data_poses[key])
      zs = np.array(zs)

      xs, ys = np.meshgrid(x_base, v_ws)

      if not (np.shape(xs) == np.shape(ys) == np.shape(zs)):
        print('Unable to plot - variables are not the same shape.')
      else:
        try:
          fig = plt.figure(figsize=figsize, dpi=100)
          ax = fig.gca(projection='3d')
          if ylim is not None:
            ax.ylim(ylim)
          fig.suptitle('%s vs Normalized Distance around Loop and v_w at h_hub' % key)
          surf = ax.plot_surface(xs, ys, zs, antialiased=True, label=key,
                                 **plot_args_final)
        except:
          print('Unable to plot %s variable.' % key)
          print('Values are:')
          print(zs)

  def plot_vectors(
      self, ys=['pqr', 'omega_hat'], ys_components=[['p','q','r']],
      x='dist_norm', label='v_w_at_h_hub', every_nth=1, plot_kwargs={},
      **kwargs):
    figsize = kwargs.get('figsize', (9, 6))
    ylims = kwargs.get('ylims', None) # None uses defaults, 'equal' sets all equal to th emax, tuple is applied to all, or list of matching length for each
    # TODO: only 'equal' or None is implemented as of now

    fig = kwargs.get('fig', None)
    if fig is None:
      fig, axes = plt.subplots(figsize=(figsize[0],figsize[1] * len(ys)),
                               nrows=len(ys), ncols=1, dpi=100)
    else:
      axes = fig.get_axes()

    if len(ys) == 1:
      # Axes must be iterable to be later zipped with the ys.
      axes = [axes]

    for ax in axes:
      colors = utils.set_rainbow_plot_cycler(n=len(self.v_ws_at_h_hub), ax=ax)
      ax.set_prop_cycle(cycler('linestyle', ['-', '--', '-.']))

    for ii, loop in enumerate(self.loops[::every_nth]):
      plot_kwargs.update({'c':colors[ii * every_nth]})
      fig = loop.plot_vectors(ys=ys, ys_components=ys_components,
          plot_kwargs=plot_kwargs, fig=fig,
          base_label=', {}m/s'.format(loop[label]))

    if ylims == 'equal':
      ymin = min([ax.get_ylim()[0] for ax in axes])
      ymax = max([ax.get_ylim()[1] for ax in axes])
    for ax, y in zip(axes, ys):
      ax.set_title('{} for various {}'.format(y, label))
      ax.set_xlabel(x)
      if ylims == 'equal':
        ax.set_ylim((ymin,ymax))

    return fig

  def plot_vectors_components(
      self, ys=['pqr', 'omega_hat'], y_components=['p','q','r'], ys_labels=None,
      x='dist_norm', label='v_w_at_h_hub', every_nth=1, plot_kwargs={},
      **kwargs):
    """Plots vector components.

    Args:
      ys: A list of variables in state to plot. Nested items (such as moments
        and forces) can be accessed via an iteratble of the nest levels.
      y_components: Labels to apply to each item in the vector.
      x: Variable in state to use as x-axis.
      label: Variable from loop object to append to labels.
      every_nth: Plot every n_th loop to unclutter the plot.
      plot_kwargs: Kwargs to pass to matplotlib plot function.

    Kwargs:
      figsize: Sets EACH figure size. Total figure size is each * number of ys
        getting plotted.
      ylims: Either 'equal' or None. None uses default scaling, while equal sets
        all limits equal to the max.
      one_legend: Boolean to show one legend for all plots. True by default.
    """
    figsize = kwargs.get('figsize', (9, 6))
    ylims = kwargs.get('ylims', None)
    # TODO: Implement tuple or list-like ylims.

    # Need to store the colors here, used below.
    colors = utils.set_rainbow_plot_cycler(n=len(self.v_ws_at_h_hub))
    full_linestyles = [
        '-', '--', '-.', ':', (0, (3, 1, 1, 1, 1, 1)),
        (0, (3, 1, 1, 1, 1, 1, 1, 1, 1, 1))]*3

    fig, axes = plt.subplots(
        figsize=(figsize[0],figsize[1] * len(y_components)), ncols=1,
        nrows=len(y_components), dpi=100)
    if len(y_components) == 1:
      # Axes must be iterable to later zipped with the ys.
      axes = [axes]

    for ax in axes:
      utils.set_rainbow_plot_cycler(n=len(self.v_ws_at_h_hub), ax=ax)
      ax.set_prop_cycle(cycler('linestyle', full_linestyles[:len(ys)]))

    for ii, loop in enumerate(self.loops[::every_nth]):
      fig = loop.plot_vectors_components(ys=ys, y_components=y_components,
          ys_labels=ys_labels,
          plot_kwargs={'c':colors[ii * every_nth]}, fig=fig,
          base_label=', {}m/s'.format(loop[label]))

    if ylims == 'equal':
      ymin = min([ax.get_ylim()[0] for ax in axes])
      ymax = max([ax.get_ylim()[1] for ax in axes])
    for ax, y_component in zip(axes, y_components):
      ax.set_title('{} component for various {}'.format(y_component, label))
      ax.set_xlabel(x)
      if ylims == 'equal':
        ax.set_ylim((ymin,ymax))

    if kwargs.get('one_legend', True):
      for ii, ax in enumerate(fig.get_axes()):
        if ii > 0:
          ax.get_legend().remove()

    return fig

  def plot_loop_data(self, x='v_w_at_h_hub',
                     ys=['power_avg_time', 'zeta_padmount_avg_time',
                         'tension_avg_time',
                         'v_a_avg_time', 'total_time',
                         'valid', 'h_hub'],
                     ylims=None, figsize=(7, 5)):

    if ylims is None:
      ylims = [None] * len(ys)

    fig, axes = plt.subplots(figsize=(figsize[0], figsize[1] * len(ys)),
                             ncols=1, nrows=len(ys), dpi=100)
    for key, ylim, ax in zip(ys, ylims, axes):
      ax.set_title('%s vs %s' % (key, x))
      ax.set_xlabel(x)
      ax.set_ylabel(key)
      ax.plot(self.data_loops[x], self.data_loops[key], label=key)
      if 'avg' in key: #then we should also show the min and max
        #pick off the avg_dist or avg_time based on number of letters
        base_key = key[:-8]
        min_key = base_key + 'min'
        max_key = base_key + 'max'
        ax.plot(self.data_loops[x], self.data_loops[min_key], '--', label=min_key)
        ax.plot(self.data_loops[x], self.data_loops[max_key], '--', label=max_key)
      if key == 'zeta_padmount_avg':
        ax.ylim(bottom=0., top=1.2*np.max(self.data_loops[key]))

      if ylim is not None:
        ax.set_ylim(ylim)

      if x == 'v_w_at_h_hub':
        if self.v_w_h_hub_cut_in > -np.inf:
          ax.axvline(x=self.v_w_h_hub_cut_in, color='g', lw=1., ls=':', label='cut in')
        if self.v_w_h_hub_cut_out < np.inf:
          ax.axvline(x=self.v_w_h_hub_cut_out, color='r', lw=1., ls=':', label='cut out')
        if self.v_w_h_hub_first_rated is not None:
          ax.axvline(x=self.v_w_h_hub_first_rated, color='orange', lw=1., ls=':', label='first rated')
      ax.grid(linewidth=0.5, linestyle=':')
      ax.legend()
    return fig

  def plot_rotor_map(self, every_nth=1, **kwargs):
    """Map out the loop on the rotor table plots.
    The xyz variable names passed must be in the state dictionary that the
    shaft_power_from_drag_power function returns."""

    fig = kwargs.get('fig', None)
    label = kwargs.get('label', 'v_w_at_h_ref')
    legend_loc = kwargs.get('legend_loc', 'upper left')
    x = kwargs.pop('x', 'omega_rotor')
    y = kwargs.pop('y', 'v_a_along_rotor_axis')
    zs = kwargs.pop('zs', ['c_t', 'c_p', 'eta_rotors'])
    path_colormap = kwargs.get('path_colormap', 'Greys')

    path_colors = [plt.cm.get_cmap(path_colormap)(l) for l in
        np.linspace(0.2, 1., len(self.loops))]

    if 'torque_shaft_max' in self.config:
      torque_shaft_max = self.config['torque_shaft_max']
    else:
      torque_shaft_max = None

    if fig is None:
      fig = rotor_model_util.PlotRotor(
          self.config['shaft_power_from_drag_power'],
          rho=self.resource['rho'], c_sound=self.resource['c_sound'],
          power_shaft_max=self.config['power_shaft_max'],
          torque_shaft_max=torque_shaft_max,
          x=x, y=y, zs=zs,
          title='Rotor maps for loops at various {}'.format(label),
          **kwargs)

    for ii, loop in enumerate(self.loops[::every_nth]):
      fig = loop.plot_rotor_map(fig=fig,
          x=x, y=y, zs=zs,
          path_color=path_colors[ii * every_nth],
          label='{} m/s'.format(loop[label]),
          **kwargs)

    for ax in fig.get_axes()[:len(zs)]:
      ax.legend(loc=legend_loc)
    return fig

  def plot_power_curve(self, v_ws_h_type='both', power_type='both',
      plot_kwargs=None, **kwargs):
    figsize = kwargs.get('figsize', (9,6))
    fig = kwargs.get('fig', None)

    # Setting a base_label this way rather than w/ get or other method since
    # that seemed to have an odd bug that would reuse the previous label if
    # the method was called again w/o passing a plot_kwarg.
    if plot_kwargs is None:
      plot_kwargs = {}
      base_label = ''
    elif 'label' not in plot_kwargs:
      base_label = ''
    else:
      base_label = plot_kwargs['label']

    if not fig:
      fig, ax = plt.subplots(figsize=figsize, dpi=100)
    ax = fig.get_axes()[0]

    ax.set_title('Power Curve')
    ax.set_xlabel('v_w [m/s]')
    ax.set_ylabel('padmount power [W]')
    if v_ws_h_type in ['ref', 'both'] and power_type in ['all', 'both']:
      plot_kwargs['label'] = base_label + ' @ h_ref'
      ax.plot(self.v_ws_at_h_ref, self.powers, '--', **plot_kwargs)
    if v_ws_h_type in ['hub', 'both'] and power_type in ['all', 'both']:
      plot_kwargs['label'] = base_label + ' @ h_virt_hub'
      ax.plot(self.v_ws_at_h_hub, self.powers, '--', **plot_kwargs)
    if v_ws_h_type in ['ref', 'both'] and power_type in ['valid', 'both']:
      plot_kwargs['label'] = base_label + ' @ h_ref, valid'
      ax.plot(self.v_ws_at_h_ref, self.powers_valid, **plot_kwargs)
    if v_ws_h_type in ['hub', 'both'] and power_type in ['valid', 'both']:
      plot_kwargs['label'] = base_label + ' @ h_virt_hub, valid'
      ax.plot(self.v_ws_at_h_hub, self.powers_valid, **plot_kwargs)
    ax.axhline(linewidth=1, label=None)
    ax.grid(linewidth=0.5, linestyle=':')
    ax.legend()
    return fig

  def _clean_power_curve(self):
    self.powers_final = []
    self.powers_valid = []
    self.powers_positive_in_limits = []

    for power, valid, v_w_at_h_hub in zip(self.powers, self.valids,
                                                 self.v_ws_at_h_hub):
      if valid:
        self.powers_valid.append(power)
        if power <= 0.:
          power = 0.
        if power > (self.config['power_shaft_max']
                     * self.config['eta_shaft_to_pad'](self.config['power_shaft_max'])):
          power = (self.config['power_shaft_max']
                   * self.config['eta_shaft_to_pad'](self.config['power_shaft_max']))
        if (v_w_at_h_hub < self.v_w_h_hub_cut_in
            or v_w_at_h_hub > self.v_w_h_hub_cut_out):
          power = 0.
        self.powers_positive_in_limits.append(power)
      else:
        self.powers_valid.append(None)
        if (v_w_at_h_hub < self.v_w_h_hub_cut_in
            or v_w_at_h_hub > self.v_w_h_hub_cut_out):
          power = 0.
        self.powers_positive_in_limits.append(power)
        power = 0.
      self.powers_final.append(power)

  def make_report_file(self, location='power_curve_report.json',
                       keys=['power', 'v_k', 'v_a', 'tension', 'cL', 'roll_angle']):
    # TODO: finish, or remove
    raise NotImplemented

    xs = self.loops[0].data_poses['dist_norm']
    self.plotters = []

    if keys == 'all':
      keys = []
      data = self.loops[0].data_poses.to_dict()
      for key in data.keys():
        keys.append(key)

    for key in keys:
      zs = []
      ys = []
      for ii, (loop, v_w) in enumerate(zip(self.loops, self.v_ws_at_h_hub)):
        ys.append(v_w)
        zs.append(loop.data_poses[key])

      self.plotters.extend(pltly.surf(ys, xs, list(zip(*zs)), plot=False))
    pltly.plot(self.plotters)

  def turbulent_power_curve(self):
    '''post processes the power curve for 10 minute average points in a class 3A turbulent environment to IEC standard'''
    # Number of points in distribution
    n = 10
    # iec standard values for class A
    I_ref = 0.16
    b = 5.6

    x = np.linspace(stats.norm.ppf(0.05), stats.norm.ppf(0.95), n)  # linear spacing between 5th and 95th percentile
    x0 = x
    pdf = stats.norm.pdf(x)

    powers = []
    for v_w in self.v_ws_at_h_ref:
      # calculating sigma of 90th quantile from IEC
      sig = (I_ref * (.75 * v_w + b))

      x = x0 * (sig / x0[-1])
      x_ea = x + v_w

      # calculates all the powers at the different points
      y_ea = np.array(interp(x_ea, self.v_ws_at_h_ref, self.powers))


      av = trapz(y_ea * pdf, x_ea) / trapz(pdf, x_ea)
      powers.append(av)

    self.powers_turb = powers

  def gen_power_curve_vec_plot_file(
      self, location='power_curve_vec_plots.json', var='force_types', **kwargs):

    output = {'frames': [],
              'times': []}

    for v_w, loop in zip(self.v_ws_at_h_ref, self.loops):
      frame = {'data': loop.gen_loop_vec_plot_file(location=None, var=var, **kwargs),
               'name': str(v_w)}
      output['frames'].append(frame)
      output['times'].append(1.0)

    if location is not None:
      with open(location,'w') as outfile:
        json.dump(output, outfile)
        print('File saved to %s.' % location)
    else:
      return output
