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
from scipy.optimize import curve_fit
from scipy.optimize import minimize
from scipy.optimize import minimize_scalar
from scipy.interpolate import interp1d
import math
import numbers
import json
import time
import pprint
import pandas as pd

from matplotlib import pyplot as plt
from matplotlib import cm
from cycler import cycler

from lib import utils
from power_calcs import kite_path
from power_calcs import kite_pose
from tools import rotor_model_util
from lib import fun
from six.moves import range
from six.moves import zip


i = 0
j = 1
k = 2


class KiteLoop(object):
  """
  Contains all the poses for a loop. This allows you to evaluate an entire loop.

  Has lots of helper functions and plotting outputs for use with Plotly
  javascript libraries.

  The path and poses can be explictly specified, or left to the
  optimizer.

  Explicitly defining poses is done through either the pose_states or
  pose_states_param kwarg, path shape though path_shape_params, and path
  location through path_location_params. Any necessary information not
  specified will automatically seed and be determined by the optimizer,
  using default settings.

  Alternatively, you can explicitly specify what to optimize over and the
  settings for each variable by the vars_to_opt kwarg.

  Args:
    resource: a dictionary that specifies a resource
      See KitePowerCurve docstring for more details

    config: a dictionary that specifies a kite configuration
      See KitePowerCurve docstring for more details

  Kwargs:
    v_w_at_h_ref:
      optional wind speed, in m/s, at resource reference height
      if not provided, it attempts to use resource['v_w_avg_h_ref']
    v_w_hat:
      optional wind direction, in array like vector format
      if not provided, nominal direction is [1, 0, 0]
    grav_mult:
      multiplier to apply to gravity - nominal is 1.
    verbose:
      Boolean - adds additional print statements about calculation.
    pose_states:
      A dictionary of lists that specifies a variable and values for all
      poses in the loop. The lists must be of length of the number of poses,
      as specified in the path_shape_params or default settings.
      See KitePose for keys needed to solve a pose.
    pose_states_param:
      A dictionary of dictionaries, where each dictionary specifies a
      parameterization type
    path_shape_params:
      a dictionary of values needed to specify the kite path.
      See KitePath for acceptable inputs
    path_location_params:
      a dictionary of values needed to specify the path location.
      See KitePath for acceptable inputs
    vars_to_opt:
      Dictionary of variables to optimize over.
      Variables need to be a variable type for pose_states, path_shape_params,
      or path_location_params. Format is:
        {<variable_name>: {'type': <parameterization_type>,
                           'values': <initial_value>,
                           'max_step': <maximum_step_size>}}

        available parameterization types are:
          spline:
            A spline fit with evenly space control points about the path.
            Seed must be array-like with number of elements equal to
            number of desired control points.
          linear_interp:
            Linear interpolation with evenly spaced control points about the
            path. Seed must be array-like with number of elements equal to
            number of desired transition points.
          constant:
            All values around the loop are the same.
          value:
            Only a single value for the entire loop. No parameterization.
            This is to be used for things like path_shape_params and
            path_location_params, where generally a single value is used.
          each:
            Every pose is individually optimized.
    opt_params:
      optional dictionary of optimization params that are passed to
      self.optimize and KitePose objects.
      Optional items:
        maxiter:
          maximum iterations for v_k optimizer
        tol:
          tolerance for power in optimizer.
          higher values will stop the optimization sooner.
          see scipy.optimize for further details
        catol:
          tolerance for constraints violations in v_k optimizer
          see scipy.optimize for further details
        store_converge_data:
          Boolean that determines if convergence data during optimization is
          stored or not. This can be memory intensive for long optimizations.
        constraint_stiffness:
          all constraints are normalized, and then multiplied by this
          parameter. high values cause the optimizer to quickly flee solutions
          that violate constraints, while low values let the optimizer
          find the best power, then work on meeting constraints.
          typically you want low values of approx. 0.0001
        constraint_penalty_norm:
          normalized multiplier on power on normalized constraint values in
          optimizer. applies penalty to violated constraints as a way
          to further penalize failing to meet constraints.
          not typically needed.
        tension_penalty_cut_in:
          the portion of positions that must be at rated power to
          start ramping in a tension penalty for v_k optimizer.
          used to steer rated power positions towards min tension solutions
          ramps penalty linearly from zero at cut in to full at rated.
        tension_penalty_norm:
          maximum tension penalty to apply for v_k optimizer.
          penalty is this * rated_power * normalized tension * ramp_in
        flap_m_mult:
          Array-like penalties to be multiplied by absolute value of current
          flap aero coefficients.
        pose_solve_options:
          dictionary that is picked off and passed to KitePose.
          see KitePose for usage

  Typical Usage Examples:
    Create a KiteLoop with a resource, and kite config.

    Use KiteLoop.solve() to fill object with results.
    Use plotting tools to inspect results, or directly pull desired data out.

    Summary loop data is contained in:
      self.data_loop
    Pose data is contained in a pandas Dataframe object in:
      self.data_poses
  """

  def __init__(self, resource, config,
               v_w_at_h_ref=None, v_w_hat=None,
               grav_mult=1.0, verbose=False,
               opt=False,
               **kwargs):

    self.resource = resource
    self.config = config
    self.grav_mult = grav_mult
    self.verbose = verbose
    self.opt = opt

    if v_w_hat is not None:
      self.v_w_hat = np.asarray(v_w_hat) / utils.vec_length(v_w_hat)
    else:
      self.v_w_hat = np.array([1.,0.,0.])

    if v_w_at_h_ref is not None:
      self.v_w_at_h_ref = v_w_at_h_ref
    else:
      self.v_w_at_h_ref = self.resource['v_w_avg_h_ref']

    self._parse_kwargs(kwargs)

  def __getitem__(self, key):
    return getattr(self, key)

  def _parse_kwargs(self, kwargs):

    # Setup default optimization params.
    opt_params = {
        'maxiter': 2000,
        'tol': 0.01,
        'catol': 0.01,
        'constraint_penalty_norm': 0.,
        'store_converge_data': True,
        'constraint_stiffness': 0.001,
        'tension_penalty_cut_in': 0.95,
        'tension_penalty_norm': 0.0005,
        'flap_penalty_factor': 0.,
        'flap_m_mult': [0., 0., 0.]}
    opt_params.update(kwargs.get('opt_params', {}))
    self.opt_params = opt_params

    self.pose_solve_options = kwargs.get('pose_solve_options', {})
    self.solver = kwargs.get('solver', 'FBL')

    # Pose states and pose_states_param may get changed during solve, so we make
    # a copy.
    self.pose_states = utils.deepcopy_lite(kwargs.get('pose_states', {}))
    self.pose_states_param = utils.deepcopy_lite(
        kwargs.get('pose_states_param', {}))

    # Pose_states_param are converted into pose_states, which means specifying
    # the same variable in both is overspecifying, and pose_states_param steps
    # on pose_states.
    if self.verbose:
      over_specified = (
          [k for k in self.pose_states.keys() if k in
           list(self.pose_states_param.keys())])
      if over_specified:
        print(str(over_specified) + ' is/are specified in both pose_state and '
              + 'pose_states_param.\nPose_states_param is overriding.')

    # Path shape and location params may get changed during solve, so we make a
    # copy.
    self.path_shape_params = utils.deepcopy_lite(
        kwargs.get('path_shape_params', {}))
    self.path_location_params = utils.deepcopy_lite(
        kwargs.get('path_location_params', {}))
    self.given_path = kwargs.get('given_path', None)

    # Vars to opt will be changed during solve, so we make a copy.
    self.vars_to_opt = utils.deepcopy_lite(kwargs.get('vars_to_opt', {}))

    if self.given_path is None:
      kite_path.KitePath.setup_optimization(
          self.path_shape_params, self.path_location_params, self.vars_to_opt)

    # Make path here, as it's needed to make good seed guesses and fill in
    # number of poses for parameterized loop args.
    # TODO: Separate making seeds from parsing kwargs.
    self._make_path(self.path_shape_params,
                    self.path_location_params, self.given_path)

    # Turn parameterized args into states for each pose.
    loop_param_args = self._setup_param_args(self.pose_states_param)
    self._update_loop_inputs(loop_param_args)

    if all(k not in self.pose_states for k in ('v_a', 'v_k')):
      if all(k not in self.vars_to_opt for k in ('v_a', 'v_k')):
        self.vars_to_opt['v_a'] = (
            {'param_type': 'spline',
             'values': [self.config['v_a_min'] + 5.] * 6,
             'max_step': 5.})

    if ('alpha' not in self.pose_states) and ('alpha' not in self.vars_to_opt):
      self.vars_to_opt['alpha'] = (
          {'param_type': 'spline',
           'values': [3.] * 6,
           'max_step': 2.})

    if ('beta' not in self.pose_states) and ('beta' not in self.vars_to_opt):
      self.vars_to_opt['beta'] = (
          {'param_type': 'spline',
           'values': [3.] * 6,
           'max_step': 2.})

    if self.config.get('aero_device', None):
      if 'aero_device_scale' not in self.pose_states:
        if 'aero_device_scale' not in self.vars_to_opt:
          self.vars_to_opt['aero_device_scale'] = (
              {'param_type': 'spline',
               'values': [0.0] * 6,
               'max_step': 0.2})

  def _initialize_poses(self):
    self.poses = []
    self._make_path(
        self.path_shape_params, self.path_location_params, self.given_path)

    for k, states in self.pose_states.items():
      assert len(states) == self.num_poses, (
          'Number of states must be same as number of poses.\n'
          'Number of poses is: ' + str(self.num_poses) + '\n' +
          'Number of states is: ' + str(len(states)) + '\n' +
          'States are: ' + k + ': ' + str(states))
    poses_state = utils.dl_to_ld(self.pose_states)

    # If optimizing (non-blank vars_to_opt), then make sure the optimizer
    # constraint tolerance is tighter than pose constraint tolerance. Without
    # this check, the optimizer may provide constraint violations that result
    # in invalid poses.
    if (('catol' in self.opt_params) and ('vars_to_opt' != {})):
      if 'contol_norm' not in self.pose_solve_options:
        self.pose_solve_options['contol_norm'] = self.opt_params['catol']
      if self.opt_params['catol'] > self.pose_solve_options['contol_norm']:
        print(
            '\nPose constraint tolerance must be greater than or equal to '
            + 'optimization constraint tolerance. Overriding pose constraint '
            + 'tolerance with opt_params catol.')
        self.pose_solve_options['contol_norm'] = self.opt_params['catol']

    # Setup poses.
    for position, pose_params in zip(self.path.positions, poses_state):
      pose = kite_pose.KitePose(
          position, self.resource, self.config,
          self.v_w_at_h_ref, self.v_w_hat, grav_mult=self.grav_mult,
          opt=self.opt, solve_options=self.pose_solve_options, **pose_params)
      self.poses.append(pose)
    self._calc_knowns()

  def _enum_neighboring_indices(self):
    num_poses = len(self.poses)
    assert num_poses == self.num_poses
    for index in range(num_poses):
      next_index = 0 if index == (num_poses - 1) else index + 1
      prev_index = index - 1 if index > 0 else num_poses - 1
      yield prev_index, index, next_index

  def _enum_neighboring_poses(self):
    for p, i, n in self._enum_neighboring_indices():
      yield self.poses[p], self.poses[i], self.poses[n]

  def _calc_segment_time(self):
    # Solve for time.
    for prev_pose, curr_pose, next_pose in self._enum_neighboring_poses():
      curr_pose.state['segment_time'] = (
          curr_pose.state['segment_length'] /
          ((next_pose.state['v_k'] + curr_pose.state['v_k']) / 2.))

  def _calc_knowns(self):
    # Calc state derivatives to close loop with strategies given.

    # Solve for time.
    self._calc_segment_time()

    total_time = 0.
    total_dist = 0.

    for pose in self.poses:
      pose.state['time'] = total_time
      pose.state['dist'] = total_dist

      total_time += pose.state['segment_time']
      total_dist += pose.position['segment_length']

    self.data_loop = {'total_time': total_time,
                      'total_dist': total_dist}

    # Solve for acceleration along path.
    for prev_pose, pose, next_pose in self._enum_neighboring_poses():
      # Segment times are from current to next pose.
      prev_accel = (
          (pose.state['v_k'] - prev_pose.state['v_k'])
          / prev_pose.state['segment_time'])
      next_accel = (
          (next_pose.state['v_k'] - pose.state['v_k'])
          / pose.state['segment_time'])

      # Weight accel along path by how close in time it is between poses.
      pose.state['accel_along_path'] = utils.slope_linear(
        prev_accel, next_accel,
        pose.state['segment_time'],
        prev_pose.state['segment_time'])

    # If kite orientation is known, calculate pqr rates and rotational accel
    # moments.
    if 'kite_axis' in self.poses[0].state:
      self._calc_body_rates()
      self._calc_body_rates_dot()

  def _calc_body_rates(self):
    """Calculates body rates and stores them in pose states."""
    def angular_diff(initial_pose, end_pose):
      d_roll = utils.angle_signed(
          initial_pose.state['kite_axis']['y'],
          end_pose.state['kite_axis']['y'],
          initial_pose.state['kite_axis']['x'])
      d_pitch = utils.angle_signed(
          initial_pose.state['kite_axis']['x'],
          end_pose.state['kite_axis']['x'],
          initial_pose.state['kite_axis']['y'])
      d_yaw = utils.angle_signed(
          initial_pose.state['kite_axis']['y'],
          end_pose.state['kite_axis']['y'],
          initial_pose.state['kite_axis']['z'])
      return np.array([d_roll, d_pitch, d_yaw])

    for prev_pose, pose, next_pose in self._enum_neighboring_poses():
      prev_pqr = angular_diff(prev_pose, pose) / prev_pose.state['segment_time']
      next_pqr = angular_diff(pose, next_pose) / pose.state['segment_time']

      # Weight pqr by how close in time it is between poses.
      pose.state['pqr'] = utils.slope_linear(
          prev_pqr, next_pqr,
          pose.state['segment_time'],
          prev_pose.state['segment_time'])

      pose._calc_omega_hat()

  def _calc_body_rates_dot(self):
    """Calculates body rate accelerations and stores them in pose states."""

    for prev_pose, pose, next_pose in self._enum_neighboring_poses():

      # Segment times are from current to next pose.
      prev_d_pqr = pose.state['pqr'] - prev_pose.state['pqr']
      next_d_pqr = next_pose.state['pqr'] - pose.state['pqr']

      # Segment time is from active pose to next pose, so d_time is the segment
      # time for the first pose in the comparison.
      prev_pqr_dot = prev_d_pqr / prev_pose.state['segment_time']
      next_pqr_dot = next_d_pqr / pose.state['segment_time']

      # Weight pqr_dot by how close in time it is between poses.
      pose.state['pqr_dot'] = utils.slope_linear(
          prev_pqr_dot, next_pqr_dot,
          pose.state['segment_time'],
          prev_pose.state['segment_time'])

  def solve(self):
    if self.vars_to_opt:
      self.optimize(self.vars_to_opt, **self.opt_params)
    else:
      self._initialize_poses()

    self._solve_poses()
    self._calc_loop_data()

    if not self.opt:
      self._extract_pose_data()

  def _solve_poses(self):
    if self.verbose:
      print('Solving poses in loop...', end='')
      start_t = time.time()

    for pose in self.poses:
      pose.solve()

    if self.verbose:
      end_t = time.time()
      print('solved. Time is %0.4fs' % (end_t - start_t))

  def _make_path(
      self, path_shape_params, path_location_params, given_path=None):
    self.path = kite_path.KitePath(
        path_shape_params, path_location_params, self.config, given_path)
    self.v_w_at_h_hub = self.resource['v_w_at_height'](
        self.path.h_hub, self.v_w_at_h_ref)

    self.num_poses = self.path.num_pos
    self.poses = []

  def _extract_pose_data(self):

    if self.verbose:
      print('Extracting pose data into DataFrame.')

    data = {}
    keys = []

    # All pose states should contain the same keys, but we grab all unique keys
    # accross all poses to be sure.
    for pose in self.poses:
      keys.extend([key for key in pose.state.keys() if key not in keys])
    for key in keys:
      data[key] = []

    data['position_index'] = []

    for ii, pose in enumerate(self.poses):
      data['position_index'].append(ii)
      for key in keys:
        try:
          data[key].append(pose.state[key])
        except KeyError:
          data[key].append(np.NaN)

    data['dist_norm'] = (
        (np.array(data['dist']) / self.data_loop['total_dist']).tolist())

    self.data_poses = pd.DataFrame(data, index=data['position_index'])

  def _calc_loop_data(self):

    if self.verbose:
      print('Loop solved, now calculating loop data.')

    temp_p = 0.
    temp_vals = {'dist': {},
                 'time': {}}

    count_pose_at_rated = 0

    self.valids = []
    self.valid = True
    self.constraints = []
    self.constraints_violated = []

    for ii, pose in enumerate(self.poses):
      self.valids.append(pose.valid)
      if not pose.valid:
        self.valid = False

      for constraint in pose.state['constraints']:
        constraint['position'] = ii
        self.constraints.append(constraint)
      for constraint_v in pose.state['constraints_violated']:
        constraint_v['position'] = ii
        self.constraints_violated.append(constraint_v)

      next_index = (ii + 1) % self.num_poses

      # Variables are weighted by time spent at each position.
      # Power must be calculated during optimization runs, so it's pulled out
      # separately.
      temp_p += (
          (pose.state['power'] + self.poses[next_index].state['power']) / 2.
          * pose.state['segment_time'])

      # If not an optimization run, store values for all number-like state
      # variables weighted by both time and distance. These are used in the stat
      # calculations below.
      if not self.opt:
        # TODO: Redo section for readability and conciseness.
        for key in pose.state.keys():
          if isinstance(pose.state[key], numbers.Number):
            # Temp_vals is weighted sum of particular variable, weighted by
            # by either time or distance. It is later divided by total
            # respective weight to get a weighted average.
            if key not in temp_vals['dist']:
              temp_vals['dist'][key] = 0.
              temp_vals['time'][key] = 0.
            temp_vals['time'][key] += (
                (pose.state[key] + self.poses[next_index].state[key]) / 2.
                 * pose.state['segment_time'])
            temp_vals['dist'][key] += (
                (pose.state[key] + self.poses[next_index].state[key]) / 2.
                 * pose.state['segment_length'])
          elif isinstance(pose.state[key], (list, np.ndarray)):
            if np.array(pose.state[key]).size==3 and all(
                [isinstance(v, numbers.Number) for v in pose.state[key]]):
              for idx, suffix in enumerate(['-x', '-y', '-z']):
                key_suff = key + suffix
                if key_suff not in temp_vals['dist']:
                  temp_vals['dist'][key_suff] = 0.
                  temp_vals['time'][key_suff] = 0.
                temp_vals['time'][key_suff] += (
                    (pose.state[key][idx]
                      + self.poses[next_index].state[key][idx]) / 2.
                     * pose.state['segment_time'])
                temp_vals['dist'][key_suff] += (
                    (pose.state[key][idx]
                      + self.poses[next_index].state[key][idx]) / 2.
                     * pose.state['segment_length'])
          # TODO: Extend to work for forces as well.
          elif key in ['moments_b']:
            # Moments and forces are nested in a dict, so we pull them out for
            # summary stats.
            for kk in pose.state[key]['type'].keys():
              # Calculate the magnitude.
              key_kk = key + '-' + kk + '-mag'
              if key_kk not in temp_vals['dist']:
                temp_vals['dist'][key_kk] = 0.
                temp_vals['time'][key_kk] = 0.
              curr_vec_len = utils.vec_length(pose.state[key]['type'][kk])
              next_vec_len = utils.vec_length(
                  self.poses[next_index].state[key]['type'][kk])
              temp_vals['time'][key_kk] += (
                  (curr_vec_len + next_vec_len) / 2.
                   * pose.state['segment_time'])
              temp_vals['dist'][key_kk] += (
                  (curr_vec_len + next_vec_len) / 2.
                   * pose.state['segment_length'])
              # Calculate the components and add suffix to name.
              for idx, suffix in enumerate(['-x', '-y', '-z']):
                key_kk_suff = key + '-' + kk + suffix
                if key_kk_suff not in temp_vals['dist']:
                  temp_vals['dist'][key_kk_suff] = 0.
                  temp_vals['time'][key_kk_suff] = 0.
                temp_vals['time'][key_kk_suff] += (
                    (pose.state[key]['type'][kk][idx]
                      + self.poses[next_index].state[key]['type'][kk][idx]) / 2.
                     * pose.state['segment_time'])
                temp_vals['dist'][key_kk_suff] += (
                    (pose.state[key]['type'][kk][idx]
                      + self.poses[next_index].state[key]['type'][kk][idx]) / 2.
                     * pose.state['segment_length'])

      pose.state['time_norm'] = (
          pose.state['time'] / self.data_loop['total_time'])

      if pose.state['power_shaft'] >= self.config['power_shaft_max']:
        count_pose_at_rated += 1

    self.r_at_rated = count_pose_at_rated / float(self.num_poses)

    # Calculate stats for all possible values. Omitted on optimization runs
    # for computation time.
    if not self.opt:
      stats = {}
      for key, value in temp_vals.items():
        # Key is either distance or time.
        for k in value.keys():
          # Special items denoted with hyphens above - we need to parse the
          # string to figure out what to do.
          if k.find('-') != -1:
            k_split = k.split('-')
            # Non-nested vectors have one hyphen, where suffix indicates axis.
            if len(k_split) == 2:
              # Separate out the suffix so we can lookup in pose.state with kk.
              kk, suffix = k_split[0], k_split[1]
              idx = {'x': 0, 'y': 1, 'z': 2}[suffix]
              stats[k + '_avg_' + key] = (
                  value[k] / self.data_loop['total_' + key])
              stats[k + '_max'] = np.max(
                  [pose.state[kk][idx] for pose in self.poses])
              stats[k + '_min'] = np.min(
                  [pose.state[kk][idx] for pose in self.poses])
            # Force and moment vectors are nested and have an additional lookup
            # to do. Assume that's what they are, and are nested under 'type'.
            elif len(k_split) == 3:
              kk, kkk, suffix = k_split[0], k_split[1], k_split[2]

              stats[k + '_avg_' + key] = (
                  value[k] / self.data_loop['total_' + key])
              if suffix == 'mag':
                magnitudes = [
                    utils.vec_length(pose.state[kk]['type'][kkk])
                    for pose in self.poses]
                stats[k + '_max'] = np.max(magnitudes)
                stats[k + '_min'] = np.min(magnitudes)
              else:
                idx = {'x': 0, 'y': 1, 'z': 2}[suffix]
                stats[k + '_max'] = np.max(
                    [pose.state[kk]['type'][kkk][idx] for pose in self.poses])
                stats[k + '_min'] = np.min(
                    [pose.state[kk]['type'][kkk][idx] for pose in self.poses])

          else:
            stats[k + '_avg_' + key] = value[k] / self.data_loop['total_' + key]
            stats[k + '_max'] = np.max(
                [pose.state[k] for pose in self.poses
                 if pose.state[k] != float('inf')
                 or pose.state[k] != float('-inf')])
            stats[k + '_min'] = np.min(
                [pose.state[k] for pose in self.poses
                 if pose.state[k] != float('inf')
                 or pose.state[k] != float('-inf')])

    # Power is required on optimization runs, so it is calculated every run.
    self.power = temp_p / self.data_loop['total_time']

    if self.verbose:
      print(utils.TextFormat.BOLD + 'Loop Valid:', self.valid,
            utils.TextFormat.END)
      if not self.valid:
        print('Type of constraints violated:\n'
              + str(list(set([c['name'] for c in self.constraints_violated]))))
      print(utils.TextFormat.BOLD +
            'Loop Mean Power is: %0.1f W' % self.power + utils.TextFormat.END)


    p_in_wind = (0.5 * self.resource['rho'] * self.v_w_at_h_hub**3)
    if p_in_wind != 0.:
      self.zeta_padmount = self.power / (p_in_wind * self.config['s'])
    else:
      self.zeta_padmount = float('-inf')

    if not self.opt:
      self.data_loop['power'] = self.power
      self.data_loop['r_at_rated'] = self.r_at_rated
      self.data_loop['valid'] = self.valid
      self.data_loop.update(stats)
      self.data_loop['v_w_at_h_ref'] = self.v_w_at_h_ref
      self.data_loop['v_w_at_h_hub'] = self.v_w_at_h_hub
      self.data_loop.update(self.path.__dict__)

  def calc_csim_v_a_sch(self):
    """
    Calculates a best fit v_a schedule and params in CSim notation.

    Results are added to loop object.
    """
    v_as = np.array(self.data_poses['v_a'].tolist())
    angles = np.array(self.data_poses['loop_angle_csim'].tolist())

    def toCSim(angle, a, b, c, phi):
      return [max(a,x) for x in b + c * np.cos(angle - phi)]

    init_guess = [self.config['v_a_min'], self.config['v_a_min']*1.5,
                  self.config['v_a_min']*0.5, 1.5]

    popt, pcov= curve_fit(toCSim, angles, v_as, init_guess)

    self.v_a_sch_params = {'a': popt[0],
                           'b': popt[1],
                           'c': popt[2],
                           'phi': popt[3]}
    self.v_a_sch = toCSim(angles, *popt)

  def _get_seeds(self, opt_vars):
    raise NotImplemented
    #TODO: finish section, revise for new parameterization
    seeds = {}

    if 'r_loop' or 'incl' in opt_vars:
      if 'r_loop' in opt_vars and 'incl' not in opt_vars:
        seeds['r_loop'] = self._get_r_loop_analytical()
      if 'r_loop' and 'incl' in opt_vars:
        pass

    if 'v_ks' in opt_vars:
      seeds['v_ks'] = []

      v_a_min = self.config['v_a_min']
      v_a_tension = math.sqrt((2.*self.config['tension_max'])
                              /(self.resource['rho'] * self.config['s']
                                * self.config['cL_oper']))
      cD = self.config['cD_from_cL'](self.config['cL_oper'])
      loyd_limit = (4./27.) * (self.config['cL_oper']**3 / cD**2)

      v_k_ideal = (
        (2./3.) * (self.config['cL_oper'] / cD) * self.v_w_at_h_hub
        * math.cos(self.path.incl))

      # TODO: Potential energy is already calculated in the pose.
      # Remove repeated calculations.
      m_pot = (self.config['m_kite'] + 0.5 * self.config['m_tether'])
      h_delta = (self.path.h_max - self.path.h_min)
      pot_e_delta = h_delta * m_pot * utils.Const.G
      k_grav = 0.5
      v_k_delta = math.sqrt(2. * pot_e_delta / m_pot)

      for pose in self.poses:
        h_norm = ((pose.state['xyz'][k] - self.path.h_min) / h_delta) - 0.5

        seeds['v_ks'].append(
            max(v_a_min,
                min(v_a_tension, v_k_ideal - h_norm * v_k_delta * k_grav)))

      return seeds

  def _get_r_loop_analytical(self):

    r_loop_max = self.config['l_tether']

    # Calculate ideal loop size using analytical model.
    # TODO: Document source for this model.
    r_loop_ideal = math.sqrt(
        (2.*self.config['l_tether'] * self.config['m_eff'])/
        (self.resource['rho'] * self.config['cL_oper'] * self.config['s']))

    if 'h_min' in self.config:
      h = (math.sin(self.path.incl) * self.config['l_tether']
           - self.config['h_min'])
      r_loop_max = h / math.cos(self.path.incl)

    r_loop = min(r_loop_max, r_loop_ideal)

    return r_loop

  def _get_incl_analytical(self):

    incl_min = None

    shear = max(0., self.resource['shear'])
    incl_ideal = math.atan(math.sqrt(shear))

    if 'h_min' in self.config:
      incl_min = (math.asin(self.config['h_min'] / self.config['l_tether'])
                  + math.asin(self.path.r_loop / self.config['l_tether']))

    incl = max(incl_min, incl_ideal)

    return incl

  def _get_r_loop_incl_analytical(self):
    raise NotImplemented

  def _get_v_k_schedule_analytical(self):
    raise NotImplemented

  def optimize(self, vars_to_opt,
               maxiter, tol, catol,
               constraint_penalty_norm, constraint_stiffness,
               tension_penalty_cut_in, tension_penalty_norm, flap_m_mult,
               flap_penalty_factor, store_converge_data):
    """
    Optimizes parameters specified in vars_to_opt for best average loop power,
    meeting constraints in KitePose.
    """

    if self.verbose:
      print('Optimizing loop...')
      start_t = time.time()

    constraint_penalty_mult = (
        constraint_penalty_norm * self.config['power_shaft_max'])

    class ConstraintsWrapper(object):
      """Defines a wrapper class for constraints.

      The sole reason for the existance of this class is to provide a fixed
      function for the optimizer to 'call.' This wrapper lets the call simply
      access a variable that we swap out with the result of each run."""

      def __init__(self, constraints):
        self.constraints = []
        for constraint in constraints:
          self.constraints.append(ConstraintWrapper(constraint))
      def get_list(self):
        # Returns a list of constraint dicts, rather than the list of
        # ConstraintWrapper objects that is self.constraints.
        out = tuple(c.constraint for c in self.constraints)
        return out

    class ConstraintWrapper(object):
      def __init__(self, constraint):
        self.constraint = {}
        # 'type' and 'fun' required for COBYLA optimizer.
        self.constraint['type'] = 'ineq'
        # 'fun' is the function call required by COBYLA.
        # Note that here we just pin it to a function that gets a variable that
        # is repeatedly swapped out. This is because the optimizer calls a
        # specific memory address function - the function itself cannot be
        # updated.
        self.constraint['fun'] = self.get_constraint_val
        self.update_constraint(constraint)
      def get_constraint_val(self, x):
        return self.constraint['opt']
      def update_constraint(self, constraint):
        self.constraint.update(constraint)
        self.constraint['opt'] = (
          self.constraint['margin_norm'] * constraint_stiffness)

    if store_converge_data:
      self.convergence_data = []

    self._vars_to_opt_param_dict = (
        self._setup_param_args(vars_to_opt, norm=True))
    self._update_loop_inputs(self._vars_to_opt_param_dict)

    if self.verbose:
      print('Variables being optimized: ', list(vars_to_opt.keys()))
      print('Initial seed is: ')
      param_vals_rounded = {}
      for k, vals in (
          list(self._get_param_values(self._vars_to_opt_param_dict).items())):
        param_vals_rounded[k] = np.round(vals, 5)
      pprint.pprint(param_vals_rounded)
      print('Parameterization types are: ')
      for items in [(k, v['param_type']) for k, v in vars_to_opt.items()]:
        print('%s: %s'%items)

    # Constraints are created at the KitePose level, so we don't know what
    # constraints the current model has until it is run. Run the model once to
    # get the current active constraints.
    loop = KiteLoop(
             self.resource, self.config, self.v_w_at_h_ref,
             self.v_w_hat, self.grav_mult,
             opt=True, pose_states=self.pose_states,
             path_shape_params=self.path_shape_params,
             path_location_params=self.path_location_params,
             opt_params=self.opt_params,
             solver=self.solver)

    loop.solve()
    cons = ConstraintsWrapper(loop.constraints)

    self.iter_count = 0
    if self.verbose:
      print('Max iter: %d, calculating iter %d..'%(maxiter, self.iter_count),
            end='')

    # Define objective function for optimizer.
    def eval_loop_power(norm_args):
      # Update parameterized args via optimizer provided normalized args.
      self._update_param_args_w_norm_args(
          self._vars_to_opt_param_dict, norm_args)
      self._update_loop_inputs(self._vars_to_opt_param_dict)

      loop = KiteLoop(
        self.resource, self.config, self.v_w_at_h_ref,
        self.v_w_hat, self.grav_mult,
        opt=True, pose_states=self.pose_states,
        path_shape_params=self.path_shape_params,
        path_location_params=self.path_location_params,
        opt_params=self.opt_params,
        solver=self.solver)
      loop.solve()

      # Ramp in tension penalty if ratio of poses at rated power is above
      # tension penalty cut in.
      tension_mult = 0.
      if loop.r_at_rated >= tension_penalty_cut_in:
        ramp = interp1d([tension_penalty_cut_in, 1.], [0., 1.])
        ramp_mult = ramp(loop.r_at_rated)
        tension_mult = (tension_penalty_norm
                        * self.config['power_shaft_max']
                        * ramp_mult)

      for pose in loop.poses:
        pose._apply_opt_penalties(
            constraint_penalty_mult, tension_mult, flap_m_mult,
            flap_penalty_factor)

      for con, constraint in zip(cons.constraints, loop.constraints):
        con.update_constraint(constraint)

      # Applying optimization penalties changes the power, so we must update
      # the summary power again by running calc_loop_data.
      loop._calc_loop_data()

      # Record intermediary optimization inputs to see how it converged.
      if store_converge_data:
        self.convergence_data.append(
            {'obj_out': loop.power, # Not necessarily power due to penalties.
             'params': utils.deepcopy_lite(
                  self._get_param_values(self._vars_to_opt_param_dict)),
             # Store a reduced set of constraint info for memory management.
             'constraints': tuple(
              {'name': c['name'],
               'margin_norm': c['margin_norm'],
               'value': c['value']} for c in cons.get_list())})

      if self.verbose:
        self.iter_count += 1
        if self.iter_count % 100 == 0:
          print('%d..'%self.iter_count, end='')
      return -loop.power

    self.optimizer_output = minimize(
        eval_loop_power,
        self._get_norm_param_args(self._vars_to_opt_param_dict),
        method='cobyla',
        constraints=cons.get_list(),
        options={'tol':tol,
                 'maxiter':maxiter,
                 'catol': catol * constraint_stiffness,
                 'rhobeg':1.})

    # Pose states and path params are updated on every call the
    # optimizer makes, but we overwrite them with the optimizer output,
    # which is in the normalized domain.
    # We do this because in the case where the optimizer fails,
    # the final iteration may not be the best run, and the optimizer
    # returns the best run.
    self._update_param_args_w_norm_args(
        self._vars_to_opt_param_dict, self.optimizer_output['x'])
    self._update_loop_inputs(self._vars_to_opt_param_dict)
    self._initialize_poses()

    # Store vars_to_opt final output in pose_state_param so user has them.
    self.pose_states_param.update(
        self._get_param_dict(self._vars_to_opt_param_dict))

    if self.verbose:
      end_t = time.time()
      print('')
      print('Optimization complete in %d iterations and %0.2fs.'
            % (self.optimizer_output['nfev'],
               (end_t - start_t)) + utils.TextFormat.BOLD)
      if self.optimizer_output['success']:
        print('Converged to solution.')
      else:
        if ((self.pose_solve_options['contol_norm'] * constraint_stiffness
             - self.optimizer_output['maxcv']) >= 0.):
          print('Converged to solution within pose constraints tolerance.')
          print('Ignore constraint violations, within constraint tolerance '
                'limit of %0.3f' % (self.pose_solve_options['contol_norm']))
        else:
          print('Did NOT converge to solution within constraints '
                'tolerance of', catol)
          print('Max violation is',
                self.optimizer_output['maxcv'] / constraint_stiffness)
          print('Constraints violated: ')
          out = {}
          for con in cons.constraints:
            if con.constraint['opt'] < -catol * constraint_stiffness:
              name = (
                  con.constraint['name'] + ' norm @ pos {:02d}'.format(
                    con.constraint['position']))
              out[name] = round(con.constraint['margin_norm'], 4)
          pprint.pprint(out)
      print(utils.TextFormat.END
            + 'Results for optimization in normalized domain: ')
      print([round(x,3) for x in self.optimizer_output['x']])
      print('Results for optimization in parameter domain: ')
      pprint.pprint(self._get_param_values(self._vars_to_opt_param_dict))
      print('Results in pose domain: ')
      pprint.pprint(self.pose_states)
      print('Results in path domain: ')
      pprint.pprint(self.path_shape_params)
      pprint.pprint(self.path_location_params)

  @staticmethod
  def _get_norm_param_args(param_args):
    norm_args = []
    for key in sorted(param_args.keys()):
      norm_args.extend(param_args[key].get_norm_values())
    return np.array(norm_args)

  @staticmethod
  def _update_param_args_w_norm_args(param_args, norm_args):

    idx_start = 0

    for key in sorted(param_args.keys()):
      idx_end = idx_start + param_args[key].num_args
      param_args[key].set_norm_values(norm_args[idx_start:idx_end])

      idx_start = idx_end

  def _setup_param_args(self, param_dict, norm=False):
    """Initializes all parameterized args."""

    param_args = {}

    for key, args in param_dict.items():
      output_xs = None
      kwargs = {}
      kwargs['param_type'] = args.get('param_type', 'value')

      # All loop lookups are currently assumed to be in the "pose domain."
      # Assuming circular paths and even spaced poses, this can be easily
      # mapped, but the assumption breaks down with arbitrary paths and spacing.

      # TODO: Add functionality to define things that vary around the
      # path in something other than pose space, such as loop angle or similar.
      if kwargs['param_type'] == 'spline':
        num_ctrl_pts = len(args['values'])
        end_offset = (self.num_poses) / (2. * num_ctrl_pts)
        kwargs['ctrl_pts_xs'] = np.linspace(end_offset,
                                            self.num_poses - end_offset,
                                            num=num_ctrl_pts)
        output_xs = list(range(self.num_poses))
      elif kwargs['param_type'] == 'linear_interp':
        num_ctrl_pts = len(args['values'])
        kwargs['ctrl_pts_xs'] = (
            np.linspace(0., self.num_poses, num_ctrl_pts))
        output_xs = list(range(self.num_poses))
      elif kwargs['param_type'] == 'constant':
        output_xs = list(range(self.num_poses))

      if norm:
        param_args[key] = utils.NormParamArg(
            args['values'], args['max_step'], **kwargs)
      else:
        param_args[key] = utils.ParamArg(args['values'], **kwargs)

      # If output_xs was defined above based on parameterization type, then set
      # it in the param_arg.
      if output_xs is not None:
        param_args[key].set_lookup_xs(output_xs)

    return param_args

  @staticmethod
  def _get_param_values(param_args):
    """Returns a dictionary of all output values of parameterized args."""

    output = {}

    for key in sorted(param_args.keys()):
      output[key] = param_args[key].get_param_values()

    return output

  @staticmethod
  def _get_param_dict(param_args):
    """Returns an input dictionary format for a dict of ParamArg objects."""

    out = {}
    for k, param_arg in param_args.items():
      out[k] = {
          'param_type': param_arg.param_type,
          'values': param_arg.get_param_values()}
    return out

  def _update_loop_inputs(self, param_args_dict):

    for key, param_arg in param_args_dict.items():
      self.pose_states[key] = param_arg.get_values()

    path_location_params = self.path_location_params
    for k in path_location_params.keys():
      if k in self.pose_states:
        path_location_params[k] = self.pose_states.pop(k)

    path_shape_params = self.path_shape_params
    for k in path_shape_params.keys():
      if k in self.pose_states:
        path_shape_params[k] = self.pose_states.pop(k)

  def plot_power_components(self, x='dist_norm', **kwargs):
    """
    Plots components of total power measured along aerodynamic axis.
    """
    figsize = kwargs.get('figsize', (9., 7.))

    keys = ['aero_power', 'tether_power', 'gravity_power', 'accel_power',
            'rotor_thrust_power']

    plt.figure(figsize=figsize)
    plt.title('Power Components vs %s @ %0.1fm/s' % (x, self.v_w_at_h_ref))
    plt.ylabel('Power [kW]')
    plt.xlabel(x)
    for k in keys:
        plt.plot(self.data_poses[x], self.data_poses[k]/1000., label=k)
    plt.grid(linestyle=':', color='gray', linewidth=0.5)
    plt.legend()

  def plot_moments(self, x='dist_norm'):
    """Plots components of moments in body frame."""
    for axis, ii in zip(['x', 'y', 'z'], [0, 1, 2]):
      plt.figure()
      for k in ['inertial', 'rotors', 'aero', 'gravity', 'tether', 'residual']:
        plt.plot(
            self.data_poses[x],
            [p.state['moments_b']['type'][k][ii] for p in self.poses], label=k)
      plt.ylabel('Nm of moment')
      plt.title('Moments about %s axis'%axis)
      plt.tight_layout()
      plt.legend()
      plt.grid(linewidth=0.5, linestyle=':')

  def plot_flap_coeff(self, x='dist_norm'):
    """Plots flap aero coefficients required to meet moment balance."""
    plt.figure()
    plt.plot(self.data_poses[x],
             [p.state['flap_aero_coeffs'] for p in self.poses])
    plt.ylabel('Flap Aero Moment Coeff')
    plt.xlabel('Normalized Distance around Loop')
    plt.tight_layout()
    plt.legend(['cl', 'cm', 'cn'])
    plt.grid(linewidth=0.5, linestyle=':')

  def plot_vectors(self, ys=['pqr', 'omega_hat'], ys_components=[['p','q','r']],
      x='dist_norm', plot_kwargs={}, **kwargs):
    """Method for plotting 3-component vectors in the pose states.

    Pass a list of corresponding component label lists. If empty or None,
    ['x','y','z'] is used for all. If length of 1, that entry is used for
    all."""

    figsize = kwargs.get('figsize', (9, 6))
    fig = kwargs.get('fig', None)
    base_label = kwargs.get('base_label', '')
    label = kwargs.get('label','v_w_at_h_hub')
    if not fig:
      fig, axes = plt.subplots(figsize=(figsize[0],figsize[1] * len(ys)),
                               nrows=len(ys), ncols=1, dpi=100)
    if ys_components is None or len(ys_components)==0:
      ys_components = [['x', 'y', 'z']] * len(ys)
    elif len(ys_components)==1:
      ys_components = ys_components * len(ys)
    assert len(ys_components)==len(ys), 'Cannot map list of ys_components to ys.'

    for jj, (ax, y, y_components) in enumerate(
        zip(fig.get_axes(), ys, ys_components)):
      if len(y_components)==0:
        y_components = ['x', 'y', 'z']
      assert len(y_components) == 3, (
          'This method only works for 3-component vectors.')
      for ii, component in enumerate(y_components):
        if type(y) is str:
          data = [p.state[y][ii] for p in self.poses]
        elif hasattr(y, '__iter__') and not isinstance(y, str):
          # Extract data that's nested a few levels
          data = [fun.nested_dict_traverse(y, p.state)[ii] for p in self.poses]
        else:
          assert(False), 'Failed to parse y: {}'.format(y)
        ax.plot(self.data_poses[x], data,
                label=component+base_label, **plot_kwargs)
        ax.set_title('{} at {} of {} m/s'.format(y, label, self[label]))
        ax.legend(bbox_to_anchor=(1.02, 1.0), loc=2, prop={'size':8})
        ax.grid(linewidth=0.5, linestyle=':')

    return fig

  def plot_vectors_components(self, ys=['pqr', 'omega_hat'],
      y_components=['p','q','r'], ys_labels=None, x='dist_norm',
      plot_kwargs={}, **kwargs):
    """Method for plotting 3-component vectors in the pose states.

    All ys in the list will have their components plotted together;
    one subplots for each component."""

    figsize = kwargs.get('figsize', (9, 6 * len(y_components)))
    fig = kwargs.get('fig', None)
    base_label = kwargs.get('base_label', '')
    label = kwargs.get('label','v_w_at_h_hub')
    if not fig:
      fig, axes = plt.subplots(
          figsize=figsize, ncols=1, nrows=len(y_components), dpi=100)
    if y_components is None or len(y_components)==0:
      y_components = ['x', 'y', 'z']

    for jj, (ax, y_component) in enumerate(zip(fig.get_axes(), y_components)):
      for ii, y in enumerate(ys):
        if type(y) is str:
          data = [p.state[y][jj] for p in self.poses]
        elif hasattr(y, '__iter__') and not isinstance(y, str):
          # Extract data that's nested a few levels
          data = [fun.nested_dict_traverse(y, p.state)[jj] for p in self.poses]
        else:
          assert(False), 'Failed to parse y: {}'.format(y)
        if ys_labels is not None:
          y_label = '{}{}'.format(ys_labels[ii], base_label)
        else:
          y_label = '{}{}'.format(y, base_label)
        ax.plot(self.data_poses[x], data,
                label=y_label, **plot_kwargs)
        ax.set_title(
            '{} components at {} of {} m/s'.format(
                y_component, label, self[label]))
        ax.legend(bbox_to_anchor=(1.02, 1.0), loc=2, prop={'size':8})
        ax.grid(linewidth=0.5, linestyle=':')

    return fig

  def _get_3d_plot_scale(self, poses):
    xs = [pose.state['xyz'][i] for pose in poses]
    ys = [pose.state['xyz'][j] for pose in poses]
    zs = [pose.state['xyz'][k] for pose in poses]

    x_min = min(xs)
    y_min = min(ys)
    z_min = min(zs)

    x_max = max(xs)
    y_max = max(ys)
    z_max = max(zs)

    # Finds the biggest dimension of positions in dataset, and sets range 1.1x
    # larger than that.
    scale = (
        1.1 * max(
            [x_max - x_min,
             y_max - y_min,
             z_max - z_min,
             x_max - self.path.gs_position[i],
             y_max - self.path.gs_position[j]])
        / 2.)
    x_mid = (x_max + x_min) / 2.
    y_mid = (y_max + y_min) / 2.
    z_mid = (z_max + z_min) / 2.

    x_range = [x_mid - scale, x_mid - scale, x_mid + scale]
    y_range = [y_mid - scale, y_mid + scale, y_mid + scale]
    z_range = [z_mid - scale, z_mid - scale, z_mid + scale]

    return x_range, y_range, z_range

  def plot_path_positions(self, **kwargs):
    fig = kwargs.get('fig', None)
    figsize = kwargs.get('figsize', (7, 7))
    label = kwargs.get('label', '{} m/s'.format(self.v_w_at_h_ref))
    path_color = kwargs.get('path_color', 'black')
    pose_colormap = kwargs.get('pose_colormap', 'rainbow')
    with_azim_incl = kwargs.get('with_azim_incl', False)

    if not fig:
      fig, ax = plt.subplots(figsize=figsize)
    ax = fig.get_axes()[0]
    # Axis not provided for cycler, as colors are set by path color.
    # Colors only needed for markers for poses.
    colors = utils.set_rainbow_plot_cycler(n=self.num_poses, cmap=pose_colormap)

    xs = self.path['crosswind_xy'].T[0]
    ys = self.path['crosswind_xy'].T[1]
    if with_azim_incl:
      # Simple Mercator projection of centroid locations,
      # will distort centroid locations to show them further from each other
      # than they really are.
      xs = xs - self.path_location_params['azim'] * self.config['l_tether']
      ys = ys + self.path_location_params['incl'] * self.config['l_tether']
      ax.set_title('Crosswind Plane Path & Poses,'
          ' with approximate azim and incl.')
      ax.axvline(linewidth=1, label=None)
      ax.axhline(linewidth=1, label=None)
    else:
      ax.set_title('Crosswind Plane Path & Poses')
      ax.scatter(0, 0, marker='+', c='k', s=250, label=None)

    ax.plot(xs, ys, c=path_color, linestyle='--', label=label, zorder=1)
    ax.quiver(xs[-1], ys[-1], xs[0]-xs[-1], ys[0]-ys[-1],
          angles='xy', scale_units='xy', scale=1.,
          linestyle='--', label=None, zorder=1, color=path_color)
    ax.scatter(xs, ys, s=75, c=colors, zorder=2)
    ax.grid(linewidth=1, linestyle=':')
    ax.axis('equal')
    if with_azim_incl:
      ax.set_ylim((-10, self.config['l_tether'] * np.pi / 3))

    return fig

  def plot_rotor_map(self, **kwargs):
    """Map out the loop on the rotor table plots."""
    fig = kwargs.get('fig', None)
    label = kwargs.get('label', '{} m/s'.format(self.v_w_at_h_ref))
    path_color = kwargs.get('path_color', 'gray')
    path_width = kwargs.get('path_width', 2)
    pose_colormap = kwargs.get('pose_colormap', 'rainbow')
    pose_markersize = kwargs.get('pose_markersize', 75)
    x = kwargs.get('x', 'omega_rotor')
    y = kwargs.get('y', 'v_a_along_rotor_axis')
    zs = kwargs.get('zs', ['c_t', 'c_p', 'eta_rotors'])

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
          **kwargs)

    colors = utils.set_rainbow_plot_cycler(n=self.num_poses, cmap=pose_colormap)
    xs = [p.state[x] for p in self.poses]
    ys = [p.state[y] for p in self.poses]
    for ax in fig.get_axes():
      ax.plot(
          xs, ys, c=path_color, linestyle='-.', label=label, zorder=2,
          linewidth=path_width)
      ax.quiver(xs[-1], ys[-1], xs[0]-xs[-1], ys[0]-ys[-1],
            angles='xy', scale_units='xy', scale=1., color=path_color,
            linestyle='--', label=None, zorder=2)
      ax.scatter(xs, ys, s=pose_markersize, c=colors, zorder=3)

    return fig


  def plot_convergence_data(self, **kwargs):
    """Plots optimization convergence data.

    Only available if loop was optimized.

    All variables being optimized and constraints are plotted.
    Constraints are violated when margins are negative."""

    # TODO: Add additional plotting kwargs.
    figsize = kwargs.get('figsize', (7,5))

    out = {}
    for ii, d in enumerate(self.convergence_data):
        for k, v in d['params'].items():
            if k not in out:
                out[k] = []
            out[k].append(v)
        constraints = {}
        for c in d['constraints']:
            if c['name'] not in constraints:
                constraints[c['name']] = []
            constraints[c['name']].append(c['margin_norm'])
        for k, v in constraints.items():
            if k not in out:
                out[k] = []
            out[k].append(constraints[k])


    # Plot crosswind path to show color scheme for constraints.
    fig = self.plot_path_positions()
    fig.suptitle('Color Key of Poses for Constraints')

    for k, v in out.items():
      plt.figure(figsize=figsize)
      ps = plt.plot(v)
      utils.set_rainbow_plot_cycler(n=len(v[0]), ax=plt.gca())
      plt.title(k)
      plt.xlabel('Iteration number')
      plt.legend(ps, [str(ii) for ii in range(len(ps))],
                 bbox_to_anchor=(1.02, 1.0), loc=2)
      plt.grid(linewidth=0.5, linestyle=':')
      plt.tight_layout()

  def gen_loop_vec_plot_file(
      self, location='plots/plot_loop_forces.json', var='force_types',
      no_show=[], nth_point=1, slicr=None, animate=False):
    """
    Takes all selected vector data in the loop and writes them out to a
    json that is formatted for use in Plotly javascript library.

    Kwargs:
      location: String of file location where data is stored.
      var: Specifies what vectors to plot, either 'all', 'force_types',
        'speeds', or 'path_def'.
      no_show: A list of strings of things not to show. Only thing currently
        active is the (...) scaling object.
      nth_point: Only plots every nth point. Used to reduce large datasets to
        be manageable.
      slicr: Tuple of start and stop positions, normalized from 0:1 of data
        to plot.
      animate: Writes out each pose as a frame, which enables plotter to
        animate poses.

    Colors are done from the 5th to the 95th percentile to avoid outliers from
    throwing it all off.
    """

    # Makes a file of the poses data, to be used for a single plot.
    plot_dict = {}

    # Maps the selected variable to the KiteLoop method that returns that
    # variable.
    methods = {'all': 'gen_all_plot_data',
               # TODO: Force components is currently broken. Fix.
               #'force_components': 'gen_force_plot_data',
               'force_types': 'gen_force_type_data',
               'speeds': 'gen_speeds_plot_data',
               'path_def': 'gen_path_def_plot_data'}

    if slicr is not None:
      start = int(slicr[0] * len(self.poses))
      end = int(slicr[1] * len(self.poses))
      poses = self.poses[start:end]
    else:
      if animate:
        poses = self.poses + [self.poses[0]]
      else:
        poses = self.poses

    if animate:
      frames = []
      times = []

    x_range, y_range, z_range = self._get_3d_plot_scale(poses)

    for ii, pose in enumerate(poses):
      if ii % nth_point == 0:
        if '(...)' not in no_show and '(...)' not in plot_dict:
          plot_dict.update({'(...)': {'name':'(...)',
                                      'type': 'scatter3d',
                                      'mode': 'lines',
                                      'x': x_range,
                                      'y': y_range,
                                      'z': z_range,
                                      'text': ['bounding box for scaling'],
                                      'line':{'width': 0.001}
                                      }})
        pose_plots = getattr(pose, methods.get(var))(no_show=(no_show + ['(...)']))
        for plot in pose_plots:
          plot['text'] = [(p+'\r\nPosition: %d \r\nV_a: %0.1f \r\nV_k: %0.1f \r\nThrust Power: %0.1f'
              % (ii, pose.state['v_a'], pose.state['v_k'], pose.state['power_thrust'])) for p in plot['text']]
          key = plot['name']
          if key in plot_dict:
            plot_dict[key]['x'].extend([None]+ plot['x'])
            plot_dict[key]['y'].extend([None] + plot['y'])
            plot_dict[key]['z'].extend([None] + plot['z'])
            plot_dict[key]['text'].extend([None] + plot['text'])
          else:
            plot_dict.update({plot['name']: plot})

        if animate:
          if ii == 0:
            frames.append({'data': list(plot_dict.values()),
                           'name': str(ii)})
          else:
            num_ease = int(pose.state['segment_time'] / (1./20.)) # 20 frames a second

            prev_frame = frames[-1]

            for num in range(num_ease):
              data = []
              for prev_val, cur_val in zip(prev_frame['data'], list(plot_dict.values())):
                vec = {}
                for p in ['x', 'y', 'z']:
                  start = interp1d([0, num_ease], [prev_val[p][0], cur_val[p][0]])
                  end = interp1d([0, num_ease], [prev_val[p][-1], cur_val[p][-1]])
                  vec[p] = [start(num).tolist(), end(num).tolist()]
                data.append(vec)
              times.append(int(pose.state['segment_time']*1000./num_ease))
              frames.append({'data': data})
          plot_dict = {}

    if animate:
      output = {'frames': frames,
                'times': times}
    else:
      output = list(plot_dict.values())

    if location is not None:
      with open(location,'w') as outfile:
        json.dump(output, outfile)
        print('File saved to %s.' % location)
    else:
      return output

  def gen_loop_positions_plot_file(
        self, location='plots/plot_loop_positions.json', label=None, no_show=[],
        var_to_color='power_shaft', path_options=None):
    """
    Takes all positions in the loop and writes them out to a json that is
    formatted for use in Plotly javascript library.

    Args:
      location: File location where data is stored.
      no_show: A list of strings of things not to show. Only thing currently
        active is the (...) scaling object.
      var_to_color: Searches the pose.state dictionary for the variable given
        and colors the marker and the point according to the value.

    Colors are done from the 5th to the 95th percentile to avoid outliers from
    throwing it all off.
    """

    plot_list = []
    var_list = []
    text_list = []
    xs = []
    ys = []
    zs = []

    if label is None:
      label = 'positions'

    if var_to_color is not None:
      for pose in self.poses:
        var = pose.state[var_to_color]
        var_list.append(var)
        text_list.append(str(var))

      prct95 = np.percentile(var_list, 95)
      prct05 = np.percentile(var_list, 5)
      color = var_list
      mode = 'lines+markers'

      marker_options = {'size':4.,
                        'color': color,
                        'cmin':prct05,
                        'cmax':prct95,
                        'colorscale':'Jet',
                        'showscale': True}

      line_options_base = {'width':2.,
                           'color': color,
                           'cmin':prct05,
                           'cmax':prct95,
                           'colorscale':'Jet'}
    else:
      line_options_base = {'width':3.,
                           'color': 'red'}

      marker_options = {}
      mode = 'lines'

    line_options = line_options_base

    if path_options is not None:
      line_options.update(path_options)

    for pose in self.poses:
      x, y, z = pose.position['xyz']
      xs.append(x)
      ys.append(y)
      zs.append(z)

    plot_list.append({ 'name': label,
                       'type': 'scatter3d',
                       'mode': mode,
                       'text': text_list,
                       'x': xs,
                       'y': ys,
                       'z': zs,
                       'marker': marker_options,
                       'line': line_options,
                       })

    x_range, y_range, z_range = self._get_3d_plot_scale(self.poses)

    if '(...)' not in no_show:
      plot_list.append({'name':'(...)',
                        'type': 'scatter3d',
                        'mode': 'lines',
                        'x': x_range,
                        'y': y_range,
                        'z': z_range,
                        'line':{'width': 0.001}
                        })

    with open(location,'w') as outfile:
      json.dump(plot_list,outfile)
      print('File saved to %s.' % location)


def plot_var_sweep(positions, resource, config,
                   var_to_sweep='v_a', var_range=(30.,90.,30.),
                   vars_to_plot=['power', 'tension'],
                   var_scales=None, var_units=None,
                   v_w_at_h_ref=7.0, alpha=5., beta=0., v_a=55.,
                   pqr=[0., 0., 0.], pqr_dot=[0., 0., 0.],
                   accel_along_path=None,
                   k_grav=0.5, grav_mult=1.0,
                   legend_key='loop_angle', every_nth=1,
                   ylims=None, plot_kwargs={},
                   **kwargs):
  """Takes a list of positions for a single kite loop, sweeps a variable for
  each pose, and plots the results.

  Each pose is evaluated completely separately. Accelerations (both rotational
  and translational) are not consistent - ie: the accelerations applied to one
  pose will not get you to the the next pose.

  This is a tool to explore the effect of changing an input and best (but not
  guaranteed) orient the kite to meet the force balance.

  Args:
    positions: A list of position dicts. See KitePose positions attribute for
      example of format.
    resource: A dict that specifies a resource. See KitePowerCurve object for
      details.
    config: A dict that specifies an energy kite. See KitePowerCurve object for
      details.

  Kwargs:
    var_to_sweep: Name of the variable to sweep. Needs to be a valid input to
      KitePose.
    var_range: Sets up range for var_to_sweep.
      Specified as (<start_value>, <end_value>, <num_steps>).
    vars_to_plot: Variable to plot. Each gets its own figure. Must be available
      in KitePose.state dict.
    var_scales: Float that scales the y-axis for each vars_to_plot.
    var_units: String that is appended to y-axis label.
    v_w_at_h_ref: Float of wind speed at reference height.
    alpha: Kite alpha, in deg. Can be a list of values specifying alpha for each
      position.
    beta: Kite beta, in deg. Can be a list of values specifying beta for each
      position.
    v_a: Kite airspeed. Can be a list of values specifying for each position.
    pqr: Array-like vector of rotational body rates. Can be a list of vectors
      specifying pqr for each position.
    pqr_dot: Array-like vector of rotational body accelerations. Can be a list
      of vectors specifying pqr_dot for each position.
    k_grav: Scalar that specifies acceleration along path for each pose.
      Represents how much of gravity acceleration along path is applied to each
      pose. If not None, this overwrites the accel_along_path variable.
    grav_mult: Multiplier applied to gravity.
    legend_key: Variable in pose.state that is used to differentiate poses in
      legend.
    every_nth: Only plot every nth position in positions.
    ylims: List-like of (<y_min>, <y_max>) for each variable in vars_to_plot.
    plot_kwargs: Dict of kwargs to pass to matplotlib plot function.
  """

  # TODO: Add more support for kwargs, such as adding to an existing
  # figure.
  figsize = kwargs.get('figsize', (10,6))

  var_range = np.linspace(var_range[0], var_range[1], var_range[2])
  if var_scales is None:
    var_scales = [1.0] * len(vars_to_plot)
  if var_units is None:
    var_units = [('','')] * len(vars_to_plot)

  if ylims == None:
    ylims = [None] * len(vars_to_plot)

  poses_data = []

  for ii, position in enumerate(positions):
    if ii % every_nth == 0:

      pose_data = {'valid': {var_to_sweep: []},
                   'invalid': {var_to_sweep: []},
                   legend_key: None}
      for key in vars_to_plot:
        pose_data['valid'][key] = []
        pose_data['invalid'][key] = []

      # Setup inputs.
      input_vars = {
        'alpha': alpha,
        'beta': beta,
        'v_a': v_a,
        'accel_along_path': accel_along_path,
        'pqr': pqr,
        'pqr_dot': pqr_dot,
        'v_w_at_h_ref': v_w_at_h_ref}
      if 'aero_device' in config:
        input_vars.update(
          {'aero_device_scale': kwargs.get('aero_device_scale', 0.)})

      # Parse the inputs to determine things that are global or per pose.
      input_dict = {}
      uniques = {}
      for k, v in input_vars.items():

        # If k_grav is provided, it steps on all accel_along_path inputs.
        if k == 'accel_along_path':
          if k_grav is not None:
            v = np.dot(
              np.array([0., 0., -1.]) * utils.Const.G * k_grav,
              position['e_path_tangent'])

        if hasattr(v, '__iter__'):
          if len(v) == len(positions):
            input_dict[k] = v[ii]
            uniques[k] = True
          else:
            uniques[k] = False
            input_dict[k] = v
        else:
          uniques[k] = False
          input_dict[k] = v

      # Cannot specify both v_a and v_k, so if v_k is the variable we sweep,
      # remove v_a from the inputs.
      if var_to_sweep == 'v_k':
        input_dict.pop('v_a')

      # Step through the variable range.
      for v in var_range:
        # Define a function to return the net force residual.
        # This will be used to optimize each pose to best meet a force balance.
        def eval_net(lift_roll_angle):
          input_dict[var_to_sweep] = v
          input_dict['lift_roll_angle'] = lift_roll_angle
          pose = kite_pose.KitePose(
              position, resource, config,
              grav_mult=grav_mult, **input_dict)
          pose.solve()
          for c in pose.state['constraints']:
            if c['name'] == 'net_margin':
              out = c['value']
          return out

        # Optimize to find the lift_roll_angle that best meets the force
        # balance. The bracket specifies the initial steps.
        result = minimize_scalar(
            eval_net, bounds=(-math.pi, math.pi), tol=0.0005,
            bracket=(-0.5, 0.), options={'maxiter': 12})

        # Create the final pose to extract the data out of.
        pose = kite_pose.KitePose(
            position, resource, config,
            grav_mult=grav_mult, **input_dict)
        pose.solve()

        # Save legend key value.
        # Unnecessarily updated every iteration.
        pose_data[legend_key] = pose.state[legend_key]

        if pose.valid:
          pose_data['valid'][var_to_sweep].append(v)
          pose_data['invalid'][var_to_sweep].append(None)
        else:
          pose_data['valid'][var_to_sweep].append(None)
          pose_data['invalid'][var_to_sweep].append(v)

        for iii, key in enumerate(vars_to_plot):
          pose_data['valid'][key].append(pose.state[key] / var_scales[iii])
          pose_data['invalid'][key].append(pose.state[key] / var_scales[iii])
      poses_data.append(pose_data)

  # Plot everything.
  for key, ylim, units in zip(vars_to_plot, ylims, var_units):
    fig = plt.figure(figsize=figsize)
    ax = fig.gca()

    ax.set_title(
        '%s vs %s for Various Positions @ %0.1f m/s'
        % (key, var_to_sweep, v_w_at_h_ref), fontsize=15)
    ax.set_xlabel(var_to_sweep + ' ' + units[0], fontsize=14)
    ax.set_ylabel(key + ' ' + units[1], fontsize=14)
    if ylim is not None:
      ax.set_ylim(ylim)

    max_legend_val = max([p[legend_key] for p in poses_data])
    min_legend_val = min([p[legend_key] for p in poses_data])

    for p in poses_data:
      color = cm.rainbow(
          (p[legend_key] - min_legend_val)/(max_legend_val - min_legend_val))
      ax.plot(p['valid'][var_to_sweep],
               p['valid'][key],
               color=color, label=legend_key + ': %0.2f' % p[legend_key],
               **plot_kwargs)
      ax.plot(p['invalid'][var_to_sweep],
               p['invalid'][key],
               color=color,
               linestyle='--', **plot_kwargs)
    state_rnd = utils.deepcopy_lite(input_dict)
    for k, v in state_rnd.items():
      state_rnd[k] = np.round(v, 2)
      if hasattr(state_rnd[k], 'tolist'):
        state_rnd[k] = state_rnd[k].tolist()
    state_rnd.pop(var_to_sweep)
    state_rnd.pop('accel_along_path')
    state_rnd.pop('lift_roll_angle')

    state_str = ''
    for ii, (k, v) in enumerate(state_rnd.items()):
      if k_grav is not None and k == 'accel_along_path':
        continue
      if uniques[k]:
        v = 'varies'
      if ii % 2 == 0 and ii != 0:
        state_str += '\n'
      state_str += str(k).replace('\'', '') + ':' + str(v) + ' '
    if k_grav is not None:
      state_str += '\nk_grav: %0.2f' % k_grav

    ax.text(1.02, 0.03, state_str
             + '\nInvalid = dashed line.\n'
             + 'Poses individually evaluated.',
             fontsize=10, transform=plt.gca().transAxes, ha='left')
    ax.legend(bbox_to_anchor=(1.01, 1.), loc=2, fontsize=14)
    ax.grid(linestyle=':', linewidth=0.5)
    fig.tight_layout()

  return fig
