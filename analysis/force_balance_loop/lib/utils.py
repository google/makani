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

from __future__ import absolute_import
from __future__ import print_function
import copy
from cycler import cycler
import numpy as np
import math
from matplotlib import cm
from matplotlib import pyplot as plt
from scipy.interpolate import CubicSpline
from six.moves import range
from six.moves import zip


def deepcopy_lite(sth, max_depth=None):
  """
  Copy tool that goes to specified depth in lists, dictionaries,
  or numpy arrays. Unspecified max_depth goes to end.

  Allows you to copy nested items without the very slow copy.deepcopy function.
  """
  def _copy_dict(d, dispatch, max_depth=None, current_depth=1):
    ret = d.copy()
    if max_depth is None or current_depth <= max_depth:
      for key, value in ret.items():
          cp = dispatch.get(type(value))
          if cp is not None:
              ret[key] = cp(value, dispatch, max_depth, current_depth)

    return ret

  def _copy_list(l, dispatch, max_depth=None, current_depth=1):
    ret = copy.copy(l)
    if max_depth is None or current_depth <= max_depth:
      current_depth += 1
      for idx, item in enumerate(ret):
          cp = dispatch.get(type(item))
          if cp is not None:
              ret[idx] = cp(item, dispatch, max_depth, current_depth)
    return ret

  def _copy_nparray(a, dispatch, max_depth=None, current_depth=1):
    ret = a.copy()
    if (max_depth is None or current_depth <= max_depth) and ret.shape != ():
      for idx, item in enumerate(ret):
          cp = dispatch.get(type(item))
          if cp is not None:
              ret[idx] = cp(item, dispatch, max_depth, current_depth)
    return ret

  _copy_dispatch = {dict: _copy_dict,
                    list: _copy_list,
                    np.ndarray: _copy_nparray}

  cp = _copy_dispatch.get(type(sth))
  if cp is None:
      return sth
  else:
      return cp(sth, _copy_dispatch, max_depth)


def axis_angle_rotation(axis, theta, axis_is_unit_vec=True):
  """
  Return the rotation matrix associated with counterclockwise rotation about
  the given axis by theta radians.
  """
  axis = np.asarray(axis)
  if not axis_is_unit_vec:
    axis = axis / vec_length(axis)
  a = math.cos(theta/2.0)
  b, c, d = -axis * math.sin(theta/2.0)
  aa, bb, cc, dd = a**2, b**2, c**2, d**2
  bc, ad, ac, ab, bd, cd = b*c, a*d, a*c, a*b, b*d, c*d
  return np.array([[aa+bb-cc-dd, 2*(bc+ad), 2*(bd-ac)],
                   [2*(bc-ad), aa+cc-bb-dd, 2*(cd+ab)],
                   [2*(bd+ac), 2*(cd-ab), aa+dd-bb-cc]])

def x_rotation(theta):
  """Returns the rotation matrix for a rotation about x by theta radians."""
  c = math.cos(theta)
  s = math.sin(theta)

  return np.array([[1., 0., 0.],
                   [0., c, s],
                   [0., -s, c]])


def y_rotation(theta):
  """Returns the rotation matrix for a rotation about y by theta radians."""
  c = math.cos(theta)
  s = math.sin(theta)

  return np.array([[c, 0., -s],
                   [0., 1., 0.],
                   [s, 0., c]])


def z_rotation(theta):
  """Returns the rotation matrix for a rotation about z by theta radians."""
  c = math.cos(theta)
  s = math.sin(theta)

  return np.array([[c, s, 0.],
                   [-s, c, 0.],
                   [0., 0., 1.]])


def vec_length(vector):
  """
  Returns scalar length of 3 dimensional vector.
  Faster than numpy.linalg.norm
  """
  return math.sqrt(np.dot(vector, vector))

def unit_vec(vector):
  """
  Returns unit vector of 3 dimensional vector.
  """
  l = vec_length(vector)

  if l == 0.:
    return np.array([0., 0., 0.])
  else:
    return vector / l

def component_along_axis(vec, axis, unit_axis=False):
  """
  Returns the component of a vector that is along axis.
  If `axis` is already normalized, set unit_axis to True.
  """
  if not unit_axis:
    axis = unit_vec(axis)
  return axis * np.dot(vec, axis)

def component_perp_axis(vec, axis, unit_axis=False):
  """
  Returns the component of a vector that is perpendicular to axis.
  If `axis` is already normalized, set unit_axis to True.
  """
  return vec - component_along_axis(vec, axis, unit_axis)

def cross(a, b):
  """
  Returns cross product of 3 dimensional vectors.
  Faster than numpy.cross
  """
  c = np.array([a[1]*b[2] - a[2]*b[1],
                a[2]*b[0] - a[0]*b[2],
                a[0]*b[1] - a[1]*b[0]])
  return c

def angle_signed(zero_vec, angle_vec, plane_normal):
  """
  Returns the signed angle from -pi to pi between zero_vec and angle_vec
  projected onto the plane normal.
  """

  plane_normal_unit = unit_vec(plane_normal)

  z_hat_perp = np.dot(zero_vec, plane_normal_unit) * plane_normal_unit
  z_hat_in_plane = zero_vec - z_hat_perp
  z_hat = unit_vec(z_hat_in_plane)

  angle_vec_out_of_plane = np.dot(angle_vec, plane_normal_unit) * plane_normal_unit
  angle_vec_in_plane = angle_vec - angle_vec_out_of_plane
  angle_vec_in_plane_hat = unit_vec(angle_vec_in_plane)

  # rounding to prevent numerical noise from causing an error
  d = round(np.dot(angle_vec_in_plane_hat, z_hat),7)
  angle_us = math.acos(d)

  if np.dot(cross(z_hat, angle_vec_in_plane_hat), plane_normal_unit) > 0:
    angle = angle_us
  else:
    angle = -angle_us

  return angle

def fourier_series_vals(coeffs, num_vals):
  """
  Returns values for a sin and cos fourier series with specified amplitudes.
  Range from 0 to 2*pi.
  """
  ys = []
  for x in np.linspace(0., 2. * math.pi, num=num_vals, endpoint=False):
    y = 0.
    y += coeffs[0]
    harmonic = 1
    for ii, coeff in enumerate(coeffs[1:]):
      if ii % 2 == 0:
        y += coeff * math.sin(harmonic * x)
      else:
        y += coeff * math.cos(harmonic * x)
        harmonic += 1
    ys.append(y)
  return ys

def spline_vals(ctrl_pts_xs, ctrl_pts_ys, output_xs, looped=True, plot=False):
  """
  Returns y values for a spline fit between specified control points at desired
  x coordinates.

  Kwargs:
    looped:
      Use for an x-range that is intended to wrap.
      Assumes the wrap is not complete (ie, ctrl_pts[0] != ctrl_pts[-1]) and
      fills in the overlap.
    plot:
      outputs a plot of the spline and control points
  """

  if looped:
    # CubicSpline periodic requires the first and last points to be the same,
    # so we extend it. In order to do so, we first check that the x spacing is
    # consistent.
    # TODO (ntucker): Make this more general.

    assert np.std(np.diff(ctrl_pts_xs)) < 1e-6, (
      'X spacing for spline control points must be constant to enable '
      'looped spline.')

    xs_extended = np.insert(
        ctrl_pts_xs, 0, ctrl_pts_xs[0] - (ctrl_pts_xs[1] - ctrl_pts_xs[0]))
    ys_extended = np.insert(ctrl_pts_ys, 0, ctrl_pts_ys[-1])
    spline = CubicSpline(xs_extended, ys_extended, bc_type='periodic',
                         extrapolate='periodic')
  else:
    spline = CubicSpline(ctrl_pts_xs, ctrl_pts_ys)

  ys = spline(output_xs)

  if plot:
    plt.figure(figsize=(6,3))
    plt.title('Ctrl Pts and Output Spline')
    plt.plot(ctrl_pts_xs, ctrl_pts_ys, marker='.', label='ctrl_pts')
    plt.plot(output_xs, ys, marker='.', label='output_pts')
    plt.legend()
    plt.show()

  return ys


def partial_deriv(fnc, *args, **kwargs):
  """
  Returns the Jacobian of a function that takes n inputs and returns n outputs.
  """

  relative_perturbs = kwargs.get('relative_perturb', np.zeros(len(args)))
  absolute_perturbs = kwargs.get('absolute_perturb', np.zeros(len(args)))
  fnc_kwargs = kwargs.get('fnc_kwargs', {})

  derivatives = []

  initial_vals = np.array(fnc(*args, **fnc_kwargs))
  num_output = len(initial_vals)

  args_final = []
  #TODO: this is lazy and just does one directional perturbation. should be centered.
  for arg, rel, absolute in zip(args, relative_perturbs, absolute_perturbs):
    args_final.append(arg + arg * rel + absolute)

  for ii in range(num_output):
    temp_args = list(args)
    temp_args[ii] = args_final[ii]

    final_vals = np.array(fnc(*temp_args, **fnc_kwargs))
    derivatives.append((final_vals - initial_vals)
                       / (temp_args[ii] - args[ii]))

  return np.array(derivatives)


def wrap_in_tuple(fnc):
  """
  Returns a function that wraps the output of a function into a tuple
  """
  def list_wrapper(*args, **kwargs):
    return (fnc(*args, **kwargs),)
  return list_wrapper


def incl_from_h_min(h_min, gs_z, l_tether, z_min):
  assert h_min is not None
  z_offset = h_min - gs_z
  return math.asin(z_offset / l_tether) - math.asin(
      z_min / l_tether)


class Const(object):
  # Speed of sound at MSL, standard conditions.  [m/s]
  C_SOUND = 343.0

  # Acceleration due to gravity at MSL.  [m/s^2]
  G = 9.80665

  hrs_per_yr = 8760.0
  days_per_yr = 365.25


class TextFormat(object):
  PURPLE = '\033[95m'
  CYAN = '\033[96m'
  DARKCYAN = '\033[36m'
  BLUE = '\033[94m'
  GREEN = '\033[92m'
  YELLOW = '\033[93m'
  RED = '\033[91m'
  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'
  END = '\033[0m'


def dl_to_ld(d):
  """
  Converts a dictionary of lists to a list of dictionaries.

  Example:
    This dictionary:
      {'foo': [1,2,3],
       'bar': ['a', 'b', 'c']}

    Would become:
      [{'bar': 'a', 'foo': 1},
       {'bar': 'b', 'foo': 2},
       {'bar': 'c', 'foo': 3}]
  """
  len_lists = len(list(d.values())[0])
  o = [{} for p in range(len_lists)]

  for key, arr in d.items():
    for ii, val in enumerate(arr):
      o[ii].update({key:val})
  return o

def ld_to_dl(l):
  """
  Converts a list of dictionaries to a dictionary of lists.

  Example:
    This list:
      [{'bar': 'a', 'foo': 1},
       {'bar': 'b', 'foo': 2},
       {'bar': 'c', 'foo': 3}]

    Would become this dictionary:
      {'foo': [1,2,3],
       'bar': ['a', 'b', 'c']}
  """
  o = {}

  for d in l:
    for k, v in d.items():
      if k not in o:
        o[k] = []
      o[k].append(v)
  return o


def convert_loop_angle_notation(loop_angles):
  """Converts a CSim loop angle into an FBL loop angle, or vice versa.

  Outputs are wrapped from 0 to 2pi."""

  return (-np.array(loop_angles) + 1.5 * np.pi) % (2. * np.pi)


def set_rainbow_plot_cycler(n=6, ax=None, cmap='rainbow'):
  """Allows plots to divide positions and spline into a color spectrum.
  Applies as cycler to given axis, if provided.

  Args:
    n: Number of slices of rainbow colors to make in cycler.
  Kwargs:
    ax: A matplotlib axis instance to apply cycler to, if provided.
    cmap: Colormap to use. Default is rainbow.

  Returns:
    A list of matplotlib format rgb colors in rainbow colormap with given number
    of n slices.

  """
  colors = [cm.get_cmap(cmap)(x) for x in
      np.linspace(0., 1., n, endpoint=True)]
  if ax is not None:
    ax.set_prop_cycle(cycler('color', colors))
  return colors


class ParamArg(object):
  """A parameterized argument, with helper functions to set and get values for
  the parameterization or output values at new x values.

  Args:
    param_type: String representing type of parameterization.
      Allowable values are:
        'spline': Periodic spline where args represent y values for control
          points. X values must be provided via ctrl_pts_xs in kwargs.
        'linear_interp': Linear interpolation, where args represent y values. X
          values must be provided via ctrl_pts_xs in kwargs.
        'constant': A simple repeat of arg for all requested output x values.
        'each': No parameterization. Output values are simply equal to input
          values, ie, input is specified for each output.
    args: Initial values for parameterized arg.
  Kwargs:
    ctrl_pts_xs: Input x values for parameterization types that require it.
      Will throw error if not required for input type.
  """

  def __init__(self, args, **kwargs):
    # If param_type is not provided, assume it's a simple single value.
    self.param_type = kwargs.get('param_type', 'value')
    if hasattr(args, '__iter__') and not isinstance(args, str):
      self.args = np.array(args)
    else:
      self.args = np.array([args])

    if self.param_type in ['spline', 'linear_interp']:
      self.ctrl_pts_xs = kwargs['ctrl_pts_xs']

  def get_param_values(self):
    return self.args

  def set_lookup_xs(self, lookup_xs):
    """Sets values for output x values. Use if output x values are going to
    remain constant for multiple calls of get_values. Only valid for certain
    parameterization types."""

    assert self.param_type in ['spline', 'linear_interp', 'constant'], (
        'Invalid param type for lookup_xs.')
    self.lookup_xs = np.array(lookup_xs)

  def get_values(self, lookup_xs=None):
    """Get non-parameterized values. Some parameterization types require
    lookup_xs for the output."""

    if lookup_xs is not None:
      self.set_lookup_xs(lookup_xs)

    if self.param_type == 'spline':
      values = spline_vals(
          self.ctrl_pts_xs, self.args, self.lookup_xs)
    elif self.param_type == 'linear_interp':
      values = np.interp(self.lookup_xs, self.ctrl_pts_xs, self.args)
    elif self.param_type == 'constant':
      values = np.tile(self.args, len(self.lookup_xs))
    elif self.param_type == 'each':
      values = self.args
    elif self.param_type == 'value':
      values = self.args[0]
    else:
      assert False, 'Invalid param_type.'

    return values

  def set_param_values(self, args):
    """Sets new parameterized values."""
    self.args = np.array(args)


class NormParamArg(ParamArg):
  """An extension of the ParamArg object to allow for a normalized output.

  Args:
    param_type: See ParamArg for usage.
    args: See ParamArg for usage.
    max_step: The normalization slope for the arg.
  Kwargs:
    See ParamArg for usage.

  Normalization is centered around 0, and 1 represents an increment of max_step.
  """

  def __init__(self, args, max_step, **kwargs):
    super(NormParamArg, self).__init__(args, **kwargs)

    self.max_step = max_step
    self.offsets = self.args.flatten()
    self.num_args = len(self.offsets)

    self._calc_norm_values()

  def get_norm_values(self):
    return self.norm_values

  def set_norm_values(self, norm_values):
    self.norm_values = np.array(norm_values)
    self._calc_param_values()

  def set_param_values(self, args):
    self.args = np.array(args)
    self._calc_norm_values()

  def reset_norm_values(self, max_step=None):
    self.offsets = self.args.flatten()
    if max_step is not None:
      self.max_step = max_step

  def _calc_norm_values(self):
    self.norm_values = (self.args.flatten() - self.offsets) / self.max_step

  def _calc_param_values(self):
    self.args = self.norm_values * self.max_step + self.offsets


def csim_to_fbl_playbook(playbook):
  """Converts a Csim playbook in full lookup table format to input for
  KitePowerCurve object.

  Args:
    playbook: A playbook dictionary from CSim in lookup table format.

  Returns:
    A dictionary of keyword arguments to re-create the playbook in FBL via the
    KitePowerCurve object.

  CSim lookup table format is available in the all_params.json that is generated
  during the system param setup. This format is required as the simpler
  parameterizations in config/m600/control/crosswind.py are heavily
  post-processed and smoothed.
  """

  fbl_power_curve_kwargs = {
      'pose_states_param': [],
      'path_location_params': [],
      'path_shape_params': []
  }
  wind_speeds = []

  lookup_loop_angles_fbl = convert_loop_angle_notation(
      playbook['entries'][0]['lookup_loop_angle'])

  # The linear interp parameter in FBL assumes from loop_angle 0 to 2pi,
  # which may not exactly align with csim lookup in FBL, so we need to
  # interpolate.
  lookup_loop_angles_fbl_interp = (
      np.linspace(0., 2. * np.pi, len(lookup_loop_angles_fbl)))

  for ii, entry in enumerate(playbook['entries']):
    if ii >= playbook['num_entries'] - 1:
      break

    fbl_entry = {}

    fbl_sorted_lookup = {}
    for key in ['alpha_lookup', 'beta_lookup', 'airspeed_lookup']:
      fbl_sorted_lookup[key] = np.interp(
          lookup_loop_angles_fbl_interp, lookup_loop_angles_fbl,
          entry[key], period=2.*np.pi)

    fbl_entry = {
      'alpha': {'param_type': 'linear_interp',
                'values': np.degrees(fbl_sorted_lookup['alpha_lookup'])},
      'beta': {'param_type': 'linear_interp',
               'values': np.degrees(fbl_sorted_lookup['beta_lookup'])},
      'v_a': {'param_type': 'linear_interp',
              'values': fbl_sorted_lookup['airspeed_lookup']}
    }

    wind_speeds.append(entry['wind_speed'])
    fbl_power_curve_kwargs['pose_states_param'].append(fbl_entry)
    fbl_power_curve_kwargs['path_shape_params'].append(
      {'type': 'circle', 'num_pos': 18, 'r_loop': entry['path_radius_target']})

    # FBL azimuth notation is reversed. (z_axis positive up)
    fbl_power_curve_kwargs['path_location_params'].append(
      {'incl': entry['elevation'], 'azim': -entry['azi_offset']})

  assert np.std(np.diff(wind_speeds)) < 1e-6, (
    'Wind speeds in playbook must be evenly spaced.')
  wind_step = wind_speeds[1] - wind_speeds[0]

  fbl_power_curve_kwargs['v_w_at_h_ref_range'] = (
      wind_speeds[0], wind_speeds[-1] + wind_step)
  fbl_power_curve_kwargs['v_w_step'] = wind_step

  return fbl_power_curve_kwargs


def fbl_to_csim_playbook(power_curve):
  """Converts an FBL KitePowerCurve object in to playbook format used by
  CSim."""

  playbook = []
  for loop in power_curve.loops:
    playbook.append(fbl_to_csim_playbook_entry(loop))
  return playbook


def fbl_to_csim_playbook_entry(loop):
  """Converts an FBL KiteLoop object into a playbook entry used by CSim."""
  key_map = {
      'azim': 'azi_offset',
      'incl': 'elevation',
      'alpha': 'alpha',
      'beta': 'beta',
      'r_loop': 'radius',
      'v_a': 'airspeed'}
  schedules = {'wind_speed': loop.v_w_at_h_ref}

  assert all([k in list(loop.pose_states_param.keys()) for k in key_map.keys()]), (
      'All playbook parameters must be present in pose_states_param, either '
      + 'provided by user or optimized.\nRequired keys: ' + str(list(key_map.keys()))
      + '\nKeys in pose_states_param: ' + str(list(loop.pose_states_param.keys())))

  for key in key_map.keys():
    param_arg = loop.pose_states_param[key]
    # Pre-process
    if key in ('alpha', 'beta', 'v_a'):
      assert ((param_arg['param_type'] == 'spline')
              or (param_arg['param_type'] == 'constant')), (
          'Alpha, beta, and v_a only support spline or constant (alpha and beta'
          + ' only) parameterization.')
    if key in ('alpha', 'beta'):
      if param_arg['param_type'] == 'constant':
        v = np.tile(np.radians(param_arg['values']), 6)
      else:
        v = np.radians(param_arg['values'])
    elif key == 'azim':
      # Ground frame is +z up in FBL and +z down in CSim, so flip azimuth.
      v = [-v for v in param_arg['values']]
    else:
      v = param_arg['values']

    # Place values in schedule with CSim name.
    if len(v) == 1:
      schedules[key_map[key]] = round(v[0], 4)
    else:
      schedules[key_map[key]] = np.round(v, 4).tolist()
  return schedules


class BridleMoments(object):
  """Creates a class the represents a bridle configuration.

  Class can be used to calculate body moments, create tables for a sweep of
  rolls and pitches, or determine neutral roll angle for tension and CG.

  More documentation and references: b/130350628
  Note that here the moments are defined about the body origin,
  rather than CG, so the inertial moments about the origin need to be
  accounted for seperately.
  """

  def __init__(self,
               bridle_pos,
               bridle_rad,
               bridle_y_offset):

    # Set up position vectors.
    # General notation of r_ab is a vector to point a with respect to point b.
    # The points of interest for the bridles are:
    # O: The kite body origin.
    # G: the kite center of mass.
    # b: The point on the bridle pitch axis above the bridle knot (at the
    # y_offset).
    # k: The bridle knot (junction).
    # k0: The bridle knot at zero pitch.
    # b/130350628

    self._r_bO = np.mean(bridle_pos, axis=0)
    # Offset average Y position with the bridle offset.
    self._r_bO[1] += bridle_y_offset

    self._r_k0b = np.array([0, 0, bridle_rad])

  def CalculateNeutralRollAngle(self, r_G):
    """Calculates neutral roll angle for bridle in the absence of all forces
    and moments other than Cg accelerations and moments resulting from
    tension.
    """

    self._r_bG = self._r_bO - r_G
    self.roll0 = np.arctan2(-self._r_bG[1], self._r_bG[2] + self._r_k0b[2])
    print('Neutral roll angle, phi_0 = %g deg' % np.rad2deg(self.roll0))
    return np.rad2deg(self.roll0)

  def CalculateMomentFromPitchRoll(self, pitch, roll):
    r_kO = self._CalcKnotVectorFromPitch(pitch)
    return self._CalcMomentFromPitchRollKnotVector(pitch, roll, r_kO)

  def MakeTables(self,
                 pitch_range=np.radians(np.linspace(-90, 30, 80)),
                 roll_range=np.radians(np.linspace(-60, 60, 60))):
    self._pitch_range = pitch_range
    self._roll_range = roll_range
    self._BuildMomentTables()

  def _BuildMomentTables(self):
    self.pitch_grid, self.roll_grid = np.meshgrid(
        self._pitch_range, self._roll_range)

    # Initialize variables.
    self.M_bx = np.empty(self.pitch_grid.shape)
    self.M_by = np.empty(self.pitch_grid.shape)
    self.M_bz = np.empty(self.pitch_grid.shape)

    # Step through all pitch values.
    for ii, pitch in enumerate(self._pitch_range):
      r_kO = self._CalcKnotVectorFromPitch(pitch)

      # Iterate through each roll angle.
      for jj, roll in enumerate(self._roll_range):
        M_b = self._CalcMomentFromPitchRollKnotVector(pitch, roll, r_kO)
        # Pull out components and store in their own array.
        self.M_bx[jj, ii] = M_b[0]  # roll
        self.M_by[jj, ii] = M_b[1]  # pitch
        self.M_bz[jj, ii] = M_b[2]  # yaw

  def GetMoments(self, pitch, roll):
    r_kO = self._CalcKnotVectorFromPitch(pitch)
    return self._CalcMomentFromPitchRollKnotVector(pitch, roll, r_kbO)

  def _CalcKnotVectorFromPitch(self, pitch):

    # 3D rotation matrix to go from r_k0b to r_kb given a pitch rotation.
    # Negative pitch because rotations are coordinate system based.
    rot_pitch = y_rotation(-pitch)

    r_kO = np.matmul(rot_pitch, self._r_k0b) + self._r_bO
    return r_kO

  def _CalcMomentFromPitchRollKnotVector(self, pitch, roll, r_kO):
    # Calculate the body tension unit vector based on current pitch and roll.
    # TODO: Replace with appropriate rotation matrices notation.
    T_hat = np.array([
        math.cos(roll) * math.sin(pitch),
        math.sin(-roll),
        math.cos(roll) * math.cos(pitch)
    ])
    # Calculate the kite body moments from the tether and bridles.
    return np.cross(r_kO, T_hat)

  def PlotMomentTables(self, **kwargs):
    plots = kwargs.get('plots', {})
    figsize = kwargs.get('figsize', (7, 7))

    try: self.Mx
    except: self.MakeTables()


    moments = ['M_bx', 'M_by', 'M_bz']
    A = np.stack((self.M_bx, self.M_by, self.M_bz), axis=2)
    titles = np.array([
        'Bridle roll moment / tether tension [Nm/N], about origin',
        'Bridle pitch moment / tether tension [Nm/N], about origin',
        'Bridle yaw moment / tether tension [Nm/N], about origin'])
    linelabelformat = np.array(['%1.1f',
                                '%1.3f',
                                '%1.1f'])

    for ii, M in enumerate(moments):
      if (np.max(A[:, :, ii]) - np.min(A[:, :, ii])) > 1e-6:
        if M in plots:
          fig = plots[M]
        else:
          plots[M] = plt.figure(figsize=figsize)
        ax = plots[M].gca()

        cs = ax.contour(np.degrees(self.roll_grid), np.degrees(self.pitch_grid),
                        A[:, :, ii], 20, colors='k')
        ax.clabel(cs, inline=1, fmt=linelabelformat[ii], fontsize=10)
        im = ax.imshow(A[:, :, ii].T, interpolation='bilinear', origin='lower',
                       cmap=cm.summer, extent=(
                           np.degrees(np.min(self.roll_grid)),
                           np.degrees(np.max(self.roll_grid)),
                           np.degrees(np.min(self.pitch_grid)),
                           np.degrees(np.max(self.pitch_grid))))
        plots[M].colorbar(im, shrink=0.8)
        im.set_clim(np.min(A), np.max(A))
        ax.set_title(titles[ii])
        ax.set_xlabel('tether roll [deg]')
        ax.set_ylabel('tether pitch [deg]')

    return plots


def slope_linear(va, vb, wa, wb):
  return (va * wa + vb * wb) / (wa + wb)

def slope_quadratic(va, vb, wa, wb):
  wa = wa ** 2.0
  wb = wb ** 2.0
  return slope_linear(va, vb, wa, wb)
