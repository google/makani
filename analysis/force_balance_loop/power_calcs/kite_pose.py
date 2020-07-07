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
import math
import json
import copy

from lib import utils
from six.moves import range


i = 0
j = 1
k = 2


class InvalidInput(Exception):
  pass


def newton_raphson_solve(fnc, init_guess, init_increment=0.01,
                         tol=0.0001, slope_tol=1e-6,
                         iter_max=7, bounds=None,
                         verbose=False, return_last=False,
                         return_converge_data=False, fnc_kwargs={}):
  """
  Returns root for provided function using newton-raphson method.
  Function can accept n inputs and return n outputs.

  Returns None if tolerance is not met within number of iterations,
  unless return_last is set to 'True', in which case it returns
  the last value that had (which is hopefully the closest).

  If multiple solutions exist, there is no guarantee it will find the intended root.

  Args:
    fnc:
      function to find root of.
      can be multidimensional, if so, must accept list of args
      and must return vars of length args.
    init_guess:
      initial guess for root
  Kwargs:
    init_increment:
      relative change of init guess for initial increment
    tol:
      absolute value of error for convergence
    slope_tol:
      prevents zero value slopes
    iter_max:
      maximum number of iterations
    bounds:
      tuple of limits for guesses - must provide min and max
    verbose:
      prints convergence messages
    return_last:
      if it doesn't converge, will return last value
    return_converge_data:
      if True, will return final value along with list of tuples containing:
        (vars_old, vars_new, errors_old, errors_new, error_deriv) for each step
    fnc_kwargs:
      dict of other params to pass into function

  """
  #initialize
  if not hasattr(init_guess, '__iter__'):
    init_guess = [init_guess]
    bounds = [bounds]
    multi_dimension = False
    nr_fnc = utils.wrap_in_tuple(fnc)
  else:
    multi_dimension = True
    nr_fnc = fnc
  init_guess = np.array(init_guess)

  num_vars = len(init_guess)
  iteration = 0
  converged = False
  all_at_bounds_iter_count = 0

  vars_old = np.array(init_guess)
  vars_new = vars_old + vars_old * init_increment

  errors_new = np.zeros(num_vars)
  errors_old = np.zeros(num_vars)

  error_derivatives = np.zeros(num_vars)
  converge_data = []

  # Get the first error
  errors_old = np.array(nr_fnc(*vars_old, **fnc_kwargs))
  errors_new = np.array(nr_fnc(*vars_new, **fnc_kwargs))

  #start iterations to converge
  for iteration in range(iter_max):
    # get the partial derivatives from old values
    vars_delta = vars_new - vars_old
    # if multidimensional, we want to do partial derivatives
    if multi_dimension:
      # returns jacobian matrix for mult-variate
      error_derivatives = utils.partial_deriv(nr_fnc, *vars_old,
                                        absolute_perturb=vars_delta)
    else:
      #if not multidimensional, manually calc derivative to save a function eval
      vars_delta = vars_new[0] - vars_old[0]
      if vars_delta == 0:
        error_derivatives[0] = 0.
      else:
        error_derivatives[0] = ((errors_new[0] - errors_old[0])
                                / (vars_new[0] - vars_old[0]))

    #make the new the old
    vars_old = copy.copy(vars_new)
    errors_old = copy.copy(errors_new)

    if multi_dimension: #if multi, then derivatives is the jacobian matrix
      vars_new = (
          vars_new
          - np.sum(np.multiply(np.linalg.inv(error_derivatives), errors_new), axis=0))
    else:
      if error_derivatives[0] != 0.:
        vars_new[0] = vars_new[0] - errors_new[0] / error_derivatives[0]

    #check bounds and clip to bounds if required
    if bounds is not None:
      for ii in range(num_vars):
        if bounds[ii] is not None:
          vars_new[ii] = np.clip(vars_new[ii], bounds[ii][0], bounds[ii][1])
      if vars_new == vars_old:
        all_at_bounds_iter_count += 1
    if all_at_bounds_iter_count > 1:
      #if we ride bounds on all variables more than once, that's it
      if verbose:
        print('All variables riding bounds - unable to converge.')
      break

    errors_new = np.array(nr_fnc(*vars_new, **fnc_kwargs))

    if return_converge_data:
      converge_data.append((vars_old, vars_new,
                            errors_old, errors_new,
                            error_derivatives))

    if all(abs(e) < tol for e in errors_new):
      converged = True
      if verbose:
        print('Converged errors to',
              [round(e,4) for e in errors_new],
              ' in %d iterations. Tolerance is %0.4g'
              % (iteration, tol))
      break

  if not multi_dimension:
    vars_new = vars_new[0]

  if not converged:
    vars_new = None if not return_last else vars_new

    if verbose == True:
      print('Did not converge within tolerance of %f and iteration limit of %d'
            % (tol, iter_max))
      print('Current error is ' + str([round(x, 5) for x in errors_new]))

  o = (vars_new, converge_data) if return_converge_data else vars_new
  return o


def sum_forces_along_axis(forces, axis, axis_is_unit_vec=True):
  """
  Returns the summed vector of a dictionary of forces along a specified axis.

  axis_is_unit_vec flag is used to save the time of normalizing the axis if
  it's already a unit vector.
  """
  if axis_is_unit_vec:
    norm_axis = axis
  else:
    norm_axis = axis / utils.vec_length(axis)
  sum_forces = np.dot(sum(forces.values()), norm_axis) * norm_axis
  return sum_forces

def sum_forces(forces):
  """Returns the summed vector of a dictionary of forces"""
  return sum(forces.values())


class KitePose(object):
  """
  A single point model of an energy kite.

  Contains methods to solve using force balance methodology, and fill object
  with solution data.

  Args:
    position:
      a dictionary that defines a single position:
        Required items:
          'xyz': An array-like vector of [x, y, z] coordinates in ground frame.
            Z is up, and default direction wind blows to is +X.
          'r_curv': the path radius of curvature at that position
          'r_curv_hat': array like unit vector pointing to the path instant
            center for this position
          'segment_length': scalar of length of this section of path
          'e_tether_rad': array like unit vector pointing to the tether ground
            attach point
          'e_path_tangent': array like unit vector of the path tangent at this
            position in the direction of travel
    resource: A dictionary that specifies a resource.
      See KitePowerCurve docstring for more details.
    config: A dictionary that specifies an energy kite.
      See KitePowerCurve docstring for more details.

  Kwargs:
    A set of state params is required to complete the solution.
    Required state param kwargs are:
      'v_a' or 'v_k'
      'alpha' and 'beta'
    Optional state param kwargs:
      lift_roll_angle: Number indicating amount of roll the lift vector has
        about the airspeed axis. Zero is defined as in plane with the straight
        line tether.
        If lift_roll_angle is NOT specifed, the pose attempts to find the
        lift_roll_angle that best meets the force balance.
      aero_device_scale:
        Number from 0 to 1 representing amount of use of an aero device. Config
        must contain a function 'aero_device' that returns aero coefficient
        deltas as a function of this device scale.

    v_w_at_h_ref:
      Optional wind speed, in m/s, at resource reference height.
      If not provided, pose attempts to use resource['v_w_avg_h_ref'].
    v_w_hat:
      Optional wind direction, in array like vector format.
      If not provided, default direction is along +x, [1, 0, 0].
    accel_along_path:
      Optional acceleration along path, specified in m/s^2.
    a_k_vec:
      Array-like vector indicating total acceleration of the kite. This is then
      broken into accelerations along and perpendicular to the path.
    grav_mult:
      Multiplier to apply to gravity - default is 1. Enables gravity to be
      scaled if desired.
    verbose:
      Boolean - adds additional print statements about calculation.
    solver:
      String selecting solver type. Options are 'CSim' or 'FBL'. The solver type
      determines what tool does the force and moment calculations given the
      state that is calculated from the pose inputs.
    solve_options:
      Optional dictionary of solution params:

      Optional items:
        contol_norm:
          Normalized constraint tolerance to ignore when
          determining validity of solution.
        power_tol_norm:
          Normalized power tolerance to round to rated power if
          power is within this range.
          Enables power limiting solvers to converge faster.

  """

  def __init__(self, position, resource, config,
               v_w_at_h_ref=None, v_w_hat=None,
               grav_mult=1.0, verbose=False,
               opt=False, **kwargs):

    # Setup position.
    self.position = position
    if 'gs_position' in config:
      self.gs_position = np.array(config['gs_position'])
    else:
      self.gs_position = np.array((0., 0., 0.))

    # Setup resource, config, and wind magnitude.
    self.resource = resource
    self.config = config
    if v_w_at_h_ref is not None:
      self.v_w_at_h_ref = v_w_at_h_ref
    else:
      self.v_w_at_h_ref = resource['v_w_avg_h_ref']

    # Setup solution settings.
    self.verbose = verbose
    self.opt = opt

    # Setup default solve options.
    solve_options = {
        'contol_norm': 0.001,
        'power_tol_norm': 1e-6}
    solve_options.update(kwargs.get('solve_options', {}))
    self.solve_options = solve_options

    # Init pose variables.
    self.valid = True
    self.state = {}
    self.state['grav_mult'] = grav_mult

    # Initialize empty variables in state.
    self.state['valid'] = True
    self.state['forces_g'] = {'parts': {}, 'type': {}}
    self.state['moments_b'] = {'parts': {}, 'type': {}}
    self.state['tension'] = None
    self.state['power_shaft'] = None
    self.state['constraints'] = []
    self.state['constraints_violated'] = []

    # Update state with position. It is useful to have it accessible there.
    self.state.update(position)

    # Setup wind direction. If not provided, wind is going to +x.
    if v_w_hat is not None:
      self.state['v_w_hat'] = np.array(v_w_hat) / utils.vec_length(v_w_hat)
    else:
      self.state['v_w_hat'] = np.array([1.,0.,0.])

    # Determine what aero model(s) are present in config.
    if 'aero_coeff_from_alpha_beta' in config:
      self._aero_type = 'alpha_beta_aero'
    elif 'body_coeff_from_alpha_beta' in config:
      self._aero_type = 'alpha_beta_body'
    else:
      raise InvalidInput(
          'Config must have one of the following aero functions:\n'
          + '\'body_coeff_from_alpha_beta\', or '
          + '\'aero_coeff_from_alpha_beta\' function.')

    self._parse_kwargs(kwargs)
    self._calc_knowns()

  def __getitem__(self, key):
    return getattr(self, key)

  def _parse_kwargs(self, kwargs):
    """
    Checks to make sure kwargs provided are enough to define a state.
    """

    # Select solver to get forces and moments.
    self.solver = kwargs.get('solver', 'FBL')

    # Must provide either a v_k or a v_a.
    assert ('v_k' in kwargs) != ('v_a' in kwargs), \
            'Must provide either v_k or v_a, not none or both.'

    # Parse acceleration inputs
    if 'accel_along_path' in kwargs:
      self.state['accel_along_path'] = kwargs['accel_along_path']
    else:
      self.state['accel_along_path'] = 0.0

    # Parse body angular rates and accelerations.
    # Defaults are zero if not provided.
    self.state['pqr'] = kwargs.pop('pqr', np.zeros(3))
    self.state['pqr_dot'] = kwargs.pop('pqr_dot', np.zeros(3))

    keys = ('v_a', 'v_k',
            'alpha', 'beta',
            'lift_roll_angle', 'aero_device_scale')

    for key in keys:
      if key in kwargs:
        self.state[key] = kwargs[key]

  def _calc_energy(self):
    self.state['potential_E'] = (
        self.position['xyz'][k]
        * (self.config['m_kite'] + self.config['m_tether'] / 2.0)
        * utils.Const.G * self.state['grav_mult'])
    # Assumes the tether mass per unit length is constant.
    self.state['kinetic_E'] = (
      0.5 * (self.config['m_kite'] + self.config['m_tether'] / 3.)
      * self.state['v_k']**2)
    self.state['total_E'] = self.state['potential_E'] + self.state['kinetic_E']

  def _calc_orientation(self, state=None):

    if state is None:
      state = self.state

    # Orientation can only be calculated if all angles relative to f frame
    # are all specified.
    # Not available, this function does nothing.
    # TODO: This probably shouldn't fail silently.
    if all([k in state for k in ['alpha',
                                 'beta',
                                 'lift_roll_angle',
                                 'DCMg2f']]):

      # Order is roll-yaw-pitch to go from f frame to body. X and z are
      # flipped from f to body where beta and lift_roll_angle are defined, so
      # rotations about those axes are flipped.
      state['DCMf2b'] = np.dot(
          utils.y_rotation(math.radians(state['alpha'])),
          np.dot(utils.z_rotation(math.radians(state['beta'])),
                 utils.x_rotation(-state['lift_roll_angle'])))
      # Flip x and z to get body coord.
      state['DCMf2b'] = np.dot(
          np.array([[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]]),
          state['DCMf2b'])
      # Rotate f frame in g to get body in g.
      state['DCMg2b'] = np.dot(
          state['DCMf2b'], state['DCMg2f'])

      # Aero frame (a) is the f frame, but rolled by lift roll angle.
      state['DCMg2a'] = np.dot(
        utils.x_rotation(-state['lift_roll_angle']), state['DCMg2f'])

      # Put kite orientation in terms of xyz vectors for user.
      state['kite_axis'] = {}
      for ii, k in enumerate(['x', 'y', 'z']):
        state['kite_axis'][k] = state['DCMg2b'][ii]
      # Put the rotor axis and offset (provided in body frame) into ground frame
      # in state.
      state['rotor_thrust_axis_g'] = (
          np.dot(state['DCMg2b'].T, self.config['rotor_thrust_axis']))

  def _calc_omega_hat(self):
    """Calculates omega_hat (reduced angular rates for body frame) and
    stores them in pose states."""

    omega_scale = (
        np.array([self.config['b'], self.config['c'], self.config['b']])
        / (2. * self.state['v_a']))

    self.state['omega_hat'] = np.multiply(self.state['pqr'], omega_scale)

  def _calc_lift_roll_from_dcm_g2b(self, state=None):
    """Compute the lift roll angle from DCMg2b."""
    if state is None:
      state = self.state
    state['DCMf2b'] = np.dot(state['DCMg2b'], state['DCMg2f'].T)
    # Obtain the DCM from the flipped fly-frame to the body frame.
    # The flipped fly-frame is the same as the fly-frame except that the X
    # and Z axes are flipped:
    # +X points opposite against the apparent wind and +Z points towards
    # the ground station from the kite.
    DCMff2b = np.dot(
        np.array([[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]]),
        state['DCMf2b'])

    # Order is roll-yaw-pitch to go from f frame to body. X and z are
    # flipped from f to body where beta and lift_roll_angle are defined, so
    # rotations about those axes are flipped.
    # Undo the rotations from alpha and beta. The remainder should be
    # the rotational matrix around +X axis of the flipped fly-frame,
    # for an angle that is negative of lift roll angle.
    lift_roll_dcm = np.dot(
        utils.z_rotation(math.radians(state['beta'])).T,
        np.dot(utils.y_rotation(math.radians(state['alpha'])).T, DCMff2b))
    # Average the two copies of the sin and cos values in that x
    # rotation matrix.
    sin_r = 0.5 * (lift_roll_dcm[1, 2] - lift_roll_dcm[2, 1])
    cos_r = 0.5 * (lift_roll_dcm[1, 1] + lift_roll_dcm[2, 2])
    # Negate the obtained angle.
    state['lift_roll_angle'] = -np.arctan2(sin_r, cos_r)
    state['DCMg2a'] = np.dot(
        utils.x_rotation(-state['lift_roll_angle']),
        state['DCMg2f'])

  def _calc_knowns(self):
    # takes a state and position and calcs trivial things from here, such as:
    # gravity, centripetal force, v_w_vec, v_a_vec, potential energy

    # Calc all the speed vectors.
    self._calc_v_a_or_v_k()

    # calc kinetic and potential energy
    self._calc_energy()

    # a_k_vec is the the total kite acceleration.
    # Required in this format for the CSim solver.
    self.state['a_k_vec'] = (
      (self.state['v_k'] ** 2 / self.state['r_curv']
       * self.state['r_curv_hat'])
      + self.state['accel_along_path'] * self.state['e_path_tangent'])

    # Defines useful vectors for solving other things.
    v_a_tension_plane_normal = utils.cross(self.state['v_a_hat'],
                                           self.state['e_tether_rad'])
    v_a_e_rad_plane_normal_hat = (
        utils.unit_vec(v_a_tension_plane_normal))

    lift_no_roll_hat = utils.unit_vec(
        utils.cross(self.state['v_a_hat'], v_a_e_rad_plane_normal_hat))

    # Define the fly (f) frame:
    # The f frame has drag along positive x. Roll is defined as zero when lift
    # is in plane defined by straight line tether and v_a.
    # Represents kite orientation in air with no roll, alpha, or beta.
    self.state['DCMg2f'] = np.array(
        [self.state['v_a_hat'],
         utils.cross(
             lift_no_roll_hat, self.state['v_a_hat']),
         lift_no_roll_hat])

    # Calc kite orientation.
    self._calc_orientation()

    # Calc omega_hat.
    self._calc_omega_hat()

  def _calc_tether_catenary_forces(self):
    # Compute tether forces.

    # Tether weight, calculated from tvanalsenoy thesis, page 43.
    # https://drive.google.com/file/d/0B9xdLU0NH-aUaExYWTBwRzNMTjA/
    self.state['forces_g']['parts']['tether_eff_weight_vec'] = (
        0.5 * self.config['m_tether'] * self.state['grav_mult'] * utils.Const.G
        * math.cos(self.state['incl']) * self.state['e_tether_incl'])

    # Tether drag.
    self.state['forces_g']['parts']['tether_drag_vec'] = (
        self._aero_force_from_coeff(self.config['cD_eff_tether'], self.state)
        * self.state['v_a_hat'])

    # Tether inertial forces.
    self.state['forces_g']['parts']['tether_centrifugal_vec'] = (
        (self.config['m_tether'] / 3. * self.state['v_k']**2)
                           / self.state['r_curv'] * -self.state['r_curv_hat'])

    self.state['forces_g']['parts']['tether_path_accel_inertial_vec'] = (
        self.config['m_tether'] / 3. * self.state['accel_along_path']
        * -self.position['e_path_tangent'])

  def _calc_aero_coeff(self):
    aero_func_lookup = {
        'alpha_beta_aero': self._get_cL_cY_cD_from_alpha_beta_aero,
        'alpha_beta_body': self._get_cL_cY_cD_from_alpha_beta_body,
        }

    aero_func_lookup[self._aero_type](self.state)
    self._apply_aero_offsets(self.state, self.config)
    if 'aero_device_scale' in self.state:
      self._apply_aero_device_offsets(self.state, self.config)

  @staticmethod
  def _apply_aero_offsets(state, config):
    # Add in aero offsets.
    # TODO: Add functionality to recalculate body coeffs based on
    # modified aero coeffs.
    for k in ['cL', 'cD', 'cY']:
      if (k + '_offset') in config:
        state[k] += config[k + '_offset']

  @staticmethod
  def _apply_aero_device_offsets(state, config):
    for k, v in config['aero_device'](
          state['aero_device_scale'], state).items():
      # Body coeffs modifications not supported since this is called after body
      # coeffs are converted to aero coeffs. Since this conversion is not
      # repeated, body coeff updates do not propagate to aero coeffs.
      # This also means aero coeffs are not consistent with body coeffs.
      # TODO: Add functionality to recalculate body coeffs based on
      # modified aero coeffs.
      assert k not in ['cx', 'cy', 'cz'], (
          'Aero drag device does not support body force coefficient offsets.')
      state[k] += v

  def _get_cL_cY_cD_from_alpha_beta_aero(self, state):
    state.update(
        self.config['aero_coeff_from_alpha_beta'](
            state['alpha'], state['beta'], state['omega_hat']))

  def _get_cL_cY_cD_from_alpha_beta_body(self, state):
    state.update(
        self.config['body_coeff_from_alpha_beta'](
            state['alpha'], state['beta'], state['omega_hat']))

    self._aero_coeff_from_body_coeff(state)

  def _calc_v_a_or_v_k(self):
    """If v_a is provided, will solve for v_k and vice versa."""

    self.state['v_w_vec'] = (
        self.resource['v_w_at_height'](
            self.position['xyz'][k], self.v_w_at_h_ref)
        * self.state['v_w_hat'])
    self.state['v_w'] = utils.vec_length(self.state['v_w_vec'])

    if 'v_k' in self.state:
      # Then solve for v_a given v_w and v_k
      self.state['v_k_vec'] = (
          self.position['e_path_tangent'] * self.state['v_k'])

      self.state['v_a_vec'] = self.state['v_w_vec'] - self.state['v_k_vec']
      self.state['v_a'] = utils.vec_length(self.state['v_a_vec'])
    else:
      # Else solve for v_k given v_w and v_a

      # This is a SSA triangle, which might have 2 solutions.
      # If v_a > v_w (which should generally be true), there is only 1.
      wind_angle = math.acos(np.dot(self.position['e_path_tangent'],
                                    utils.unit_vec(self.state['v_w_vec'])))
      sin_wind_angle = math.sin(wind_angle)
      kiting_angle = (
          math.asin(self.state['v_w'] * sin_wind_angle / self.state['v_a']))
      v_a_v_w_angle = math.pi - wind_angle - kiting_angle

      self.state['v_k'] = (
          self.state['v_a'] * math.sin(v_a_v_w_angle) / sin_wind_angle)
      self.state['v_k_vec'] = (
          self.state['v_k'] * self.position['e_path_tangent'])
      # NOTE: Due to floating point errors with this calculation, |v_a_vec| only approximate v_a!
      self.state['v_a_vec'] = self.state['v_w_vec'] - self.state['v_k_vec']

    self.state['v_k_hat'] = utils.unit_vec(self.state['v_k_vec'])
    self.state['v_a_hat'] = utils.unit_vec(self.state['v_a_vec'])

  @staticmethod
  def _aero_coeff_from_body_coeff(state):
    """Returns aero coefficients from a state that contains body coefficients.

    Does NOT apply aero offsets that may be in config."""
    alpha_rad = math.radians(state['alpha'])
    beta_rad = math.radians(state['beta'])

    sin_alpha = math.sin(alpha_rad)
    cos_alpha = math.cos(alpha_rad)
    sin_beta = math.sin(beta_rad)
    cos_beta = math.cos(beta_rad)

    state['cL'] = state['cx'] * sin_alpha - state['cz'] * cos_alpha
    state['cY'] = (-state['cx'] * cos_alpha * sin_beta
                    + state['cy'] * cos_beta
                    - state['cz'] * sin_alpha * sin_beta)
    state['cD'] = (-state['cx'] * cos_alpha * cos_beta
                    - state['cz'] * sin_alpha * cos_beta
                    - state['cy'] * sin_beta)

  def _aero_force_from_coeff(self, c, state):
    # Takes a known aero coefficient, c, and state with v_a and calculates lift.
    force = 0.5 * self.resource['rho'] * self.config['s'] * c * state['v_a']**2
    return force

  def _aero_coeff_from_force(self, force, state):
    # Take a known force and state with v_a and calculates an aero coefficient.
    c = (2. * force) / (self.resource['rho'] * self.config['s'] * state['v_a']**2)
    return c

  def solve(self):
    """
    Solves pose force balance with a known aero coefficient, as derived by
    alpha and beta.

    If lift roll angle is unknown, it is solved for in order to
    balance the forces. As long the lift is able to complete this balance, the
    residual will be zero.

    If lift roll angle is provided, orientation is fully specified. It is not
    guaranteed or even likely that this balances the forces,
    as will be indicated by a residual force.

    Does not attempt to meet constraints or limits - will return results
    regardless of validity.

    Fills pose.state with results of solution.
    pose.state['valid'] is True if valid force balance is found.
    """

    # We expect the self._state['forces_g'] to be a dictionary like {
    # 'type': The dictionary of major force components in vector form.
    #        This field is required and must follow the same format.
    # 'parts': Details or further breakdowns of each component in 'type'.
    #          E.g., 'aero' --> 'lift_vec', 'side_lift_vec', etc.
    #          This field is OPTIONAL, mostly to debug a particular solver.
    #          Follow-up computation should NOT use fields here.
    # We may need to add a 'scalar' version, just to store the norm of
    # each component of 'type'.
    # self._state['moments_b'] has a similar structure.

    # get cL, cD, and cY from aero params
    self._calc_aero_coeff()

    # Determine solve type and kick off appropriate solver.
    if self.solver == 'FBL':
      self._solve_FBL()
    elif self.solver == 'CSim':
      self._solve_CSim()

    self._calc_shaft_power()
    self._calc_power_padmount()
    self._calc_summary_data()
    self._apply_constraints()

  def _solve_FBL(self):
    # Calculate and sum forces.
    self._force_bal()

    expected_forces = ['rotors', 'aero', 'gravity', 'tether',
                       'inertial', 'residual']
    for s in expected_forces:
      assert s in self.state['forces_g']['type'], \
            "%s is missing from KitePose._state['forces_g']['type']" % s

    # Tension scalar needed for moment solve and tension penalties.
    if not self.state.get('tension', None):
      self.state['tension'] = utils.vec_length(
          self.state['forces_g']['type']['tether'])

    if not self.state.get('rotor_force', None):
      # Rotor force scalar needed for shaft power calc.
      # Dot product used as sign is important.
      # Positive is thrusting, negative is gen.
      self.state['rotor_force'] = np.dot(
          self.state['forces_g']['type']['rotors'],
          self.state['rotor_thrust_axis_g'])

    self._moment_balance()

    expected_moments = ['rotors', 'aero', 'gravity', 'tether',
                        'inertial', 'residual']
    for s in expected_moments:
      assert s in self.state['moments_b']['type'], \
          "%s is missing from KitePose._state['moments_b']['type']" % s

  def _solve_csim(self):
    raise NotImplemented

  def print_forces(self):
    print('---------------------------------')
    for s in self.state['forces_g']['type']:
      print('%s: %s' % (s, self.state['forces_g']['type'][s]))

  def _calc_summary_data(self):
    """Calculates things that are useful to the user, but are not required for
    solving."""

    # If an optimization run, skip to save compute time.
    if not self.opt:
      # TODO: Add function to calculate components of force vectors
      # along axes that are useful to user.

      # TODO: Uncomment when components are made, as noted in TODO
      # above.
      # self.state['k_tension_to_lift'] = (
      #   self.state['tension'] / self.state['lift'])

      # Calc L/D for kite only.
      self.state['cL_over_cD_kite'] = self.state['cL'] / self.state['cD']

      # Calc L/D for system (kite + tether).
      self.state['cL_over_cD_sys'] = (
        self.state['cL'] / (self.state['cD'] + self.config['cD_eff_tether']))

      # Calc thrust power. Negative is gen, positive is thrusting.
      self.state['power_thrust'] = (
        self.state['rotor_force'] * self.state['v_a'])

      # Beta inertial is beta of the aero frame with respect to the path, as
      # opposed to the kite body. This shows the beta that would result from the
      # nose of the kite on the flight path (but still rolled as needed).
      self.state['beta_inertial'] = math.degrees(utils.angle_signed(
          -self.state['v_a_hat'],
          self.state['e_path_tangent'],
          self.state['DCMg2a'][2]))

      # Calc rotor drag fraction, k
      # Loyd limit suggests an optimal at 0.5
      # TODO: Use component force when we've added function to break
      # main forces into components so it doesn't need to be calculated again.
      self.state['k_rotor_drag_frac'] = (
          -self.state['rotor_force']
          / utils.vec_length(
              np.dot(self.state['forces_g']['type']['aero'],
                     self.state['DCMg2a'][0])))

      # Calc the portion of path acceleration along the gravity vector, and
      # normalize by G.
      k_path_along_g = np.dot(
          np.array([0., 0., -1.]), self.state['e_path_tangent'])

      # Accel_Gs_along_G is the acceleration of the kite on the path in Gs in
      # the direction of gravity. Basically, how much acceleration along the
      # kite path is coming from gravity, in Gs.
      self.state['accel_Gs_along_G'] = (
          self.state['accel_along_path'] * k_path_along_g
          / (utils.Const.G * self.state['grav_mult']))

      # K_grav is how much of the possible acceleration from gravity along the
      # path we are putting into kite acceleration.
      if k_path_along_g == 0:
        self.state['k_grav'] = 0.
      else:
        self.state['k_grav'] = (
          self.state['accel_Gs_along_G'] / k_path_along_g)

      # Calc the angular difference between the straight line tether and the
      # net tension. This is the catenary angle at the kite.
      tension_rad = np.dot(
          self.state['forces_g']['type']['tether'],
          self.state['e_tether_rad'])

      # Calc tether catenary angle.
      self.state['tether_catenary_angle'] = math.acos(
          min(1.0, max(-1.0, tension_rad/self.state['tension'])))
      # Calc tether catenary direction. Zero is in direction of path. Positive
      # means tether angle is coming from kite's right, negative from left.
      self.state['tether_catenary_direction'] = utils.angle_signed(
          self.state['v_k_hat'],
          self.state['forces_g']['type']['tether'],
          self.state['e_tether_rad'])

      self.state['kiting_angle'] = math.acos(
          round(np.dot(-self.state['v_k_hat'], self.state['v_a_hat']), 6))
      self.state['lift_to_vk_vw_plane_angle'] = utils.angle_signed(
        self.state['v_w_hat'],
        self.state['forces_g']['type']['aero'],
        self.state['v_a_hat'])

      p_in_wind = (0.5 * self.resource['rho'] * self.state['v_w']**3)

      self._calc_powers()

      if p_in_wind != 0.:
        self.state['zeta_padmount'] = (
            self.state['power'] / (p_in_wind * self.config['s']))
      else:
        if self.state['power'] > 0.:
          self.state['zeta_padmount'] = float('inf')
        else:
          self.state['zeta_padmount'] = float('-inf')

  def _calc_powers(self):
    """Calculates various components of power along v_a and stores to state"""

    def power_along_axis(force, axis, speed):
      return np.dot(force, axis) * speed

    aero_vec_total = self.state['forces_g']['type']['aero']
    self.state['aero_power'] = power_along_axis(
        self.state['forces_g']['type']['aero'],
        self.state['v_a_hat'],
        -self.state['v_a'])
    self.state['tether_power'] = power_along_axis(
        self.state['forces_g']['type']['tether'],
        self.state['v_a_hat'],
        -self.state['v_a'])
    self.state['gravity_power'] = power_along_axis(
        self.state['forces_g']['type']['gravity'],
        self.state['v_a_hat'],
        -self.state['v_a'])
    self.state['accel_power'] = power_along_axis(
        self.state['forces_g']['type']['inertial'],
        self.state['v_a_hat'],
        -self.state['v_a'])
    self.state['rotor_thrust_power'] = power_along_axis(
        self.state['forces_g']['type']['rotors'],
        self.state['v_a_hat'],
        -self.state['v_a'])
    self.state['no_grav_power'] = power_along_axis(
        self.state['forces_g']['type']['aero']
        + self.state['forces_g']['type']['tether']
        + self.state['forces_g']['type']['inertial'],
        self.state['v_a_hat'],
        -self.state['v_a'])

  def _apply_constraints(self):
    v_a_margin = self.state['v_a'] - self.config['v_a_min']
    power_shaft_margin = (
        self.config['power_shaft_max'] - abs(self.state['power_shaft']))
    tension_max_margin = self.config['tension_max'] - self.state['tension']
    net = utils.vec_length(self.state['forces_g']['type']['residual'])

    #TODO: cleanup section with function so it's not so much copy paste
    #but theres so many somewhat special cases!
    self.state['constraints'].append(
        {'name': 'v_a_min_margin',
         'value': self.state['v_a'],
         'limit': self.config['v_a_min'],
         'lim_type': 'min',
         'margin': v_a_margin,
         'margin_norm': v_a_margin / self.config['v_a_min']})

    self.state['constraints'].append(
        {'name': 'power_shaft_margin',
         'value': self.state['power_shaft'],
         'limit': self.config['power_shaft_max'],
         'lim_type': 'max',
         'margin': power_shaft_margin,
         'margin_norm': (power_shaft_margin
                         / self.config['power_shaft_max'])})

    # TODO: Make net constraint offset and normalization a parameter.
    # The magnitude of the residual force (net) will never be negative.
    # In order to help the optimizer find solutions, we offset the normalized
    # margin by a small amount.
    self.state['constraints'].append(
        {'name': 'net_margin',
         'value': net,
         'limit': 0.,
         'lim_type': 'zero',
         'margin': -net,
         'margin_norm': (
            (-net + self.config['m_kite'] * 0.005 * utils.Const.G)
            / (self.config['m_kite'] * 10. * utils.Const.G))})

    self.state['constraints'].append(
        {'name': 'max_tension',
         'value': self.state['tension'],
         'limit': self.config['tension_max'],
         'lim_type': 'max',
         'margin': tension_max_margin,
         'margin_norm': (tension_max_margin / self.config['tension_max'])})

    if 'min_turn_r' in self.config:
      r_turn_min_margin = self.state['r_curv'] - self.config['min_turn_r']
      self.state['constraints'].append(
          {'name': 'min_turning_radius',
           'value': self.state['r_curv'],
           'limit': self.config['min_turn_r'],
           'lim_type': 'min',
           'margin': r_turn_min_margin,
           'margin_norm': (r_turn_min_margin / self.config['min_turn_r'])})

    if 'v_a_max' in self.config:
      v_a_max_margin = self.config['v_a_max'] - self.state['v_a']
      self.state['constraints'].append(
          {'name': 'v_a_max_margin',
           'value': self.state['v_a'],
           'limit': self.config['v_a_max'],
           'lim_type': 'max',
           'margin': v_a_max_margin,
           'margin_norm': v_a_max_margin / self.config['v_a_max']})

    if 'alpha_max' in self.config:
      alpha_max_margin = self.config['alpha_max'] - self.state['alpha']
      self.state['constraints'].append(
          {'name': 'alpha_max_margin',
           'value': self.state['alpha'],
           'limit': self.config['alpha_max'],
           'lim_type': 'max',
           'margin': alpha_max_margin,
           'margin_norm': alpha_max_margin / 5.})
      # TODO: Alpha min may be 0., hence the fixed normalization.
      # Come up with a better way to do this.

    if 'alpha_min' in self.config:
      alpha_min_margin = self.state['alpha'] - self.config['alpha_min']
      self.state['constraints'].append(
          {'name': 'alpha_min_margin',
           'value': self.state['alpha'],
           'limit': self.config['alpha_min'],
           'lim_type': 'min',
           'margin': alpha_min_margin,
           'margin_norm': alpha_min_margin / 5.})
      # TODO: Alpha min may be 0., hence the fixed normalization.
      # Come up with a better way to do this.

    if 'cL_max' in self.config:
      cL_max_margin = self.config['cL_max'] - self.state['cL']
      self.state['constraints'].append(
          {'name': 'cL_max_margin',
           'value': self.state['cL'],
           'limit': self.config['cL_max'],
           'lim_type': 'max',
           'margin': cL_max_margin,
           'margin_norm': (cL_max_margin / self.config['cL_max'])})

    if 'cL_min' in self.config:
      cL_min_margin = self.state['cL'] - self.config['cL_min']
      self.state['constraints'].append(
          {'name': 'cL_min_margin',
           'value': self.state['cL'],
           'limit': self.config['cL_min'],
           'lim_type': 'min',
           'margin': cL_min_margin,
           'margin_norm': (cL_min_margin / 1.)})

    if 'cY_max' in self.config:
      cY_max_margin = self.config['cY_max'] - self.state['cY']
      self.state['constraints'].append(
          {'name': 'cY_max_margin',
           'value': self.state['cY'],
           'limit': self.config['cY_max'],
           'lim_type': 'max',
           'margin': cY_max_margin,
           'margin_norm': (cY_max_margin / self.config['cY_max'])})

    if 'cY_min' in self.config:
      cY_min_margin = self.state['cY'] - self.config['cY_min']
      self.state['constraints'].append(
          {'name': 'cY_min_margin',
           'value': self.state['cY'],
           'limit': self.config['cY_min'],
           'lim_type': 'min',
           'margin': cY_min_margin,
           'margin_norm': (cY_min_margin / self.config['cY_min'])})

    if 'beta_max' in self.config:
      beta_max_margin = self.config['beta_max'] - self.state['beta']
      self.state['constraints'].append(
          {'name': 'beta_max_margin',
           'value': self.state['beta'],
           'limit': self.config['beta_max'],
           'lim_type': 'max',
           'margin': beta_max_margin,
           'margin_norm': (beta_max_margin / abs(self.config['beta_max']))})

    if 'beta_min' in self.config:
      beta_min_margin = self.state['beta'] - self.config['beta_min']
      self.state['constraints'].append(
          {'name': 'beta_min_margin',
           'value': self.state['beta'],
           'limit': self.config['beta_min'],
           'lim_type': 'min',
           'margin': beta_min_margin,
           'margin_norm': (beta_min_margin / abs(self.config['beta_min']))})

    if 'cl_residual_max' in self.config:
      cl_residual_max_margin = (
          self.config['cl_residual_max'] - self.state['flap_aero_coeffs'][0])
      self.state['constraints'].append(
          {'name': 'cl_residual_max_margin',
           'value': self.state['flap_aero_coeffs'][0],
           'limit': self.config['cl_residual_max'],
           'lim_type': 'max',
           'margin': cl_residual_max_margin,
           'margin_norm': (
              cl_residual_max_margin / abs(self.config['cl_residual_max']))})

    if 'cl_residual_min' in self.config:
      cl_residual_min_margin = (
          self.state['flap_aero_coeffs'][0] - self.config['cl_residual_min'])
      self.state['constraints'].append(
          {'name': 'cl_residual_min_margin',
           'value': self.state['flap_aero_coeffs'][0],
           'limit': self.config['cl_residual_min'],
           'lim_type': 'min',
           'margin': cl_residual_min_margin,
           'margin_norm': (
              cl_residual_min_margin / abs(self.config['cl_residual_min']))})

    if 'cm_residual_max' in self.config:
      cm_residual_max_margin = (
          self.config['cm_residual_max'] - self.state['flap_aero_coeffs'][1])
      self.state['constraints'].append(
          {'name': 'cm_residual_max_margin',
           'value': self.state['flap_aero_coeffs'][1],
           'limit': self.config['cm_residual_max'],
           'lim_type': 'max',
           'margin': cm_residual_max_margin,
           'margin_norm': (
              cm_residual_max_margin / abs(self.config['cm_residual_max']))})

    if 'cm_residual_min' in self.config:
      cm_residual_min_margin = (
          self.state['flap_aero_coeffs'][1] - self.config['cm_residual_min'])
      self.state['constraints'].append(
          {'name': 'cm_residual_min_margin',
           'value': self.state['flap_aero_coeffs'][1],
           'limit': self.config['cm_residual_min'],
           'lim_type': 'min',
           'margin': cm_residual_min_margin,
           'margin_norm': (
              cm_residual_min_margin / abs(self.config['cm_residual_min']))})

    if 'cn_residual_max' in self.config:
      cn_residual_max_margin = (
          self.config['cn_residual_max'] - self.state['flap_aero_coeffs'][2])
      self.state['constraints'].append(
          {'name': 'cn_residual_max_margin',
           'value': self.state['flap_aero_coeffs'][2],
           'limit': self.config['cn_residual_max'],
           'lim_type': 'max',
           'margin': cn_residual_max_margin,
           'margin_norm': (
              cn_residual_max_margin / abs(self.config['cn_residual_max']))})

    if 'cn_residual_min' in self.config:
      cn_residual_min_margin = (
          self.state['flap_aero_coeffs'][2] - self.config['cn_residual_min'])
      self.state['constraints'].append(
          {'name': 'cn_residual_min_margin',
           'value': self.state['flap_aero_coeffs'][0],
           'limit': self.config['cn_residual_min'],
           'lim_type': 'min',
           'margin': cn_residual_min_margin,
           'margin_norm': (
              cn_residual_min_margin / abs(self.config['cn_residual_min']))})

    if 'h_min' in self.config:
      h_min_margin = self.state['xyz'][k] - self.config['h_min']
      self.state['constraints'].append(
          {'name': 'h_min_margin',
           'value': self.state['xyz'][k],
           'limit': self.config['h_min'],
           'lim_type': 'min',
           'margin': h_min_margin,
           'margin_norm': h_min_margin / self.config['h_min']})

    if 'tether_roll_min' in self.config:
      # Positive tether roll points lift towards the path center.
      value = self.state['tether_roll_angle']
      roll_min_margin = value - self.config['tether_roll_min']
      self.state['constraints'].append(
          {'name': 'tether_roll_min_margin',
           'value': value,
           'limit': self.config['tether_roll_min'],
           'lim_type': 'min',
           'margin': roll_min_margin,
           'margin_norm': (
               roll_min_margin / abs(self.config['tether_roll_min']))})

    if 'tether_roll_max' in self.config:
      value = self.state['tether_roll_angle']
      roll_max_margin = self.config['tether_roll_max'] - value
      self.state['constraints'].append(
          {'name': 'tether_roll_max_margin',
           'value': value,
           'limit': self.config['tether_roll_max'],
           'lim_type': 'max',
           'margin': roll_max_margin,
           'margin_norm': (
               roll_max_margin / abs(self.config['tether_roll_max']))})

    if 'incl_max' in self.config:
      incl_margin = self.config['incl_max'] - self.state['incl']
      self.state['constraints'].append(
          {'name': 'incl_margin',
           'value': self.state['incl'],
           'limit': self.config['incl_max'],
           'lim_type': 'max',
           'margin': incl_margin,
           'margin_norm': incl_margin / self.config['incl_max']})

    # Aero device scale needs to be a stiff constraint. If not, the optimizer
    # will heavily cheat with it early on and end up in an unsolveable space.
    if 'aero_device_scale' in self.state:
      aero_device_scale_margin = 1.0 - self.state['aero_device_scale']
      self.state['constraints'].append(
          {'name': 'aero_device_scale_max_margin',
           'value': self.state['aero_device_scale'],
           'limit': 1.0,
           'lim_type': 'max',
           'margin': aero_device_scale_margin,
           'margin_norm': aero_device_scale_margin * 10.})

      aero_device_scale_margin = self.state['aero_device_scale']
      self.state['constraints'].append(
          {'name': 'aero_device_scale_min_margin',
           'value': self.state['aero_device_scale'],
           'limit': 0.0,
           'lim_type': 'min',
           'margin': aero_device_scale_margin,
           'margin_norm': aero_device_scale_margin * 10.})

    if 'torque_shaft_max' in self.config:
      torque_shaft_margin = self.config['torque_shaft_max'] - abs(self.state['torque_shaft'])
      self.state['constraints'].append(
          {'name': 'torque_shaft_margin',
           'value': self.state['torque_shaft'],
           'limit': self.config['torque_shaft_max'],
           'lim_type': 'max',
           'margin': torque_shaft_margin,
           'margin_norm': (torque_shaft_margin
                           / self.config['torque_shaft_max'])})

    if 'aero_thrust_p_max' in self.config:
      aero_thrust_p = self.state['rotor_force'] * self.state['v_a']
      aero_thrust_p_margin = (
          self.config['aero_thrust_p_max'] - aero_thrust_p)
      self.state['constraints'].append(
          {'name': 'aero_thrust_p_max_margin',
           'value': aero_thrust_p,
           'limit': self.config['aero_thrust_p_max'],
           'lim_type': 'max',
           'margin': aero_thrust_p_margin,
           'margin_norm': (
              aero_thrust_p_margin / self.config['aero_thrust_p_max'])})

    if 'aero_thrust_p_min' in self.config:
      aero_thrust_p = self.state['rotor_force'] * self.state['v_a']
      aero_thrust_p_margin = (
          aero_thrust_p - self.config['aero_thrust_p_min'])
      self.state['constraints'].append(
          {'name': 'aero_thrust_p_min_margin',
           'value': aero_thrust_p,
           'limit': self.config['aero_thrust_p_min'],
           'lim_type': 'min',
           'margin': aero_thrust_p_margin,
           'margin_norm': (
              aero_thrust_p_margin / abs(self.config['aero_thrust_p_min']))})

    # Determine validity of pose based on specified normalized constraint
    # tolerance.
    for constraint in self.state['constraints']:
      if constraint['margin_norm'] < - self.solve_options['contol_norm']:
        self.state['constraints_violated'].append(constraint)
        self.state['valid'] = False

    self.valid = self.state['valid']

    if self.verbose:
      if not self.valid:
        print('Pose is not valid. Constraints violated are: ')
        print(self.state['constraints_violated'])

  def _apply_opt_penalties(
      self, constraint_mult, tension_mult, flap_m_mult, flap_penalty_factor):

    penalty = 0.

    if constraint_mult != 0.:
      for constraint in self.state['constraints']:
        if constraint['margin_norm'] < 0:
          penalty += constraint['margin_norm']**4 * constraint_mult

    if tension_mult != 0.:
      if (round(self.state['power_shaft'])
          >= round(self.config['power_shaft_max'])):
        penalty += (
            abs(self.state['tension']) / self.config['tension_max']
            * -tension_mult)

    if any([f!=0. for f in flap_m_mult]):
      # Scale the penalty as if it was a drag power, which scales with v_a**3.
      aero_factor = self.resource['rho']/2. * self.state['v_a']**3
      penalty -= (
          flap_penalty_factor *
          np.sum(
            np.multiply(aero_factor,
              np.multiply(flap_m_mult, self.state['flap_aero_coeffs'])**2)))

    self.state['power'] += penalty

  def _calc_power_padmount(self):
    self.state['eta_shaft_to_pad'] = (
        self.config['eta_shaft_to_pad'](self.state['power_shaft']))

    if self.state['power_shaft'] > 0.:
      self.state['power'] = (self.state['power_shaft']
                             * self.state['eta_shaft_to_pad'])
    else:
      self.state['power'] = (self.state['power_shaft']
                             / self.state['eta_shaft_to_pad'])

  def _calc_shaft_power(self, state=None):
    # Setup
    if state is None:
      state = self.state

    # Only use portion of airspeed along rotor axis.
    # Edgewise airspeed effects are ignored.
    state['v_a_along_rotor_axis'] = np.dot(
        -state['rotor_thrust_axis_g'], state['v_a_vec'])

    power_update = self.config['shaft_power_from_drag_power'](
        self.resource['rho'], self.resource['c_sound'],
        state['v_a_along_rotor_axis'],
        state['rotor_force'])

    if 'constraints' not in power_update:
      # Assume that if the rotor is not returning constraints, it is a dumb
      # model that is not checking for local betz limits, so do that check here.
      betz_limit = (
          (16./27.) * 0.5 * self.resource['rho'] * self.config['rotor_area']
          * state['v_a']**3)
      betz_margin = betz_limit - power_update['power_shaft']

      power_update['constraints'] = [{'name': 'betz_margin',
                                      'value': power_update['power_shaft'],
                                      'limit': betz_limit,
                                      'lim_type': 'max',
                                      'margin': betz_margin,
                                      'margin_norm': betz_margin / betz_limit}]

    # Move constraint over to state constraints.
    state['constraints'].extend(power_update.pop('constraints'))
    state.update(power_update)

  def _calc_lift_roll_angle(self, state=None):
    """
    Points the lift to counteract forces perpendicular to v_a-e_tether_rad
    plant (fly y axis).

    Starts with a with a provided state, which must have C_L, C_Y, and C_D.
    If not specified, is self.state.

    State must already be populated with all forces that have a component
    perpendicular to plane defined by fly y-axis, in forces_g['type']

    Fills state provided with complete balanced forces in state['forces_g'].
    """

    print('Warning: Solving for lift roll angle has not been updated for '
      + 'defined rotor axis. Results will have residual and may not solve.')

    # TODO: Revise for specified rotor thrust axis.

    if state is None:
      state = self.state

    lift_total = math.sqrt(state['lift']**2 + state['side_lift']**2)
    lift_angle = math.atan2(state['cY'], state['cL'])

    # Calc forces perpendicular to the fly frame xz axes, which is the plane
    # defined by v_a and straight line tether.
    initial_force_along_f_y_vec = (
        sum_forces_along_axis(
            self.state['forces_g']['type'], self.state['DCMg2f'][1]))
    initial_force_along_f_y = (
        np.dot(initial_force_along_f_y_vec,
               self.state['DCMg2f'][1]))

    if lift_total < abs(initial_force_along_f_y) or lift_total == 0.:
      # If lift is not enough to meet required perp force, point all the lift
      # the kite has to resist it. This will not be a valid solution and will
      # show up in constraints as a non-zero net force.
      net_lift_roll_angle = math.pi / 2.
    else:
      net_lift_roll_angle = math.asin(
          initial_force_along_f_y / lift_total)

    state['lift_roll_angle'] = -(net_lift_roll_angle + lift_angle)

  def _force_bal(self, state=None):
    """
    Solves the force balance for a known C_L, C_Y, and C_D, with orientation
    also specified.

    Starts with a with a provided state, which must have C_L, C_Y, and C_D.
    If state is not specified, it is self.state.
    Fills state provided with complete balanced forces in
    state['forces_g']['type'].
    """

    if state is None:
      state = self.state

    # Calc inertial forces.
    # Acceleration forces first:
    kite_force = self.config['m_kite'] * state['accel_along_path']

    # Inertial forces, so they are in the opposite direction (sign) of
    # the acceleration.
    F_inertial_path_accel = (
        kite_force * -state['e_path_tangent'])
    # Centrifugal forces on kite and tether.
    state['r_curv_vec'] = state['r_curv'] * state['r_curv_hat']
    F_inertial_centrifugal_accel = (
        (self.config['m_kite'] * state['v_k']**2)
         / state['r_curv'] * -state['r_curv_hat'])
    state['forces_g']['type']['inertial'] = (
      F_inertial_centrifugal_accel + F_inertial_path_accel)

    # Calculate  gravity forces.
    weight = utils.Const.G * self.config['m_kite'] * state['grav_mult']
    state['forces_g']['type']['gravity'] = np.array([0., 0., -weight])

    # Calculate tether catenary forces (perpendicular to e_rad).
    self._calc_tether_catenary_forces()
    # Sum all currently known portions of tension.
    state['forces_g']['type']['tether'] = (
        state['forces_g']['parts']['tether_centrifugal_vec']
        + state['forces_g']['parts']['tether_eff_weight_vec']
        + state['forces_g']['parts']['tether_drag_vec']
        + state['forces_g']['parts']['tether_path_accel_inertial_vec'])

    # Calculate aero forces.
    state['lift'] = self._aero_force_from_coeff(state['cL'], state)
    state['side_lift'] = self._aero_force_from_coeff(state['cY'], state)
    state['drag_kite'] = self._aero_force_from_coeff(state['cD'], state)

    if 'lift_roll_angle' not in state:
      self._calc_lift_roll_angle(state)
      self._calc_orientation()

    # Forces along aero frame that is right hand coord sys with cL, cY, cD.
    state['forces_g']['type']['aero'] = (
      state['drag_kite'] * state['DCMg2a'][0]
      + state['side_lift'] * state['DCMg2a'][1]
      + state['lift'] * state['DCMg2a'][2])

    # Now that aero forces are calculated and known, we can calculate the
    # remaining portion of tension along the straight line tether.
    self._calc_tension_radial(state)
    # And then add it to the other tether forces.
    state['forces_g']['type']['tether'] += (
        state['forces_g']['parts']['tether_tension_rad_vec'])

    # Rotors balance all forces along rotor axis.
    state['forces_g']['type']['rotors'] = -(
      sum_forces_along_axis(state['forces_g']['type'],
                            self.state['rotor_thrust_axis_g']))

    state['forces_g']['type']['residual'] = sum_forces(state['forces_g']['type'])

  def _calc_tension_radial(self, state=None):
    """Fills provided state that contains aero and accel forces for kite and
    tether with radial component of tension that balances those forces."""
    if state is None:
      state = self.state

    # Sum all forces in fly (f) frame z, which is along unrolled lift. Ie:
    # Sum forces in tension_rad and rotor_axis plane that are perpendicular to
    # rotor_axis. Total rotor thrust is still unknown.

    # Calc forces perp to rotor axis.
    f_perp_rotors = utils.component_perp_axis(
        sum_forces(state['forces_g']['type']),
        self.state['rotor_thrust_axis_g'], unit_axis=True)

    # The magnitude of tension_rad is the inverse of this force scaled by how
    # much is reacted by the tether_rad. Ie, dot(e_tether_rad, f_perp_rotors).
    tension_rad = (
        -utils.vec_length(f_perp_rotors)
        / np.dot(state['e_tether_rad'], utils.unit_vec(f_perp_rotors)))
    state['forces_g']['parts']['tether_tension_rad_vec'] = (
        tension_rad * state['e_tether_rad'])

  def _calc_tether_angles(self, state=None):

    if state is None:
      state = self.state

    state['tether_roll_angle'] = utils.angle_signed(
        state['kite_axis']['z'],
        state['forces_g']['type']['tether'],
        state['kite_axis']['x'])
    state['tether_pitch_angle'] = utils.angle_signed(
        state['kite_axis']['z'],
        state['forces_g']['type']['tether'],
        state['kite_axis']['y'])

  def _moment_balance(self, state=None):
    """Calculates moments about body origin and stores them in
    state['moments_b']['type'].

    Note that body origin may not be co-incident with CG. Moments about CG are
    calculated in calc_summary_data for user."""

    if state is None:
      state = self.state

    # Calc aero moments.
    aero_moment_scale = np.multiply(
        0.5 * self.resource['rho'] * self.config['s'] * state['v_a']**2,
        np.array([self.config['b'], self.config['c'], self.config['b']]))
    aero_moment_coeffs = np.array([state['cl'], state['cm'], state['cn']])
    state['moments_b']['type']['aero'] = (
        np.multiply(aero_moment_coeffs, aero_moment_scale))

    # Calc inertial moments.
    M_inertial_angular_accel = (
        -np.dot(self.config['inertia'], state['pqr_dot']))
    # We need the forces in g frame put into body frame.
    f_inertial_b = np.dot(
        state['DCMg2b'], state['forces_g']['type']['inertial'])
    M_inertial_CG_accel = utils.cross(f_inertial_b, self.config['CG'])
    # Moment from rotation not about a principle axis of inertia.
    M_inertial_off_axis = -utils.cross(
        state['pqr'], np.dot(self.config['inertia'], state['pqr']))
    state['moments_b']['type']['inertial'] = (
        M_inertial_angular_accel + M_inertial_CG_accel + M_inertial_off_axis)

    # Calc gravity accel moment.
    f_grav_b = np.dot(
        state['DCMg2b'], state['forces_g']['type']['gravity'])
    state['moments_b']['type']['gravity'] = utils.cross(
        f_grav_b, self.config['CG'])

    # Calc rotor thrust moment.
    f_rotors_b = np.dot(
        state['DCMg2b'], state['forces_g']['type']['rotors'])
    state['moments_b']['type']['rotors'] = utils.cross(
        f_rotors_b, self.config['rotor_thrust_center'])

    # Calc bridle moments.
    self._calc_tether_angles(state)
    # Bridle_Mt is moment per unit tension.
    state['bridle_Mt'] = (
        self.config['bridle_moment_from_tether_pitch_roll'](
            state['tether_pitch_angle'], state['tether_roll_angle']))
    state['moments_b']['type']['tether'] = state['bridle_Mt'] * state['tension']

    # Calc moment residual.
    state['moments_b']['type']['residual'] = (
      state['moments_b']['type']['aero']
      + state['moments_b']['type']['inertial']
      + state['moments_b']['type']['gravity']
      + state['moments_b']['type']['rotors']
      + state['moments_b']['type']['tether'])

    # Put in terms of aero moment coefficient needed to balance residuals.
    state['flap_aero_coeffs'] = np.multiply(
        -state['moments_b']['type']['residual'],
        1. / aero_moment_scale)


  def gen_all_plot_data(self, no_show=None):
    plot_list = self.gen_force_type_data(no_show)

    no_show = ['...', 'straight line tether']

    if 'kite_axis' in self.state:
      plot_list.extend(
          self._plot_vec_data({}, self.state['kite_axis'],
                              10., 1, ' Kite coordinate sys', no_show))

    plot_list.extend(self.gen_speeds_plot_data(no_show))

    return plot_list

  def gen_force_plot_data(self, no_show=None):
    scale = 1./1500.

    colors = {'tether_tension_rad_vec': 'moccasin',
              'tether_centrifugal_vec': 'orchid',
              'tether_drag_vec': 'paleturquoise',
              'tether_eff_weight_vec': 'lightpink',
              'lift_vec':'blue',
              'drag_kite_vec':'darkslateblue',
              'centrifugal_vec':'purple',
              'weight_vec':'pink',
              'rotors_force_vec':'green',
              'v_k_accel_vec':'grey',
              'straight line tether':'#D0D3D4'}

    plot_list = self._plot_vec_data(
        colors, self.state['forces_g']['type'], scale, 0.001, 'kN', no_show)
    return plot_list

  def gen_force_type_data(self, no_show=None):
    scale = 1./1500.

    colors = {'gravity': 'moccasin',
              'aero': 'orchid',
              'tether': 'paleturquoise',
              'rotors': 'lightpink',
              'inertial':'blue',
              'residual':'red',
              'straight line tether':'#D0D3D4'}

    plot_list = self._plot_vec_data(
        colors, self.state['forces_g']['type'], scale, 0.001, 'kN', no_show)
    return plot_list

  def gen_speeds_plot_data(self, no_show=None):
    scale = 1.

    colors = {'v_a_vec':'blue',
              'v_k_vec':'black',
              'v_w_vec':'#add8e6',
              'straight line tether':'#D0D3D4'}

    d = {'v_a_vec': self.state['v_a_vec'],
         'v_k_vec': self.state['v_k_vec'],
         'v_w_vec': self.state['v_w_vec']}

    plot_list = self._plot_vec_data(colors, d, scale, 1., 'm/s', no_show)

    return plot_list

  def gen_path_def_plot_data(self, no_show=None):
    scale = 1.

    colors = {'r_curv_vec': '#800000',
              'e_tether_rad': 'orange',
              'lift_no_roll_hat': 'black',
              'e_path_tangent': 'pink',
              'v_a_e_rad_plane_normal_hat': 'green',
              'straight line tether': '#D0D3D4'}

    d = {'r_curv_vec': self.state['r_curv_vec'],
         'lift_no_roll_hat': self.state['lift_no_roll_hat'] * 10.,
         'e_path_tangent': self.state['e_path_tangent'] * 10.,
         'v_a_e_rad_plane_normal_hat': (
              self.state['v_a_e_rad_plane_normal_hat'] * 10.),
         'e_tether_rad': self.state['e_tether_rad'] * 10.}

    plot_list = self._plot_vec_data(colors, d, scale, 1., 'm', no_show)

    if 'kite_axis' in self.state:
      plot_list.extend(
          self._plot_vec_data({}, self.state['kite_axis'],
                              10., 1, ' Kite coordinate sys', no_show))

    return plot_list

  def _plot_vec_data(self, colors, d, scale, mag_scale, mag_unit, no_show):
    # returns a dict of the pose data, to be used to make lots of plots
    plot_list = []
    pos = self.position['xyz']

    for key in sorted(d.keys()):
      if key not in no_show:

        mag = utils.vec_length(d[key]) * mag_scale
        plot_list.append(
            {'name':key,
             'type': 'scatter3d',
             'mode': 'lines+markers',
             'text' : [('Magnitude: %0.2f'%mag + mag_unit +' \n')] * 2,
             'x': [pos[i], pos[i]+d[key][i]*scale],
             'y': [pos[j], pos[j]+d[key][j]*scale],
             'z': [pos[k], pos[k]+d[key][k]*scale],
             'line':{'width': 4.},
             'marker': {'color':colors.get(key, ''),
                        'size':6.}})
    if 'straight line tether' not in no_show:
      plot_list.append(
          {'name':'straight line tether',
           'type': 'scatter3d',
           'mode': 'lines+markers',
           'text': [],
           'x': [self.gs_position[i], pos[i]],
           'y': [self.gs_position[j], pos[j]],
           'z': [self.gs_position[k], pos[k]],
           'marker': {'color':colors.get('straight line tether', ''),
                      'size':4.},
           })
    if '(...)' not in no_show:
      plot_list.append({'name':'(...)',
                        'type': 'scatter3d',
                        'mode': 'lines',
                        'text': ['bounding box for scaling'],
                        'x': [0, 0, 600],
                        'y': [-300, 300, 300],
                        'z': [0, 0, 600],
                        'line':{'width': 0.001}
                        })
    return plot_list

  def gen_plot_file(self, location='plots/plot_pose_forces.json',
                    plot_type='all', no_show=[]):

    lookup = {'all': self.gen_all_plot_data,
              'forces': self.gen_force_plot_data,
              'path': self.gen_path_def_plot_data,
              'speeds': self.gen_speeds_plot_data}

    # makes a file of the pose data, to be used for a single plot
    plot_list = lookup[plot_type](no_show=no_show)

    with open(location,'w') as outfile:
      json.dump(plot_list,outfile)
      print('File saved to %s.' % location)

