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

"""Playbook Helper Functions."""

import copy

from makani.control import control_types as m
import numpy as np
from scipy import interpolate


def _GenerateLookupTable(lookup_fn,
                         num_output_points=m.CROSSWIND_SCHEDULE_TABLE_LENGTH,
                         continuous=True):
  lookup_x = np.linspace(0, 2 * np.pi, num_output_points)
  lookup_y = lookup_fn(lookup_x)
  if continuous:
    assert abs(lookup_y[-1] - lookup_y[0]) < 1e-6
    lookup_y[-1] = lookup_y[0]
  return lookup_y.tolist()


def GeneratePeriodicSplineLookupTable(control_points):
  """Generate a lookup table based on a periodic spline in the FBL format.

  FBL splines are equally spaced around the FBL loop (going clockwise with zero
  at 12 o'clock).

  Args:
    control_points: List of points defining the spline.
  Returns:
    A lookup table from 0 to 2 pi as a list.
  """
  # The periodic cubic spline is built from the control points, reusing the
  # last point at the start to close out the period.
  # The control points are positioned in the center of equally divided segments
  # of the loop, leading to a half segment offset, to match FBL convention.
  spline_x = (np.linspace(0., 1., len(control_points)+1, endpoint=True)
              - 0.5 / len(control_points))
  spline_y = np.array([control_points[-1]] + control_points)
  spline_fit = interpolate.CubicSpline(spline_x, spline_y, bc_type='periodic',
                                       extrapolate='periodic')
  # The x coordinates in the crosswind controller domain from
  # _GenerateLookupTable are transformed from the controller to FBL domain
  # before sampling.
  return _GenerateLookupTable(
      lambda x: spline_fit((-x + 1.5 * np.pi) / (2.0 * np.pi)))


def GenerateSinusoidalLookupTable(mean, amplitude, phase):
  return _GenerateLookupTable(lambda x: mean + amplitude * np.cos(x - phase))


def ConvolveSmooth(array, weights, edges=None, allow_scaling=False):
  """Generate a smoothed array from input array.

  Outputs an n-dimensional array by convolution of
  n-dimensional weights over array.

  Args:
    array: Array-like structure to be smoothed
    weights: Array-like structure of weights for convolution.
      For example, [1.0, 1.0, 1.0] would smooth via a centered moving
      average over one dimensional input array.
    edges: A list of edge treatment options for each dimension.
      End of each dimension is treated the same.
      Inputs can be:
        'extend': Default option. Repeats end points, effectively
          weighting edges more than middle.
        'continuous': Tiles input array in that dimension.
          Assumes overlap, ie: dim[0] = dim[-1]
        'cutoff': Ignores out of index points.
          Rescales weights to keep total weight constant.
    allow_scaling: A boolean if scaling via weights is allowed.
      If false, values in weights are normalized such that sum(weights) == 1.
      If true, output is scaled by sum(weights).

  Returns:
      A new array representing convolution of array with weights.
  """

  # Initialize variables.
  a = np.array(array)
  w = np.array(weights)
  a_out = np.zeros(a.shape)

  # Weights must be centered, so dimensions must be odd.
  for dim in w.shape:
    assert dim % 2 != 0, 'Each dimension of weights must be odd'

  # Weights can scale the output if sum weights != 1
  w_scaling_factor = np.sum(w)

  # If scaling is not allowed, normalize weights.
  if not allow_scaling:
    w /= w_scaling_factor
    w_scaling_factor = np.sum(w)

  # Find indices for middle of weights.
  w_middle_indices = [(dim - 1) / 2 for dim in w.shape]

  # If edges are not specified, set default to 'extend'.
  if edges is None:
    edges = ['extend'] * len(a.shape)

  # Check that number of edges matches number of dimensions in array.
  assert len(edges) == len(a.shape), (
      'Number of edges must match number of dimensions of array')

  # Iterate through list of all indices in input array
  for a_indices in np.ndindex(*a.shape):
    # Sum of weights is used to adjust if some weights are cutoff.
    w_sum = 0.
    # Iterate through all indices in weights
    for w_indices in np.ndindex(*w.shape):
      # Make adjusted indices offset by location in weights from middle.
      a_adj_indices = (
          np.array(a_indices)
          + np.array(w_indices)
          - np.array(w_middle_indices))

      # Reset cut_off flag
      cut_off = False
      # For each dimension, further adjust indices based on edge options.
      for e, edge in enumerate(edges):
        if edge == 'cutoff':
          # If outside array, set cut_off flag to True.
          if a_adj_indices[e] < 0 or a_adj_indices[e] > a.shape[e] - 1:
            cut_off = True
        elif edge == 'continuous':
          # If outside array, adjust indices to wrap around.
          # Account for overlap of end points in offset.
          if a_adj_indices[e] >= a.shape[e] - 1:
            a_adj_indices[e] -= a.shape[e] - 1
          elif a_adj_indices[e] < 0:
            a_adj_indices[e] -= 1
        elif edge == 'extend':
          # If outside array, clip indices to min and max for dimension.
          a_adj_indices[e] = np.clip(a_adj_indices[e], 0, (a.shape[e] -1))
        else:
          assert False, 'Edge type is invalid. See docstring for details.'

      if not cut_off:
        # Keep track of sum of weights used for cutoff adjustment.
        w_sum += w[tuple(w_indices)]
        # Add value in array at adjusted indices times weight to output
        a_out[tuple(a_indices)] += a[tuple(a_adj_indices)] * w[tuple(w_indices)]
    # Adjust output value if weights was cutoff.
    # If weights was not cutoff, w_sum == w_scaling_factor,
    # and no adjustment will occur.
    a_out[tuple(a_indices)] *= w_scaling_factor / w_sum
  return a_out.tolist()


def PlaybookEntries2Dict(playbook):
  """Converts a playbook from list of dicts to dict of lists.

  Compiles all the values from each entry together.

  Args:
      playbook: A playbook in lookup table format.

  Returns:
      A playbook in dict of lists format.
  """

  output = {}
  for entry in playbook['entries']:
    for key, values in entry.iteritems():
      if key != 'lookup_loop_angle':
        if key not in output:
          output[key] = []
        output[key].append(values)

  # check if all lookup_loop_angles are the same, then copy
  for ii in range(len(playbook['entries']) - 1):
    assert (playbook['entries'][ii]['lookup_loop_angle']
            == playbook['entries'][ii-1]['lookup_loop_angle'])
  output['lookup_loop_angle'] = playbook['entries'][0]['lookup_loop_angle']

  return output


# pylint: disable=g-doc-return-or-yield
def MakePlaybook(playbook, adjust_wind_params=None):
  """Generate a playbook schedule based on the playbook definition provided.

  Each playbook solution contains the following fields:
    wind_speed: [m/s] The wind speed the solution was found at.
    alpha: [rad] A list of spline points defining the alpha schedule around the
        loop.
    beta: [rad] A list of spline points defining the beta schedule around the
        loop.
    airspeed: [m/s] A list of spline points defining the airspeed schedule
        around the loop.
    transout_airspeed: [m/s]  A list of spline points defining the airspeed
    schedule around the loop in PrepTransOut.
    radius: [m] The path radius at that solution.
    elevation: [rad] The loop elevation of that solution.
    azi_offset: [rad] The azimuth offset from downwind of the solution.

  Args:
    playbook: A list of playbook solutions at different wind speeds, in order
        of ascending wind speed.
    adjust_wind_params: A dictionary of parameters to adjust wind speed.
      Params are:
        'min_offset': sets the minimum allowed offset
        'crossover': point where adjusted wind = input wind
        'scaling_factor': multipler applied to wind, centered at crossover

  Returns: A playbook structure with lookup tables for use in the controller.
  """
  # pylint: enable=g-doc-return-or-yield

  # This function is intended to allow for a solution conservative at both
  # high and low winds by compressing the wind speed lookup around the
  # crossover point.
  def _AdjustWindSpeed(wind_speed, min_offset, crossover, scaling_factor):
    """Adjusts wind speed."""

    adjusted_wind = (
        max(wind_speed + min_offset,
            crossover + (wind_speed - crossover) * scaling_factor))

    return adjusted_wind

  # Default is no scaling
  if adjust_wind_params is None:
    adjust_wind_params = {
        'min_offset': 0.0,
        'crossover': 0.0,
        'scaling_factor': 1.0}

  configs = []
  last_wind_speed = 0.0
  for params in playbook:
    assert params['wind_speed'] > last_wind_speed
    configs.append({
        'wind_speed': _AdjustWindSpeed(params['wind_speed'],
                                       **adjust_wind_params),
        'alpha_lookup': GeneratePeriodicSplineLookupTable(params['alpha']),
        'beta_lookup': GeneratePeriodicSplineLookupTable(params['beta']),
        'airspeed_lookup':
            GeneratePeriodicSplineLookupTable(params['airspeed']),
        'transout_airspeed_lookup':
            GeneratePeriodicSplineLookupTable(params['transout_airspeed']),
        'path_radius_target': params['radius'],
        'elevation': params['elevation'],
        'azi_offset': params['azi_offset'],
        'lookup_loop_angle': [
            2.0 * np.pi * i / (m.CROSSWIND_SCHEDULE_TABLE_LENGTH - 1)
            for i in range(m.CROSSWIND_SCHEDULE_TABLE_LENGTH)],
    })
    last_wind_speed = params['wind_speed']

  return {'num_entries': len(playbook),
          'entries': configs}


def MakeFallbackPlaybook(params):
  num_control_points = 6
  return {
      'wind_speed': 1.0,
      'alpha_lookup': GeneratePeriodicSplineLookupTable(
          [params['alpha_cmd']] * num_control_points),
      'beta_lookup': GeneratePeriodicSplineLookupTable(
          [params['beta_cmd']] * num_control_points),
      'airspeed_lookup': GenerateSinusoidalLookupTable(
          params['airspeed_mean'],
          params['airspeed_variation_0'] +
          params['airspeed_mean'] * params['airspeed_variation_slope'],
          params['airspeed_variation_loop_angle_offset']),
      'transout_airspeed_lookup': GenerateSinusoidalLookupTable(
          params['airspeed_mean'],
          params['airspeed_variation_0'] +
          params['airspeed_mean'] * params['airspeed_variation_slope'],
          params['airspeed_variation_loop_angle_offset']),
      'path_radius_target': params['path_radius_target'],
      'elevation': params['elevation'],
      'azi_offset': params['azi_offset'],
      'lookup_loop_angle': [
          2.0 * np.pi * i / (m.CROSSWIND_SCHEDULE_TABLE_LENGTH - 1)
          for i in range(m.CROSSWIND_SCHEDULE_TABLE_LENGTH)],
  }


def SmoothPlaybook(playbook, smoothing_weights):
  """Smooth playbook using smoothing weights.

  Args:
    playbook: A playbook dictionary in lookup table format
    smoothing_weights: A dictionary with the following format:
      {<key>: {'weights': Array of weights to apply to ConvolveSmooth,
               'edges': List of edge treatment options for each dimension}
      <key> is the values in the playbook to apply smoothing options to

  Returns: A new playbook dictionary structure with smoothed values.

  """
  # change into format that is easier to apply smoothing to
  pb_dict = PlaybookEntries2Dict(playbook)

  smoothed_vars = {}
  for key, smooth_entry in smoothing_weights.iteritems():
    smoothed_vars[key] = ConvolveSmooth(
        pb_dict[key], smooth_entry['weights'],
        edges=smooth_entry['edges'])

  # place smoothed values into playbook
  playbook_output = copy.deepcopy(playbook)
  for key, values in smoothed_vars.iteritems():
    for ii, entry in enumerate(values):
      playbook_output['entries'][ii][key] = entry

  return playbook_output


def FillPlaybook(playbook):
  """Repeat the last entry of the playbook to fill the reserved memory space."""
  while len(playbook['entries']) < m.NUM_PLAYBOOK_ENTRIES:
    playbook['entries'].append(playbook['entries'][-1])
  return playbook
