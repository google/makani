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

import numpy as np
import math
import copy
from matplotlib import pyplot as plt

from lib import utils
from six.moves import range
from six.moves import zip

i = 0
j = 1
k = 2


def set_vars_to_opt(params, vars_to_opt, key, var_type, values, max_step,
                    default=None):
  if key not in params:
    params[key] = values if default is None else default
    if key not in vars_to_opt:
      vars_to_opt[key] = (
          {'param_type': var_type,
           'values': values,
           'max_step': max_step})


def leading_segment_len_2d(xys):
  """Compute the lengths of segments following each each 2D point."""
  segment_lens_2d = []
  num_points = len(xys)
  for ii in range(num_points):
    next_index = (ii + 1) % num_points
    next_p = np.array([xys[next_index][0], xys[next_index][1]])
    p_diff = next_p - xys[ii]
    segment_len_2d = np.linalg.norm(p_diff)
    segment_lens_2d.append(segment_len_2d)
  return segment_lens_2d


def centered_segment_len_2d(xys):
  """Compute the lengths of segments centered at each 2D point."""
  segment_lens = leading_segment_len_2d(xys)
  segment_lens.append(segment_lens[0])
  return [0.5 * (segment_lens[i] + segment_lens[i + 1])
          for i in range(len(segment_lens) - 1)]


def map_2d_path_to_sphere(xys, l_tether, azi, gs_position, **kwargs):
  """Convert 2d path to tether sphere.

  Assuming xys centroid is at [0, 0] in crosswind plane.
  Only and only one of 'incl' and 'h_min' has to be specified in kwargs.
  """
  centered_seg_lengths = np.array(centered_segment_len_2d(xys))
  total_dist = np.sum(centered_seg_lengths)

  mean_path_radius = np.sum(
    np.linalg.norm(xys, axis=1) * centered_seg_lengths / total_dist)

  # The least distortion occurs when the mean path radius is on the tether
  # sphere. Determine the x_g location that does this.
  if mean_path_radius < l_tether:
    dist_mean_radius = math.sqrt(l_tether**2 - mean_path_radius**2)
  # Need to handle mean_path_radius > l_tether gracefully for optimizer.
  else:
    dist_mean_radius = 0.

  # Map from 2d to 3d coordinates
  xyzs = np.array([[dist_mean_radius, -x, y] for x, y in xys])
  norm = np.linalg.norm(xyzs, axis=1)
  xyzs *= np.expand_dims(l_tether / norm, 1).repeat(3, axis=1)

  # determine inclination from min height
  xs, ys, zs = list(zip(*xyzs.tolist()))
  z_min = min(zs)

  if kwargs.get('incl', None) is None:
      assert 'h_min' in kwargs
      incl = incl_from_h_min(kwargs['h_min'], gs_position[2], l_tether, z_min)
  else:
      assert kwargs.get('h_min', None) is None
      incl = kwargs['incl']

  # rotate positions by incl and azim
  xyzs_out = []
  rot_matrix_incl = utils.axis_angle_rotation(np.array([0., -1., 0.]), incl)
  rot_matrix_azim = utils.axis_angle_rotation(np.array([0., 0., 1.]), azi)

  for xyz in xyzs:
    xyz = np.dot(rot_matrix_incl, xyz)
    xyz = np.dot(rot_matrix_azim, xyz)
    xyz += gs_position
    xyzs_out.append(xyz)

  return xyzs_out


def _recenter_path_2d(cartesian):
    # Recenter the path
    cartesian = np.array(cartesian)
    segments = centered_segment_len_2d(cartesian)
    total_len = np.sum(segments)
    # Convert segments to weights (N x 2) and make it as two identical columns.
    weights = np.repeat(np.expand_dims(segments / total_len, 1), 2, axis=1)
    center = np.sum(weights * cartesian, axis=0)
    cartesian -= center
    return cartesian


def hermite(points, dxy_ds, closed=True, num_pts_per_edge=50):
    points = copy.copy(points)
    output_points = []

    if closed:
        points.append(points[0])
        dxy_ds.append(dxy_ds[0])

    h = np.array(
        [[2, -2, 1, 1],
         [-3, 3, -2, -1],
         [0, 0, 1, 0],
         [1, 0, 0, 0]])
    s = np.linspace(0., 1., num_pts_per_edge + 1)
    S = np.array([s**3,
                  s**2,
                  s,
                  np.ones(len(s))])

    for ii in range(len(points)-1):
        x1 = points[ii][0]
        y1 = points[ii][1]
        x2 = points[ii+1][0]
        y2 = points[ii+1][1]

        x1_slope = dxy_ds[ii][0]
        y1_slope = dxy_ds[ii][1]
        x2_slope = dxy_ds[ii+1][0]
        y2_slope = dxy_ds[ii+1][1]

        C = np.array([[x1, y1],
                      [x2, y2],
                      [x1_slope, y1_slope],
                      [x2_slope, y2_slope]])

        p = np.matmul(np.matmul(S.T, h), C)

        output_points.extend(p[:-1])
    return output_points


def ortho_hermite(width_r, height_r, top_r, bottom_r,
               left_y, right_y, top_x, bottom_x,
               d_middle_left, d_top_left, d_top_right,
               d_middle_right, d_bottom_right, d_bottom_left, num_pts):
  points = [
      (-width_r, left_y),
      (top_x - top_r, height_r),
      (top_x + top_r, height_r),
      (width_r, right_y),
      (bottom_x + bottom_r, -height_r),
      (bottom_x - bottom_r, -height_r),
  ]
  dxy_ds = [
      (0, d_middle_left),
      (d_top_left, 0),
      (d_top_right, 0),
      (0, -d_middle_right),
      (-d_bottom_right, 0),
      (-d_bottom_left, 0),
  ]
  assert num_pts % 6 == 0
  return hermite(points, dxy_ds, num_pts_per_edge=(num_pts // 6))


def f8_hermite(slope, lobe_d, sharpness, centroid_sharpness_ratio, asymmetry,
    keystone_angle, tilt, num_pts):
  """Returns a 2D path of a 'figure-8' shape via a hermite spline.

  It uses 3 colinear control points: the center (used twice), and the outer
  edge of each lobe; and the slope at each point. Default direction creates
  a gentle downstroke, but can be switched by tilting pi radians.

  Args (all angles in radians):
    slope: Targent angle at the centroid, relative to horizontal (for 0 tilt).
    lobe_d: Approximate diameter of each lobe; nominal distance from the
      centroid to the outer lobe side.
    sharpness: The baseline strength of the tangency at the control points,
      relative to loop_d (tangency is sharpness*lobe_d).
      This is the main driver of aspect ratio, or how 'tall' the lobes are.
    centroid_sharpness_ratio: How much to emphasize the
      tangency at the centroid vs that at the lobe. 1 is balanced.
    asymmetry: Relative amount to shift the lobe edges left or right, relative
      to the centroid. 0 is balanced and 0.5 would make the right lobe_d
      3x as large as the left. The relative tangency at each lobe is also
      adjusted.
    keystone_angle: How much to adjust the slope of the tangency at the outer
      lobe edge. Positive values make the upper corners sharper and flatten out
      the bottoms.
    tilt: Overall rotation of all points and angles. Positive is CCW.
    num_pts: Number of positions to return. Must be multiple of 4.
  """

  # Setup base control points array
  points = np.array([
            [0., 0.],
            [lobe_d * (1 + asymmetry), 0],
            [0., 0.],
            [-lobe_d * (1 - asymmetry), 0]
                     ])
  # Rotate by the tilt.
  # This is slightly unconventional way of setting up the rotation matrix
  # multiplication since applying to an array of x,y pairs, so it needs
  # the negative sign to work the way you would expect.
  points = np.matmul(points, np.array([
            [np.cos(-tilt), -np.sin(-tilt)],
            [np.sin(-tilt), np.cos(-tilt)]
                     ]))

  # Setup base control tangency angles.
  angles = np.array([
      -slope,
      np.pi/2 - keystone_angle,
      np.pi + slope,
      np.pi/2 + keystone_angle,
                    ]) + tilt
  # Break angles into components and multiply by the weightings.
  dxy_ds = np.array([np.cos(angles), np.sin(angles)]).T * np.array([
      [centroid_sharpness_ratio * sharpness * lobe_d],
      [(1 / centroid_sharpness_ratio) * sharpness * lobe_d * (1 + asymmetry)],
      [centroid_sharpness_ratio * sharpness * lobe_d],
      [(1 / centroid_sharpness_ratio) * sharpness * lobe_d * (1 - asymmetry)]
                                  ])

  assert num_pts % 4 == 0
  return hermite([tuple(xy_pair) for xy_pair in points],
                 [tuple(xy_pair) for xy_pair in dxy_ds],
                 num_pts_per_edge=(num_pts // 4))


def radius_offsets(base_radius, offset_ratios):
  """Returns a 2d path with origin at centroid. Path defined as base_radius
  with radius multipliers specified in offset_ratios for each position.

  Postions are are evenly distributed by loop angle.
  Number of positions is specified by length of offset_ratios.
  """
  num_points = len(offset_ratios)
  theta = np.linspace(np.pi * 0.5, -np.pi * 1.5, num=num_points + 1)[:-1]
  radius = base_radius * (np.array(offset_ratios) + 1.0)
  cartesian = np.array([np.cos(theta) * radius, np.sin(theta) * radius]).T
  return _recenter_path_2d(cartesian)


def fourier_path(base_radius, harmonics, num_points):
  """Returns a 2d path with origin at centroid for specified base radius and
  harmonics.

  Harmonics are specified by list of amplitude and phase offset pairs.
  """
  dir_angles = np.linspace(0.5 * np.pi, -np.pi * 1.5, num_points + 1)[:-1]
  rs = np.zeros(num_points)

  for ii, theta in enumerate(dir_angles):
    # Specify the zero-th harmonic.
    r = base_radius

    for h, (amplitude, phase) in enumerate(harmonics):
      # Starts from the first harmonics.
      r += amplitude * np.cos((h + 1) * theta + phase)
    rs[ii] = r
  cartesian = np.array([np.cos(dir_angles) * rs, np.sin(dir_angles) * rs]).T

  return _recenter_path_2d(cartesian)


def dir_harmonics(base_radius, curvature_harmonics, num_points):
    """Parameterize the 2D path with harmonics over direction of travel."""

    # See http://go/makani-travel-harmonically.
    dir_angles = np.linspace(0.5 * np.pi, -np.pi * 1.5, num_points + 1)[:-1]
    # Set radii to the baseline radius.
    radii = base_radius
    # Add harmonic components to the radius at every dir angle.
    for i, (amp, phase) in enumerate(curvature_harmonics):
      h = i + 2
      amp /= h  # scale amplitude down for higher harmonics
      r2 = amp * np.cos(h*dir_angles + phase)
      radii += r2
    cartesian = []
    # Start anywhere and trace the path.
    x=0.
    y=0.
    cartesian.append([x, y])
    for i, theta in enumerate(dir_angles[:-1]):
      theta = dir_angles[i]
      dtheta = dir_angles[i+1] - theta
      dl = dtheta * radii[i]
      x += dl*np.cos(theta)
      y += dl*np.sin(theta)
      cartesian.append([x, y])
    return _recenter_path_2d(cartesian)

class KitePath(object):
  """
  Container for position data that defines a flight path
  Can create positions for given params for a circular flight path,
  or be given a pre-prepared list of positions.

  Required:
    shape_params:
      dictionary that specifies path shape that must include:
        type:
          either 'circle', 'radius_offsets', 'fourier', 'dir_harmonics',
          'hermite', or 'ortho_hermite'.          .
        If type is circle:
          r_loop: radius of loop
        If type is radius_offsets:
          Parameterizes loop by a base radius, with radius offsets for each
          pose.
          Specified via:
            base_radius: Scalar for nominal radius.
            offset_ratios: List-like of base_radius offsets normalized by base
              radiusfor each pose.
          If being optimized, num_poses must be provided.
        If type is fourier:
          Parameterizes path using fourier coefficients in polar space.
          Specified via:
            base_radius: zero-th harmonic amplitude
            harmonics: list-like of amplitude and phase offset pairs
        If type is dir_harmonics:
          Parameterizes path specifying curvature along path using fourier
          coefficients.
          Specified via:
            base_radius: Zero-th harmonic amplitude
            harmonics: List-like of amplitude and phase offset pairs for
              2nd harmonic on up (1st is skipped).
        If type is ortho_hermite:
          Parameterizes path using an orthogonal hermite curve.
          See go/makani-ortho-hermite for more.
          Specificied via:
            See go link and code
          The num_pos must be a multiple of 6.
        If type is hermite:
          Parameterizes path using a hermite curve.
          Specificied via:
            points: list of (x, y) pairs that serve as control points.
            dxy_ds: list of (dx/ds, dy/ds) pairs that control the amount of
              tangency at each control point.
          The num_pos must be a multiple of the length of these lists.
          This method is not yet directly supported in optimization.

    location_params:
      Dictionary that must include:
        azim: Azimuth of path center, in radians unless specified otherwise.
        incl: Inclination of path center, in radians unless specified
          otherwise.
      Optional:
        incl_unit: Specifies units for inclination, either 'rad' or 'deg.'
        azim_unit: Specifies units for azimuth, either 'rad' or 'deg.'

  Optional:
    given_positions:
      A list of position data, already in the required format.
      Used to feed in a manually made set of positions, for example, from
      flight data or generated in another tool.
    Note:
      Summary path stats assume each position is weighted the same.

  Each position is a dictionary that must contain:
    'xyz': an array like list of [x, y, z] coordinates in ground frame
    'r_curv': the path radius of curvature at that position
    'r_curv_hat': array like unit vector pointing to the path instant center
      for that point
    'segment_length': scalar of length of this section of path
    'e_tether_rad': array like unit vector pointing to the tether ground
      attach point
    'e_path_tangent': array like unit vector of the path tangent at this point
  """

  def __init__(self, shape_params, location_params,
               config, given_positions=None):

    self.config = config
    self.gs_position = self.config.get('gs_position', np.array([0.,0.,0.]))

    self.given_positions = given_positions

    deg_rad_lookup = {'deg': lambda x: math.radians(x),
                      'rad': lambda x: x}

    if self.given_positions is None:
      self.incl_unit = location_params.get('incl_unit', 'rad')
      self.azim_unit = location_params.get('azim_unit', 'rad')

      self.incl = self.h_min = None

      if 'incl' in location_params:
        self.incl = deg_rad_lookup[self.incl_unit](location_params['incl'])
      elif 'h_lowerbound' in location_params:
        self.h_min = location_params['h_lowerbound']

      self.azim = deg_rad_lookup[self.azim_unit](location_params['azim'])

      self.shape_params = shape_params
      self._calc_positions()

    else:
      self.num_pos = len(self.given_positions)
      self.positions = self.given_positions
      self._calc_path_data()

      # Calc summary data.
      # Note that these assume a constant weighting per position.
      # TODO: Weight each by length of associated segment.
      self.l_tether = np.mean(
          [utils.vec_length(position['xyz']) for position in self.positions])
      self.r_loop = np.mean([position['r_curv'] for position in self.positions])

      self.incl = math.asin(self.h_hub / self.l_tether)
      self.azim = math.atan2(self.centroid[j], self.centroid[i])

  def __getitem__(self, key):
    return getattr(self, key)

  @staticmethod
  def setup_optimization(shape_params, location_params, vars_to_opt):
    if 'h_lowerbound' not in location_params:
      set_vars_to_opt(location_params,
                      vars_to_opt, 'incl', 'value', 0.4, 0.05)
    set_vars_to_opt(location_params,
                    vars_to_opt, 'azim', 'value', 0., -0.05)

    # If type is not provided, make the default a circle.
    shape_params['type'] = shape_params.get('type', 'circle')

    if shape_params['type'] == 'circle':
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'r_loop', 'value', 120, 20.0)
      if 'num_pos' not in shape_params:
        shape_params['num_pos'] = 18
    elif shape_params['type'] == 'radius_offsets':
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'base_radius', 'value', 120, 20.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'offset_ratios', 'spline', [0.0] * 6, 0.03)
      if 'num_pos' not in shape_params:
        shape_params['num_pos'] = 18
    elif shape_params['type'] == 'fourier':
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'base_radius', 'value', 120, 10.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'amp1', 'value', 0.0, 30.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'phase1', 'value', 0.0, 0.1)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'amp2', 'value', 0.0, 30.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'phase2', 'value', 0.0, 0.1)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'amp3', 'value', 0.0, 30.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'phase3', 'value', 0.0, 0.1)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'amp4', 'value', 0.0, 30.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'phase4', 'value', 0.0, 0.1)
    elif shape_params['type'] == 'dir_harmonics':
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'base_radius', 'value', 120, 10.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'amp2', 'value', 0.0, 30.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'phase2', 'value', 0.0, 0.1)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'amp3', 'value', 0.0, 30.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'phase3', 'value', 0.0, 0.1)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'amp4', 'value', 0.0, 30.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'phase4', 'value', 0.0, 0.1)
    elif shape_params['type'] == 'ortho_hermite':
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'width_r', 'value', 120, 20.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'height_r', 'value', 120, 10.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'top_r', 'value', 20.0, 20.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'bottom_r', 'value', 20.0, 20.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'left_y', 'value', -0, 10.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'right_y', 'value', 0, 10.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'top_x', 'value', 0, 10.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'bottom_x', 'value', 0, 10.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'd_middle_left', 'value', 150, 25.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'd_top_left', 'value', 150, 25.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'd_top_right', 'value', 150, 25.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'd_middle_right', 'value', 150, 25.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'd_bottom_right', 'value', 150, 25.0)
      set_vars_to_opt(shape_params,
                      vars_to_opt, 'd_bottom_left', 'value', 150, 25.0)
      if 'num_pos' not in shape_params:
        shape_params['num_pos'] = 36
    elif shape_params['type'] == 'f8_hermite':
      set_vars_to_opt(shape_params, vars_to_opt,
                      'slope', 'value', np.pi/4, np.pi/8)
      set_vars_to_opt(shape_params, vars_to_opt,
                      'lobe_d', 'value', 300., 50.)
      set_vars_to_opt(shape_params, vars_to_opt,
                      'sharpness', 'value', 1.5, 0.4)
      set_vars_to_opt(shape_params, vars_to_opt,
                      'centroid_sharpness_ratio', 'value', 1.0, 0.4)
      set_vars_to_opt(shape_params, vars_to_opt,
                      'asymmetry', 'value', 0., 0.2)
      set_vars_to_opt(shape_params, vars_to_opt,
                      'keystone_angle', 'value', 0.0, 0.4)
      set_vars_to_opt(shape_params, vars_to_opt,
                      'tilt', 'value', 0., 0.3)
      if 'num_pos' not in shape_params:
        shape_params['num_pos'] = 36
    elif shape_params['type'] == 'hermite':
      assert False, (
          'Hermite path type is not supported for direct use in optimization.')
    else:
      assert False, ('{} type is not supported'.format(shape_params['type']))

  def _calc_positions(self):
    self.positions = []

    if self.shape_params['type'] == 'circle':
      self.num_pos = self.shape_params.get('num_pos', 18)
      self.r_loop = self.shape_params['r_loop']

      thetas = np.linspace(0., 2 * np.pi, num=self.num_pos, endpoint=False)
      xys = [(self.r_loop * math.sin(theta), self.r_loop * math.cos(theta))
             for theta in thetas]

    elif self.shape_params['type'] == 'radius_offsets':
      self.offset_ratios = self.shape_params['offset_ratios']
      self.r_loop = self.shape_params['base_radius']
      self.num_pos = self.shape_params.get('num_pos', len(self.offset_ratios))
      xys = radius_offsets(self.r_loop, self.offset_ratios)

    elif self.shape_params['type'] == 'hermite':
      self.points = self.shape_params['points']
      self.dxy_ds = self.shape_params['dxy_ds']
      self.num_pos = self.shape_params.get('num_pos', 18)
      assert self.num_pos % len(self.points) == 0
      xys = hermite(self.points, self.dxy_ds,
                    num_pts_per_edge=self.num_pos//len(self.points))

    elif self.shape_params['type'] == 'ortho_hermite':
      self.width_r = self.shape_params['width_r']
      self.height_r = self.shape_params['height_r']
      self.top_r = self.shape_params['top_r']
      self.bottom_r = self.shape_params['bottom_r']

      self.left_y = self.shape_params['left_y']
      self.right_y = self.shape_params['right_y']
      self.top_x = self.shape_params['top_x']
      self.bottom_x = self.shape_params['bottom_x']
      self.d_middle_left = self.shape_params['d_middle_left']
      self.d_top_left = self.shape_params['d_top_left']
      self.d_top_right = self.shape_params['d_top_right']
      self.d_middle_right = self.shape_params['d_middle_right']
      self.d_bottom_right = self.shape_params['d_bottom_right']
      self.d_bottom_left = self.shape_params['d_bottom_left']
      self.num_pos = self.shape_params.get('num_pos', 36)

      xys = ortho_hermite(
           self.width_r, self.height_r, self.top_r, self.bottom_r,
           self.left_y, self.right_y, self.top_x, self.bottom_x,
           self.d_middle_left, self.d_top_left, self.d_top_right,
           self.d_middle_right, self.d_bottom_right, self.d_bottom_left,
           self.num_pos)

    elif self.shape_params['type'] == 'f8_hermite':
      self.slope = self.shape_params['slope']
      self.lobe_d = self.shape_params['lobe_d']
      self.sharpness = self.shape_params['sharpness']
      self.centroid_sharpness_ratio = self.shape_params['centroid_sharpness_ratio']
      self.asymmetry = self.shape_params['asymmetry']
      self.keystone_angle = self.shape_params['keystone_angle']
      self.tilt = self.shape_params['tilt']
      self.num_pos = self.shape_params.get('num_pos', 36)

      xys = f8_hermite(
           self.slope, self.lobe_d, self.sharpness,
           self.centroid_sharpness_ratio, self.asymmetry,
           self.keystone_angle, self.tilt, self.num_pos)

    elif self.shape_params['type'] == 'dir_harmonics':
      if 'harmonics' in self.shape_params:
        self.curvature_harmonics = self.shape_params['harmonics']
      else:
        # TODO: Add a wrapper for the ParamArg to generalize it.
        self.curvature_harmonics = [
            (self.shape_params['amp2'], self.shape_params['phase2']),
            (self.shape_params['amp3'], self.shape_params['phase3']),
            (self.shape_params['amp4'], self.shape_params['phase4']),
        ]
      self.base_radius = self.shape_params['base_radius']
      self.num_pos = self.shape_params.get('num_pos', 18)
      xys = dir_harmonics(self.base_radius, self.curvature_harmonics, self.num_pos)

    elif self.shape_params['type'] == 'fourier':
      if 'harmonics' in self.shape_params:
        self.curvature_harmonics = self.shape_params['harmonics']
      else:
        self.curvature_harmonics = [
            (self.shape_params['amp1'], self.shape_params['phase1']),
            (self.shape_params['amp2'], self.shape_params['phase2']),
            (self.shape_params['amp3'], self.shape_params['phase3']),
            (self.shape_params['amp4'], self.shape_params['phase4']),
        ]
      self.base_radius = self.shape_params['base_radius']
      self.num_pos = self.shape_params.get('num_pos', 18)
      xys = fourier_path(self.base_radius, self.curvature_harmonics, self.num_pos)

    else:
      assert False, (
        'Path type not supported: {}'.format(self.shape_params['type']))

    xyzs = map_2d_path_to_sphere(
        xys, self.config['l_tether'], self.azim, self.gs_position,
        incl=self.incl, h_min=self.h_min)
    self.xys = xys
    self.positions = self.calc_pos_data_from_xyzs(
        xyzs, gs_position=self.gs_position)
    self._calc_path_data()

  @staticmethod
  def calc_pos_data_from_xyzs(xyzs, gs_position=[0., 0., 0.]):
    """Compute path-related states for KitePose given points around the path."""
    gs_position = np.array(gs_position)
    positions = []

    for ii, xyz in enumerate(xyzs):
        prev_index = ii - 1 if ii > 0 else (len(xyzs) - 1)
        next_index = ii + 1 if ii < (len(xyzs) - 1) else 0

        xyz_prev = xyzs[prev_index]
        xyz_next = xyzs[next_index]

        prev_length_vec = xyz_prev - xyz
        next_length_vec = xyz - xyz_next

        len_A = np.linalg.norm(prev_length_vec)
        len_B = np.linalg.norm(next_length_vec)
        len_C = np.linalg.norm(xyz_prev - xyz_next)

        plane_normal = np.cross(prev_length_vec, next_length_vec)

        l_ab = np.linalg.norm(xyz - xyz_prev)
        l_bc = np.linalg.norm(xyz_next - xyz)
        slope_ab = (xyz - xyz_prev) / l_ab
        slope_bc = (xyz_next - xyz) / l_bc
        e_path_tang = utils.slope_linear(
            slope_ab, slope_bc, l_bc, l_ab)

        r_vec = np.cross(plane_normal, e_path_tang)
        r_hat = r_vec / np.linalg.norm(r_vec)

        p = (len_A + len_B + len_C)/2.
        area = math.sqrt(p*(p-len_A)*(p-len_B)*(p-len_C))

        r_curve = (len_A * len_B * len_C)/(4.*area)

        xyz_gs_coord = xyz - gs_position

        e_tether_rad = -xyz_gs_coord / np.linalg.norm(xyz)

        segment_len = len_B

        incl = math.atan2(
            xyz_gs_coord[k], math.sqrt(xyz_gs_coord[i]**2 + xyz_gs_coord[j]**2))
        azim = math.atan2(xyz_gs_coord[j], xyz_gs_coord[i])

        e_tether_incl_base = np.array([0., 0., -1.])
        rm_incl = utils.axis_angle_rotation(np.array([0., -1., 0.]), incl)
        rm_azim = utils.axis_angle_rotation(np.array([0., 0., 1.]), azim)
        e_tether_incl = np.dot(rm_azim, np.dot(rm_incl, e_tether_incl_base))

        position = {'xyz': xyz,
                    'r_curv': r_curve,
                    'r_curv_hat': r_hat,
                    'segment_length': segment_len,
                    'e_tether_rad': e_tether_rad,
                    'e_path_tangent': utils.unit_vec(e_path_tang),
                    'e_tether_incl': e_tether_incl,
                    'incl': incl,
                    'azim': azim}

        positions.append(position)
    return positions

  def _rotate_pos(self, position, axis, angle):
    rot = utils.axis_angle_rotation(axis, angle)

    position['xyz'] -= self.gs_position
    position['xyz'] = np.dot(rot, position['xyz'])
    position['xyz'] += self.gs_position

    position['r_curv_hat'] = np.dot(rot, position['r_curv_hat'])
    position['e_tether_rad'] = np.dot(rot, position['e_tether_rad'])
    position['e_path_tangent'] = np.dot(rot, position['e_path_tangent'])

  def _calc_path_data(self):
    """Updates positions with information that is useful to user, but not
    required for solution"""

    # Calc centroid.
    self.centroid = self._calc_centroid(self.positions)

    # Calc crosswind xy coord.
    self._calc_crosswind_data(self.centroid, self.gs_position, self.positions)

    self.h_hub = self.centroid[k]
    self.h_max = max([position['xyz'][k] for position in self.positions])
    self.h_min = min([position['xyz'][k] for position in self.positions])

  @staticmethod
  def _calc_centroid(positions):
    # center the path on 0,0
    total_dist = sum([p['segment_length'] for p in positions])
    centroid = np.array([0., 0., 0.])
    for p in positions:
      centroid[i] += p['xyz'][i] * p['segment_length'] / total_dist
      centroid[j] += p['xyz'][j] * p['segment_length'] / total_dist
      centroid[k] += p['xyz'][k] * p['segment_length'] / total_dist
    return centroid

  @staticmethod
  def _calc_crosswind_data(centroid, gs_position, positions):
    """Takes a numpy array centroid and gs_position and updates list of position
    dictionaries with crosswind xy coordinates and angles, in FBL and CSim
    notation."""
    crosswind_x_axis = utils.unit_vec(
          utils.cross(centroid - gs_position, np.array([0., 0., 1.])))
    crosswind_y_axis = utils.unit_vec(
          utils.cross(centroid - gs_position, -crosswind_x_axis))

    for p in positions:
      p_from_crosswind_center = p['xyz'] - centroid
      p['crosswind_xy'] = np.array(
          [np.dot(p_from_crosswind_center, crosswind_x_axis),
           np.dot(p_from_crosswind_center, crosswind_y_axis)])
      # Defines a loop angle, where 0 is top of the loop
      # increasing clockwise looking at kite from gs_position.
      p['loop_angle'] = math.atan2(
          round(p['crosswind_xy'][i], 2), round(p['crosswind_xy'][j], 2))
      p['loop_angle_csim'] = -p['loop_angle'] + 6. * math.pi / 4.

      for key in ['loop_angle', 'loop_angle_csim']:
        if p[key] < 0.:
          p[key] += 2. * math.pi
        elif p[key] >= 2. * math.pi:
          p[key] -= 2. * math.pi

  def __getitem__(self, key):
    assert self.positions
    return np.array([p[key] for p in self.positions])

  def keys(self):
    if not self.positions:
      return []
    else:
      return list(self.positions[0].keys())

  def CSimXg(self):
    """Get XYZ in Sim's ground frame assuming +X is where wind comes from."""
    return self.ConvertToCSimFrameG(self['xyz'].copy())

  def ConvertToCSimFrameG(self, xyz):
    """Convert 3D points to Sim's ground frame assuming +X is where wind comes
    from."""
    xyz[:, 0] *= -1
    xyz[:, 2] *= -1
    return xyz

  def plot_2d_path(self, path_type='crosswind', plot_kwargs={}, **kwargs):
    """Plots the 2D path.

    Args:
      path_type: ['raw', 'crosswind', 'both'] What type of 2D path to plot.
      plot_kwargs: kwargs to pass to the plotter (line color, style, etc).
    Kwargs:
      figsize: figsize
      fig: A figure object to plot the path on. If none, creates a new fig.
      show_legend: Wether to add the legend. 'label' needs to have been passed
        to plot_kwargs
    Example:
      fig = path.plot_2d_path(plot_kwargs={'color':'b', 'label':'circle'})
      fig = other_path.plot_2d_path(fig=fig, show_legend=True, plot_kwargs={
          'color':'g', 'linestyle':'dashed', 'label':'other_path'})
    """
    figsize = kwargs.get('figsize', (6,6))
    show_legend = kwargs.get('show_legend', False)
    fig = kwargs.get('fig', None)

    titles = {'raw': '2d paths: raw xys',
              'crosswind': '2d paths: crosswind_xys',
              'both': '2d paths: crosswind_xys projection and raw xys'}

    if not fig:
      fig, ax = plt.subplots(figsize=figsize)
    ax = fig.get_axes()[0]

    if path_type in ['crosswind', 'both']:
      xs = [pos['crosswind_xy'][0] for pos in self.positions]
      ys = [pos['crosswind_xy'][1] for pos in self.positions]
      cw_line = ax.plot(xs, ys, **plot_kwargs)
      plot_kwargs.update({'label': None,
                          'color': cw_line[0].get_color()})
      # Close the loop and indicate travel direction.
      ax.quiver(xs[-1], ys[-1], xs[0]-xs[-1], ys[0]-ys[-1],
          angles='xy', scale_units='xy', scale=1., **plot_kwargs)
    if path_type == 'both':
      plot_kwargs.update({'label': cw_line[0].get_label()+' raw xys',
        'linewidth': 0.5,
        'linestyle': 'dashed'})
    if path_type in ['raw', 'both']:
      xs = [xy[0] for xy in self.xys]
      ys = [xy[1] for xy in self.xys]
      raw_line = ax.plot(xs, ys, **plot_kwargs)
    if path_type == 'raw':
      plot_kwargs.update({'label': None,
                          'color': raw_line[0].get_color()})
      # Close the loop and indicate travel direction.
      ax.quiver(xs[-1], ys[-1], xs[0]-xs[-1], ys[0]-ys[-1],
          angles='xy', scale_units='xy', scale=1., **plot_kwargs)

    ax.scatter(0, 0, marker='+', c='k', s=250)
    ax.axis('equal')
    ax.set(xlabel='x [m]', ylabel='y [m]',
           title=titles[path_type])
    if show_legend:
      ax.legend().set_visible(True)
    else:
      ax.legend().set_visible(False)
    return fig
