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

"""Control system monitoring parameters."""

from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():
  # The following constraints are specified in "Constrained Hover Envelope
  # Parker Ranch", at https://goo.gl/e6WjOV accessed on 2018-09-05
  num_vertices_xy_cross = 7
  # Coordinates [m] of the cross-section of constraint envelope at z = 0.0.
  z_low = -4.3
  xs_low = np.array([-17.0, -8.5, -5.5, -5.5, -8.5, -17.1, -17.0, 0.0, 0.0,
                     0.0])
  ys_low = np.array([4.0, 4.0, 2.0, -2.0, -4.0, -4.0, 4.0, 0.0, 0.0, 0.0])
  # Coordinates [m] of the cross-section of constraint envelope at z = -10.0.
  z_high = -8.4
  xs_high = np.array([-15.4, -8.7, -5.9, -5.9, -8.7, -15.4, -15.4, 0.0,
                      0.0, 0.0])
  ys_high = np.array([3.2, 3.5, 1.9, -1.9, -3.5, -3.2, 3.2, 0.0, 0.0, 0.0])
  # Initialization of the coordinates at mid z
  xs_mid = np.zeros(xs_low.shape)
  ys_mid = np.zeros(ys_low.shape)
  # Interpolate to find cross-section of constraint envelope at an
  # intermediate altitude.
  z_mid = (z_low + z_high) / 2.0
  t = (z_mid - z_low) / (z_high - z_low)
  xs_mid = xs_low + t * (xs_high - xs_low)
  ys_mid = ys_low + t * (ys_high - ys_low)

  # Azimuth [rad] of the indicator frame relative to the ground frame; this
  # is the azimuth of the nominal upwind direction.
  azi_g2indicator = np.deg2rad(35.0)

  # Plot limits
  plot_xlim = [-20.0, 0.0]
  plot_ylim = [-10.0, 6.0]

  # Coordinates [m] of the cross-section of constraint envelope at y = 0.0.
  num_vertices_xz_cross = 5
  xs_vertical = np.array([-5.5, -5.9, -15.4, -17.0, -5.5, 0.0, 0.0, 0.0, 0.0,
                          0.0])
  zs_vertical = np.array([-4.3, -6.1, -8.4, -4.3, -4.3, 0.0, 0.0, 0.0, 0.0,
                          0.0])

  constraint_window = {
      'num_vertices_xy_cross': num_vertices_xy_cross,
      'xs_low': xs_low.tolist(),
      'ys_low': ys_low.tolist(),
      'xs_mid': xs_mid.tolist(),
      'ys_mid': ys_mid.tolist(),
      'xs_high': xs_high.tolist(),
      'ys_high': ys_high.tolist(),
      'num_vertices_xz_cross': num_vertices_xz_cross,
      'xs_vertical': xs_vertical.tolist(),
      'zs_vertical': zs_vertical.tolist(),
      'azi_g2indicator': azi_g2indicator,
      'plot_xlim': plot_xlim,
      'plot_ylim': plot_ylim
  }

  return {
      # Limits for the altitude estimate difference [m].
      'alt_sens_diff': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 2.0,
          'very_high': np.finfo('d').max
      },

      # Limits for the altitude Kalman covariance [m^2].
      'alt_kalman_cov': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 10.0,
          'very_high': np.finfo('d').max
      },

      # Limits for perch offset [m].
      'perch_offset': {
          'very_low': np.finfo('d').min,
          'low': np.finfo('d').min,
          'high': 0.3,
          'very_high': np.finfo('d').max
      },

      # Parameterization of the constraint window coordinates.
      'constraint_window': constraint_window,
  }
