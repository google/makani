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

"""Loadcell parameters."""

from makani.avionics.network import aio_labels
from makani.config import mconfig
import numpy as np


def _MakeBasis(x, y):
  assert abs(np.dot(x, y)) < 1e-6
  z = np.cross(x, y)
  for v in (x, y, z):
    assert abs(np.linalg.norm(v) - 1.0) <= 1e-6
  return [x, y, z]


@mconfig.Config()
def MakeParams():
  # Loadcell basis vectors depend on bridle hardpoint plate angles.
  # These are provided by go/makani-loadcell-coordinate-system.
  # The loadcells do not measure forces along their respective z-axes.

  # Basis vectors for the original loadcell frames.
  port_basis_b = _MakeBasis(np.array([0.057189, 0.754616, 0.653669]),
                            np.array([-0.996195, -0.000000, 0.087156]))

  star_basis_b = _MakeBasis(np.array([0.050616, -0.814083, 0.578539]),
                            np.array([0.996195, -0.000000, -0.087156]))

  # Check that +z-directions have positive z-component in the body frame. This
  # is required by the tether tension estimation scheme.
  assert port_basis_b[2][2] > 0.0
  assert star_basis_b[2][2] > 0.0

  # Loadcell data is zeroed and calibrated to Newtons on the board.
  #
  # TODO: Add crosstalk corrections to the controller once we figure
  # out how those apply to the force angle.
  return [
      {
          'channels': [
              {
                  'cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},

                  # Location of this channel's strain reading:
                  #   i_msg: Index of the LoadcellMessage, in an array of all
                  #       LoadcellMessages, ordered by loadcell node.
                  #   i_strain: Index into the LoadcellMessage's
                  #       loadcell_data.strain field.
                  'strain_location': {
                      'i_msg': aio_labels.kLoadcellNodePortA,
                      'i_strain': 0
                  }
              },
              {
                  'cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
                  'strain_location': {
                      'i_msg': aio_labels.kLoadcellNodePortA,
                      'i_strain': 1
                  }
              }],

          # DCM from this loadcell's frame to the body frame.
          'dcm_loadcell2b': {'d': np.array(port_basis_b).T.tolist()},

          # Mapping from a vector of the channel inputs on this loadcell to the
          # xy-component of force in the loadcell's frame.
          #
          # Channel 0 maps to the negative y-direction, while channel 1 maps to
          # the positive x-direction.
          'channels_to_force_local_xy': {
              'd': [[0.0, 1.0],
                    [-1.0, 0.0]],
          }
      },

      {
          'channels': [
              {
                  'cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
                  'strain_location': {
                      'i_msg': aio_labels.kLoadcellNodeStarboardA,
                      'i_strain': 0
                  }
              },
              {
                  'cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
                  'strain_location': {
                      'i_msg': aio_labels.kLoadcellNodeStarboardA,
                      'i_strain': 1
                  }
              }],

          'dcm_loadcell2b': {'d': np.array(star_basis_b).T.tolist()},

          # Channel 0 maps to the positive y-direction, while channel 1 maps to
          # the negative x-direction.
          'channels_to_force_local_xy': {
              'd': [[0.0, -1.0],
                    [1.0, 0.0]],
          }
      }
  ]
