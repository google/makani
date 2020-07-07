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

"""Landing zone parameters."""

from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():
  # TODO: Write helper function to do the landing zone(s)
  # coordinate assembly and rotation.

  # Coordinates of prepared area along East-West radial,
  # in GS reference frame when flying EW radial.
  primary_radial = np.deg2rad(91.0)
  primary_x_low = -500.0
  primary_x_high = 0.0
  primary_y_low = -100.0
  primary_y_high = 20.0

  primary_radial_coords = np.array([[primary_x_low, primary_x_high,
                                     primary_x_high, primary_x_low],
                                    [primary_y_low, primary_y_low,
                                     primary_y_high, primary_y_high]])

  primary_vertices_list = np.transpose(primary_radial_coords).tolist()

  # Coordinates of prepared area along North-South radial,
  # in GS reference frame when flying EW radial.
  secondary_radial = np.deg2rad(10.0)
  secondary_x_low = -500.0
  secondary_x_high = 0.0
  secondary_y_low = -20.0
  secondary_y_high = 100.0

  radial_diff = secondary_radial - primary_radial
  dcm_secondary_radial = np.array([[np.cos(radial_diff), -np.sin(radial_diff)],
                                   [np.sin(radial_diff), np.cos(radial_diff)]])
  secondary_radial_coords = np.array([[secondary_x_low, secondary_x_high,
                                       secondary_x_high, secondary_x_low],
                                      [secondary_y_low, secondary_y_low,
                                       secondary_y_high, secondary_y_high]])

  secondary_vertices = np.dot(dcm_secondary_radial, secondary_radial_coords)
  secondary_vertices_list = np.transpose(secondary_vertices).tolist()

  # Parker Ranch pad coordinates (no-landing zone).
  pr_pad_coords = np.array([[-18.288, 30.48, 30.48, -30.48, -30.48,
                             -57.912000000000006, -74.0664, -102.108, -103.632,
                             -99.6696, -89.91600000000001, -51.816, -30.48,
                             -18.288],
                            [-30.48, -30.48, 30.48, 30.48, -6.096, -6.096,
                             -18.288, -59.436, -68.58, -77.724, -79.248, -68.58,
                             -50.292, -30.48]])
  pr_pad_vertices_list = np.transpose(pr_pad_coords).tolist()

  # Nominal flight path angle [rad] of a gliding descent at
  # WingSave-trimmed angle of attack, empirically adjusted to account
  # for motor drag.
  gs = -0.122

  return {
      'primary_vertices': primary_vertices_list,

      'secondary_vertices': secondary_vertices_list,

      'pr_pad_vertices': pr_pad_vertices_list,

      # Flight path angle [rad] to display on manual monitor.
      'glideslope': gs,

      # Downwind and upwind x-locations [m] of landing area, respectively.
      'threshold': primary_x_low,
      'overrun': primary_x_high
  }
