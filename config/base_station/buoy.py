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

"""Buoy parameters."""

from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():
  """Make buoy parameters."""
  # Inertia tensor [kg-m^2] about the buoy center of mass, and center of mass of
  # the buoy [m], expressed in the Global Coordinate System of the spar Finite
  # Element Model. Source:
  # docs.google.com/spreadsheets/d/1o7B96gTwsQPzowRA4b3Y5zZqc-0jZl1wUfGL-Lit3fU
  inertia_tensor_fem = np.array([[0.27272e12, 0.65323e07, -0.79852e08],
                                 [0.65323e07, 0.27269e12, 0.28349e08],
                                 [-0.79852e08, 0.28349e08, 0.43703e10]]) * 1e-3

  center_of_mass_pos_fem = np.array([-17.743, 7.4421, 15593.0]) * 1e-3

  # Buoy system mass [kg]. This includes: spar, ballast, mooring lines, tower
  # and ground station.
  mass = 756480.0

  # Rotation matrix and translation vector from the Global Coordinate System of
  # the spar FE model to the vessel (v) frame. The document above defines the
  # transformation between the FEM model GCS and the platform (p) frame.
  # The vessel frame and the (p) frame share origin and orientation when the
  # platform is not rotated (see control/coordinate_systems.md).
  dcm_fem2v = np.array([[-1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, -1.0]])

  trans_fem2v_fem = np.array([0.0, 0.0, 63.396])

  # Inertia tensor [kg-m^2] about the buoy CM in vessel (v) frame.
  inertia_tensor_v = np.matmul(np.matmul(dcm_fem2v, inertia_tensor_fem),
                               dcm_fem2v.T)

  # Buoy center of mass position [m] in vessel (v) frame.
  center_of_mass_pos_v = np.matrix(dcm_fem2v) * np.matrix(
      center_of_mass_pos_fem - trans_fem2v_fem).T

  return {
      # Mass [kg], center-of-mass [m] and inertia tensor [kg*m^2] about the
      # center-of-mass expressed in the vessel (v) coordinate system.
      # These properties include the spar, the solid ballast, the tower and the
      # ground station.
      'mass': mass,
      'center_of_mass_pos': center_of_mass_pos_v.T.tolist()[0],
      'inertia_tensor': {'d': inertia_tensor_v.tolist()},

      # Height [m] and radius [m] of the spar and the tower (transition piece).
      # See General Arrangement Profile and Plans, Revision P0, and Transition
      # Piece and Details, Revision 1 on Drive:
      # https://drive.google.com/file/d/1rzKtJq-To-TsCzD8mlS5gA5isjSt1_Au/view
      # https://drive.google.com/file/d/1qIHHdyEp1zeWserY2DFDLF5L65pFVF3H/view
      'bottom_deck_pos_z_v': 63.396,
      'top_deck_pos_z_v': 5.3,
      'spar_height': 63.396 - 5.3,
      'spar_diameter': 4.5,
      'tower_height': 5.300,
      'tower_bottom_radius': 1.605,
      'tower_top_radius': 0.770,
  }
