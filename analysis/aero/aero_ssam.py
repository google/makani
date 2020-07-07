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

"""Contains the formulation of the SSAM model.

  Summary:
    This code calculates the local angle of attack and sideslip on the kite
    aerodynamic surfaces assuming rigid body mechanics about the c.g. of the
    kite.
"""

from makani.analysis.aero.hover_model import hover_model
import numpy as np


class SSAMModel(object):
  """Class used to determine local angles of attack on kite sections."""

  def __init__(self, wing_model, wing_serial):
    """Initializes the SSAM model.

    Args:
      wing_model: Wing model (e.g. 'm600').
      wing_serial: String giving the desired wing serial number (e.g. '01').
    """

    # Use the hover model to obtain the kite main wing panels.
    self._kite_params = hover_model.GetParams(wing_model, wing_serial,
                                              use_wake_model=False)

    # Rigid body mechanics require understanding of the c.g. location as the
    # wing is defined to rotate about the body c.g. and the freestream velocity
    # is assumed uniform everywhere.
    self._cg_loc_b = self._kite_params['center_of_mass_pos']

  def GetMainWingAlphas(self, angular_rate_b, apparent_wind_b):
    """Computes the local alpha values on the main wing.

    Args:
      angular_rate_b: Array of shape (n,3) containing kite body rates.
      apparent_wind_b: Array of shape (n,3) containing apparent wind velocity
                       components.

    Returns:
      main_wing_alphas_deg: Array of shape (n, x) containing the kite main wing
                            local alpha values, where x is the number of kite
                            main wing panels.
    """
    assert len(np.shape(angular_rate_b)) == 2
    assert np.shape(angular_rate_b)[1]
    assert np.shape(angular_rate_b) == np.shape(apparent_wind_b)

    # Pitch rate is ignored as it does not participate in the heaving motion of
    # any of the wing airfoils.
    angular_rate_b[:, 1] = 0.0

    # Compute alpha values for each plane contained in the hover model where the
    # panel is located on the main wing.
    main_wing_alphas_deg = np.array([])
    for panel in self._kite_params['panels']:
      if panel['name'].startswith('Wing'):
        panel_ac_pos_b = panel['pos_b']
        panel_relative_incidence_deg = np.rad2deg(panel['relative_incidence'])

        # It is assumed that the kite rotates about its c.g.
        r_panel = panel_ac_pos_b - self._cg_loc_b

        # Expand the stationary r-position and reorient to match the omega
        # array size to enable the cross product.
        r_panel = np.repeat(np.expand_dims(r_panel, axis=1).transpose(),
                            np.shape(angular_rate_b)[0], axis=0)
        panel_alpha_deg, _ = _ComputeRelativeAlphaBeta(angular_rate_b, r_panel,
                                                       apparent_wind_b)

        # Account for washout if necessary.
        panel_alpha_deg += panel_relative_incidence_deg

        panel_alpha_deg = np.expand_dims(panel_alpha_deg, axis=1)
        if np.shape(main_wing_alphas_deg)[0] != np.shape(panel_alpha_deg)[0]:
          main_wing_alphas_deg = panel_alpha_deg
        else:
          main_wing_alphas_deg = np.concatenate((main_wing_alphas_deg,
                                                 panel_alpha_deg), axis=1)
    return main_wing_alphas_deg


def _ComputeRelativeAlphaBeta(omega_b, position_b, apparent_wind_b):
  """Computes the relative alpha and beta values, in degrees, from kinematics.

  Args:
    omega_b: Array of size (n, 3). Body rates of the kite [rad/s].
    position_b: Array of size (1, 3). Position of the surface to compute local
                alpha/beta [m].
    apparent_wind_b: Array of size (n,3). Apparent wind vector from the state
                     estimator [m/s].

  Returns:
    local_alpha_deg, local_beta_deg: The values of local alpha and beta.

  The math for a relative angle of attack at a given section is as follows:
  (1) Kinematically:
    v_section_b = apparent_wind_b - omega_b X position_b

  (2) By definition:
    alpha_rad = atan2(-v_section_b_z, -v_section_b_x)
    beta_rad  = asin(-v_section_b_y,  mag(v_section_b))
    where _x, _y, _z denote the unit basis vectors in the body coordinates.
  """

  assert np.shape(omega_b) == np.shape(apparent_wind_b)

  # The subtraction is because the cross product is the rigid body motion
  # but the reference frame for the aero has the opposite effect of the
  # motion of the rigid body motion frame.
  local_vel = apparent_wind_b - np.cross(omega_b, position_b, axisa=1,
                                         axisb=1)
  local_vel_mag = np.linalg.norm(local_vel, axis=1)
  local_alpha_deg = np.rad2deg(np.arctan2(-1.0 * local_vel[:, 2],
                                          -1.0 * local_vel[:, 0]))
  local_beta_deg = np.rad2deg(np.arcsin(-1.0 * local_vel[:, 1]
                                        / local_vel_mag))
  return local_alpha_deg, local_beta_deg
