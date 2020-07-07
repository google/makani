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

"""Offshore buoy simulator parameters."""

from makani.analysis.control import geometry
from makani.config import mconfig
import numpy as np


@mconfig.Config(deps={
    'sea': 'common.sim.sea_sim',
    'buoy': 'base_station.buoy',
})
def MakeParams(params):
  """Creates a dictionary containing buoy simulation parameters.

  Args:
    params: dict containing information about the physical buoy and sea model.

  Returns:
    dict containing information about the buoy position, hydrodynamic
    characteristics, mooring line connectors, mass, and initial conditions.
  """
  # Parameters for the 2019 off-shore demo buoy. See go/makani-buoy-sim-params
  # for details.

  # Initial buoy (vessel) attitude relative to its parent (ground) frame.
  dcm_g2v_0 = geometry.AngleToDcm(0.0, 0.0, 0.0)
  q_g2v_0 = geometry.DcmToQuat(dcm_g2v_0)

  # Yaw restoring torque coefficients [s^-2]. See the section "Buoy Model
  # Development" in go/makani-buoy-sim-params.
  restoring_torque_coeff_z = 0.3843

  # Damping torque coefficients [unitless].
  damping_torque_coeff_x = -1.1649
  damping_torque_coeff_y = -1.1649
  damping_torque_coeff_z = -0.8328

  # Effective mooring line attachment point [m].
  mooring_attach_z_offset_from_com_v = 10.002
  mooring_attach_pos_z_v = (params['buoy']['center_of_mass_pos'][2] +
                            mooring_attach_z_offset_from_com_v)

  # Vertical position of the mean sea level.
  # Assumes vessel and ground frames are coincidental at initialization.
  water_density = params['sea']['water_density']
  wet_height = (4.0 * params['buoy']['mass'] / np.pi / water_density /
                params['buoy']['spar_diameter']**2)
  msl_pos_z_g = params['buoy']['bottom_deck_pos_z_v'] - wet_height

  return {
      'msl_pos_z_g': msl_pos_z_g,

      'hydrodynamics': {
          # Torsional damping coefficients [N*m/(rad/s)^2].
          'torsional_damping_x': (damping_torque_coeff_x *
                                  params['buoy']['inertia_tensor']['d'][0][0]),
          'torsional_damping_y': (damping_torque_coeff_y *
                                  params['buoy']['inertia_tensor']['d'][1][1]),
          'torsional_damping_z': (damping_torque_coeff_z *
                                  params['buoy']['inertia_tensor']['d'][2][2]),

          # Buoyancy damping coefficient [N*s^2/m^2].
          'buoyancy_damping_coeff': 9.0e4,

          # Spar segment drag coefficient.
          # Setting this to zero now (no hydrodynamic drag) and relying on
          # damping coefficients instead.
          # TODO: Obtain values for the drag coefficient.
          'Cd': 0.0,

          # Added mass parameters.
          'Ca': 1.0,     # Added mass coefficient [-].
          'Dh': 6.8219,  # Heave added mass efective diameter [m].
          'ki': 1.0188,  # Added inertia scaling factor [-].

          'uncertainties': {
              'torsional_damping_x_scale': 1.0,
              'torsional_damping_y_scale': 1.0,
              'torsional_damping_z_scale': 1.0,
              'buoyancy_damping_coeff_scale': 1.0,
              'Ca_scale': 1.0,
              'Dh_scale': 1.0,
              'ki_scale': 1.0,
          },
      },

      'mooring_lines': {
          # Definition of the heading [rad] of the +X axis of the vessel frame
          # for which there is no restoring yaw torque, clockwise from North.
          # This is assumed to be derived from the mooring system installation.
          # The value in this parameter will override the yaw angle specified in
          # the initial quaternion q_0 below, in order to have the buoy at rest
          # at the initial time.
          # TODO(b/144859123): Currently this is set to zero to work around a
          # bug in which the vessel heading is used by the sim even in onshore
          # scenarios, where it should have no effect.  The long-term fix should
          # be to remove the buoy model from onshore simulations entirely.
          'yaw_equilibrium_heading': np.deg2rad(0.0),

          # For the coefficients below, see derivation in
          # go/makani-buoy-sim-params. Equivalent yaw torsional stiffness
          # coefficients [N*m/rad] accounting for hydrostatic action and for the
          # mooring lines. The subscript indicates the axis of rotation, in
          # vessel frame (v).
          'torsional_stiffness_z': restoring_torque_coeff_z *
                                   params['buoy']['inertia_tensor']['d'][2][2],

          # Mooring line attachment point [m] in vessel frame.
          'mooring_attach_v': [0., 0., mooring_attach_pos_z_v],

          # Mooring line model parameters.
          'kt0': 2.2e+03,    # Proportional spring coefficient [N/m].
          'kt1': 32.0,       # Quadratic spring coefficient [N/m^2].
          'ct': 1.1078e+05,  # Translational damping coefficient [N*s^2/m^2].

          'uncertainties': {
              'yaw_equilibrium_heading_delta': 0.0,
              'torsional_stiffness_z_scale': 1.0,
              'mooring_attach_pos_x_delta': 0.0,
              'mooring_attach_pos_y_delta': 0.0,
              'mooring_attach_pos_z_delta': 0.0,
              'kt0_scale': 1.0,
              'kt1_scale': 1.0,
              'ct_scale': 1.0,
          },
      },

      'mass_prop_uncertainties': {
          # Offset [m] for the center-of-mass in vessel coordinates.
          'center_of_mass_offset': [0.0, 0.0, 0.0],

          # Buoy mass multiplier [#].
          'mass_scale': 1.0,

          # Buoy moment of inertia multipliers [#].
          'moment_of_inertia_scale': [1.0, 1.0, 1.0],
      },

      # Initial buoy attitude.
      # This attitude will be modified according to the value of the parameter
      # yaw_equilibrium_heading above.
      'q_0': q_g2v_0.T.tolist()[0],

      # Initial buoy angular velocity [rad/s] relative to its parent (ground)
      # frame, expressed in vessel coordinates.
      'omega_0': [0.0, 0.0, 0.0],

      # Initial buoy position [m] relative to the ground frame, expressed in
      # ground coordinates.
      'Xg_0': [0.0, 0.0, 0.0],

      # Initial buoy velocity [m/s] relative to the ground frame, expressed in
      # ground coordinates.
      'Vg_0': [0.0, 0.0, 0.0],
  }
