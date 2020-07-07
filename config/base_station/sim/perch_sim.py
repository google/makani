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

"""Perch simulator parameters."""

from makani.analysis.control import geometry
from makani.config import mconfig
from makani.config import physics_util
from makani.config.sensor_util import MakeEncoderParams
from makani.control import system_types
from makani.sim import sim_types as m
import numpy as np


@mconfig.Config(deps={
    'common_params': 'common.common_params',
    'gs_model': 'base_station.gs_model',
    'perch': 'base_station.perch',
    'sim_options': 'common.sim.sim_options',
    'flight_plan': 'common.flight_plan'
})
def MakeParams(params):
  """Make perch simulator parameters."""
  # pylint: disable=invalid-name
  A, B = physics_util.CalcTwoLinkageDynamics(
      params['perch']['I_perch_and_drum'], params['perch']['I_drum'],
      params['perch']['b_perch'], params['perch']['b_drum'])

  flight_plan = params['flight_plan']
  # Pick an initial azimuth [rad] in the ground frame to rotate the
  # wing and the perch.  azi_g is 0 when the wing is on the negative
  # xg axis.
  azi_g = 0.0

  # Set perch angle based on flight plan.
  if (flight_plan in [m.kFlightPlanStartDownwind]
      and not params['sim_options'] & m.kSimOptConstraintSystem):
    theta_p_0 = m.Wrap(azi_g, -np.pi, np.pi)
    initialize_in_crosswind_config = True
  elif flight_plan in [m.kFlightPlanDisengageEngage,
                       m.kFlightPlanHighHover,
                       m.kFlightPlanHoverInPlace,
                       m.kFlightPlanLaunchPerch,
                       m.kFlightPlanManual,
                       m.kFlightPlanTurnKey]:
    theta_p_0 = m.Wrap(azi_g + np.pi, -np.pi, np.pi)
    initialize_in_crosswind_config = False
  else:
    assert False

  # The tophat has one perch azimuth encoder; GSv1 has none.
  perch_azi_enabled = [
      params['gs_model'] == system_types.kGroundStationModelTopHat, False]

  return {
      # Radius [m] of the levelwind.
      'levelwind_radius': 1.5,

      # Position [m] of the levelwind in perch x and y coordinates
      # when the levelwind elevation is zero.
      'levelwind_hub_p': [1.8, -1.5],

      # Minimum tension [N] for the levelwind to engage.
      'levelwind_engage_min_tension': 1e3,

      # The matrices describing the linearized dynamics of the perch and
      # winch drum system.
      'A': {'d': A.tolist()},
      'B': {'d': B.tolist()},

      # Initial angle [rad] of the perch relative to ground coordinates.
      'theta_p_0': theta_p_0,

      # Boolean [#] that describes whether the perch begins in the
      # crosswind configuration (i.e. winch drum angle is 0.0 rad) or
      # a reeled-in configuration.
      'initialize_in_crosswind_config': initialize_in_crosswind_config,

      # Properties of the perch panel.  The perch panel is modeled as
      # the union of two cylinders, which are pitched and rolled about
      # the perch axes by the specified angles, then truncated at
      # planes parallel to the perch z-plane. The port cylinder
      # corresponds to the port wing side and vice versa.
      #
      # Parameters are from hachtmann on 2015-08-21 (with corrections on
      # 2015-09-22), and are confirmed using the drawings located here:
      # go/makaniwiki/perch-geometry.
      'panel': {
          # For each panel, the center [m] and radius [m] describe the
          # cylinder modeling it. The z_extents_p [m] specify the planes,
          # parallel to the perch z-plane, at which the cylinders are
          # cut.
          'port': {
              'center_panel': [3.122, 0.763],
              'radius': 4.0,
              'z_extents_p': [-0.625, 2.656],
          },

          'starboard': {
              'center_panel': [3.203, -1.103],
              'radius': 4.0,
              'z_extents_p': [-0.306, 2.656],
          },

          'origin_pos_p': [0.0, 0.0, 0.0],

          # Rotation matrix from the perch frame to the panel frame.
          'dcm_p2panel': {'d': geometry.AngleToDcm(
              np.deg2rad(0.0), np.deg2rad(6.0), np.deg2rad(-7.0)).tolist()},

          # Y extents [m] of the panel apparatus in the panel coordinate
          # system.
          'y_extents_panel': [-1.9, 2.0],
      },

      # Sensor parameters for perch encoders.  The biases of one or
      # two degrees are estimated typical biases.  The noise level is
      # chosen to be pessimistic but not completely unrealistic.
      'ts': params['common_params']['ts'],
      'levelwind_ele_sensor': [MakeEncoderParams(), MakeEncoderParams()],
      'perch_azi_sensor': [
          MakeEncoderParams(bias=np.deg2rad(-1.0), noise_level_counts=0.25,
                            scale=1.0 if perch_azi_enabled[0] else 0.0),
          MakeEncoderParams(bias=np.deg2rad(2.0), noise_level_counts=0.25,
                            scale=1.0 if perch_azi_enabled[1] else 0.0)]

  }
