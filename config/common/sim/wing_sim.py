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

"""Simulation wing parameters."""

from makani.analysis.control import geometry
from makani.config import mconfig
from makani.sim import sim_types as m
import numpy as np


@mconfig.Config(deps={
    'buoy_sim': 'base_station.sim.buoy_sim',
    'ground_station': 'base_station.ground_station',
    'gs02_sim': 'base_station.sim.gs02_sim',
    'gs_model': 'base_station.gs_model',
    'perch': 'base_station.perch',
    'perch_sim': 'base_station.sim.perch_sim',
    'sim_options': 'common.sim.sim_options',
    'system': mconfig.WING_MODEL + '.system_params',
    'tether': mconfig.WING_MODEL + '.tether',
    'ground_frame': 'base_station.ground_frame'
})
def MakeParams(params):
  flight_plan = params['system']['flight_plan']

  # Set initial wing position and orientation in the perch frame.
  if flight_plan == m.kFlightPlanStartDownwind:
    Xp_0 = np.array([-0.98, 0.0, -0.022]) * params['tether']['length']
    wing_azi_p = np.pi
    start_perched = False

  elif flight_plan == m.kFlightPlanManual:
    # The sim begins in OffTether; release the kite from a 60 degree
    # elevation.
    # TODO(b/132663390): Set a realistic initial velocity and attitude.
    Xp_0 = np.array([-0.5, 0.0, -0.866]) * params['tether']['length']
    wing_azi_p = np.pi
    start_perched = False

  elif flight_plan in [m.kFlightPlanTurnKey, m.kFlightPlanLaunchPerch,
                       m.kFlightPlanHighHover]:
    # We drop the wing from slightly above the perched position, so its
    # contactors are not initially in contact with the perch. The wing must be
    # moved towards the perch by the same distance, so the tether's initial
    # length is short enough to keep it on the perch.
    drop_height = 0.06
    if params['gs_model'] == m.kGroundStationModelGSv2:
      Xp_0 = (np.array(params['ground_station']['gs02']['perched_wing_pos_p'])
              + np.array([0.0, -drop_height, -drop_height]))
    else:
      Xp_0 = (np.array(params['perch']['perched_wing_pos_p'])
              + np.array([-drop_height, 0.0, -drop_height]))

    wing_azi_p = 0.0
    start_perched = True

  elif flight_plan == m.kFlightPlanDisengageEngage:
    # DisengageEngage starts with the tether almost fully payed out.
    Xp_0 = [0.0, 428.0, -6.0]
    wing_azi_p = np.pi / 2.0
    start_perched = False

  elif flight_plan == m.kFlightPlanHoverInPlace:
    Xp_0 = [0.0, 55.0, -6.0]  # Hand-tuned for GS02 hover simulations.
    wing_azi_p = np.pi / 2.0
    start_perched = False

  else:
    assert False

  if params['gs_model'] == m.kGroundStationModelGSv2:
    _, pitch, roll = geometry.QuatToAngle(
        np.matrix([[d] for d in params['buoy_sim']['q_0']]))
    yaw = (params['buoy_sim']['mooring_lines']['yaw_equilibrium_heading'] -
           params['ground_frame']['heading'])
    dcm_g2v = geometry.AngleToDcm(yaw, pitch, roll)
    dcm_v2p = geometry.AngleToDcm(params['gs02_sim']['initial_platform_azi'],
                                  0.0, 0.0)
    dcm_g2p = dcm_g2v * dcm_v2p
  else:
    # TODO: Make a SWIG wrapper for CalcDcmGToP.
    dcm_g2p = geometry.AngleToDcm(params['perch_sim']['theta_p_0'], 0.0, 0.0)

  if start_perched:
    if params['gs_model'] == m.kGroundStationModelGSv2:
      dcm_p2panel = np.matrix(params['gs02_sim']['panel']['dcm_p2panel']['d'])
    else:
      assert wing_azi_p == 0.0
      dcm_p2panel = np.matrix(params['perch_sim']['panel']['dcm_p2panel']['d'])

    dcm_panel2b = geometry.AngleToDcm(np.pi, np.pi / 2.0, 0.0)
    dcm_p2b = dcm_panel2b * dcm_p2panel
  else:
    dcm_p2b = geometry.AngleToDcm(np.pi + wing_azi_p, np.pi / 2.0, 0.0)

  q_g2b = geometry.DcmToQuat(dcm_p2b * dcm_g2p)

  return {
      'mass_prop_uncertainties': {
          # Offset [m] for the simulated wing's center-of-mass in body
          # coordinates.
          'center_of_mass_offset': [0.0, 0.0, 0.0],

          # Multiplier [#] on the kite mass for simulation.
          'mass_scale': 1.0,

          # Multipliers [#] on the kite inertia tensor for simulation.
          'moment_of_inertia_scale': [1.0, 1.0, 1.0],
      },

      'Xg_0': np.dot(dcm_g2p.T, Xp_0).tolist()[0],
      'Vb_0': [0.0, 0.0, 0.0],
      'q_0': q_g2b.T.tolist()[0],
      'omega_0': [0.0, 0.0, 0.0]
  }
