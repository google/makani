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

"""Calibration information for the ground station."""

from makani.config import mconfig
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={'gs_model': 'base_station.gs_model'})
def MakeParams(params):
  """Make ground station parameters."""
  if params['gs_model'] == system_types.kGroundStationModelTopHat:
    # Azimuths are measured relative to -x_g.
    azi_ref_offset = np.pi

  elif params['gs_model'] == system_types.kGroundStationModelGSv1:
    # Azimuths are measured relative to -x_g.
    azi_ref_offset = np.pi

  elif params['gs_model'] == system_types.kGroundStationModelGSv2:
    # Azimuths are measured relative to north.
    azi_ref_offset = 0.0

  else:
    assert False, 'Unsupported ground station model.'

  return {
      # Offset [rad] of the azimuth reference direction from +x_g.
      # Note that azimuth angles are also wrapped to [-pi, pi).
      'azi_ref_offset': azi_ref_offset,

      'gs02': {
          # Position of the GSG [m] in the drum frame.
          'gsg_pos_drum': [-2.1, 0.0, 1.348],

          # Radius of the drum plus 1/2 the outer diameter of the tether [m].
          'drum_radius': 1.88,

          # Origin of the drum frame [m] in the platform frame.
          'drum_origin_p': [0.417, 0.0, -3.05],

          # Origin of the levelwind frame [m] in the platform frame.
          'levelwind_origin_p': [-1.25, 0.4, -5.3],

          # Origin of the cassette frame [m] in the levelwind frame.
          'cassette_origin_l': [-0.2, 1.15, -0.12],

          # Conversion [mm/rad] between the winch drum rotation angle and the
          # x-position of the levelwind shuttle used by the visualizer. The
          # various coefficients were calculated from a polyfit of the caming
          # table used by the PLC for GS02-01.
          'caming_table_drum_angle_rad': [-9.5, -3.8],
          'caming_table_low_pitch_mm': [-6.5961, 374.6522],
          'caming_table_high_pitch_mm': [-68.8652, -246.6691],
          'caming_table_min_offset_mm': 19.9,

          # Coefficients of an empirical linear regression to calculate an
          # approximation of what could have been the levelwind shoulder and
          # wrist measurements leading to the elevation measurement passed on to
          # the flight controller. Values computed by snolet from CW-11
          # levelwind_ele data. This approximation is only used by the
          # visualizer.
          'levelwind_ele_to_shoulder': [-1.4904, 0.4],
          'levelwind_ele_to_wrist': [0.49045, -0.15],

          # Gimbal yoke and termination angles during reel. Based on empirical
          # data.
          'gsg_yoke_angle_in_reel_rad': -1.453,
          'gsg_termination_angle_in_reel_rad': 0.07318,

          # Max acceleration of the winch drum [rad/s^2] in reel modes.
          #
          # TODO(b/118814184): Import this from the MAT controller.
          'max_drum_accel_in_reel': 0.2,

          # Perched position [m] of the wing in the platform frame.
          #
          # TODO: This is determined empirically in the sim so that
          # the wing will initialize on the perch. We should update this based
          # on actual position during testing.
          'perched_wing_pos_p': [0.4, 7.8, -4.3],

          # Estimated length of tether [m] that is wrapped around the racetrack.
          # The racetrack is the portion of the tether track that connects the
          # main drum body to the GSG.
          #
          # This is used to provide an initial estimate of the position at
          # which the winch will be when the kite is perched.
          'racetrack_tether_length': 5.1,

          # Normal distance [m] from platform frame origin to the line which
          # points to the perch panel along the perch bore sight.
          'anchor_arm_length': 0.484,

          # Azimuth [rad] of the idealized boom in the platform frame about
          # the platform's z-axis.
          'boom_azimuth_p': np.pi / 2.0,

          # Elevation angle of the detwist axis in relationship to the drum
          # frame's xy-plane.
          'detwist_elevation': np.deg2rad(22.5),

          # Angle [rad] by which the ground station azimuth is offset from
          # the wing azimuth.
          'reel_azi_offset_from_wing': -np.pi / 2.0,

          'drum_angles': {
              # Maximum drum angle [rad].
              'max': 0.0,

              # Upper limit of drum angles [rad] for which the tether anchor
              # point is along the racetrack.
              'racetrack_high': np.deg2rad(-180.0),

              # Lower limit of drum angles [rad] for which the tether anchor
              # point is along the racetrack. This is also the upper limit for
              # the wide-wrap section.
              'racetrack_low': np.deg2rad(-360.0),

              # Lower limit of drum angles [rad] for which the tether anchor
              # point is along the wide-wrap section. This is also the upper
              # limit for the main-wrap section.
              'wide_wrap_low': np.deg2rad(-360.0 - 270.0),
          }
      }
  }
