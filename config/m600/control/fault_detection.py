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

"""Fault-detection parameters."""
from makani.config import mconfig
from makani.control import control_types as m
from makani.control import system_types
import numpy as np


@mconfig.Config(deps={
    'common_params': 'common.common_params',
    'gs_model': 'base_station.gs_model',
    'system': mconfig.WING_MODEL + '.system_params',
    'wing_serial': 'common.wing_serial'
})
def MakeParams(params):
  ts = params['common_params']['ts']

  # Expect a sensor reading to change within this many counts [#].
  default_no_update_counts = int(0.5 / ts)

  # Flags indicating that a sensor is disabled due to a lack of
  # calibration or specific test conditions.
  disabled = {
      'controllers': [False, True, True],
      'imus': [False, False, False],
      'gs_compass': False,
      'gsg_azimuth': [False, False],
      'gsg_elevation': [False, False],
      'gs02': False,
      'levelwind_ele': [False, False],
      'detwist': False,
      'drum': False,
      'loadcells': False,
      'pitot': False,
      'perch_azi': [False, False],
      'proximity_sensor': False,
      'winch': False,
      'wing_gps': [False, False, False, False]
  }

  if params['wing_serial'] == m.kWingSerial01:
    # For early M600 tests the Pitot tubes will not be installed
    # (though the sensors themselves may be populated).
    disabled['pitot'] = True
    disabled['wing_gps'] = [False, False, True, True]

  if params['gs_model'] == m.kGroundStationModelTopHat:
    # Assume tophat GS.  The tophat has only one perch azimuth
    # encoder.  It does not have a GPS compass, nor a levelwind (nor
    # the associated encoders).
    disabled['gs_compass'] = True
    disabled['gsg_azimuth'] = [False, True]
    disabled['gs02'] = True
    disabled['levelwind_ele'] = [True, True]
    disabled['drum'] = True
    disabled['perch_azi'] = [False, True]
    disabled['proximity_sensor'] = True
    disabled['winch'] = True
  elif params['gs_model'] == m.kGroundStationModelGSv1:
    # Assume GSv1.  On GSv1, perch azimuth encoders are inoperative.
    disabled['gsg_azimuth'] = [True, True]
    disabled['gs02'] = True
    disabled['detwist'] = True
    disabled['drum'] = True
    disabled['perch_azi'] = [True, True]
  elif params['gs_model'] == m.kGroundStationModelGSv2:
    disabled['gsg_azimuth'] = [False, False]
    if params['system']['test_site'] == system_types.kTestSiteParkerRanch:
      # Disable levelwind A due to encoder issues [b/117862206, b/134973868].
      disabled['levelwind_ele'] = [True, False]
    else:
      disabled['levelwind_ele'] = [False, False]
    # The winch subsystem is populated from TetherPlc, which is unused in GS02.
    # The GS02 drum status is encoded in TetherGroundStation.
    # TODO: Consider updating the winch subsystem in the case of GS02
    # to properly reflect the drum status.
    disabled['winch'] = True
  else:
    assert False, 'Unsupported ground station model.'

  return {
      'control': {
          # Number of control cycles before declaring another controller
          # as not updating.
          'no_update_counts_limit': int(0.2 / ts)
      },

      # Flags indicating that a sensor is disabled due to a lack of
      # calibration or specific test conditions.
      'disabled': disabled,

      'ground_estimator': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'ground_station': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'gs_compass': {
          'no_update_counts_limit': default_no_update_counts,
          'max_latency': default_no_update_counts * ts,
      },

      'gs_gps': {
          'no_update_counts_limit': default_no_update_counts,

          # Maximum position [m] and velocity [m/s] standard
          # deviations.
          'pos_sigma_max': 10.0,
      },

      'gsg': {
          'no_update_counts_limit': [int(1.0 / ts), int(1.0 / ts)],
          'signal_min': [-np.pi, -np.pi],
          'signal_max': [np.pi, np.pi],
      },

      'imu': {
          'no_update_counts_limits': [default_no_update_counts,
                                      default_no_update_counts,
                                      default_no_update_counts],

          # Maximum reported latency [s] from the IMU.
          #
          # The IMU is sampled at a rate faster than the control loop.
          # TODO: Lower this number after confirming the
          # timing of the IMU.
          'max_latency': 0.05,

          # Maximum reported magnetometer latency [s] from the IMU.
          #
          # The magnetometer on the ADIS16488A updates at 100 Hz.  It
          # is possible for there to be an additional delay before this
          # data is sent on the network.
          'mag_max_latency': 0.05,

          # Limits to plausible magnitude of the Earth's magnetic field [gauss].
          'mag_min_plausible': 0.2,
          'mag_max_plausible': 0.8,
      },

      'joystick': {
          'no_update_counts_limit': default_no_update_counts
      },

      'levelwind_ele': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'loadcell': {
          'no_update_counts_limit': default_no_update_counts
      },

      'motor': {
          # MotorStatusMessages are broadcast at 10 Hz.
          'no_update_counts_limit': max(int(0.5 / ts), 1),
      },

      'perch_azi': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'pitot': {
          'no_update_counts_limit': default_no_update_counts,

          # Maximum latency [s] reported by the TMS570 before an error
          # is flagged.
          'max_latency': 0.1,
      },

      'proximity_sensor': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'weather': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'winch': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'wind': {
          'no_update_counts_limit': default_no_update_counts,
      },

      'wing_gps': {
          'no_update_counts_limit': [default_no_update_counts,
                                     default_no_update_counts],
          # Maximum allowable latency [s] reported for the
          # NovAtelLogBestXyz message.
          'max_latency': default_no_update_counts * ts,
      },
  }
