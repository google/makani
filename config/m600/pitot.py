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

"""Pitot tube calibration."""
from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={
    'wing_serial': 'common.wing_serial'})
def MakeParams(params):
  # There is currently one physical Pitot tube connected to two
  # sets of pressure sensors.  Pitot data is reported in SI units
  # (Pascals).
  sensors = [None for _ in range(system_types.kNumPitotSensors)]

  if params['wing_serial'] in [system_types.kWingSerial04Hover,
                               system_types.kWingSerial04Crosswind]:
    sensors[system_types.kPitotSensorHighSpeed] = {
        'max_pressure': 6895.0,
        'stat_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'alpha_cal': {'scale': 1.0, 'bias': 9.5, 'bias_count': 0},
        'beta_cal': {'scale': 1.0, 'bias': 4.3, 'bias_count': 0},
        'dyn_cal': {'scale': 1.0, 'bias': -2.3, 'bias_count': 0}
    }

    sensors[system_types.kPitotSensorLowSpeed] = {
        'max_pressure': 600.0,
        'stat_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'alpha_cal': {'scale': 1.0, 'bias': -3.93, 'bias_count': 0},
        'beta_cal': {'scale': 1.0, 'bias': -3.01, 'bias_count': 0},
        'dyn_cal': {'scale': 1.0, 'bias': -5.00, 'bias_count': 0}
    }

  elif params['wing_serial'] in [system_types.kWingSerial06Hover,
                                 system_types.kWingSerial06Crosswind]:

    sensors[system_types.kPitotSensorHighSpeed] = {
        'max_pressure': 6895.0,
        'stat_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'alpha_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'beta_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'dyn_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
    }

    sensors[system_types.kPitotSensorLowSpeed] = {
        'max_pressure': 600.0,
        'stat_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'alpha_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'beta_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'dyn_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
    }

  elif params['wing_serial'] in [system_types.kWingSerial07Hover,
                                 system_types.kWingSerial07Crosswind]:

    sensors[system_types.kPitotSensorHighSpeed] = {
        'max_pressure': 6895.0,
        'stat_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'alpha_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'beta_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'dyn_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
    }

    sensors[system_types.kPitotSensorLowSpeed] = {
        'max_pressure': 600.0,
        'stat_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'alpha_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'beta_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'dyn_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
    }

  else:

    print 'Warning: No Pitot calibration information for this wing serial.'

    sensors[system_types.kPitotSensorHighSpeed] = {
        'max_pressure': 6895.0,
        'stat_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'alpha_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'beta_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
        'dyn_cal': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
    }

    sensors[system_types.kPitotSensorLowSpeed] = (
        sensors[system_types.kPitotSensorHighSpeed])

  return {
      # Position of the Pitot tube [m] in body coordinates.  Provided
      # by seanchou on 2016-08-12 (b/30795383).
      'pos': [3.213, 0.0, 0.443],

      # Direction of the Pitot tube.  Provided by seanchou on
      # 2016-08-12 (b/30795383).
      'dcm_b2p': {'d': [[0.998629534754574, 0.0, 0.052335956242944],
                        [0.0, 1.0, 0.0],
                        [-0.052335956242944, 0.0, 0.998629534754574]]},

      # Theoretical angle [rad] between the axis of symmetry of the Pitot
      # tube and it's off axis ports.  This was determined empirically
      # by fitting a spherical pressure distribution to the AeroProbe
      # calibration for tube 15239-3.
      'port_angle': 0.47,

      'sensors': sensors,

      # Local pressure coefficient [#] at the center (dynamic
      # pressure) port on the air data probe, due to the aerodynamic
      # influence of the nearby mass balance tube. The pressure
      # coefficient, C_P, is related to local airspeed through the
      # equation:
      #
      #    C_P = 1 - (v_local / v_freestream)^2
      'local_pressure_coeff': 0.168,
  }
