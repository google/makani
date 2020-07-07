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

"""Scoring functions relating to the crosswind controller."""

from makani.lib.python.batch_sim import scoring_functions
import numpy as np
import scoring_functions_util as scoring_util


class AirspeedMinScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if there is sufficient airspeed when entering crosswind."""

  def __init__(self, good_limit, bad_limit, severity):
    super(AirspeedMinScoringFunction, self).__init__(
        'Min. Crosswind Airspeed', 'm/s', good_limit, bad_limit, severity)
    assert good_limit > bad_limit

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['crosswind_airspeed_min']

  def GetOutput(self, timeseries):
    return {
        'crosswind_airspeed_min': np.min(timeseries['crosswind_airspeed'])
    }

  def GetTimeSeries(self, params, sim, control):
    airspeed = self._SelectTelemetry(
        sim, control, 'airspeed', flight_modes='kFlightModeCrosswindNormal')

    return {'crosswind_airspeed': airspeed}


class AirspeedErrorScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the airspeed error in crosswind flight is small enough."""

  def __init__(self, good_limit, bad_limit, severity):
    super(AirspeedErrorScoringFunction, self).__init__(
        'Airspeed Error', 'm/s', good_limit, bad_limit, severity)
    self.SetSourcePriority(['control'])

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['crosswind_airspeed_error_max']

  def GetOutput(self, timeseries):
    return {
        'crosswind_airspeed_error_max':
            np.max(timeseries['crosswind_airspeed_error'])
    }

  def GetTimeSeries(self, params, sim, control):
    flight_modes = ['kFlightModeCrosswindNormal',
                    'kFlightModeCrosswindPrepTransOut']
    airspeeds, airspeed_cmds = self._SelectTelemetry(
        sim, control, ['airspeed', 'airspeed_cmd'],
        flight_modes=flight_modes)
    error = np.fabs(airspeeds - airspeed_cmds)

    return {'crosswind_airspeed_error': error}


class RadiusErrorScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Test if the radius error in crosswind flight is small enough."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(RadiusErrorScoringFunction, self).__init__(
        'Crosswind Radius Error', 'm', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return np.array([output['crosswind_radius_err_min'],
                     output['crosswind_radius_err_max']])

  def GetOutput(self, timeseries):
    error = timeseries['crosswind_radius_err']
    if np.size(error) == 0:
      min_error = max_error = float('nan')
    else:
      max_error = np.max(error)
      min_error = np.min(error)
    return {
        'crosswind_radius_err_max': max_error,
        'crosswind_radius_err_min': min_error
    }

  def GetTimeSeries(self, params, sim, control):
    flight_modes = ['kFlightModeCrosswindNormal',
                    'kFlightModeCrosswindPrepTransOut']
    path_radius_target, wing_pos_cw = self._SelectTelemetry(
        sim, control, ['path_radius_target', 'wing_pos_cw'],
        flight_modes=flight_modes)

    if not scoring_util.IsSelectionValid(wing_pos_cw):
      return {
          'crosswind_radius_err': np.array([]),
      }

    radii = np.hypot(wing_pos_cw['x'], wing_pos_cw['y'])
    radius_cmds = path_radius_target
    error = radii - radius_cmds

    return {'crosswind_radius_err': error}
