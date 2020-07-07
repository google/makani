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

"""Scoring functions relating to the buoy."""
from makani.lib.python.batch_sim import scoring_functions
from makani.lib.python.h5_utils import numpy_utils
import numpy as np


class BuoyWaterLineScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Score to evaluate the highest point that the water line reaches."""

  def __init__(self, good_limit, bad_limit, severity):
    super(BuoyWaterLineScoringFunction, self).__init__(
        'Buoy Min. Water Line Distance to Threshold', 'm', good_limit,
        bad_limit, severity)
    assert good_limit > bad_limit

  def GetSystemLabels(self):
    return ['offshore', 'buoy']

  def GetValue(self, output):
    return output['water_line_min']

  def GetOutput(self, timeseries):
    return {'water_line_min': np.min(timeseries['water_line'])}

  def GetTimeSeries(self, params, sim, control):
    water_line = self._SelectTelemetry(sim, control, 'water_line')
    return {'water_line': water_line}


class BuoyYawAngleScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Score to evaluate the maximum/minimum yaw angle."""

  def __init__(self, good_limit, bad_limit, severity):
    super(BuoyYawAngleScoringFunction, self).__init__(
        'Buoy Peak Yaw Angle From Equilibrium', 'deg', good_limit,
        bad_limit, severity)
    assert good_limit < bad_limit

  def GetSystemLabels(self):
    return ['offshore', 'buoy']

  def GetValue(self, output):
    return output['peak_buoy_yaw_angle_deg']

  def GetOutput(self, timeseries):
    buoy_yaw_angle_from_eq_deg = timeseries['buoy_yaw_angle_from_eq_deg']
    return {'peak_buoy_yaw_angle_deg': np.max(
        np.fabs(buoy_yaw_angle_from_eq_deg))}

  def GetTimeSeries(self, params, sim, control):
    buoy_yaw_angle_from_eq = self._SelectTelemetry(sim, control,
                                                   'buoy_yaw_angle_from_eq')
    return {'buoy_yaw_angle_from_eq_deg': np.degrees(buoy_yaw_angle_from_eq)}


class BuoyVesselOriginAccelScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Score to evaluate the maximum acceleration of the vessel frame origin."""

  def __init__(self, good_limit, bad_limit, severity):
    super(BuoyVesselOriginAccelScoringFunction, self).__init__(
        'Buoy Vessel Origin Acceleration', 'g', good_limit,
        bad_limit, severity)
    assert good_limit < bad_limit

  def GetSystemLabels(self):
    return ['offshore', 'buoy']

  def GetValue(self, output):
    return output['peak_buoy_accel_norm_gs']

  def GetOutput(self, timeseries):
    buoy_accel_norm_gs = timeseries['buoy_accel_norm_gs']
    return {'peak_buoy_accel_norm_gs': np.max(buoy_accel_norm_gs)}

  def GetTimeSeries(self, params, sim, control):
    buoy_accel_g = self._SelectTelemetry(sim, control, 'buoy_accel_g')

    try:
      buoy_accel_g_norm = np.sum(
          np.abs(numpy_utils.Vec3ToArray(buoy_accel_g))**2, axis=-1)**(1./2)
    except (TypeError, ValueError):
      buoy_accel_g_norm = np.array([float('nan')])

    return {'buoy_accel_norm_gs': buoy_accel_g_norm / 9.81}
