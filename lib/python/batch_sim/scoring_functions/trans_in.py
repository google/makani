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

"""Scoring functions relating to the trans-in controller."""

from makani.lib.python.batch_sim import scoring_functions
import numpy as np


class TransInPitchForwardDurationScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if the trans-in pitch forward duration lies in an acceptable range."""

  def __init__(self, alpha_threshold_deg, good_limit, bad_limit, severity):
    super(TransInPitchForwardDurationScoringFunction, self).__init__(
        'TransIn Pitch Forward Duration', 's', good_limit, bad_limit, severity)
    self._alpha_threshold = np.deg2rad(alpha_threshold_deg)

  def GetSystemLabels(self):
    return ['aero', 'controls']

  def GetValue(self, output):
    return output['pitch_forward_duration']

  def GetTimeSeries(self, params, sim, control):
    time, alpha = self._SelectTelemetry(sim, control, ['time', 'alpha'])
    indices = np.argwhere(alpha < self._alpha_threshold)

    pitch_forward_duration = float('nan')
    if indices.size > 0:
      pitch_forward_duration = time[indices[0][0]] - time[0]

    return {'pitch_forward_duration': pitch_forward_duration}


class AfterPitchForwardScoringFunction(scoring_functions.ScoringFunctionFilter):
  """Test a condition after the initial pitch forward is complete."""

  def __init__(self, alpha_threshold_deg, scoring_function):
    self._scoring_function = scoring_function
    # We only monitor the score after the first time it dips below the
    # angle-of-attack threshold.
    self._alpha_threshold = np.deg2rad(alpha_threshold_deg)
    name = scoring_function.name + ' After Pitch Forward'
    super(AfterPitchForwardScoringFunction, self).__init__(
        name, self._scoring_function)

  def GetSystemLabels(self):
    return ['controls']

  def GetTimeSeries(self, params, sim, control):
    time, alpha = self._SelectTelemetry(sim, control, ['time', 'alpha'])
    indices = np.argwhere(alpha < self._alpha_threshold)

    return self._FilteredOutput(
        params, sim, control, indices, time)


class TransInFinalLateralVelocityErrorScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if the lateral velocity error is too large."""

  def __init__(self, good_limit, bad_limit, severity):
    super(TransInFinalLateralVelocityErrorScoringFunction, self).__init__(
        'Final Lateral Velocity Error', 'm/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['final_vel_err_ti_y']

  def GetOutput(self, timeseries):
    return {
        'final_vel_err_ti_y': timeseries['vel_err_ti_y'][-1]
    }

  def GetTimeSeries(self, params, sim, control):
    wing_vel_trans_in, wing_vel_ti_y_cmd = self._SelectTelemetry(
        sim, control, ['wing_vel_trans_in', 'wing_vel_trans_in_y_cmd'])
    wing_vel_ti_y = wing_vel_trans_in['y']

    return {
        'vel_err_ti_y': np.abs(wing_vel_ti_y - wing_vel_ti_y_cmd)
    }


class FinalAirspeedScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test airspeed on the last sample."""

  def __init__(self, good_limit, bad_limit, severity):
    super(FinalAirspeedScoringFunction, self).__init__(
        'Final Airspeed', 'm/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['final_airspeed']

  def GetOutput(self, timeseries):
    return {'final_airspeed': timeseries['airspeed'][-1]}

  def GetTimeSeries(self, params, sim, control):
    airspeed = self._SelectTelemetry(sim, control, 'airspeed')
    return {'airspeed': airspeed}
