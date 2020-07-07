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

"""Scoring functions relating to aerodynamics."""

from makani.analysis.aero import aero_ssam
from makani.analysis.aero import apparent_wind_util
from makani.analysis.log_analysis import loop_averager
from makani.control import system_types
from makani.lib.python import c_helpers
from makani.lib.python.batch_sim import scoring_functions
from makani.lib.python.h5_utils import numpy_utils
from makani.system import labels as system_labels

import numpy as np
from scipy import interpolate
import scoring_functions_util as scoring_util

_FLAP_LABEL_HELPER = c_helpers.EnumHelper('FlapLabel', system_labels,
                                          prefix='kFlap')
_WING_MODEL_HELPER = c_helpers.EnumHelper('WingModel', system_types)
_WING_SERIAL_HELPER = c_helpers.EnumHelper('WingSerial', system_types)


class AirspeedMaxScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the airspeed falls outside of acceptable limits."""

  def __init__(self, good_limit, bad_limit, severity):
    super(AirspeedMaxScoringFunction, self).__init__(
        'Max Airspeed', 'm/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['airspeed_max']

  def GetOutput(self, timeseries):
    return {'airspeed_max': np.max(timeseries['airspeed'])}

  def GetTimeSeries(self, params, sim, control):
    airspeed = self._SelectTelemetry(sim, control, 'airspeed')
    return {'airspeed': airspeed}


class AirspeedMinScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the airspeed falls outside of acceptable limits."""

  def __init__(self, good_limit, bad_limit, severity):
    super(AirspeedMinScoringFunction, self).__init__(
        'Min Airspeed', 'm/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['airspeed_min']

  def GetOutput(self, timeseries):
    return {'airspeed_min': np.min(timeseries['airspeed'])}

  def GetTimeSeries(self, params, sim, control):
    airspeed = self._SelectTelemetry(sim, control, 'airspeed')
    return {'airspeed': airspeed}


class MainWingAlphaScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if angle of attack on the main wing exceeds a limit."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, airspeed_threshold=1.0,
               steady_flight=False):
    super(MainWingAlphaScoringFunction, self).__init__(
        'Main Wing SSAM AoA%s' % (' (w/o initial transients)' if steady_flight
                                  else ''),
        'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    assert airspeed_threshold > 0.0
    self._airspeed_threshold = airspeed_threshold

  def GetSystemLabels(self):
    return ['aero']

  def GetValue(self, output):
    return np.array([output['wing_alpha_min'], output['wing_alpha_max']])

  def GetFailureCount(self, timeseries):
    if timeseries is None:
      return None
    # Confirm that the arrays have the same size.
    assert len(timeseries['alphas_min']) == len(timeseries['alphas_max'])
    # Confirm that at any index, alphas_max is going to be greater than
    # alphas_min.
    assert np.all(timeseries['alphas_max'] >= timeseries['alphas_min'])
    return np.logical_or(
        timeseries['alphas_min'] < self._bad_lower_limit,
        timeseries['alphas_max'] > self._bad_upper_limit).sum()

  def GetIndexOfFirstFailure(self, timeseries):
    if timeseries is None:
      return None
    if self.GetFailureCount(timeseries) == 0:
      return -1
    else:
      return np.argmax(
          np.logical_or(timeseries['alphas_min'] < self._bad_lower_limit,
                        timeseries['alphas_max'] > self._bad_upper_limit))

  def GetOutput(self, timeseries):
    return {
        'wing_alpha_min': np.min(timeseries['alphas_min']),
        'wing_alpha_max': np.max(timeseries['alphas_max'])
    }

  def GetTimeSeries(self, params, sim, control):
    airspeeds, angular_rates, app_wind_b = self._SelectTelemetry(
        sim, control, ['airspeed', 'body_rates', 'apparent_wind_vector'])

    # Converts the indices into an (n,) sized array for 2D array masking.
    data_indices = np.reshape(
        np.argwhere(airspeeds > self._airspeed_threshold), -1)

    if data_indices.size == 0:
      wing_alphas_max = np.array([float('nan')])
      wing_alphas_min = np.array([float('nan')])

    else:
      # Mask the body rates for telemetry that crosses the threshold.
      omega_b = numpy_utils.Vec3ToArray(angular_rates)[data_indices]

      # Mask the cartesian wind for telemetry that crosses the threshold.
      wind_b = numpy_utils.Vec3ToArray(app_wind_b)[data_indices]

      # Compute the kinematic-based local values of alpha.
      # TODO: Wing_model should be mapped to enumerate wing models.
      wing_model = _WING_MODEL_HELPER.ShortName(
          int(params['system_params']['wing_model'][0]))
      wing_serial = _WING_SERIAL_HELPER.Name(int(
          params['system_params']['wing_serial'][0]))
      ssam = aero_ssam.SSAMModel(wing_model, wing_serial)
      wing_alphas_deg = ssam.GetMainWingAlphas(omega_b, wind_b)

      # Provide telemetry of maximum alphas anywhere along the main wing.
      # As per GetMainWingAlphas the expected size of wing_alphas_deg is (n, m)
      # where m is the number of wing panels and n is the number of elements in
      # the time series.
      wing_alphas_max = np.amax(wing_alphas_deg, axis=1)
      wing_alphas_min = np.amin(wing_alphas_deg, axis=1)

    return {'alphas_max': wing_alphas_max, 'alphas_min': wing_alphas_min}


class AlphaDegScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the angle-of-attack exceeds a limit when airspeed is high."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, airspeed_threshold=1.0,
               steady_flight=False):
    super(AlphaDegScoringFunction, self).__init__(
        'Angle-of-attack%s' % (' (w/o initial transients)' if steady_flight
                               else ''),
        'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    assert airspeed_threshold > 0.0
    self._airspeed_threshold = airspeed_threshold

  def GetSystemLabels(self):
    return ['aero']

  def GetValue(self, output):
    return np.array([output['alpha_min'], output['alpha_max']])

  def GetOutput(self, timeseries):
    alpha = timeseries['alpha']
    return {
        'alpha_max': np.max(alpha),
        'alpha_min': np.min(alpha)
    }

  def GetTimeSeries(self, params, sim, control):
    airspeeds, alpha, time = self._SelectTelemetry(
        sim, control, ['airspeed', 'alpha', 'time'])
    data_indices = np.argwhere(airspeeds > self._airspeed_threshold)
    if data_indices.size == 0:
      alpha = np.array([float('nan')])
    else:
      chord = params['system_params']['wing']['c']
      # Choose 5 main wing flow-over time constants, 2 sigma for peak rejection
      alpha = scoring_util.FilterByWindowAveraging(
          time[data_indices], alpha[data_indices], airspeeds[data_indices],
          chord, num_tau=5, num_sigma=2)

    return {'alpha': np.rad2deg(alpha)}


class MinAlphaDegScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the angle-of-attack exceeds a limit when airspeed is high."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, airspeed_threshold=1.0):
    super(MinAlphaDegScoringFunction, self).__init__(
        'Min. Angle-of-attack', 'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    assert airspeed_threshold > 0.0
    self._airspeed_threshold = airspeed_threshold

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['min_alpha']

  def GetOutput(self, timeseries):
    return {'min_alpha': np.min(timeseries['alpha'])}

  def GetTimeSeries(self, params, sim, control):
    # TODO: Review if this scoring function is working as
    # intended. Update/remove as needed.
    airspeeds, alpha, time = self._SelectTelemetry(
        sim, control, ['airspeed', 'alpha', 'time'])
    data_indices = np.argwhere(airspeeds > self._airspeed_threshold)
    if data_indices.size == 0:
      alpha = np.array([float('nan')])
    else:
      chord = params['system_params']['wing']['c']
      # Choose 5 main wing flow-over time constants, 2 sigma for peak rejection
      alpha = scoring_util.FilterByWindowAveraging(
          time[data_indices], alpha[data_indices], airspeeds[data_indices],
          chord, num_tau=5, num_sigma=2)

    return {'alpha': np.rad2deg(alpha)}


class BetaDegScoringFunction(scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the side-slip exceeds a limit when airspeed is high."""

  def __init__(self, bad_lower_limit, good_lower_limit,
               good_upper_limit, bad_upper_limit, severity,
               airspeed_threshold=1.0, steady_flight=False):
    super(BetaDegScoringFunction, self).__init__(
        'Side-slip%s' % (' (w/o initial transients)' if steady_flight else ''),
        'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    assert airspeed_threshold > 0.0
    self._airspeed_threshold = airspeed_threshold

  def GetSystemLabels(self):
    return ['aero']

  def GetValue(self, output):
    return np.array([output['beta_min'], output['beta_max']])

  def GetOutput(self, timeseries):
    beta = timeseries['beta']
    return {
        'beta_max': np.max(beta),
        'beta_min': np.min(beta)
    }

  def GetTimeSeries(self, params, sim, control):
    airspeeds, beta, time = self._SelectTelemetry(
        sim, control, ['airspeed', 'beta', 'time'])
    data_indices = np.argwhere(airspeeds > self._airspeed_threshold)
    if data_indices.size == 0:
      beta = np.array([float('nan')])
    else:
      chord = params['system_params']['wing']['c']
      # Choose 5 main wing flow-over time constants, 2 sigma for peak rejection
      beta = scoring_util.FilterByWindowAveraging(
          time[data_indices], beta[data_indices], airspeeds[data_indices],
          chord, num_tau=5, num_sigma=2)

    return {'beta': np.rad2deg(beta)}


class AlphaDegErrorScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the angle-of-attack error in crosswind flight is small enough."""

  def __init__(self, good_limit, bad_limit, severity,
               airspeed_threshold=1.0, steady_flight=False):
    super(AlphaDegErrorScoringFunction, self).__init__(
        'Angle-of-attack Error%s' % (' (w/o initial transients)'
                                     if steady_flight else ''),
        'deg', good_limit, bad_limit, severity)
    assert airspeed_threshold > 0.0
    self._airspeed_threshold = airspeed_threshold
    self.SetSourcePriority(['control'])

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['alpha_error_max']

  def GetOutput(self, timeseries):
    return {'alpha_error_max': np.max(timeseries['alpha_error'])}

  def GetTimeSeries(self, params, sim, control):
    airspeeds, alphas, alpha_cmds, time = self._SelectTelemetry(
        sim, control, ['airspeed', 'alpha', 'alpha_cmd', 'time'])

    data_indices = np.argwhere(airspeeds > self._airspeed_threshold)
    if data_indices.size == 0:
      alpha_error = np.array([float('nan')])
    else:
      chord = params['system_params']['wing']['c']
      # Choose 5 main wing flow-over time constants, 2 sigma for peak rejection
      alphas = scoring_util.FilterByWindowAveraging(
          time[data_indices], alphas[data_indices], airspeeds[data_indices],
          chord, num_tau=5, num_sigma=2)
      alpha_error = np.fabs(alphas - alpha_cmds[data_indices])

    return {'alpha_error': np.rad2deg(alpha_error)}


class BetaDegErrorScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the sideslip angle error in crosswind flight is small enough."""

  def __init__(self, good_limit, bad_limit, severity,
               airspeed_threshold=1.0, steady_flight=False):
    super(BetaDegErrorScoringFunction, self).__init__(
        'Sideslip Error%s' % (' (w/o initial transients)'
                              if steady_flight else ''),
        'deg', good_limit, bad_limit, severity)
    assert airspeed_threshold > 0.0
    self._airspeed_threshold = airspeed_threshold
    self.SetSourcePriority(['control'])

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['beta_error_max']

  def GetOutput(self, timeseries):
    return {'beta_error_max': np.max(timeseries['beta_error'])}

  def GetTimeSeries(self, params, sim, control):
    airspeeds, betas, beta_cmds, time = self._SelectTelemetry(
        sim, control, ['airspeed', 'beta', 'beta_cmd', 'time'])

    data_indices = np.argwhere(airspeeds > self._airspeed_threshold)
    if data_indices.size == 0:
      beta_error = np.array([float('nan')])
    else:
      chord = params['system_params']['wing']['c']
      # Choose 5 main wing flow-over time constants, 2 sigma for peak rejection
      betas = scoring_util.FilterByWindowAveraging(
          time[data_indices], betas[data_indices], airspeeds[data_indices],
          chord, num_tau=5, num_sigma=2)
      beta_error = np.fabs(betas - beta_cmds[data_indices])

    return {'beta_error': np.rad2deg(beta_error)}


# TODO: Alpha, Beta and Airspeed RMS error scoring functions
# should be evaluated on per loop basis. The GetTimeseries method should be like
# the one in SurfaceSaturationScoringFunction.
class AlphaRmsScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """RMS angle-of-attack error."""

  def __init__(self, good_limit, bad_limit, severity):
    super(AlphaRmsScoringFunction, self).__init__(
        'AoA RMS', 'deg', good_limit, bad_limit, severity)
    self.SetSourcePriority(['control'])

  def GetSystemLabels(self):
    return ['experimental', 'controls']

  def GetValue(self, output):
    return output['alpha_rms']

  def GetOutput(self, timeseries):
    return {'alpha_rms': np.rad2deg(timeseries['alpha_rms'])}

  def GetTimeSeries(self, params, sim, control):
    alphas, alpha_cmds = self._SelectTelemetry(
        sim, control, ['alpha', 'alpha_cmd'])
    return {
        'alpha_rms': np.mean((alphas - alpha_cmds)**2.0)**0.5
        }


class BetaRmsScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """RMS side-slip error."""

  def __init__(self, good_limit, bad_limit, severity):
    super(BetaRmsScoringFunction, self).__init__(
        'Side-slip RMS', 'deg', good_limit, bad_limit, severity)
    self.SetSourcePriority(['control'])

  def GetSystemLabels(self):
    return ['experimental', 'controls']

  def GetValue(self, output):
    return output['beta_rms']

  def GetOutput(self, timeseries):
    return {'beta_rms': np.rad2deg(timeseries['beta_rms'])}

  def GetTimeSeries(self, params, sim, control):
    betas, beta_cmds = self._SelectTelemetry(
        sim, control, ['beta', 'beta_cmd'])
    return {
        'beta_rms': np.mean((betas - beta_cmds)**2.0)**0.5
        }


class AirspeedRmsScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """RMS airspeed error."""

  def __init__(self, good_limit, bad_limit, severity):
    super(AirspeedRmsScoringFunction, self).__init__(
        'Airspeed RMS', 'm/s', good_limit, bad_limit, severity)
    self.SetSourcePriority(['control'])

  def GetSystemLabels(self):
    return ['experimental', 'controls']

  def GetValue(self, output):
    return output['airspeed_rms']

  def GetOutput(self, timeseries):
    return {'airspeed_rms': timeseries['airspeed_rms']}

  def GetTimeSeries(self, params, sim, control):
    airspeeds, airspeed_cmds = self._SelectTelemetry(
        sim, control, ['airspeed', 'airspeed_cmd'])
    return {
        'airspeed_rms': np.mean((airspeeds - airspeed_cmds)**2.0)**0.5
        }


class SurfaceSaturationScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests for the percent time the control surface is saturated.

     If the saturation scoring function is activated during a crosswind flight
     mode, then the characteristic percentage time corresponds to the maximum
     percentage time a surface was saturated during any particular single loop.

     If the saturation scoring function is activated during a non-crosswind
     flight mode, then the characteristic percentage time corresponds to the
     maximum saturation percentage achieved on all the telemetry data for
     that flight mode.
  """

  def __init__(self, flap_labels, good_limit, bad_limit, severity):
    self._flap_indices = [_FLAP_LABEL_HELPER.Value(l) for l in flap_labels]
    super(SurfaceSaturationScoringFunction, self).__init__(
        '%s %% Saturated' % '/'.join(flap_labels), '% time',
        good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['aero', 'controls']

  def GetValue(self, output):
    return output['percent_time_sat']

  def GetOutput(self, timeseries):
    largest_saturation_loop_mask = timeseries['largest_saturation_loop_mask']
    if largest_saturation_loop_mask is None:
      return {'percent_time_sat': float('nan')}
    saturation_counter = np.where(largest_saturation_loop_mask)[0]
    return {
        'percent_time_sat': float(np.size(saturation_counter)) /
                            np.size(largest_saturation_loop_mask) * 100.0
    }

  def GetTimeSeries(self, params, sim, control):
    # "_i" is used to denote the as-imported variable.
    (omega_i, airspeed_i, alpha_i, beta_i, flaps, loop_angles) = (
        self._SelectTelemetry(sim, control, [
            'body_rates', 'airspeed', 'alpha', 'beta', 'flaps', 'loop_angle']))
    if not scoring_util.IsSelectionValid(flaps):
      return {'saturation_mask': None, 'largest_saturation_loop_mask': None}
    deflections = np.rad2deg(flaps[:, self._flap_indices])

    # Observations of actual control surface deflections show that when
    # saturated, they are not exactly at the control limit. Adding or
    # subtracting 0.25 degrees from the limit allows for most saturations
    # to be flagged by the criteria. A4 and A5 are often very close to the
    # upper limit of zero with very small magnitudes of deflection. An offset
    # of only 0.005 degrees is applied to the upper flap limit for flaps A4
    # and A5 to prevent normal operation from being flagged erroneously.
    # The deflection limits of the rudder are altered to account for the
    # wing-fuselage junction loads limit, per b/112267831.
    lower_deflection_limit = (
        np.rad2deg(np.array(params['control_params']['crosswind']['output']
                            ['lower_flap_limits'][0, self._flap_indices]))
        + 0.25)
    if (self._flap_indices == [system_labels.kFlapA4] or
        self._flap_indices == [system_labels.kFlapA5]):
      upper_deflection_limit = (np.rad2deg(np.array(
          params['control_params']['crosswind']['output']
          ['upper_flap_limits'][0, self._flap_indices])) - 0.005)

    elif self._flap_indices == [system_labels.kFlapRud]:
      # Check if body rates data exist. These may not exist if the relevant
      # flight modes do not exist.
      if not scoring_util.IsSelectionValid(omega_i):
        return {'saturation_mask': float('nan')}
      # Position [m] of the vtail aerodynamic center. Only the x-coordinate is
      # relevant here.
      # TODO: Add position of aerodynamic center to params in the h5 log.
      r_vtail = np.array([-7.0, 0.0, 0.0])

      vapp = np.array(airspeed_i)
      alpha = np.array(alpha_i)
      beta = np.array(beta_i)

      omega = np.array([omega_i['x'],
                        omega_i['y'],
                        omega_i['z']])
      # Account for motion of empennage relative to kite origin as it affects
      # apparent wind, alpha, beta.
      v_kite = apparent_wind_util.ApparentWindSphToCart(vapp, alpha, beta).T
      v_omega = np.cross(omega, r_vtail, axis=0)
      vapp, alpha, beta = (
          apparent_wind_util.ApparentWindCartToSph((v_kite + v_omega).T))
      rudder_limits = self._GetRudderLimits(beta, vapp)
      lower_deflection_limit = rudder_limits['lower_limit']
      upper_deflection_limit = rudder_limits['upper_limit']

      # Check that there is not a dimension problem coming out of the rudder
      # deflection limit table. Deflections is expected shape (N,1). The rudder
      # table uses the apparent_wind_util that may create arrays of shape (N,).
      if np.shape(lower_deflection_limit) != np.shape(deflections):
        lower_deflection_limit = np.reshape(lower_deflection_limit,
                                            np.shape(deflections))
      if np.shape(upper_deflection_limit) != np.shape(deflections):
        upper_deflection_limit = np.reshape(upper_deflection_limit,
                                            np.shape(deflections))

    else:
      upper_deflection_limit = (np.rad2deg(np.array(
          params['control_params']['crosswind']['output']
          ['upper_flap_limits'][0, self._flap_indices])) - 0.25)

    # Fraction of a limit at which a surface is considered nearly saturated.
    saturation_fraction = 0.9
    new_upper_limit = saturation_fraction * upper_deflection_limit
    new_lower_limit = saturation_fraction * lower_deflection_limit

    is_saturated_upper = np.greater_equal(deflections, new_upper_limit)
    is_saturated_lower = np.less_equal(deflections, new_lower_limit)
    saturation_mask = np.logical_or(is_saturated_upper, is_saturated_lower)
    largest_saturation_loop_mask = saturation_mask

    # Identify the number of loops in the data and cycle over them to find which
    # loop has the highest percentage of saturation. If there are no loops then
    # simply take all the data as the scoring function is likely operating in a
    # non-crosswind mode, e.g. hover.
    endloop_indices = loop_averager.GetEndLoopIndices(loop_angles)
    if np.size(endloop_indices):
      # Go through each independent loop to find the mask array that contains
      # the largest amount of masked values and return that array for scoring.
      for i, endloop_indx in enumerate(endloop_indices):
        if i == 0:
          start_loop_indx = 0
          largest_pct_saturated = 0.0
          largest_saturation_loop_mask = saturation_mask[start_loop_indx:
                                                         endloop_indx+1]
        else:
          start_loop_indx = endloop_indices[i-1]

        # Figure out how long the surface has been saturated during this loop.
        this_loop_mask = saturation_mask[start_loop_indx:endloop_indx + 1]

        saturation_counter = np.where(this_loop_mask)[0]
        percent_saturated = (float(np.size(saturation_counter)) /
                             np.size(this_loop_mask) * 100.0)

        if percent_saturated > largest_pct_saturated:
          largest_saturation_loop_mask = this_loop_mask
          largest_pct_saturated = percent_saturated

    return {'saturation_mask': saturation_mask,
            'largest_saturation_loop_mask': largest_saturation_loop_mask}

  def GetFailureCount(self, timeseries):
    """Returns the counts of True in the `saturation_mask` array."""
    if timeseries is None:
      return None
    return np.sum(timeseries['saturation_mask'])

  def GetIndexOfFirstFailure(self, timeseries):
    """Returns the index of the first True in the `saturation_mask` array."""
    if timeseries is None:
      return None
    if self.GetFailureCount(timeseries) == 0:
      return -1
    else:
      return np.where(timeseries['saturation_mask'])[0][0]

  def _GetRudderLimits(self, beta, airspeed):
    # Tables below are copied from https://goo.gl/BmyhAe. See spreadsheet for
    # description of how the tables are derived.
    betas = np.array([-20.0, -10.0, 0.0, 10.0, 20.0])
    airspeeds = np.array([50.0, 55.0, 60.0, 65.0, 70.0, 75.0,
                          80.0, 85.0, 90.0, 95.0, 100.0])
    rudder_limit_lower = np.array([[-22.00, -22.00, -22.00, -22.00, -22.00],
                                   [-22.00, -22.00, -22.00, -22.00, -22.00],
                                   [-22.00, -22.00, -22.00, -22.00, -16.42],
                                   [-22.00, -22.00, -22.00, -22.00, -10.57],
                                   [-22.00, -22.00, -22.00, -19.54, -5.92],
                                   [-22.00, -22.00, -22.00, -15.72, -2.16],
                                   [-22.00, -22.00, -22.00, -12.59, 0.91],
                                   [-22.00, -22.00, -22.00, -10.00, 3.45],
                                   [-22.00, -22.00, -21.31, -7.83, 5.58],
                                   [-22.00, -22.00, -19.49, -5.99, 7.39],
                                   [-22.00, -22.00, -17.94, -4.43, 8.93]])
    rudder_limit_upper = np.array([[22.00, 22.00, 22.00, 22.00, 22.00],
                                   [17.67, 22.00, 22.00, 22.00, 22.00],
                                   [9.87, 22.00, 22.00, 22.00, 22.00],
                                   [3.80, 17.64, 22.00, 22.00, 22.00],
                                   [-1.02, 12.83, 22.00, 22.00, 22.00],
                                   [-4.91, 8.94, 21.97, 22.00, 22.00],
                                   [-8.09, 5.76, 18.88, 22.00, 22.00],
                                   [-10.73, 3.12, 16.32, 22.00, 22.00],
                                   [-12.94, 0.91, 14.17, 22.00, 22.00],
                                   [-14.81, -0.96, 12.35, 22.00, 22.00],
                                   [-16.40, -2.55, 10.80, 22.00, 22.00]])

    lower_limit_lookup = interpolate.interp2d(
        betas, airspeeds, rudder_limit_lower, kind='linear')
    upper_limit_lookup = interpolate.interp2d(
        betas, airspeeds, rudder_limit_upper, kind='linear')

    lower_limit = np.zeros(len(beta))
    upper_limit = np.zeros(len(beta))
    for ii in range(len(beta)):
      lower_limit[ii] = lower_limit_lookup(
          np.rad2deg(beta[ii]), airspeed[ii]) + 0.25
      upper_limit[ii] = upper_limit_lookup(
          np.rad2deg(beta[ii]), airspeed[ii]) - 0.25

    return {
        'lower_limit': lower_limit,
        'upper_limit': upper_limit
    }
