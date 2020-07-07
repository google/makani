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

"""Scoring functions relating to the kite's kinematics."""

from makani.lib.python.batch_sim import scoring_functions
from makani.lib.python.h5_utils import numpy_utils
import numpy as np
import scoring_functions_util as scoring_util


class PitchRateScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if there is excessive pitch rate."""

  def __init__(self, good_limit, bad_limit, severity):
    super(PitchRateScoringFunction, self).__init__(
        'Pitch Rate', 'rad/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['aero', 'controls']

  def GetValue(self, output):
    return np.max((np.abs(output['pitch_rate_min']),
                   np.abs(output['pitch_rate_max'])))

  def GetOutput(self, timeseries):
    return {
        'pitch_rate_min': np.min(timeseries['pitch_rate']),
        'pitch_rate_max': np.max(timeseries['pitch_rate'])
    }

  def GetTimeSeries(self, params, sim, control):
    body_rates = self._SelectTelemetry(sim, control, 'body_rates')
    return {'pitch_rate': body_rates['y']}


class AzimuthDegNoGoZoneScoringFunction(
    scoring_functions.WrappedLimitScoringFunction):
  """Test if the azimuth is within a no-go zone."""

  def __init__(self, bad_start_limit, bad_end_limit, tol,
               wrap_left, wrap_right, severity):
    super(AzimuthDegNoGoZoneScoringFunction, self).__init__(
        'Azimuth in No Go Zone', 'deg', bad_start_limit, bad_end_limit, tol,
        wrap_left, wrap_right, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['azim_in_no_go_max']

  def GetOutput(self, timeseries):
    return {'azim_in_no_go_max': np.max(timeseries['azim_in_no_go'])}

  def GetTimeSeries(self, params, sim, control):
    wing_pos_g = self._SelectTelemetry(sim, control, 'wing_xg')
    azimuth = np.arctan2(wing_pos_g['y'], wing_pos_g['x'])
    azimuth = np.rad2deg(azimuth)

    azim_in_no_go = scoring_util.GetDistToWrappedLimits(
        azimuth, self._bad_start_limit, self._bad_end_limit,
        self._wrap_left, self._wrap_right)

    return {'azim_in_no_go': azim_in_no_go}


class MaxWingOverflyGSScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if the wing overflies the GS by some limit into wind.

  Positive values indicate wing has crossed vertical plane at
  ground station perpendicular to prevailing wind direction.
  """

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxWingOverflyGSScoringFunction, self).__init__(
        'Max. Wing Overfly GS', 'm', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['wing_overfly_gs_max']

  def GetOutput(self, timeseries):
    return {
        'wing_overfly_gs_max': np.nanmax(timeseries['overfly_dist'])
    }

  def GetTimeSeries(self, params, sim, control):
    wing_g, wind_g = self._SelectTelemetry(
        sim, control, ['wing_xg', 'wind_g_vector_f_slow'])

    # Only want xy ground components of wind and position.
    wing_xy = numpy_utils.Vec3ToArray(wing_g)[:, :2]
    wind_xy = numpy_utils.Vec3ToArray(wind_g)[:, :2]

    wind_mag = np.linalg.norm(wind_xy, axis=1)

    # Protect against divide by zero.
    # Overfly distance will be nan if wind_mag is zero.
    wind_mag[wind_mag == 0.0] = [float('nan')]

    # New axis for numpy to broadcast correctly for division.
    wind_unit_vec = wind_xy / wind_mag[:, np.newaxis]

    # Vectorized dot product of wing_xy and wind_unit_vec.
    overfly_dist = -np.sum(wing_xy * wind_unit_vec, axis=1)

    if np.all(np.isnan(overfly_dist)):
      return {'overfly_dist': float('nan')}
    else:
      return {'overfly_dist': overfly_dist}


class MaxElevationAngleDegScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if the maximum elevation angle is in bounds."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MaxElevationAngleDegScoringFunction, self).__init__(
        'Max. Elevation Angle', 'deg', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['elevation_angle_g_max']

  def GetOutput(self, timeseries):
    return {
        'elevation_angle_g_max': np.max(timeseries['elevation_angle_g'])
        }

  def GetTimeSeries(self, params, sim, control):
    wing_xg = self._SelectTelemetry(sim, control, 'wing_xg')
    elevation_angle_g = np.arctan2(-wing_xg['z'],
                                   np.hypot(wing_xg['x'], wing_xg['y']))

    return {'elevation_angle_g': np.rad2deg(elevation_angle_g)}


# TODO: This scoring function has params hardcoded which could are
# specific to the m600. Update this function so that the params can account
# properly for both oktoberkite and m600 configs.
class TetherSphereDeviationScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test how far the kite deviates from the nominal tether sphere."""

  def __init__(self, good_limit, bad_limit, severity,
               mean_tether_tension=100e3):
    super(TetherSphereDeviationScoringFunction, self).__init__(
        'Max. Tether Sph Dev (mean tension = %g kN)'
        % (mean_tether_tension / 1e3), 'm', good_limit, bad_limit, severity)
    self._mean_tether_tension = mean_tether_tension

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['tether_sphere_deviation_max']

  def GetOutput(self, timeseries):
    deviation = timeseries['tether_sphere_deviation']
    if np.size(deviation):
      max_deviation = np.abs(np.min(deviation))
    else:
      max_deviation = float('nan')

    return {'tether_sphere_deviation_max': max_deviation}

  def GetTimeSeries(self, params, sim, control):
    # Tether sphere radius at mean tether tension.
    # - The 0.75 factor applied to the tether tensile stiffness is used to
    #   account for catenary effects.
    # - The extra 0.16 meter offset is used to account for the distance from
    #   the bridle pivot to the origin of the body frame.
    tether_params = params['system_params']['tether']
    wing_params = params['system_params']['wing']
    mean_tether_sphere_radius = (
        tether_params['length'] +
        self._mean_tether_tension / (0.75 * tether_params['tensile_stiffness'] /
                                     tether_params['length']) +
        wing_params['bridle_rad'] +
        0.16)

    wing_xg, tether_xg_start = self._SelectTelemetry(
        sim, control, ['wing_xg', 'tether_xg_start'])

    if (scoring_util.IsSelectionValid(wing_xg) and
        scoring_util.IsSelectionValid(tether_xg_start)):
      tether_norm = np.linalg.norm(
          numpy_utils.Vec3ToArray(wing_xg)
          - numpy_utils.Vec3ToArray(tether_xg_start), axis=1)
      deviation = tether_norm - mean_tether_sphere_radius
    else:
      deviation = np.array([], dtype=float)

    return {'tether_sphere_deviation': deviation}


class KiteHeightAglScoringFunctionWrapper(
    scoring_functions.ScoringFunctionWrapper):
  """Wrapper class to evaluate the kite height above ground level."""

  def __init__(self, scoring_function):
    if not (isinstance(scoring_function, MinKiteHeightAglScoringFunction) or
            isinstance(scoring_function, KiteHeightAglScoringFunction)):
      raise TypeError('The wrapped scoring function can only be of type'
                      'MinKiteHeightAglScoringFunction or'
                      'KiteHeightAglScoringFunction.')
    self._scoring_function = scoring_function
    super(KiteHeightAglScoringFunctionWrapper, self).__init__(
        scoring_function.name, scoring_function)

  def GetSystemLabels(self):
    return ['controls']

  def GetTimeSeries(self, params, sim, control):
    height = self._SelectTelemetry(sim, control, 'wing_xg')['z']
    ground_z = params['system_params']['ground_frame']['ground_z']

    # Maintain backwards compatibility with RPX-09 and previous logs.
    if 'tail_spike_pos' in params['system_params']['wing'].dtype.names:
      tail_spike_pos = [params['system_params']['wing']['tail_spike_pos']['x'],
                        params['system_params']['wing']['tail_spike_pos']['y'],
                        params['system_params']['wing']['tail_spike_pos']['z']]
      r_kite_sphere = np.linalg.norm(tail_spike_pos)
    else:
      # Position [m] of the tip of the tail in body coordinates.
      h_tail_pos = [params['system_params']['wing']['horizontal_tail_pos']['x'],
                    params['system_params']['wing']['horizontal_tail_pos']['y'],
                    params['system_params']['wing']['horizontal_tail_pos']['z']]
      r_kite_sphere = np.linalg.norm(h_tail_pos)

    clearance_agl = ground_z - (height + r_kite_sphere)

    return {'clearance_agl': clearance_agl}


class MinKiteHeightAglScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the kite has flown below a minimum threshold."""

  def __init__(self, good_limit, bad_limit, severity):
    super(MinKiteHeightAglScoringFunction, self).__init__(
        'Min Kite Height Above Ground Level', 'm', good_limit, bad_limit,
        severity)

  def GetSystemLabels(self):
    raise NotImplementedError('This class should be wrapped by the'
                              'KiteHeightAglScoringFunctionWrapper, which'
                              'implements this method.')

  def GetValue(self, output):
    return output['kite_height_agl_min']

  def GetOutput(self, timeseries):
    return {
        'kite_height_agl_min': np.min(timeseries['clearance_agl'])
    }

  def GetTimeSeries(self):
    raise NotImplementedError('This class should be wrapped by the'
                              'KiteHeightAglScoringFunctionWrapper, which'
                              'implements this method.')


class KiteHeightAglScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the kite has flown outside of an allowable range."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(KiteHeightAglScoringFunction, self).__init__(
        'Kite Height Above Ground Level', 'm', bad_lower_limit,
        good_lower_limit, good_upper_limit, bad_upper_limit, severity)

  def GetValue(self, output):
    return np.array([output['kite_height_agl_min'],
                     output['kite_height_agl_max']])

  def GetOutput(self, timeseries):
    return {
        'kite_height_agl_min': np.min(timeseries['clearance_agl']),
        'kite_height_agl_max': np.max(timeseries['clearance_agl']),
    }
