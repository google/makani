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

"""Scoring functions relating to the hover controller."""

from makani.analysis.control import geometry
from makani.avionics.common import plc_messages
from makani.lib.python import c_helpers
from makani.lib.python.batch_sim import scoring_functions
from makani.lib.python.h5_utils import numpy_utils
import numpy as np
import pandas as pd
import scoring_functions_util as scoring_util

_GROUND_STATION_MODE_HELPER = c_helpers.EnumHelper(
    'GroundStationMode', plc_messages)


class TetherElevationScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests the tether elevation."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, transform_stages=None,
               sustained_duration=0.0, extra_system_labels=None):
    super(TetherElevationScoringFunction, self).__init__(
        ('Tether Elevation %s' % transform_stages if transform_stages
         else 'Tether Elevation'),
        'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    self._sustained_duration = sustained_duration
    self._transform_stages = transform_stages
    self._system_labels = ['controls', 'tether elevation']
    if extra_system_labels:
      self._system_labels += extra_system_labels

  def GetSystemLabels(self):
    return self._system_labels

  def GetValue(self, output):
    if (np.isnan(output['tether_elevation_min']) or
        np.isnan(output['tether_elevation_max'])):
      return float('nan')

    return np.array([output['tether_elevation_min'],
                     output['tether_elevation_max']])

  def GetOutput(self, timeseries):
    tether_elevation = timeseries['tether_elevation']

    if tether_elevation is None:
      return {
          'tether_elevation_max': float('nan'),
          'tether_elevation_min': float('nan')
      }

    if self._transform_stages:
      gs02_mode = timeseries['gs02_mode']
      gs02_transform_stage = timeseries['gs02_transform_stage']
      mask = gs02_mode == _GROUND_STATION_MODE_HELPER.Value('Transform')

      stage_mask = np.zeros((np.size(tether_elevation),), dtype=bool)
      for s in self._transform_stages:
        stage_mask |= gs02_transform_stage == s

      mask &= stage_mask
      tether_elevation = tether_elevation[mask]

      if not np.size(tether_elevation):
        return {
            'tether_elevation_max': float('nan'),
            'tether_elevation_min': float('nan')
        }
      else:
        return {
            'tether_elevation_max': max(tether_elevation),
            'tether_elevation_min': min(tether_elevation),
        }
    else:
      if self._sustained_duration is None:
        return {
            'tether_elevation_max': max(tether_elevation),
            'tether_elevation_min': min(tether_elevation),
        }
      min_sustained_tether_ele, max_sustained_tether_ele = (
          scoring_util.GetSustainedValue(
              tether_elevation, self._good_lower_limit, self._good_upper_limit,
              self._sustained_duration, self._t_samp))
      return {
          'tether_elevation_max': max_sustained_tether_ele,
          'tether_elevation_min': min_sustained_tether_ele,
      }

  def GetTimeSeries(self, params, sim, control):
    # TODO: Use the Gs02TransformStageFilter.
    if self._transform_stages:
      gs02_mode, gs02_transform_stage, tether_elevation = (
          self._SelectTelemetry(
              sim, control, ['gs02_mode', 'gs02_transform_stage',
                             'tether_elevation']))

      if (not scoring_util.IsSelectionValid(gs02_mode) or
          not scoring_util.IsSelectionValid(gs02_transform_stage) or
          not scoring_util.IsSelectionValid(tether_elevation)):
        return {
            'gs02_mode': None,
            'gs02_transform_stage': None,
            'tether_elevation': None
        }
      else:
        tether_elevation_deg = np.rad2deg(tether_elevation)
        return {
            'gs02_mode': gs02_mode,
            'gs02_transform_stage': gs02_transform_stage,
            'tether_elevation': tether_elevation_deg
        }
    else:
      time, tether_elevation = self._SelectTelemetry(
          sim, control, ['time', 'tether_elevation'])
      if not (scoring_util.IsSelectionValid(tether_elevation) or
              scoring_util.IsSelectionValid(time)):
        return {
            'tether_elevation': None
        }
      tether_elevation_deg = np.rad2deg(tether_elevation)

      # Low pass, symmetrically (2nd order) filter the tether elevation.
      cut_off_freq = 0.4
      tether_elevation_deg_f = scoring_util.LpFiltFiltTimeSeries(
          time, tether_elevation_deg, cut_off_freq)
      self._t_samp = scoring_util.GetTimeSamp(time)

      # Return filtered data.
      return {
          'tether_elevation': tether_elevation_deg_f
      }


class TetherElevationOscillationScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the tether elevation oscillations."""

  def __init__(self, good_limit, bad_limit, severity, cut_off_freq=1.0/30.0):
    super(TetherElevationOscillationScoringFunction, self).__init__(
        ('Tether Elevation Oscillations'),
        'deg', good_limit, bad_limit, severity)
    self._cut_off_freq = cut_off_freq
    self._t_window = 2./self._cut_off_freq

  def GetSystemLabels(self):
    return ['controls', 'tether elevation']

  def GetValue(self, output):
    return output['tether_elevation_std']

  def GetOutput(self, timeseries):
    return {'tether_elevation_std': np.nanmax(
        timeseries['tether_elevation_std'])}

  def GetTimeSeries(self, params, sim, control):
    time, tether_elevation, tether_elevation_valid = self._SelectTelemetry(
        sim, control, ['time', 'tether_elevation', 'tether_elevation_valid'])
    if not (scoring_util.IsSelectionValid(tether_elevation) or
            scoring_util.IsSelectionValid(time)):
      return {'tether_elevation_std': np.array([float('nan')])}
    tether_elevation_deg = np.rad2deg(
        tether_elevation[tether_elevation_valid == 1])

    # Low pass, symmetrically (2nd order) filter the tether elevation.
    tether_elevation_deg_f = scoring_util.LpFiltFiltTimeSeries(
        time, tether_elevation_deg, self._cut_off_freq)

    # Calculate a rolling StDev of the difference between the raw signal and
    # the filtered signal.
    t_samp = scoring_util.GetTimeSamp(time)
    if np.isnan(t_samp):
      return {'tether_elevation_std': np.array([float('nan')])}
    n_window = int(self._t_window / t_samp)
    elevation_deviation_df = pd.DataFrame(
        tether_elevation_deg - tether_elevation_deg_f)
    tether_elevation_std = elevation_deviation_df.rolling(
        n_window, min_periods=n_window).std().values.flatten()

    return {'tether_elevation_std': tether_elevation_std}


class PanelAzimuthTrackingScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests whether the tether azimuth in p-frame is within acceptable limits."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, extra_system_labels=None):
    super(PanelAzimuthTrackingScoringFunction, self).__init__(
        'Panel Azimuth Limit', 'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    # TODO(b/143912508): Fix the numerous false positive cases before removing
    # the experimental label.
    self._system_labels = ['controls', 'experimental']
    if extra_system_labels:
      self._system_labels += extra_system_labels

  def GetSystemLabels(self):
    return self._system_labels

  def GetValue(self, output):
    if (np.isnan(output['tether_azimuth_min']) or
        np.isnan(output['tether_azimuth_max'])):
      return float('nan')

    return np.array([output['tether_azimuth_min'],
                     output['tether_azimuth_max']])

  def GetOutput(self, timeseries):
    tether_azimuth = timeseries['tether_azimuth_p']

    if tether_azimuth is None or tether_azimuth.size == 0:
      return {
          'tether_azimuth_max': float('nan'),
          'tether_azimuth_min': float('nan')
      }
    else:
      return {
          'tether_azimuth_max': max(tether_azimuth),
          'tether_azimuth_min': min(tether_azimuth),
      }

  def GetTimeSeries(self, params, sim, control):
    tether_azimuth, platform_azi = (
        self._SelectTelemetry(sim, control, ['tether_azimuth', 'platform_azi']))

    if not scoring_util.IsSelectionValid(tether_azimuth):
      return {
          'tether_azimuth_p': None,
      }
    else:
      # Only evaluate panel azimuth tracking when the kite is in the air.
      tether_azimuth = numpy_utils.Wrap(
          (tether_azimuth - platform_azi), -np.pi, np.pi)
      return {
          'tether_azimuth_p': np.rad2deg(tether_azimuth),
      }


class HoverPositionScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the maximum error in hover position."""

  def __init__(self, dim, good_limit, bad_limit, severity,
               dist_from_perch_limit=8.5):
    super(HoverPositionScoringFunction, self).__init__(
        'Position Error ' + dim, 'm', good_limit, bad_limit, severity)
    self._dim = dim
    assert dist_from_perch_limit > 0.0
    self._dist_from_perch_limit = dist_from_perch_limit

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['max_pos_err_g']

  def GetOutput(self, timeseries):
    return {'max_pos_err_g': np.max(timeseries['pos_err_g'])}

  def GetTimeSeries(self, params, sim, control):
    wing_xg, wing_pos_g_cmd = self._SelectTelemetry(
        sim, control, ['wing_xg', 'wing_pos_g_cmd'])

    pos_err_g = np.abs(wing_pos_g_cmd[self._dim] - wing_xg[self._dim])

    return {'pos_err_g': pos_err_g}


class PerchedPositionScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Evaluates the error between the perched position target and actual."""

  def __init__(self, good_limit, bad_limit, severity):
    super(PerchedPositionScoringFunction, self).__init__(
        'Perched Position Error', 'm', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    # Manually add descend flight mode since mode filter doesn't seem to work.
    # TODO(b/145831658): Figure out why flight mode filter doesn't work.
    return ['controls', 'kFlightModeHoverDescend']

  def GetValue(self, output):
    return output['xy_perched_pos_err']

  def GetOutput(self, timeseries):
    return {
        'xy_perched_pos_err': timeseries['xy_perched_pos_err']
    }

  def GetTimeSeries(self, params, sim, control):
    wing_xg, buoy_xg, dcm_g2v, platform_azi, gain_ramp = self._SelectTelemetry(
        sim, control, ['wing_xg', 'buoy_xg', 'dcm_g2v', 'platform_azi',
                       'hover_gain_ramp_scale'],
        flight_modes='kFlightModeHoverDescend')
    hover_path_params = params['control_params']['hover']['path']

    gain_ramp_down_idx = np.where(gain_ramp < 1e-8)

    if (np.size(gain_ramp_down_idx) == 0 or
        not scoring_util.IsSelectionValid(platform_azi)):
      xy_perched_pos_err = float('nan')
    else:
      last_gain_ramp_down_idx = gain_ramp_down_idx[0][0]
      dcm_v2p = geometry.AngleToDcm(
          platform_azi[last_gain_ramp_down_idx], 0.0, 0.0, 'ZYX')
      dcm_g2p = np.matmul(np.matrix(dcm_g2v[last_gain_ramp_down_idx, :, :]),
                          dcm_v2p)
      final_wing_pos_g = np.array(wing_xg[last_gain_ramp_down_idx].tolist())
      final_buoy_pos_g = np.array(buoy_xg[last_gain_ramp_down_idx].tolist())
      final_wing_pos_p = np.matmul(dcm_g2p, final_wing_pos_g - final_buoy_pos_g)
      perch_wing_pos_p = hover_path_params['perched_wing_pos_p'].tolist()
      perched_wing_pos_err = final_wing_pos_p - perch_wing_pos_p
      xy_perched_pos_err = np.sqrt(
          perched_wing_pos_err[0, 0]**2 + perched_wing_pos_err[0, 1]**2)

    return {'xy_perched_pos_err': xy_perched_pos_err}


class HoverAngleScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the approximate maximum error in hover attitude."""

  def __init__(self, dim, good_limit, bad_limit, severity,
               dist_from_perch_limit=8.5):
    super(HoverAngleScoringFunction, self).__init__(
        '(Approx.) Angle Error ' + dim, 'rad', good_limit, bad_limit, severity)
    self._dim = dim
    assert dist_from_perch_limit > 0.0
    self._dist_from_perch_limit = dist_from_perch_limit

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['approx_angle_error_b_max']

  def GetOutput(self, timeseries):
    return {
        'approx_angle_error_b_max': np.max(timeseries['approx_angle_error_b'])
    }

  def GetTimeSeries(self, params, sim, control):
    hover_angles, hover_angles_cmd = self._SelectTelemetry(
        sim, control, ['hover_angles', 'hover_angles_cmd'])

    # TODO: This angle error is approximate because it is
    # treating the axis-angle command and measured values as vectors.
    # The correct angle error is calculated in hover_angles.c.
    approx_angle_error_b = np.abs(hover_angles_cmd[self._dim]
                                  - hover_angles[self._dim])

    return {'approx_angle_error_b': approx_angle_error_b}


class HoverTensionControlScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests the efficacy of hover tension control."""

  def __init__(self, good_limit, bad_limit, severity):
    super(HoverTensionControlScoringFunction, self).__init__(
        'Hover Tension Error (RMS)', 'kN', good_limit, bad_limit, severity)
    self._sources = ['control']

  def GetSystemLabels(self):
    return ['controls', 'experimental']

  def GetValue(self, output):
    return output['tension_error_rms']

  def GetOutput(self, timeseries):
    return {
        'tension_error_rms': (
            np.sqrt(np.mean(timeseries['tension_error']**2.0))/1e3)
    }

  def GetTimeSeries(self, params, sim, control):
    tension, tension_cmd = self._SelectTelemetry(
        sim, control, ['tether_tension', 'tether_tension_cmd'])
    return {'tension_error': tension_cmd - tension}
