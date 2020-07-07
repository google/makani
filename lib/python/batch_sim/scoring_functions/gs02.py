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

"""Scoring functions relating to the ground station v2."""

from makani.avionics.common import pack_avionics_messages
from makani.lib.python.batch_sim import scoring_functions
import numpy as np
from scipy.signal import periodogram
import scoring_functions_util as scoring_util


def diff_wrap_angle(n_revs, angle):
  """Returns angle difference wrapped between [0, 2*pi*n_revs).

  Args:
    n_revs: Integer indicating the number of revolutions.
    angle: Numpy array with the angles to be diff'ed and wrapped.

  Returns:
    Numpy array with the input angle diff'ed and wrapped.
  """
  wrapping_angle = n_revs * np.pi
  angle_diff = np.diff(np.mod(angle, 2.0 * wrapping_angle))
  angle_diff[np.where(angle_diff >= wrapping_angle)] -= 2.0 * wrapping_angle
  return angle_diff


class GsgYokeAnglesScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the gsg yoke angle falls outside its physical limits."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(GsgYokeAnglesScoringFunction, self).__init__(
        'Gsg Yoke Range', 'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['gs02']

  def GetValue(self, output):
    return np.array([output['gsg_yoke_min'],
                     output['gsg_yoke_max']])

  def GetOutput(self, timeseries):
    gsg_yoke = timeseries['gsg_yoke']
    return {
        'gsg_yoke_max': np.max(gsg_yoke),
        'gsg_yoke_min': np.min(gsg_yoke)
    }

  def GetTimeSeries(self, params, sim, control):
    gsg_yoke = self._SelectTelemetry(sim, control, 'gsg_yoke')
    return {'gsg_yoke': np.rad2deg(gsg_yoke)}


class GsgTerminationAnglesScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the gsg termination angle falls outside its physical limits."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(GsgTerminationAnglesScoringFunction, self).__init__(
        'Gsg Termination Range', 'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['gs02']

  def GetValue(self, output):
    return np.array([output['gsg_termination_min'],
                     output['gsg_termination_max']])

  def GetOutput(self, timeseries):
    gsg_termination = timeseries['gsg_termination']
    return {
        'gsg_termination_max': np.max(gsg_termination),
        'gsg_termination_min': np.min(gsg_termination)
    }

  def GetTimeSeries(self, params, sim, control):
    gsg_termination = self._SelectTelemetry(sim, control, 'gsg_termination')
    return {'gsg_termination': np.rad2deg(gsg_termination)}


class GsAzimuthErrorScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Checks if azimuth error falls within design limits in high tension mode."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(GsAzimuthErrorScoringFunction, self).__init__(
        'GS02 Azimuth Error Range', 'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['gs02']

  def GetValue(self, output):
    return np.array([output['gs_azimuth_error_min'],
                     output['gs_azimuth_error_max']])

  def GetOutput(self, timeseries):
    gs_azimuth_error = timeseries['gs_azimuth_error']
    return {
        'gs_azimuth_error_max': np.max(gs_azimuth_error),
        'gs_azimuth_error_min': np.min(gs_azimuth_error)
    }

  def GetTimeSeries(self, params, sim, control):
    gs_azimuth_error = self._SelectTelemetry(sim, control, 'gs_azimuth_error')
    return {'gs_azimuth_error': np.rad2deg(gs_azimuth_error)}


class GsDetwistCommandJumpScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """"Checks if jumps in the detwist command fall within acceptable limits."""

  def __init__(self, good_limit, bad_limit, severity):
    super(GsDetwistCommandJumpScoringFunction, self).__init__(
        'Detwist Command Jump', 'deg', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['gs02', 'control', 'experimental']

  def GetValue(self, output):
    return output['max_detwist_cmd_jump']

  def GetOutput(self, output):
    return {
        'max_detwist_cmd_jump': np.max(np.abs(output['gs_detwist_cmd_diff']))
    }

  def GetTimeSeries(self, params, sim, control):
    # The detwist command is in [0, TETHER_DETWIST_REVS * 2 * pi).
    gs_detwist_cmd = self._SelectTelemetry(sim, control, ['gs_detwist_cmd'])
    diff = diff_wrap_angle(
        pack_avionics_messages.TETHER_DETWIST_REVS, gs_detwist_cmd)

    return {'gs_detwist_cmd_diff': np.rad2deg(diff)}


class GsDetwistCommandRateScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """"Checks if the detwist command derivative is within acceptable limits."""

  def __init__(self, good_limit, bad_limit, severity):
    super(GsDetwistCommandRateScoringFunction, self).__init__(
        'Detwist Command Rate', 'deg/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['gs02', 'control']

  def GetValue(self, output):
    return output['max_detwist_cmd_rate']

  def GetOutput(self, output):
    return {
        'max_detwist_cmd_rate': np.max(np.abs(output['gs_detwist_cmd_rate']))
    }

  def GetTimeSeries(self, params, sim, control):
    # The detwist command is in [0, TETHER_DETWIST_REVS * 2 * pi).
    time, gs_detwist_cmd = self._SelectTelemetry(
        sim, control, ['time', 'gs_detwist_cmd'])
    dt = scoring_util.GetTimeSamp(time)
    if np.isnan(dt):
      return {'gs_detwist_cmd_rate': np.array([float('nan')])}

    gs_detwist_cmd_diff = diff_wrap_angle(
        pack_avionics_messages.TETHER_DETWIST_REVS, gs_detwist_cmd)
    gs_detwist_cmd_deriv = gs_detwist_cmd_diff / dt

    return {'gs_detwist_cmd_rate': np.rad2deg(gs_detwist_cmd_deriv)}


class GsDetwistOscillationsScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """"Checks if there exist oscillations in the detwist angle."""

  def __init__(self, good_limit, bad_limit, severity):
    super(GsDetwistOscillationsScoringFunction, self).__init__(
        'Detwist Oscillations', 'dB', good_limit, bad_limit, severity)
    self.SetSourcePriority(['control'])

  def GetSystemLabels(self):
    return ['gs02', 'control']

  def GetValue(self, output):
    return output['gs_detwist_max_ratio']

  def GetOutput(self, output):
    if output['gs_detwist_ratios_per_loop']:
      lobe_power_ratio = [r[2] for r in output['gs_detwist_ratios_per_loop']]
    else:
      lobe_power_ratio = np.nan

    return {
        'gs_detwist_max_ratio': np.max(lobe_power_ratio)
    }

  def GetTimeSeries(self, params, sim, control):
    time, gs_detwist_pos, loop_angle = self._SelectTelemetry(
        sim, control, ['time', 'gs_detwist_pos', 'loop_angle'])
    accum_loops = np.cumsum(np.diff(loop_angle) > np.deg2rad(350.))
    dt = scoring_util.GetTimeSamp(time)
    if np.isnan(dt):
      return {'gs_detwist_ratios_per_loop': []}

    # Obtain the derivative of the detwist angle to remove the steadily
    # decreasing ramp and the jump at every revolution. The frequency content
    # is preserved.
    # Wrapping first the detwist position to [0, 2*pi).
    gs_detwist_pos_diff = diff_wrap_angle(
        pack_avionics_messages.TETHER_DETWIST_REVS, gs_detwist_pos)
    gs_detwist_pos_deriv = gs_detwist_pos_diff / dt

    # Obtain the number of points in the FFT based on the desired frequency
    # resolution and the sampling time: frequency_resolution ~= fs/nfft Hz.
    frequency_resolution = 0.1  # [Hz]
    nfft = 2.0 ** np.ceil(np.log2(1.0 / dt / frequency_resolution))

    # Iterate through all loops.
    num_loops = np.floor(accum_loops[-1])
    if np.isnan(num_loops):
      num_loops = 0
    gs_detwist_ratios_per_loop = []
    for loop in range(1, int(num_loops) + 1):
      detwist_deriv_this_loop = gs_detwist_pos_deriv[
          np.where(np.floor(accum_loops).astype(int) == loop)]

      f_this_loop, pxx_this_loop = periodogram(detwist_deriv_this_loop,
                                               fs=1.0 / dt, nfft=int(nfft),
                                               scaling='spectrum',
                                               detrend=False)

      # Find the peaks in the spectrum.
      # Peaks are where the derivative changes sign and the second derivative
      # is negative.
      pxx_diff_this_loop = np.diff(10.0 * np.log10(pxx_this_loop))
      pxx_diff_sign_change_this_loop = (pxx_diff_this_loop[1:] *
                                        pxx_diff_this_loop[0:-1])
      pxx_diff_diff_this_loop = np.diff(pxx_diff_this_loop)
      min_freq = 0.25  # [Hz]
      pxx_peaks_idx_this_loop = np.where(
          (pxx_diff_sign_change_this_loop < 0.) &
          (pxx_diff_diff_this_loop < 0.) &
          (f_this_loop[0:-2] > min_freq))[0] + 1

      if pxx_peaks_idx_this_loop.size == 0:
        # No peaks were found.
        continue

      # Get the highest secondary lobe.
      max_peak_this_loop_db = 10.0 * np.log10(
          np.max(pxx_this_loop[pxx_peaks_idx_this_loop]))
      max_ratio_this_loop_db = (max_peak_this_loop_db -
                                10.0 * np.log10(pxx_this_loop[0]))
      f_secondary_this_loop = f_this_loop[
          pxx_peaks_idx_this_loop[
              np.argmax(pxx_this_loop[pxx_peaks_idx_this_loop])]]

      # Store tuple (loop number, lobe frequency [Hz], lobe power ratio [dB]).
      gs_detwist_ratios_per_loop.append((loop + 1, f_secondary_this_loop,
                                         max_ratio_this_loop_db))

    return {'gs_detwist_ratios_per_loop': gs_detwist_ratios_per_loop}


class GSTetherTwistScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """"Checks the number of tether twists."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(GSTetherTwistScoringFunction, self).__init__(
        'Tether twists', '#', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['gs02', 'control']

  def GetValue(self, output):
    return output['peak_tether_twists']

  def GetOutput(self, output):
    max_value = np.max(output['tether_twists'])
    min_value = np.min(output['tether_twists'])
    return {
        'peak_tether_twists': max_value if
                              max_value > np.abs(min_value) else min_value
    }

  def GetTimeSeries(self, params, sim, control):
    accum_kite_loops = self._SelectTelemetry(sim, control, ['accum_kite_loops'])
    accum_detwist_loops = self._SelectTelemetry(sim, control,
                                                ['accum_detwist_loops'])
    return {'tether_twists': accum_kite_loops - accum_detwist_loops}
