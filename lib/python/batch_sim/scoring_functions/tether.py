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

"""Scoring functions relating to the tether."""

from makani.lib.python.batch_sim import scoring_functions
import numpy as np
import pandas as pd
import scoring_functions_util as scoring_util


class TetherHeightAglMinScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if a minimum tether height AGL limit is met."""

  def __init__(self, good_limit, bad_limit, severity):
    super(TetherHeightAglMinScoringFunction, self).__init__(
        'Tether Height AGL Min.', 'm', good_limit, bad_limit, severity)
    assert good_limit > bad_limit

  def GetSystemLabels(self):
    return ['controls']

  def GetValue(self, output):
    return output['tether_height_agl_min']

  def GetOutput(self, timeseries):
    return {
        'tether_height_agl_min': np.min(timeseries['tether_height_agl'])
    }

  def GetTimeSeries(self, params, sim, control):
    # TODO(b/110039916): This scoring function considers ALL tether
    # nodes, including inactive ones for which the z coordinate is
    # reported as zero.
    tether_xg_nodes = self._SelectTelemetry(sim, control, 'tether_xg_nodes')
    if np.size(tether_xg_nodes) == 1:
      tether_xg_nodes_z = np.array([float('nan')])
    else:
      tether_xg_nodes_z = tether_xg_nodes['z']

    tether_agl = (
        params['system_params']['ground_frame']['ground_z']
        - tether_xg_nodes_z)

    return {'tether_height_agl': tether_agl}


class TensionScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if tension limits are met."""

  def __init__(self, name, good_limit, bad_limit, severity):
    super(TensionScoringFunction, self).__init__(
        name, 'kN', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls', 'loads', 'tether']

  def GetTimeSeries(self, params, sim, control):
    tether_tension, time = self._SelectTelemetry(
        sim, control, ['tether_tension', 'time'])

    if not scoring_util.IsSelectionValid(tether_tension):
      return {'time': [], 'tension_kn': []}
    else:
      return {'time': time, 'tension_kn': tether_tension.flatten() * 1e-3}

  def GetOutput(self, output):
    raise NotImplementedError

  def GetValue(self, output):
    raise NotImplementedError

  def GetFailureCount(self, timeseries):
    if timeseries is None:
      return None
    return super(TensionScoringFunction, self).GetFailureCount(
        {'tension_kn': timeseries['tension_kn']})

  def GetIndexOfFirstFailure(self, timeseries):
    if timeseries is None:
      return None
    if self.GetFailureCount(timeseries) == 0:
      return -1
    else:
      return super(TensionScoringFunction, self).GetIndexOfFirstFailure(
          {'tension_kn': timeseries['tension_kn']})


class TensionMaxScoringFunction(TensionScoringFunction):
  """Tests if a maximum tension limit is met."""

  def __init__(self, good_limit, bad_limit, severity):
    super(TensionMaxScoringFunction, self).__init__(
        'Max Tether Tension', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls', 'loads', 'tether']

  def GetValue(self, output):
    return output['tension_max']

  def GetOutput(self, timeseries):
    tension = timeseries['tension_kn']
    if np.size(tension):
      tension_max = np.max(tension)
    else:
      tension_max = float('nan')
    return {'tension_max': tension_max}


class TensionMinScoringFunction(TensionScoringFunction):
  """Tests if a minimum tension limit is met."""

  def __init__(self, good_limit, bad_limit, severity):
    super(TensionMinScoringFunction, self).__init__(
        'Min Tether Tension', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls', 'loads', 'tether']

  def GetValue(self, output):
    return output['tension_min']

  def GetOutput(self, timeseries):
    tension = timeseries['tension_kn']
    time = timeseries['time']

    if (scoring_util.IsSelectionValid(tension) and
        scoring_util.IsSelectionValid(time)):
      # Low pass, symmetrically (2nd order) filter the tension signal.
      cut_off_freq = 4.0
      tension_f = scoring_util.LpFiltFiltTimeSeries(
          time, tension, cut_off_freq)
      tension_min = np.min(tension_f)
    else:
      tension_min = float('nan')

    return {'tension_min': tension_min}


class TensionActualToExpectedScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Checks if tension is close to expected tension for commands."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(TensionActualToExpectedScoringFunction, self).__init__(
        'Tension Actual-to-Expected Ratio', '-',
        bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['controls', 'tether']

  def GetValue(self, output):
    return np.array([output['tension_ratio_min'],
                     output['tension_ratio_max']])

  def GetOutput(self, timeseries):
    return {
        'tension_ratio_min':
            np.min(timeseries['tension_actual_to_expected_cmd']),
        'tension_ratio_max':
            np.max(timeseries['tension_actual_to_expected_cmd'])
    }

  def GetTimeSeries(self, params, sim, control):
    telemetry_keys = ['tether_tension', 'airspeed_cmd', 'alpha_cmd']

    tether_tension, airspeed_cmd, alpha_cmd = (
        self._SelectTelemetry(sim, control, telemetry_keys))

    rhoair = params['sim_params']['phys_sim']['air_density']
    sref = params['system_params']['wing']['A']
    lift_coeff_at_zero_alpha = (
        params['control_params']['simple_aero_model']['CL_0'])
    lift_coeff_alpha_slope = (
        params['control_params']['simple_aero_model']['dCL_dalpha'])

    lift_coeff_cmd = (
        lift_coeff_alpha_slope * alpha_cmd + lift_coeff_at_zero_alpha)

    # Assume tension nearly equal to lift.
    tether_tension_expect_cmd = (
        0.5 * rhoair * sref * lift_coeff_cmd * airspeed_cmd**2)

    return {
        'tension_actual_to_expected_cmd': (
            tether_tension / tether_tension_expect_cmd)
    }


class PitchDegScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the tether pitch exceeds a limit after a given start time."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, tension_threshold=0.0):
    super(PitchDegScoringFunction, self).__init__(
        'Tether Pitch Range (tension > %g kN)' % (tension_threshold / 1e3),
        'deg', bad_lower_limit, good_lower_limit, good_upper_limit,
        bad_upper_limit, severity)
    self._tension_threshold = tension_threshold

  def GetSystemLabels(self):
    return ['controls', 'tether']

  def GetValue(self, output):
    return np.array([output['tether_pitch_min'],
                     output['tether_pitch_max']])

  def GetOutput(self, timeseries):
    tether_pitch = timeseries['tether_pitch']
    return {
        'tether_pitch_max': np.max(tether_pitch),
        'tether_pitch_min': np.min(tether_pitch)
    }

  def GetTimeSeries(self, params, sim, control):
    tether_tension, tether_pitch = self._SelectTelemetry(
        sim, control, ['tether_tension', 'tether_pitch'])

    data_indices = np.argwhere(tether_tension > self._tension_threshold)
    if data_indices.size > 0:
      tether_pitch = tether_pitch[data_indices]
    else:
      tether_pitch = np.deg2rad(np.array([(self._good_lower_limit +
                                           self._good_upper_limit) / 2]))

    return {'tether_pitch': np.rad2deg(tether_pitch)}


class RollDegScoringFunction(scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the tether roll exceeds limits."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, tension_threshold=0.0):
    super(RollDegScoringFunction, self).__init__(
        'Tether Roll Range (tension > %g kN)' % (tension_threshold / 1e3),
        'deg', bad_lower_limit, good_lower_limit, good_upper_limit,
        bad_upper_limit, severity)
    self._tension_threshold = tension_threshold

  def GetSystemLabels(self):
    return ['controls', 'tether']

  def GetValue(self, output):
    return np.array([output['tether_roll_min'],
                     output['tether_roll_max']])

  def GetOutput(self, timeseries):
    tether_roll = timeseries['tether_roll']
    return {
        'tether_roll_max': np.max(tether_roll),
        'tether_roll_min': np.min(tether_roll)
    }

  def GetTimeSeries(self, params, sim, control):
    tether_tension, tether_roll = self._SelectTelemetry(
        sim, control, ['tether_tension', 'tether_roll'])

    data_indices = np.argwhere(tether_tension > self._tension_threshold)
    if data_indices.size > 0:
      tether_roll = tether_roll[data_indices]
    else:
      tether_roll = np.deg2rad(np.array([(self._good_lower_limit +
                                          self._good_upper_limit) / 2]))

    return {'tether_roll': np.rad2deg(tether_roll)}


class PitchMomentCoefficientScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the pitch moment coefficient due to the tether is too large."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity):
    super(PitchMomentCoefficientScoringFunction, self).__init__(
        'Tether Pitch Moment Coefficient', '#', bad_lower_limit,
        good_lower_limit, good_upper_limit, bad_upper_limit, severity)

  def GetSystemLabels(self):
    return ['controls', 'tether']

  def GetValue(self, output):
    return np.array([output['cm_tether_min'], output['cm_tether_max']])

  def GetOutput(self, timeseries):
    return {
        'cm_tether_max': np.max(timeseries['cm_tether']),
        'cm_tether_min': np.min(timeseries['cm_tether'])
    }

  def GetTimeSeries(self, params, sim, control):
    area = params['system_params']['wing']['A']
    c = params['system_params']['wing']['c']
    airspeed, tether_moment = self._SelectTelemetry(
        sim, control, ['airspeed', 'tether_moment'])
    dynamic_pressure = (0.5 * params['system_params']['phys']['rho']
                        * airspeed**2.0)
    if scoring_util.IsSelectionValid(tether_moment):
      cm_tether = tether_moment['y'] / (dynamic_pressure * area * c)
    else:
      cm_tether = np.array([float('nan')])

    return {'cm_tether': cm_tether}


# TODO: GetTimeseries must return loop based pitch timeseries data.
class SustainedPitchDegScoringFunction(
    scoring_functions.DoubleSidedLimitScoringFunction):
  """Tests if the tether pitch exceeds limits for a sustained duration."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, sustained_duration=0.0):
    super(SustainedPitchDegScoringFunction, self).__init__(
        'Tether Pitch Range (tension = 0 kN, duration = %g s)'
        % sustained_duration, 'deg', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    self._sustained_duration = sustained_duration

  def GetSystemLabels(self):
    return ['controls', 'tether']

  def GetValue(self, output):
    return np.array([output['min_sustained_tether_pitch'],
                     output['max_sustained_tether_pitch']])

  def GetTimeSeries(self, params, sim, control):
    time, tether_pitch = self._SelectTelemetry(sim, control,
                                               ['time', 'tether_pitch'])
    if (not scoring_util.IsSelectionValid(time) or
        not scoring_util.IsSelectionValid(tether_pitch)):
      return {
          'min_sustained_tether_pitch': float('nan'),
          'max_sustained_tether_pitch': float('nan')
      }

    tether_pitch_deg = np.rad2deg(tether_pitch)

    # Low pass, symmetrically (2nd order) filter the tether pitch.
    cut_off_freq = 2.0
    tether_pitch_deg_f = scoring_util.LpFiltFiltTimeSeries(
        time, tether_pitch_deg, cut_off_freq)

    min_sustained_tether_pitch, max_sustained_tether_pitch = (
        scoring_util.GetSustainedValue(
            tether_pitch_deg_f, self._good_lower_limit, self._good_upper_limit,
            self._sustained_duration, scoring_util.GetTimeSamp(time)))

    return {
        'min_sustained_tether_pitch': min_sustained_tether_pitch,
        'max_sustained_tether_pitch': max_sustained_tether_pitch
    }


class RollPeriodScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Tests if the period of the roll mode in hover is below allowable limit."""

  def __init__(self, good_limit, bad_limit, severity):
    super(RollPeriodScoringFunction, self).__init__(
        'Hover Roll Period', 's', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['controls', 'tether']

  def GetValue(self, output):
    return output['max_roll_period']

  def GetOutput(self, timeseries):
    roll_period = timeseries['roll_period']
    return {'max_roll_period': np.max(roll_period)}

  def GetTimeSeries(self, params, sim, control):
    tension, tether_roll, tether_pitch, time = self._SelectTelemetry(
        sim, control, ['tether_tension', 'tether_roll', 'tether_pitch', 'time'])
    bridle_y_offset = params['system_params']['wing']['bridle_y_offset']
    bridle_pos_z = params['system_params']['wing']['bridle_pos']['z'][0, 0]
    bridle_rad = params['system_params']['wing']['bridle_rad']
    center_of_mass = params['system_params']['wing']['center_of_mass_pos']
    roll_inertia = np.matrix(params['system_params']['wing']['I']['d'])[0, 0]

    if scoring_util.IsSelectionValid(tension):
      bridle_roll_stiffness = tension * (
          - bridle_y_offset * np.sin(tether_roll) * np.cos(tether_pitch)
          + bridle_rad * np.cos(tether_roll) * np.cos(tether_pitch)
          + center_of_mass['y'] * np.sin(tether_roll) * np.cos(tether_pitch)
          - (center_of_mass['z'] - bridle_pos_z) * np.cos(tether_roll))

      # Low pass, symmetrically (2nd order) filter the computed roll stiffness.
      cut_off_freq = 0.5
      bridle_roll_stiffness_f = scoring_util.LpFiltFiltTimeSeries(
          time, bridle_roll_stiffness, cut_off_freq)

      roll_period = np.divide(2 * np.pi,
                              np.sqrt(np.maximum(bridle_roll_stiffness_f, 1e-3)
                                      / roll_inertia))
      return {'roll_period': roll_period}

    else:
      return {'roll_period': np.array([float('nan')])}


class TetherTensionOscillationScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Detects tether tension oscillations."""

  def __init__(self, good_limit, bad_limit, severity, cut_off_freq=1.0/30.0):
    super(TetherTensionOscillationScoringFunction, self).__init__(
        ('Tether Tension Oscillations'),
        'kN', good_limit, bad_limit, severity)
    self._cut_off_freq = cut_off_freq
    self._t_window = 2./self._cut_off_freq

  def GetSystemLabels(self):
    return ['controls', 'tether', 'experimental']

  def GetValue(self, output):
    return output['tether_tension_std']

  def GetOutput(self, timeseries):
    return {'tether_tension_std': np.nanmax(timeseries['tether_tension_std'])}

  def GetTimeSeries(self, params, sim, control):
    time, tether_tension = self._SelectTelemetry(
        sim, control, ['time', 'tether_tension'])
    if not scoring_util.IsSelectionValid(tether_tension):
      return {'tether_tension_std': np.array([float('nan')])}

    # Low pass, symmetrically (2nd order) filter the tether tension.
    tether_tension_f = scoring_util.LpFiltFiltTimeSeries(
        time, tether_tension, self._cut_off_freq)

    # Calculate a rolling StDev of the difference between the raw signal and
    # the filtered signal.
    t_samp = scoring_util.GetTimeSamp(time)
    if np.isnan(t_samp):
      return {'tether_tension_std': np.array([float('nan')])}
    n_window = int(self._t_window / t_samp)
    tension_deviation_df = pd.DataFrame(tether_tension - tether_tension_f)
    tether_tension_std = tension_deviation_df.rolling(
        n_window, min_periods=n_window).std().values.flatten()

    return {'tether_tension_std': tether_tension_std * 1e-3}
