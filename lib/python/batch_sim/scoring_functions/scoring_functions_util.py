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

"""Utilities for scoring functions."""
import logging
from makani.lib.python.batch_sim import flight_modes as flight_modes_module
import numpy as np
from scipy import signal as sp_signal
from scipy import stats
from scipy.interpolate import interp1d

# Define telemetry selectors.
# To expand the dictionary, add variable in the format:
# 'name':{
# 'source_a': lambda a: a[path to 'name' in source_a dictionary],
# 'source_b': lambda b: b[path to 'name' in source_b dictionary],
# 'method': 'interpolation method', default is 'linear',
# ...}
_TELEMETRY_SELECTORS = {
    'time': {
        'sim': lambda s: s['time'],
        'control': lambda c: c['time']},
    'flight_mode': {
        'control': lambda c: c['flight_mode'],
        'method': 'nearest'},
    'flight_mode_time': {
        'control': lambda c: c['flight_mode_time']},
    'gs02_mode': {
        'control': lambda c: c['control_input']['gs_sensors']['mode'],
        'sim': lambda s: s['gs02']['mode']},
    'gs02_transform_stage': {
        'sim': lambda s: s['gs02']['transform_stage'],
        'control':
            lambda c: c['control_input']['gs_sensors']['transform_stage']},
    'airspeed': {
        'sim': lambda s: s['wing']['apparent_wind_b']['v'],
        'control': lambda c: c['state_est']['apparent_wind']['sph_f']['v']},
    'airspeed_cmd': {
        'control': lambda c: c['crosswind']['airspeed_cmd']},
    'apparent_wind_vector': {
        'control': lambda c: c['state_est']['apparent_wind']['vector']},
    'body_rates': {
        'sim': lambda s: s['wing']['omega'],
        'control': lambda c: c['state_est']['pqr']},
    'alpha': {
        'sim': lambda s: s['wing']['apparent_wind_b']['alpha'],
        'control': lambda c: c['state_est']['apparent_wind']['sph_f']['alpha']},
    'alpha_cmd': {
        'control': lambda c: c['crosswind']['alpha_cmd']},
    'beta': {
        'sim': lambda s: s['wing']['apparent_wind_b']['beta'],
        'control': lambda c: c['state_est']['apparent_wind']['sph_f']['beta']},
    'beta_cmd': {
        'control': lambda c: c['crosswind']['beta_cmd']},
    'gs_azimuth_error': {
        'sim': lambda s: s['gs02']['a_error']},
    'platform_azi': {
        'sim': lambda s: s['gs02']['azimuth'],
        'control': lambda c: c['control_input']['perch']['perch_azi'][:, 0]},
    'gs_detwist_cmd': {
        'control': lambda c: c['control_output']['detwist_cmd']},
    'gs_detwist_pos': {
        'control': lambda c: c['control_input']['gs_sensors']['detwist_pos']},
    'gsg_yoke': {
        'sim': lambda s: s['gsg']['gsg_yoke'],
        'control': lambda c: c['control_input']['gsg']['azi'][:, 0]},
    'gsg_termination': {
        'sim': lambda s: s['gsg']['gsg_termination'],
        'control': lambda c: c['control_input']['gsg']['ele'][:, 0]},
    'path_radius_target': {
        'control': lambda c: c['crosswind']['path_radius_target']},
    'payout': {
        'control': lambda c: c['state_est']['winch']['payout']},
    'wing_pos_cw': {
        'control': lambda c: c['crosswind']['current_pos_cw']},
    'wing_pos_g_cmd': {
        'control': lambda c: c['hover']['wing_pos_g_cmd']},
    'wing_xg': {
        'sim': lambda s: s['wing']['Xg'],
        'control': lambda c: c['state_est']['Xg']},
    'wing_acc': {
        'sim': lambda s: s['wing']['Ab'],
        'control': lambda c: c['state_est']['Ab_f']},
    'hover_angles': {
        'control': lambda c: c['hover']['angles']},
    'hover_angles_cmd': {
        'control': lambda c: c['hover']['angles_cmd']},
    'hover_gain_ramp_scale': {
        'control': lambda c: c['hover']['gain_ramp_scale']},
    'angular_acc': {
        'sim': lambda s: s['wing']['domega']},
    'tether_elevation': {
        # Because the vessel and platform frames differ only by a rotation
        # around the z-axis, elevations with respect to the two frames are
        # numerically equal.
        'sim': lambda s: s['tether']['Xv_start_elevation'],
        'control':
            lambda c: c['state_est']['tether_ground_angles']['elevation_p']},
    'tether_elevation_valid': {
        'control':
            lambda c: c['state_est']['tether_ground_angles']['elevation_valid']
    },
    'tether_azimuth': {
        'sim': lambda s: s['tether']['Xv_start_azimuth']},
    'tether_tension': {
        'sim': lambda s: s['wing']['tether_force_b']['tension'],
        'control':
            lambda c: c['state_est']['tether_force_b']['sph']['tension']},
    'tether_tension_cmd': {
        'control':
            lambda c: c['hover']['tension_cmd']},
    'tether_pitch': {
        'sim': lambda s: s['wing']['tether_force_b']['pitch'],
        'control': lambda c: c['state_est']['tether_force_b']['sph']['pitch']},
    'tether_roll': {
        'sim': lambda s: s['wing']['tether_force_b']['roll'],
        'control': lambda c: c['state_est']['tether_force_b']['sph']['roll']},
    'tether_moment': {
        'sim': lambda s: s['wing']['fm_tether']['moment']},
    'tether_xg_start': {
        'sim': lambda s: s['tether']['Xg_start'],
        'control': lambda c: c['state_est']['tether_anchor']['pos_g']},
    'tether_xg_end': {
        'sim': lambda s: s['tether']['Xg_end']},
    'tether_xg_nodes': {
        'sim': lambda s: s['tether']['Xg_nodes']},
    'rotor_speeds': {
        'sim': lambda s: abs(s['rotors']['omega']),
        # The controller telemetry already reports back absolute values.
        'control': lambda c: c['control_input']['rotors']},
    'rotor_freestream_speeds': {
        'sim': lambda s: s['rotors']['v_freestream'],
        'control': lambda c: c['v_app_locals']},
    'rotor_gyro_moments': {
        'sim': lambda s: s['rotors']['gyro_moment']},
    'rotor_thrusts': {
        'sim': lambda s: s['rotors']['thrust']},
    'motor_torques': {
        'sim': lambda s: s['stacked_power_sys']['motor_torques']},
    'thrust_moment': {
        'control': lambda c: c['thrust_moment']},
    'thrust_moment_avail': {
        'control': lambda c: c['thrust_moment_avail']},
    'electric_power': {
        'sim': lambda s: s['power_sys']['P_elec']},
    'aero_power': {
        'sim': lambda s: s['rotors']['aero_power']},
    'flaps': {
        'sim': lambda s: s['wing']['flaps'],
        'control': lambda c: c['control_input']['flaps']},
    'servo_shaft_torques': {
        'sim': lambda s: s['servo_sensor']['external_shaft_torques']},
    'wing_vel_trans_in': {
        'control': lambda c: c['trans_in']['wing_vel_ti']},
    'wing_vel_trans_in_y_cmd': {
        'control': lambda c: c['trans_in']['wing_vel_ti_y_cmd']},
    'wind_g_vector_f_slow': {
        'control': lambda c: c['state_est']['wind_g']['vector_f_slow']},
    'ground_voltage': {
        'sim': lambda s: s['stacked_power_sys']['ground_voltage']},
    'tether_current': {
        'sim': lambda s: s['stacked_power_sys']['tether_current']},
    'block_voltages': {
        'sim': lambda s: s['stacked_power_sys']['block_voltages']},
    'loop_angle': {
        'control': lambda c: c['crosswind']['loop_angle'],
        'method': 'nearest'},
    'dcm_g2b': {
        'sim': lambda s: s['wing']['dcm_g2b']['d'],
        'control': lambda c: c['state_est']['dcm_g2b']['d'],
        'method': 'nearest'},
    'dcm_g2v': {
        'sim': lambda s: s['buoy']['dcm_g2v']['d'],
        'control': lambda c: c['state_est']['vessel']['dcm_g2v']['d'],
        'method': 'nearest'},
    'buoy_xg': {
        'sim': lambda s: s['buoy']['Xg'],
        'control': lambda c: c['state_est']['vessel']['pos_g']},
    'accum_kite_loops': {
        'control': lambda c: c['crosswind']['loop_count']},
    'accum_detwist_loops': {
        'control': lambda c: c['detwist_loop_count']},
    'water_line': {
        'sim': lambda s: s['buoy']['water_line_pos_z_v']},
    'buoy_yaw_angle_from_eq': {
        'sim': lambda s: s['buoy']['yaw_angle_from_eq']},
    'buoy_accel_g': {
        'sim': lambda s: s['buoy']['vessel_origin_accel_g']},
    }


def GetDistToWrappedLimits(value, start_limit, end_limit,
                           wrap_left, wrap_right):
  """Returns value for min distance from value to limits on wrapped scale.

  Arguments:
    value: Value to be evaluated. Can be list-like or single value. Values must
      be between wrap_left and wrap_right.
    start_limit: The beginning of a range on wrapped scale.
    end_limit: The end of a range on wrapped scale.
    wrap_left: Minimum value for wrapping scale.
    wrap_right: Maximum value for wrapping scale.
  Returns:
    Minimum distance that value is from range limits.
    Positive values indicate value is between range specified by start_limit
    and end_limit. Negative values indicate value is outside of range.
  """
  wrap_range = wrap_right - wrap_left

  if not hasattr(value, '__iter__'):
    value = [value]

  # Unwrap end limit if needed so limits are in order.
  if end_limit < start_limit:
    end_limit_ordered = end_limit + wrap_range
  else:
    end_limit_ordered = end_limit

  for ii, v in enumerate(value):
    assert v >= wrap_left and v <= wrap_right, (
        'Values must be between wrap_left and wrap_right.')

    if end_limit < start_limit and v < end_limit:
      # If limits go around wrap and value was in limits before wrap,
      # unwrap value.
      v += wrap_range

    if v > start_limit and v < end_limit_ordered:
      # If inside the bad range, give positive value
      value[ii] = min(abs(v - start_limit),
                      abs(v - end_limit_ordered))
    else:
      # If outside bad range, give negative value.
      value[ii] = -min(abs(v - start_limit),
                       abs(v - end_limit_ordered),
                       # Also check wrapped values to limits.
                       abs(v + wrap_range - end_limit_ordered),
                       abs(v - wrap_range - start_limit))

  if len(value) == 1:
    return value[0]
  else:
    return value


def _GetValueAndSource(sim, control, name, sources):
  """Returns value of specified telemetry 'name' and 'source'.

  Arguments:
    sim: Simulator telemetry dictionary.
    control: Controller telemetry dictionary.
    name: [string] Telemetry variable e.g. 'airspeed' or 'alpha'.
    sources: [list of strings] The list of telemetry sources in their
             priority order. Data is returned from the first source that's
             available, and data from other sources are interpolated
             accordingly.
  Raises:
    ValueError: Requested 'name' not available.
  """
  if name not in _TELEMETRY_SELECTORS:
    raise ValueError('Requested name "%s" not available.' % name)

  all_sources = _TELEMETRY_SELECTORS[name].keys()
  for source in sources:
    if source in all_sources:
      selector = _TELEMETRY_SELECTORS[name][source]
      telemetry = None
      if source == 'sim' and sim is not None:
        telemetry = sim
      elif source == 'control' and control is not None:
        telemetry = control

      if telemetry is not None:
        try:
          return selector(telemetry), source
        except ValueError:
          logging.error('Cannot find "%s" in %s".', name, source)
          return None, None
  return None, None


def _GetFlightModesIndices(flight_mode_timeseries, flight_modes):
  """Returns indices corresponding flight mode specified.

  Arguments:
    flight_mode_timeseries: 'flight_mode' timeseries data.
    flight_modes: [string or list of strings] Optional flight mode.
        For example, 'kFlightModeCrosswindNormal' or
        ['kFlightModeCrosswindNormal', 'kFlightModeCrosswindPrepTransOut'].
  """
  if isinstance(flight_modes, str):
    flight_modes = [flight_modes]

  modes_indices = np.empty(0, dtype=int)
  for flight_mode in flight_modes:
    mode_indices = np.argwhere(
        flight_mode_timeseries
        == flight_modes_module.GetFlightModes()[flight_mode])
    modes_indices = np.append(modes_indices, mode_indices)

  return np.sort(modes_indices)


# Note: Since interpolation of integer array is performed here, there is a
# possibility of offsetting flight mode transitions by a cycle.
def _GetInterpolatedValue(sim_time, control_time, data_value, method):
  """Returns control telemetry data_value interpolated to simulator time."""
  if not method:
    method = 'linear'
  def _Interpolate(sim_time, control_time, data_value):  # pylint: disable=missing-docstring
    assert data_value.shape

    if len(data_value.shape) == 1:
      if np.size(data_value) == 1:
        return data_value.repeat(sim_time.size)
      else:
        return interp1d(control_time, data_value, kind=method,
                        bounds_error=False, axis=0,
                        fill_value=(data_value[0], data_value[-1]))(sim_time)
    else:
      # If this an N-D array where N > 1 (e.g., motor_voltages[:, 8]),
      # each slice of this array needs to be interpolated.
      new_shape = (sim_time.shape[0],) + data_value.shape[1:]
      data_out = np.empty(new_shape, dtype=data_value.dtype)
      for i in np.nditer(data_value.shape[1:]):
        slice_index = [slice(None)] + list(i)
        source_value = data_value[slice_index]
        data_out[slice_index] = interp1d(
            control_time, source_value, kind=method, bounds_error=False,
            axis=0, fill_value=(source_value[0], source_value[-1]))(
                sim_time)
      return data_out

  if isinstance(data_value, dict):
    all_fields = data_value.keys()
    data_value_out = {}
  elif isinstance(data_value, np.ndarray) and data_value.dtype.names:
    all_fields = data_value.dtype.names
    new_shape = (len(sim_time),) + data_value.shape[1:]
    data_value_out = np.empty(new_shape, dtype=data_value.dtype)
  else:
    if np.isnan(data_value).any():
      return data_value
    else:
      return _Interpolate(sim_time, control_time, data_value)

  for field in all_fields:
    if np.isnan(data_value[field]).any():
      data_value_out[field] = data_value[field]
    else:
      data_value_out[field] = _Interpolate(sim_time, control_time,
                                           data_value[field])
  return data_value_out


def SelectTelemetry(sim, control, names, flight_modes=None, sources=None):
  """Returns a list of selected telemetry.

  The 'control' fields are interpolated to 'sim' time when telemetry from mixed
  sources is requested.

  Arguments:
    sim: Simulator telemetry dictionary.
    control: Controller telemetry dictionary.
    names: [string or list of strings] Telemetry variable(s) e.g.
           'airspeed' or ['airspeed', 'alpha'].
    flight_modes: [string or list of strings] Optional flight mode.
           For example, 'kFlightModeCrosswindNormal' or
           ['kFlightModeCrosswindNormal', 'kFlightModeCrosswindPrepTransOut'].
    sources: [list of strings] The list of telemetry sources in their
           priority order. Data is returned from the first source that's
           available, and data from other sources are interpolated accordingly.

  Returns:
    A list of selected telemetry for specified 'flight_modes'.
  """
  if isinstance(names, str):
    names = [names]

  is_flight_mode_added = False

  if flight_modes is not None and 'flight_mode' not in names:
    names.append('flight_mode')
    is_flight_mode_added = True

  assert sources
  primary_source = sources[0]
  assert primary_source in ['sim', 'control']

  telem_data = {}
  source_set = set()
  for name in names:
    data_value, data_source = _GetValueAndSource(sim, control, name, sources)
    telem_data[name] = {'value': data_value, 'source': data_source}
    source_set.add(data_source)

  if len(source_set) > 1 and primary_source in source_set:
    sim_time, _ = _GetValueAndSource(sim, control, 'time', sources=['sim'])
    control_time, _ = _GetValueAndSource(
        sim, control, 'time', sources=['control'])
    for name in names:
      data_value = telem_data[name]['value']
      if data_value is None:
        continue
      data_source = telem_data[name]['source']
      interp_method = _TELEMETRY_SELECTORS[name].get('method')
      if primary_source == 'sim' and data_source == 'control':
        telem_data[name]['value'] = _GetInterpolatedValue(
            sim_time, control_time, data_value, interp_method)
      elif primary_source == 'control' and data_source == 'sim':
        telem_data[name]['value'] = _GetInterpolatedValue(
            control_time, sim_time, data_value, interp_method)

  if is_flight_mode_added:
    names.remove('flight_mode')

  selected_data = []
  for name in names:
    if telem_data[name]['value'] is None:
      selected_data.append(np.array([float('nan')]))
    elif flight_modes is None:
      selected_data.append(telem_data[name]['value'])
    else:
      modes_indices = _GetFlightModesIndices(telem_data['flight_mode']['value'],
                                             flight_modes)
      if modes_indices.size == 0:
        selected_data.append(np.array([float('nan')]))
      else:
        selected_data.append((telem_data[name]['value'])[modes_indices])

  if len(selected_data) == 1:
    return selected_data[0]
  else:
    return selected_data


def IsSelectionValid(selected_data):
  """Checks whether a selection returned by SelectTelemetry is valid."""

  if not np.shape(selected_data):
    # Invalid if the selected data is a nan.
    if np.isnan(selected_data):
      return False
  else:
    # Invalid if the selected data is an empty list.
    if not selected_data.shape[0]:
      return False

  # Invalid if the selelcted data is [float('nan')].
  if (np.size(selected_data) == 1 and
      selected_data.dtype.names is None and
      np.isnan(selected_data.flatten()[0])):
    return False

  return True


def FilterByWindowAveraging(time, values, speed, scale, num_tau, num_sigma):
  """Filters values in a window average whose width is set by a time scale, tau.

  This takes the array of values and returns its windowed sample average
  over a width of num_tau multiplied by a time constant which is proportional
  to scale / speed. It returns the filtered averaged values after rejecting
  values within the window that are outside of the prescribed number of
  standard deviations.

  Args:
    time: numpy array of time
    values: numpy array of the values to be filtered
    speed: numpy array of the reference speeds
    scale: scalar value of the reference scale
    num_tau: number of time constants over which to average (sets window width)
    num_sigma: number of standard deviations outside of which to ignore data

  Returns:
    filtered_values - filtered/windowed mean value array
  """
  # Check for input vector lengths/sizes
  assert (time.shape == values.shape and values.shape == speed.shape
          and scale > 0 and num_tau > 0
          and num_sigma > 0), ('FilterByWindowAveraging input is incorrect')

  # Time constants
  time_constants = np.divide(num_tau * scale * np.ones(speed.shape),
                             speed)

  # Return vector of statistically averaged quantities
  filtered_values = np.zeros(values.shape)
  time_it = np.nditer(time, flags=['f_index'])
  while not time_it.finished:
    current_time = time[time_it.index]
    begin_time = current_time - time_constants[time_it.index] / 2.0
    end_time = current_time + time_constants[time_it.index] / 2.0

    # Get relevant filter window
    window_time = time[np.logical_and(time < end_time, time > begin_time)]

    # Force window symmetry
    window_begin_index = np.sum(window_time < current_time)
    window_end_index = np.sum(window_time > current_time)
    if window_begin_index < window_end_index:
      window_equal_index = window_begin_index
    else:
      window_equal_index = window_end_index

    index_begin = time_it.index - window_equal_index
    index_end = time_it.index + window_equal_index
    windowed_values = values[index_begin : index_end + 1]

    # Get stddev to remove extreme samples
    sampled_stddev = np.std(windowed_values)
    stddev_factor = num_sigma * sampled_stddev

    # Try to remove extreme samples
    windowed_values = windowed_values[windowed_values <=
                                      windowed_values + stddev_factor]
    windowed_values = windowed_values[windowed_values >=
                                      windowed_values - stddev_factor]

    # Now take average
    filtered_values[time_it.index] = np.mean(windowed_values)

    # Increment iterator
    time_it.iternext()

  return filtered_values


def GetIntervals(i_outside_allowable_range):
  """Detects intervals during which a time series is outside allowable range.

  Args:
    i_outside_allowable_range: list of indices where the value is
                               above or below the allowable range of values.

  Returns:
    A list of couple of indices marking the beginning and the end of an
    interval of values outside of the allowable range.
  """
  intervals = []

  if i_outside_allowable_range.size > 0:
    i_begin = i_outside_allowable_range[0]

    for i in range(len(i_outside_allowable_range) - 1):
      if i_outside_allowable_range[i + 1] > i_outside_allowable_range[i] + 1:
        intervals.append((i_begin, i_outside_allowable_range[i]))
        i_begin = i_outside_allowable_range[i + 1]
    intervals.append((i_begin, i_outside_allowable_range[-1]))

  return intervals


def GetSustainedValue(time_series, lower_limit, upper_limit, duration,
                      sampling_period):
  """Returns the min and max values outside of the good upper and lower limits.

  Args:
    time_series: (N,) numpy array of data to be evaluated.
    lower_limit: Lower limit of the acceptable range of values.
    upper_limit: Upper limit of the acceptable range of values.
    duration: Continuous time (in seconds) over which the time_series must lie
              outside of the good limits in order to be reported.
    sampling_period: Sampling period (in seconds) used to create the
                     time_series.

  Returns:
    A tuple including the minimum and maximum sustained bad values.
  """

  if np.isnan(sampling_period):
    return (np.array([float('nan')]), np.array([float('nan')]))

  # Number of elements spanning 'duration'
  n_sustained_duration = int(duration / sampling_period)

  # Initialize the min and max sustained bad values.
  min_sustained_value = (lower_limit + upper_limit) / 2.0
  max_sustained_value = (lower_limit + upper_limit) / 2.0

  # Look for telemetry values outside of the allowable range.
  i_low = np.where(time_series < lower_limit)
  i_high = np.where(time_series > upper_limit)

  # Check if data stays above or below allowable range for 'duration' seconds.
  for interval in GetIntervals(i_low[0]):
    if interval[1] - interval[0] > n_sustained_duration:
      min_over_interval = np.min(time_series[interval[0]:interval[1] + 1])
      min_sustained_value = min(min_sustained_value, min_over_interval)

  for interval in GetIntervals(i_high[0]):
    if interval[1] - interval[0] > n_sustained_duration:
      max_over_interval = np.max(time_series[interval[0]:interval[1] + 1])
      max_sustained_value = max(max_sustained_value, max_over_interval)

  return (min_sustained_value, max_sustained_value)


def GetTimeSamp(time):
  """Empirically estimate the sampling period of the telemetry."""

  # Set a lower limit on the length of data that is feasible to use.
  if len(time) < 10:
    return float('nan')

  # Determine the most likely sampling frequency.
  # Flight logs have the larger telemetry_sample_period param but don't use it.
  # Mean would be biased by any large gaps in time.
  # Taking the first element in the diff operation diff would fail if there is
  # a gap after the very first entry.
  return stats.mode(np.around(np.diff(time), decimals=3)).mode[0]


def LpFiltFiltTimeSeries(time, time_series, f_cutoff):
  """Returns a low-pass, symmetrically filtered timeseries, accounting for gaps.

  Args:
    time: (N,) Numpy array of the time values associated with each data point
          in the time_series.
    time_series: (N,) Numpy array of the data values of interest.
    f_cutoff: Cutoff frequency (Hz) to use.
  """

  t_samp = GetTimeSamp(time)
  if np.isnan(t_samp):
    return np.array([float('nan')])

  # Break the data into a list of continuous arrays.
  discont_ind = np.where(np.diff(time) > (1.5 * t_samp))[0] + 1
  split_time_series = np.split(time_series, discont_ind)

  # Filter each chunk of continuous data independently.
  # Note: The filtfilt function transforms a first-order Butterworth filter
  # into a second-order filter.
  time_series_f = []
  for cont_time_series in split_time_series:
    nyquist_frac = f_cutoff / (0.5 / t_samp)
    b, a = sp_signal.butter(1, nyquist_frac, 'lowpass')
    time_series_f.append(sp_signal.filtfilt(
        b, a, cont_time_series, padtype=None))

  return np.concatenate(time_series_f)
