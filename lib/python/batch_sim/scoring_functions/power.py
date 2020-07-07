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

"""Scoring functions relating to the power-train."""

from makani.lib.python.batch_sim import scoring_functions
import numpy as np
import scipy.signal as signal
import scoring_functions_util as scoring_util


class BreakerCurrentScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if breaker limit (i.e. average ac current) is met."""

  # Breaker limit is an attempt to convert from ground power to
  # AC breaker current.  Voltage is assumed to be 480 and inverter
  # inefficiency is ignored.  Voltage in CL is set slightly high which
  # should offset the 97% efficiency of the inverters so this seems a
  # reasonable assumption.  Filter is run on the square of the current
  # to act like a thermal time constant.

  def __init__(self, good_limit, bad_limit, severity):
    super(BreakerCurrentScoringFunction, self).__init__(
        'Current Max 30 minute average', 'amps', good_limit, bad_limit,
        severity)

  def GetSystemLabels(self):
    return ['power']

  def GetValue(self, output):
    return output['current_breaker_max']

  def GetOutput(self, timeseries):
    return {'current_breaker_max': np.max(timeseries['current_breaker'])}

  def GetTimeSeries(self, params, sim, control):
    ground_voltage, tether_current = self._SelectTelemetry(
        sim, control, ['ground_voltage', 'tether_current'])
    current_ac = (np.multiply(ground_voltage, tether_current) /
                  (480.0 * np.sqrt(3) * 6.0))

    # Simple 30 minute filter with Nyquist = 50 Hz.
    b, a = signal.butter(1, 1.0 / (30.0 * 60.0) / 50.0)

    # Filter current^2 to get thermally weighted average.
    filtered_power = signal.lfilter(b, a, np.square(current_ac))

    # Take the square root to get current.
    return {'current_breaker': np.sqrt(filtered_power)}


class PowerConsumedMaxScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if a negative power limit (i.e. power used) is met."""

  # Instantaneous power consumption limit.  Note, negative power is
  # motoring power and positive power is generation.

  def __init__(self, good_limit, bad_limit, severity):
    super(PowerConsumedMaxScoringFunction, self).__init__(
        'Power Consumed Max', 'kW', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['power']

  def GetValue(self, output):
    return output['power_consumed_max']

  def GetOutput(self, timeseries):
    return {
        'power_consumed_max': -np.min(timeseries['power'])
    }

  def GetTimeSeries(self, params, sim, control):
    ground_voltage, tether_current = self._SelectTelemetry(
        sim, control, ['ground_voltage', 'tether_current'])
    power = np.multiply(ground_voltage, tether_current)
    return {'power': power / 1e3}


class PowerGeneratedMaxScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if generation power limit (i.e. power produced) is met."""

  # Instantaneous power generation limit.  Note, negative power is
  # motoring power and positive power is generation.

  def __init__(self, good_limit, bad_limit, severity):
    super(PowerGeneratedMaxScoringFunction, self).__init__(
        'Power Generated Max', 'kW', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['power']

  def GetValue(self, output):
    return output['power_generated_max']

  def GetOutput(self, timeseries):
    return {
        'power_generated_max': np.max(timeseries['power'])
    }

  def GetTimeSeries(self, params, sim, control):
    ground_voltage, tether_current = self._SelectTelemetry(
        sim, control, ['ground_voltage', 'tether_current'])
    power = np.multiply(ground_voltage, tether_current)
    return {'power': power / 1e3}


class PowerTransientScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Test if power transients exceed allowable over window."""

  # Power transients took out microgrid.  This looks for transients
  # that exceed a threshold with a focus on transients that are long
  # enough to cause a significant power variation.
  # TODO: Make window settable.

  def __init__(self, good_limit, bad_limit, severity):
    super(PowerTransientScoringFunction, self).__init__(
        'Power Rate of Change Max', 'kW/s', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['experimental', 'power']

  def GetValue(self, output):
    return output['power_slope_max']

  def GetOutput(self, timeseries):
    return {'power_slope_max': np.max(timeseries['power_slope'])}

  def GetTimeSeries(self, params, sim, control):
    ground_voltage, tether_current = self._SelectTelemetry(
        sim, control, ['ground_voltage', 'tether_current'])
    power = np.multiply(ground_voltage, tether_current)
    if not scoring_util.IsSelectionValid(power):
      return {
          'power_slope': np.array([float('nan')])
      }

    # Now figure out the indexing to support desired window time (s).
    window_time = 1.0
    sample_step = params['system_params']['ts'][0]
    window_size = int(window_time / sample_step)

    # Recalculate the actual window time we get for an achievable window size.
    window_time = float(window_size) * sample_step

    delta_power = np.subtract(power[0:-window_size], power[window_size:])

    return {
        # 1.0 factor accounts for 1 / 1.0 seconds.
        'power_slope': 1.0 / window_time * np.abs(delta_power) / 1e3
    }


# TODO: The GetTimeSeries method should return ground and kite
# power values on a loop averaged basis. Implement a GetOutput method to average
# these timeseries values.
class PowerGeneratedMeanScoringFunction(
    scoring_functions.SingleSidedLimitScoringFunction):
  """Score performance based on average generated power in crosswind."""

  def __init__(self, good_limit, bad_limit, severity):
    super(PowerGeneratedMeanScoringFunction, self).__init__(
        'Mean Generated Power', 'kW', good_limit, bad_limit, severity)

  def GetSystemLabels(self):
    return ['experimental', 'power', 'performance']

  def GetValue(self, output):
    return output['crosswind_stable_ground_power_mean'] / 1e3

  def GetTimeSeries(self, params, sim, control):
    block_voltages, ground_voltage, tether_current, loop_angle, time = (
        self._SelectTelemetry(sim, control,
                              ['block_voltages', 'ground_voltage',
                               'tether_current', 'loop_angle', 'time'],
                              flight_modes='kFlightModeCrosswindNormal'))

    crosswind_ground_power_mean = float('nan')
    crosswind_stable_ground_power_mean = float('nan')
    crosswind_kite_power_mean = float('nan')
    crosswind_stable_kite_power_mean = float('nan')

    if scoring_util.IsSelectionValid(block_voltages):

      kite_powers = np.sum(block_voltages, 1) * tether_current
      ground_powers = ground_voltage * tether_current

      loop_indexes = []
      last_time = None
      # Determine times of each loop 9 o'clock.
      for ind in range(1, len(loop_angle)):
        if loop_angle[ind] > 2 * np.pi - 0.5 and loop_angle[ind - 1] < 0.5:
          if not last_time or time[ind] - last_time > 5.0:
            loop_indexes.append(ind)
            last_time = time[ind]

      # Compute average power across all whole loops flown.
      if len(loop_indexes) >= 2:
        crosswind_kite_power_mean = np.mean(
            kite_powers[loop_indexes[0]:loop_indexes[-1]])
        crosswind_ground_power_mean = np.mean(
            ground_powers[loop_indexes[0]:loop_indexes[-1]])

      # Throw away the first 4 loops to allow measurement to stabilize.
      if len(loop_indexes) >= 6:
        crosswind_stable_kite_power_mean = np.mean(
            kite_powers[loop_indexes[4]:loop_indexes[-1]])
        crosswind_stable_ground_power_mean = np.mean(
            ground_powers[loop_indexes[4]:loop_indexes[-1]])

    return {
        'crosswind_ground_power_mean': crosswind_ground_power_mean,
        'crosswind_stable_ground_power_mean':
            crosswind_stable_ground_power_mean,
        'crosswind_kite_power_mean': crosswind_kite_power_mean,
        'crosswind_stable_kite_power_mean': crosswind_stable_kite_power_mean,
    }
