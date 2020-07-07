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

"""Functions used to evaluate controller performance."""

import collections
import copy
import csv
import time

from makani.avionics.common import plc_messages
from makani.lib.python import c_helpers
from makani.lib.python.batch_sim import batch_sim_util
from makani.lib.python.batch_sim import flight_modes as flight_mode_util
from makani.lib.python.batch_sim.scoring_functions import scoring_functions_util as scoring_util
import numpy


_GROUND_STATION_MODE_HELPER = c_helpers.EnumHelper(
    'GroundStationMode', plc_messages)


class LimitsException(Exception):
  pass


class ScoringFunction(object):
  """Abstract class representing a 'score' used in a disturbance simulation."""

  def __init__(self, name, units, severity):
    self.name = name
    self.units = units
    self.severity = int(severity)
    assert self.severity >= 0 & self.severity <= 5
    self._InitSourcePriority(['sim', 'control'])

  def GetName(self):
    """Returns the display name of this score (including units)."""
    return '%s [%s]' % (self.name, self.units)

  def GetSeverity(self):
    """Return the severity set for this score."""
    return self.severity

  def GetSystemLabels(self):
    return ['']

  def GetValue(self, output):
    """Returns the display value for this score."""
    raise NotImplementedError

  def GetScore(self, output):
    """Returns the score for this output.  1.0 is bad and 0.0 is good."""
    raise NotImplementedError

  def GetFailureCount(self, timeseries):
    """Returns the number of samples outside a limit."""
    raise NotImplementedError

  def GetIndexOfFirstFailure(self, timeseries):
    """Returns the index where a limit is triggered for the first time.

    Args:
      timeseries: Array of numbers.

    Returns:
      Index where a limit is triggered for the first time. If the limit is never
      triggered, it returns -1.
    """
    raise NotImplementedError

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    """Returns the timeseries values used for this score in a dict."""
    raise NotImplementedError

  def GetOutput(self, timeseries):
    """Returns the output values used for this score in a dict."""
    return timeseries

  def Class(self):
    return self.__class__

  def Limits(self):
    """Return the limits set for this score."""
    raise NotImplementedError

  def SetSourcePriority(self, sources):
    """Set source priority."""
    for source in sources:
      assert source in ['control', 'sim']
    self._sources = copy.copy(sources)

  def _InitSourcePriority(self, sources):
    """Set default source priority."""
    self.SetSourcePriority(sources)

  def SourcePriority(self):
    """Get the source priority."""
    return self._sources

  def _SelectTelemetry(self, sim, control, names, flight_modes=None):
    """Returns a list of selected telemetry.

    The 'control' fields are interpolated to 'sim' time when telemetry from
    mixed sources is requested.

    Arguments:
      sim: Simulator telemetry dictionary.
      control: Controller telemetry dictionary.
      names: [string or list of strings] Telemetry variable(s) e.g.
             'airspeed' or ['airspeed', 'alpha'].
      flight_modes: [string or list of strings] Optional flight mode.
             For example, 'kFlightModeCrosswindNormal' or
             ['kFlightModeCrosswindNormal', 'kFlightModeCrosswindPrepTransOut'].

    Returns:
      A list of selected telemetry for specified 'flight_modes'.
    """
    return scoring_util.SelectTelemetry(
        sim, control, names, flight_modes, self.SourcePriority())


class ScoringFunctionWrapper(ScoringFunction):
  """Base class to wrap a child scoring function."""

  def __init__(self, name, scoring_function):
    self._scoring_function = scoring_function
    super(ScoringFunctionWrapper, self).__init__(name, scoring_function.units,
                                                 scoring_function.severity)

  def SetSourcePriority(self, sources):
    self._scoring_function.SetSourcePriority(sources)

  def _InitSourcePriority(self, unused_sources):
    """Wrappers should not have a source priority. It is up to the child."""
    pass

  def SourcePriority(self):
    return self._scoring_function.SourcePriority()

  def GetValue(self, output):
    if not output:
      return float('nan')
    else:
      return self._scoring_function.GetValue(output)

  def GetScore(self, output):
    if not output:
      return float('nan')
    else:
      return self._scoring_function.GetScore(output)

  def GetFailureCount(self, timeseries):
    if not timeseries:
      return None
    else:
      return self._scoring_function.GetFailureCount(timeseries)

  def GetIndexOfFirstFailure(self, timeseries):
    if not timeseries:
      return None
    else:
      return self._scoring_function.GetIndexOfFirstFailure(timeseries)

  def GetOutput(self, timeseries):
    if not timeseries:
      return {}
    else:
      return self._scoring_function.GetOutput(timeseries)

  def Class(self):
    return self._scoring_function.Class()

  def Limits(self):
    return self._scoring_function.Limits()


class ScoringFunctionFilter(ScoringFunctionWrapper):
  """Base class to filter input telemetry for the child score function."""

  def GetSystemLabels(self):
    return self._scoring_function.GetSystemLabels()

  def _FilteredOutput(self, params, sim_telemetry, control_telemetry,
                      indices, flight_time):
    """Filter the telemetry with `indices` and return output of child scores."""

    if indices.size == 0:
      return {}
    else:
      indices = numpy.reshape(indices, (indices.size,))
      indices.sort()
      # If the indices are contiguous, replace them with a slice. This allows
      # numpy to provide views into arrays, which do not copy data.
      if (numpy.diff(indices) == 1).all():
        indices = slice(indices[0], indices[-1] + 1)

    t_start, t_end = flight_time[indices][[0, -1]]

    if self.SourcePriority()[0] == 'sim':
      if sim_telemetry is not None:
        # This indices is based on sim_telemetry.
        sim_telemetry = sim_telemetry[indices]
        if control_telemetry is not None:
          # Filter control_telemetry based on time.
          control_indices = batch_sim_util.GetTelemetryIndices(
              control_telemetry, t_start, t_end)
          if control_indices is None:
            return {}
          control_telemetry = control_telemetry[control_indices]
      elif control_telemetry is not None:
        # Fall back to use control_telemetry as the primary source.
        control_telemetry = control_telemetry[indices]
    elif self.SourcePriority()[0] == 'control':
      if control_telemetry is not None:
        # This indices is based on control_telemetry.
        control_telemetry = control_telemetry[indices]
        if sim_telemetry is not None:
          # Filter sim_telemetry based on time.
          sim_indices = batch_sim_util.GetTelemetryIndices(
              sim_telemetry, t_start, t_end)
          if sim_indices is None:
            return {}
          sim_telemetry = sim_telemetry[sim_indices]
      elif sim_telemetry is not None:
        # Fall back to use sim_telemetry as the primary source.
        sim_telemetry = sim_telemetry[indices]
    else:
      assert False

    return self._scoring_function.GetTimeSeries(params, sim_telemetry,
                                                control_telemetry)


class CrashScoringFunction(ScoringFunctionWrapper):
  """Labels a scoring function as a crash criteria."""

  def __init__(self, scoring_function):
    self._scoring_function = scoring_function
    # All crash scoring functions are assigned a severity of 5.
    self._scoring_function.severity = 5

    name = '%s %s' % ('-Crash-', self._scoring_function.name)
    super(CrashScoringFunction, self).__init__(
        name, self._scoring_function)

  def GetSystemLabels(self):
    return ['crash'] + self._scoring_function.GetSystemLabels()

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    return self._scoring_function.GetTimeSeries(params, sim_telemetry,
                                                control_telemetry)


class PerchingFilter(ScoringFunctionFilter):
  """Restricts a scoring function to only apply to the landing maneuver."""

  def __init__(self, scoring_function, filter_name_prefix='Perching'):
    """Initializes a perching filter.

    Args:
      scoring_function: The child scoring function to filter for.
      filter_name_prefix: String to apply to scoring function name when
          `flight_mode` provided is a list of flight mode(s).
    """
    name = '%s %s' % (filter_name_prefix, scoring_function.name)

    super(PerchingFilter, self).__init__(name, scoring_function)

  def GetSystemLabels(self):
    system_labels = self._scoring_function.GetSystemLabels()
    # Manually add descend flight mode.
    # TODO(b/145831658): Figure out why flight mode filter doesn't work.
    system_labels.insert(0, 'Perching')
    system_labels.insert(1, 'kFlightModeHoverDescend')
    return system_labels

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    param_names = ['flight_mode', 'hover_gain_ramp_scale', 'time']

    flight_mode, gain_ramp_scale, flight_time = (
        self._SelectTelemetry(sim_telemetry, control_telemetry, param_names))

    selection_mask = numpy.logical_and(
        flight_mode ==
        flight_mode_util.GetFlightModes()['kFlightModeHoverDescend'],
        numpy.logical_and(gain_ramp_scale > 0.01, gain_ramp_scale < 0.95))

    indices = numpy.argwhere(selection_mask)[:, 0]

    return self._FilteredOutput(
        params, sim_telemetry, control_telemetry, indices, flight_time)


class FlightModeFilter(ScoringFunctionFilter):
  """Restricts a scoring function to only apply to a given flight mode."""

  def __init__(self, flight_mode, scoring_function,
               flight_mode_time_threshold=0.0, filter_name_prefix=None):
    """Initializes a flight mode filter.

    Using multiple flight_modes with flight_mode_time_threshold has some
    unexpected behavior.  The time threshold must be met for each new flight
    mode, and will reset with each mode change, even if the threshold was
    already met with the prior flight mode.

    Args:
      flight_mode: String or list of strings with flight mode(s) to filter e.g.
        'kFlightModeTransIn' or
        ['kFlightModeTransIn', 'kFlightModeHoverFullLength'].
      scoring_function: The child scoring function to filter for.
      flight_mode_time_threshold: The minimum amount of time spent in each
          flight mode to activate the scoring.
      filter_name_prefix: String to apply to scoring function name when
          `flight_mode` provided is a list of flight mode(s).
    """
    if isinstance(flight_mode, str):
      name = '%s %s' % (flight_mode[len('kFlightMode'):], scoring_function.name)
      flight_mode = [flight_mode]
    elif filter_name_prefix:
      name = '%s %s' % (filter_name_prefix, scoring_function.name)
    else:
      name = scoring_function.name

    for mode in flight_mode:
      assert mode.startswith('kFlightMode')

    self._flight_modes = flight_mode
    self._flight_mode_time_threshold = flight_mode_time_threshold

    super(FlightModeFilter, self).__init__(name, scoring_function)

  def GetSystemLabels(self):
    system_labels = self._scoring_function.GetSystemLabels()
    for mode in self._flight_modes:
      system_labels.insert(0, mode)
    return system_labels

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    param_names = ['flight_mode', 'flight_mode_time', 'time']

    descend_mode = 'kFlightModeHoverDescend'
    if descend_mode in self._flight_modes:
      param_names.append('hover_gain_ramp_scale')
      flight_mode, flight_mode_time, flight_time, gain_ramp_scale = (
          self._SelectTelemetry(sim_telemetry, control_telemetry, param_names))
    else:
      flight_mode, flight_mode_time, flight_time = self._SelectTelemetry(
          sim_telemetry, control_telemetry, param_names)

    flight_mode_masks = [
        flight_mode == flight_mode_util.GetFlightModes()[mode]
        for mode in self._flight_modes if mode != descend_mode]

    if descend_mode in self._flight_modes:
      descend_mode_mask = numpy.logical_and(
          flight_mode == flight_mode_util.GetFlightModes()[descend_mode],
          gain_ramp_scale > 0.95)
      flight_mode_masks.append(descend_mode_mask)

    selection_mask = numpy.logical_and(
        flight_mode_masks,
        flight_mode_time >= self._flight_mode_time_threshold)

    indices = numpy.argwhere(selection_mask)[:, 1]

    return self._FilteredOutput(
        params, sim_telemetry, control_telemetry, indices, flight_time)


class Gs02TransformStageFilter(ScoringFunctionFilter):
  """Restricts a scoring function to only apply to given transform stages."""

  def __init__(self, transform_stages, scoring_function):
    """Initializes a ground station tranform stage filter.

    Args:
      transform_stages: An integer or a list of integers of transform stages
          to be filtered.
      scoring_function: The child scoring function to filter for.
    """
    if isinstance(transform_stages, int):
      self._transform_stages = [transform_stages]
    elif isinstance(transform_stages, list):
      for stage in transform_stages:
        assert isinstance(stage, int) and 0 <= stage <= 4
      self._transform_stages = copy.copy(transform_stages)
    else:
      assert False

    name = 'Transform Stages %s %s' % (
        self._transform_stages, scoring_function.name)

    super(Gs02TransformStageFilter, self).__init__(name, scoring_function)

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    gs02_mode, gs02_transform_stage, flight_time = (
        self._SelectTelemetry(
            sim_telemetry, control_telemetry,
            ['gs02_mode', 'gs02_transform_stage', 'time']))

    transform_stage_masks = [
        gs02_transform_stage == stage for stage in self._transform_stages]

    selection_mask = numpy.logical_and(
        transform_stage_masks,
        gs02_mode == _GROUND_STATION_MODE_HELPER.Value('Transform'))

    indices = numpy.argwhere(selection_mask)[:, 1]

    return self._FilteredOutput(
        params, sim_telemetry, control_telemetry, indices, flight_time)


class SampleFilter(ScoringFunctionFilter):
  """Score for every N samples."""

  def __init__(self, sample_rate, scoring_function, show_rate=True):
    """Initialize it with the sample rate (e.g 0.2 means every 5 samples)."""
    if show_rate:
      name = '%s/%d' % (scoring_function.name, numpy.round(1.0/sample_rate))
    else:
      name = scoring_function.name
    self._sample_rate = sample_rate
    super(SampleFilter, self).__init__(name, scoring_function)

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    sample_interval = int(numpy.round(1.0 / self._sample_rate))
    sim_telemetry = sim_telemetry[::sample_interval]
    control_telemetry = control_telemetry[::sample_interval]
    return self._scoring_function.GetTimeSeries(params, sim_telemetry,
                                                control_telemetry)


class PayoutFilter(ScoringFunctionFilter):
  """Filter the telemetry based on tether payout."""

  def __init__(self, scoring_function, min_payout=None, max_payout=None,
               flight_mode_filter=None, flight_mode_time_threshold=0.0):
    if min_payout is None:
      assert max_payout is not None
      tag = '<%d' % max_payout
      min_payout = 0.0
    elif max_payout is None:
      max_payout = float('inf')
      tag = '>%d' % min_payout
    else:
      tag = '(%d, %d)' % (min_payout, max_payout)

    self._min_payout = min_payout
    self._max_payout = max_payout

    if isinstance(flight_mode_filter, str):
      flight_mode_filter = [flight_mode_filter]

    flight_mode_dict = {
        'PayOut': ['kFlightModeHoverPayOut',
                   'kFlightModeHoverPrepTransformGsUp'],
        'ReelIn': ['kFlightModeHoverReelIn']
    }

    if flight_mode_filter is not None:
      if all(flight_mode in flight_mode_util.GetFlightModes().keys()
             for flight_mode in flight_mode_filter):
        scoring_function = FlightModeFilter(
            flight_mode_filter, scoring_function, flight_mode_time_threshold)
      elif all(flight_mode in ['PayOut', 'ReelIn']
               for flight_mode in flight_mode_filter):
        filter_name_prefix = 'Hover -'
        flight_mode_list = []
        for flight_mode in flight_mode_filter:
          filter_name_prefix += (' - %s' % flight_mode)
          unused_assignment = [flight_mode_list.append(f)
                               for f in flight_mode_dict[flight_mode]]
        scoring_function = FlightModeFilter(
            flight_mode_list, scoring_function, flight_mode_time_threshold,
            filter_name_prefix=filter_name_prefix)
      else:
        raise ValueError('Invalid flight mode filter.')

    name = '%s [m] payout, %s' % (tag, scoring_function.name)
    super(PayoutFilter, self).__init__(name, scoring_function)

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    param_names = ['payout', 'time']
    payout, flight_time = self._SelectTelemetry(
        sim_telemetry, control_telemetry, param_names)

    if scoring_util.IsSelectionValid(payout):
      indices = numpy.ones(payout.shape, dtype=bool)
      if self._min_payout is not None:
        indices = numpy.logical_and(indices, payout >= self._min_payout)
      if self._max_payout is not None:
        indices = numpy.logical_and(indices, payout <= self._max_payout)
      indices = numpy.argwhere(indices).flatten()

    else:
      indices = numpy.array([], dtype=int)
    # The current selector only supports a continuous segment of the log.
    if numpy.size(indices):
      indices = numpy.arange(indices[0], indices[-1] + 1)

    return self._FilteredOutput(
        params, sim_telemetry, control_telemetry, indices, flight_time)


class MinAltitudeFilter(ScoringFunctionFilter):
  """Score only if the kite is above some minimum altitude."""

  def __init__(self, min_altitude, scoring_function):
    """Initialize it with the minimum altitude threshold."""
    self._max_z = -min_altitude
    name = '>%d[m] AGL, %s' % (-self._max_z, scoring_function.name)
    super(MinAltitudeFilter, self).__init__(name, scoring_function)

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    param_names = ['wing_xg', 'time']
    wing_xg, flight_time = self._SelectTelemetry(
        sim_telemetry, control_telemetry, param_names)

    if wing_xg.dtype.names and 'z' in wing_xg.dtype.names:
      indices = numpy.argwhere(wing_xg['z'] <= self._max_z).flatten()
    else:
      indices = numpy.array([], dtype=int)
    # The current selector only supports a continuous segment of the log.
    if numpy.size(indices):
      indices = numpy.arange(indices[0], indices[-1] + 1)

    return self._FilteredOutput(
        params, sim_telemetry, control_telemetry, indices, flight_time)


class CrosswindFilter(ScoringFunctionFilter):
  """Base class to filter crosswind telemetry for scoring."""

  def __init__(self, scoring_function, min_altitude=20.0,
               filter_by_min_altitude=True,
               flight_mode_time_threshold=0.0):

    scoring_function = FlightModeFilter(
        'kFlightModeCrosswindNormal', scoring_function,
        flight_mode_time_threshold)

    if filter_by_min_altitude:
      scoring_function = MinAltitudeFilter(min_altitude, scoring_function)

    super(CrosswindFilter, self).__init__(
        scoring_function.name, scoring_function)

  def GetTimeSeries(self, params, sim_telemetry, control_telemetry):
    return self._scoring_function.GetTimeSeries(
        params, sim_telemetry, control_telemetry)


class SingleSidedLimitScoringFunction(ScoringFunction):
  """Abstract class for a score determined by comparing a value against limits.

  If good_limit < bad_limit, then:
    * 1.0 or greater is reported if GetValue() >= bad_limit,
    * 0.0 or less is reported if GetValue() <= good_limit

  When bad_limit < good_limit, then:
    * 1.0 or greater is reported if GetValue() <= bad_limit,
    * 0.0 or less is reported if GetValue() => good_limit

  Linear interpolation provides the remaining values.  If GetValue() returns
  an array, the worst score is taken.
  """

  def __init__(self, name, units, good_limit, bad_limit, severity):
    super(SingleSidedLimitScoringFunction, self).__init__(name, units, severity)
    if good_limit == bad_limit:
      raise LimitsException()
    self._bad_limit = bad_limit
    self._good_limit = good_limit

  def GetScore(self, output):
    if not output:
      return float('nan')
    return numpy.max((numpy.array(self.GetValue(output)) - self._good_limit)
                     / (self._bad_limit - self._good_limit))

  def GetFailureCount(self, timeseries):
    if timeseries is None:
      return None
    if len(timeseries.keys()) > 1:
      raise NotImplementedError(
          '`timeseries` contains more than one key, therefore `GetFailureCount`'
          ' must be overriden in the "%s" scoring function class.' %
          self.GetName())
    timeseries = timeseries[timeseries.keys()[0]]
    if self._good_limit < self._bad_limit:
      return (numpy.array(timeseries) >= self._bad_limit).sum()
    else:
      return (numpy.array(timeseries) <= self._bad_limit).sum()

  def GetIndexOfFirstFailure(self, timeseries):
    if timeseries is None:
      return None
    if len(timeseries.keys()) > 1:
      raise NotImplementedError(
          '`timeseries` contains more than one key, therefore '
          '`GetIndexOfFirstFailure` must be overriden in the "%s" scoring '
          'function class.' % self.GetName())
    if self.GetFailureCount(timeseries) == 0:
      return -1
    timeseries = timeseries[timeseries.keys()[0]]
    if self._good_limit < self._bad_limit:
      return numpy.where(numpy.array(timeseries) >= self._bad_limit)[0][0]
    else:
      return numpy.where(numpy.array(timeseries) <= self._bad_limit)[0][0]

  def Limits(self):
    return (self._good_limit, self._bad_limit)


class DoubleSidedLimitScoringFunction(ScoringFunction):
  """Abstract class for a score computed by comparing a value against intervals.

  Derives a score from GetValue():
    * 1.0 or greater is reported if GetValue() <= bad_lower_limit,
    * 0.0 or less is reported if good_lower_limit <= GetValue()
          and GetValue() <= good_upper_limit,
    * 1.0 or greater is reported if bad_upper_limit <= GetValue().

  Linear interpolation provides the remaining values.  If GetValue() returns
  an array, the worst score is taken.
  """

  def __init__(self, name, units, bad_lower_limit, good_lower_limit,
               good_upper_limit, bad_upper_limit, severity):
    super(DoubleSidedLimitScoringFunction, self).__init__(name, units, severity)
    if not (bad_lower_limit < good_lower_limit
            and good_lower_limit < good_upper_limit
            and good_upper_limit < bad_upper_limit):
      raise LimitsException()
    self._bad_lower_limit = bad_lower_limit
    self._good_lower_limit = good_lower_limit
    self._bad_upper_limit = bad_upper_limit
    self._good_upper_limit = good_upper_limit

  def Limits(self):
    return (self._bad_lower_limit, self._good_lower_limit,
            self._good_upper_limit, self._bad_upper_limit)

  def GetScore(self, output):
    if not output:
      return float('nan')
    value = numpy.array(self.GetValue(output))
    return numpy.max((numpy.max(
        (value - self._good_upper_limit) /
        (self._bad_upper_limit - self._good_upper_limit)), numpy.max(
            (self._good_lower_limit - value)
            / (self._good_lower_limit - self._bad_lower_limit))))

  def GetFailureCount(self, timeseries):
    if timeseries is None:
      return None
    if len(timeseries.keys()) > 1:
      raise NotImplementedError(
          '`timeseries` contains more than one key, therefore `GetFailureCount`'
          ' must be overriden in the "%s" scoring function class.' %
          self.GetName())
    timeseries = timeseries[timeseries.keys()[0]]
    below_bad_limit = (numpy.array(timeseries) <= self._bad_lower_limit).sum()
    above_bad_limit = (numpy.array(timeseries) >= self._bad_upper_limit).sum()
    return below_bad_limit + above_bad_limit

  def GetIndexOfFirstFailure(self, timeseries):
    if timeseries is None:
      return None
    if len(timeseries.keys()) > 1:
      raise NotImplementedError(
          '`timeseries` contains more than one key, therefore '
          '`GetIndexOfFirstFailure` must be overriden in the "%s" scoring '
          'function class.' % self.GetName())
    if self.GetFailureCount(timeseries) == 0:
      return -1
    timeseries = timeseries[timeseries.keys()[0]]
    timeseries_array = numpy.array(timeseries)
    return (numpy.amin(numpy.where(numpy.logical_or(
        timeseries_array <= self._bad_lower_limit,
        timeseries_array >= self._bad_upper_limit))))


class WrappedLimitScoringFunction(SingleSidedLimitScoringFunction):
  """Abstract class for a score computed with wrapped limits.

  A subset of SingleSidedLimitScoringFunction that allows for
  additional parameters to be passed in to allow the scoring function
  to access them.

  Behaves like SingleSidedLimitScoringFunction with 0.0 as good_limit and
  tol as bad_limit.

  See scoring_functions_util.GetDistToWrappedLimits for other arguments.

  Note: This simply sets up the scoring function to access additional
      values. The scoring function should use
      scoring_functions_util.GetDistToWrappedLimits (or similar) to
      utilize these additional parameters.
  """

  def __init__(self, name, units, bad_start_limit,
               bad_end_limit, tol, wrap_left, wrap_right, severity):
    super(WrappedLimitScoringFunction, self).__init__(name, units, 0.0, tol,
                                                      severity)

    self._bad_start_limit = bad_start_limit
    self._bad_end_limit = bad_end_limit
    self._tol = tol
    self._wrap_left = wrap_left
    self._wrap_right = wrap_right

  def Limits(self):
    return (0.0, self._tol)

  def GetScore(self, output):
    return self.GetValue(output) / self._tol


class DurationScoringFunction(DoubleSidedLimitScoringFunction):
  """Tests if the duration lies in an acceptable range."""

  def __init__(self, bad_lower_limit, good_lower_limit, good_upper_limit,
               bad_upper_limit, severity, extra_system_labels=None):
    super(DurationScoringFunction, self).__init__(
        'Duration', 's', bad_lower_limit, good_lower_limit,
        good_upper_limit, bad_upper_limit, severity)
    self._extra_system_labels = extra_system_labels

  def GetSystemLabels(self):
    base_system_labels = super(DurationScoringFunction, self).GetSystemLabels()
    if isinstance(self._extra_system_labels, list):
      return self._extra_system_labels + base_system_labels
    else:
      return base_system_labels

  def GetValue(self, output):
    return output['duration']

  def GetTimeSeries(self, params, unused_sim, control):
    duration = float('nan')
    if control.size > 2:
      duration = control['time'][-1] - control['time'][0]

    return {
        'duration': duration
    }


def ScoreLogFile(scoring_functions, data, control_telem_only=False):
  """Apply a list of scoring functions to the contents of a log file.

  Args:
    scoring_functions: List of ScoringFunction objects.
    data: Log file data loaded, for example, using h5py.File.
    control_telem_only: Use only controller telemetry to evaluate scores.

  Returns:
    A tuple with the following elements:
      - A dict whose keys are scoring function names and whose values are tuples
    (score, value, severity, failure_count, index_of_first_failure).
      - A dict mapping scoring function names to execution times.
  """
  parameters = data['parameters']
  messages = data['messages']
  # Make compatible with both 'command center' and 'wing' flight logs.
  all_controller_fields = messages['kAioNodeControllerA'].keys()
  if 'kMessageTypeControlDebug' in all_controller_fields:
    control_telem = (messages['kAioNodeControllerA']
                     ['kMessageTypeControlDebug']['message'])
  elif 'kMessageTypeControlTelemetry' in all_controller_fields:
    control_telem = (messages['kAioNodeControllerA']
                     ['kMessageTypeControlTelemetry']['message'])
  else:
    assert False

  # Make compatible with both sim and flight logs.
  sim_telem = None
  # Load sim telemetry if available and if scoring is not user restricted to
  # use controller telemetry for evaluation.
  if messages['kAioNodeSimulator'].keys() and not control_telem_only:
    sim_telem = (messages['kAioNodeSimulator']['kMessageTypeSimTelemetry']
                 ['message'])

  scores = collections.OrderedDict()
  score_times = collections.OrderedDict()
  for scoring_function in scoring_functions:
    start_time = time.time()
    timeseries = scoring_function.GetTimeSeries(parameters, sim_telem,
                                                control_telem)
    output = scoring_function.GetOutput(timeseries)
    try:
      failure_count = scoring_function.GetFailureCount(timeseries)
      assert isinstance(failure_count, int) or failure_count is None
    except NotImplementedError:
      failure_count = None
    try:
      index_of_first_failure = scoring_function.GetIndexOfFirstFailure(
          timeseries)
      assert (isinstance(index_of_first_failure, int) or
              index_of_first_failure is None)
    except NotImplementedError:
      index_of_first_failure = None
    scores[scoring_function.GetName()] = (
        scoring_function.GetScore(output),
        scoring_function.GetValue(output),
        scoring_function.GetSeverity(),
        failure_count,
        index_of_first_failure
    )
    end_time = time.time()
    score_times[scoring_function.GetName()] = end_time - start_time
  return scores, score_times


def FormatScores(scores, print_failure_count=False,
                 print_index_of_first_failure=False):
  """Provide a color-coded formatting of the output of ScoreLogFile."""
  ok = '\033[94m'
  warn = '\033[93m'
  fail = '\033[91m'
  result = []
  for name, (score, value, severity, failure_count,
             index_of_first_failure) in scores.iteritems():
    if score <= 1.0 / 3.0:
      color = ok
    elif score <= 2.0 / 3.0:
      color = warn
    else:
      color = fail
    norm_score = 999 * min(max(score, 0.0) / 10.0, 1.0)
    line_result = color + '%83s (S-%s) | %3.0f ' % (name, severity, norm_score)
    if print_failure_count:
      if failure_count is not None:
        line_result += '| %6d ' % failure_count
      else:
        line_result += '| %6s ' % 'N/A'
    if print_index_of_first_failure:
      if index_of_first_failure is not None:
        line_result += '| %6d ' % index_of_first_failure
      else:
        line_result += '| %6s ' % 'N/A'
    line_result += '| %s\033[0m' % GetDisplayedString(value)
    result.append(line_result)
  return '\n'.join(result)


def FormatScoreTimes(score_times):
  """Provide a color-coded formatting of the output of ScoreLogFile."""
  ok = '\033[94m'
  warn = '\033[93m'
  fail = '\033[91m'
  result = []
  for name, score_time in score_times.iteritems():
    if score_time < 0.2:
      color = ok
    elif score_time < 2.0:
      color = warn
    else:
      color = fail
    result.append(color + '%80s | %5.3f\033[0m'
                  % (name, score_time))
  return '\n'.join(result)


def GetDisplayedString(value):
  """Round the value to two decimal places for display."""
  if isinstance(value, (list, numpy.ndarray)):
    return [GetDisplayedString(v) for v in value]
  elif isinstance(value, float) or isinstance(value, numpy.float64):
    if not numpy.isnan(value):
      value = int(value * 100) / 100.0
  return value


def SaveScoresToCsv(scores, csv_filename):
  """Saves scores to file in CSV format.

  Args:
    scores: Dictionary whose keys are scoring function names and whose values
        are tuples (score, value, severity, failure_count,
        index_of_first_failure).
    csv_filename: Output file name.
  """
  delimiter = ';'
  with open(csv_filename, 'w') as csv_file:
    csv_writer = csv.writer(csv_file, delimiter=delimiter)
    csv_writer.writerow(('score_name', 'normalized_score', 'score_values',
                         'severity'))
    for name, (score, value, severity, _, _) in scores.iteritems():
      csv_writer.writerow((name, score, value, severity))
