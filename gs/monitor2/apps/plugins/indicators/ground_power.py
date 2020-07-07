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

"""Monitor indicators for ground power."""
import math

from makani.avionics.ground_power.q7 import inverter_types
from makani.avionics.linux.swig import aio_util
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins.indicators import ground_power_faults

_INVERTER_TIMEOUT_SEC = 25.0
_INVERTER_MAX_STALE_COUNT = 6


class BaseAllInvertersIndicator(indicator.BaseAttributeIndicator):
  """Base class for ground power."""

  def __init__(self, name, show_label, field, **base_kwargs):
    super(BaseAllInvertersIndicator, self).__init__(
        [('GroundPowerStatus', 'GroundPowerQ7A')], name, **base_kwargs)

    self._messages = [None] * inverter_types.kNumInverters
    self._last_updated_time = [0] * inverter_types.kNumInverters
    self._show_label = show_label
    self._str_length_per_field = 15
    self._field = field

  def _AggregatedFilter(self, messages):
    blank = '--'.rjust(self._str_length_per_field)
    text = self._GetHeadline()
    str_format = '%% %d.1f' % self._str_length_per_field
    text.append(
        '  '.join((str_format % self._GetCTypeFieldByString(m, self._field))
                  if m else blank for m in messages))
    return '\n'.join(text), stoplights.STOPLIGHT_NORMAL

  def _Filter(self, ground_power):
    current_time_sec = aio_util.ClockGetUs() * 1.0e-6
    if ground_power:
      assert ground_power.id < inverter_types.kNumInverters
      if ground_power.stale_count <= _INVERTER_MAX_STALE_COUNT:
        self._messages[ground_power.id] = ground_power
        self._last_updated_time[ground_power.id] = current_time_sec
      else:
        self._messages[ground_power.id] = None

    any_missing = False
    all_missing = True
    for n in range(inverter_types.kNumInverters):
      if current_time_sec - self._last_updated_time[n] > _INVERTER_TIMEOUT_SEC:
        self._messages[n] = None
      if self._messages[n] is None:
        any_missing = True
      else:
        all_missing = False

    if all_missing:
      comm_stoplight = stoplights.STOPLIGHT_ERROR
    elif any_missing:
      comm_stoplight = stoplights.STOPLIGHT_WARNING
    else:
      comm_stoplight = stoplights.STOPLIGHT_NORMAL

    text, stoplight = self._AggregatedFilter(self._messages)
    stoplight = stoplights.MostSevereStoplight(stoplight, comm_stoplight)
    return text, stoplight

  def _GetHeadline(self):
    if self._show_label:
      return [
          '  '.join([('Inverter %d' % (n + 1)).rjust(self._str_length_per_field)
                     for n in range(inverter_types.kNumInverters)])]
    else:
      return []


class BaseAllInvertersDictChart(indicator.BaseAttributeDictChart):
  """The base dict chart for inverter status."""

  def __init__(self, name, banks, field, **widget_kwargs):
    super(BaseAllInvertersDictChart, self).__init__(
        [('GroundPowerStatus', 'GroundPowerQ7A')],
        [self._GetLabel(n) for n in banks], name, **widget_kwargs)
    self._messages = {n: None for n in banks}
    self._last_updated_time = {n: 0 for n in banks}
    self._banks = banks
    self._field = field

  def _GetLabel(self, inverter_id):
    return 'Inverter.%d' % (inverter_id + 1)

  def _Filter(self, ground_power):
    current_time_sec = aio_util.ClockGetUs() * 1.0e-6
    if ground_power:
      if ground_power.id in self._banks:
        self._messages[ground_power.id] = ground_power
        self._last_updated_time[ground_power.id] = current_time_sec

    for n in self._banks:
      if current_time_sec - self._last_updated_time[n] > _INVERTER_TIMEOUT_SEC:
        self._messages[n] = None
    return self._FilterDictValues(self._messages)

  def _FilterDictValues(self, messages):
    values = {}
    timestamps = {}
    any_missing = False
    all_missing = True
    for bank in self._banks:
      label = self._GetLabel(bank)
      if bank in messages and messages[bank]:
        ground_power = messages[bank]
        values[label] = getattr(ground_power, self._field)
        timestamps[label] = ground_power.capture_info['timestamp']
        all_missing = False
      else:
        values[label] = None
        timestamps[label] = None
        any_missing = True

    if all_missing:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif any_missing:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return timestamps, values, stoplight


class SummaryTransformerTempIndicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryTransformerTempIndicator, self).__init__(
        'Transformer Temperatures [&deg;C]', show_label,
        'transformer_temp')


class SummaryDCVoltageIndicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryDCVoltageIndicator, self).__init__(
        'DC Voltage [V]', show_label, 'v_dc')


class SummaryDCCurrentIndicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryDCCurrentIndicator, self).__init__(
        'DC Current [A]', show_label, 'i_dc')


class SummaryMeanCommonModeVoltsIndicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryMeanCommonModeVoltsIndicator, self).__init__(
        'Mean Common Mode Volts [V]', show_label, 'mean_common_mode_v')


class SummaryInstCommonModeVoltsIndicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryInstCommonModeVoltsIndicator, self).__init__(
        'Inst. Common Mode Volts [V]', show_label, 'inst_common_mode_v')


class SummaryCBAirTempIndicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryCBAirTempIndicator, self).__init__(
        'Ctrl Board Air Temperature [&deg;C]', show_label, 'cb_air_temp')


class SummaryHeatSink1T1Indicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryHeatSink1T1Indicator, self).__init__(
        'Heatsink 1 Temperature 1 [&deg;C]', show_label, 'heatsink1_temp1')


class SummaryHeatSink1T2Indicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryHeatSink1T2Indicator, self).__init__(
        'Heatsink 1 Temperature 2 [&deg;C]', show_label, 'heatsink1_temp2')


class VoltageIndicator(BaseAllInvertersDictChart):

  def __init__(self, banks, **widget_kwargs):
    super(VoltageIndicator, self).__init__(
        'Voltage %s [V]' % [n + 1 for n in banks], banks, 'v_dc',
        trail_length=120, **widget_kwargs)


class CurrentIndicator(BaseAllInvertersDictChart):

  def __init__(self, banks, **widget_kwargs):
    super(CurrentIndicator, self).__init__(
        'Current %s [A]' % [n + 1 for n in banks], banks, 'i_dc',
        trail_length=120, **widget_kwargs)


class SummaryFaultInductorIndicator(BaseAllInvertersIndicator):

  def __init__(self, show_label):
    super(SummaryFaultInductorIndicator, self).__init__(
        'Fault Inductor [1 | 0]', show_label, 'fault_inductor_status')

  def _AggregatedFilter(self, messages):
    blank = '--'.rjust(self._str_length_per_field)
    text = self._GetHeadline()
    statuses = []
    warning = False
    str_format = '%% %dd' % self._str_length_per_field
    for m in messages:
      if m:
        fault = self._GetCTypeFieldByString(m, self._field)
        statuses.append(str_format % fault)
        # Raise a warning if the fault inductor is fired (fault == 1).
        if fault:
          warning = True
      else:
        statuses.append(blank)
    text.append('  '.join(statuses))
    stoplight = (stoplights.STOPLIGHT_WARNING if warning
                 else stoplights.STOPLIGHT_NORMAL)
    return '\n'.join(text), stoplight


class SummaryFaultIndicator(BaseAllInvertersIndicator):
  """The fault indicator for inverters."""

  def __init__(self, name, word_index, show_label, max_visible_faults=3):
    super(SummaryFaultIndicator, self).__init__(name, show_label, None)
    self._word_index = word_index
    self._max_visible_faults = max_visible_faults

  def _GetFaults(self, word_index, messages):
    key = 'fault_word%d' % (word_index + 1)
    max_faults_per_inverter = 0
    warning = False
    error = False
    faults = []
    for m in messages:
      inverter_faults = []
      if m:
        fault = self._GetCTypeFieldByString(m, key)
        if fault:
          for n in range(16):
            if fault & (1 << n):
              fault_str = ground_power_faults.INVERTER_FAULTS[word_index][n]
              inverter_faults.append(fault_str)
              if fault_str == 'Command Stop':
                warning = True
              else:
                error = True
      faults.append(inverter_faults)
      max_faults_per_inverter = max(
          max_faults_per_inverter, len(inverter_faults))
    return faults, error, warning, max_faults_per_inverter

  def _AggregatedFilter(self, messages):
    faults, error, warning, max_faults_per_inverter = self._GetFaults(
        self._word_index, messages)

    num_visible_faults = min(self._max_visible_faults, max_faults_per_inverter)
    blank = '--'.rjust(self._str_length_per_field)
    text = self._GetHeadline()
    for n in range(num_visible_faults):
      text.append(
          '  '.join(f[n].rjust(self._str_length_per_field)
                    if n < len(f) else blank for f in faults))
    if max_faults_per_inverter > num_visible_faults:
      text.append('There are more faults that are not shown.')

    if error:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL
    return '\n'.join(text), stoplight


class SummaryIndicator(SummaryFaultIndicator):
  """Summarize the inverter status by gathering all faults.

  The indicator is red if the inverter is disconnected or has faults.
  It is yellow if the inverter is stopped but ready to start.
  """

  _NUM_FAULT_WORDS = 8

  def __init__(self, name):
    # word_index and show_label do not apply here.
    super(SummaryIndicator, self).__init__(
        name, None, False, max_visible_faults=1)
    self._cycle = 0

  def _AggregatedFilter(self, messages):
    faults = set()
    any_value = False
    fault_inductor_fired = False
    for m in messages:
      if m:
        any_value = True
        if m.fault_inductor_status:
          fault_inductor_fired = True

    for n in range(self._NUM_FAULT_WORDS):
      partial_faults = self._GetFaults(n, messages)[0]
      for f in partial_faults:
        faults.update(f)

    error = False
    warning = False
    warning_faults = set(['Command Stop'])
    # Warning faults (e.g. 'Command Stop') cause other faults.
    if faults & warning_faults:
      warning = True
    elif faults:
      error = True

    if fault_inductor_fired:
      warning = True
      faults.add('Fault inductor has fired')

    if not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif error:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    if not faults:
      text = 'Normal' if any_value else '--'
    else:
      faults = sorted(faults)
      num_faults = len(faults)
      num_groups = int(
          math.ceil(float(num_faults) / float(self._max_visible_faults)))
      second = aio_util.ClockGetUs() * 1.0e-6
      group_index = int(second) % num_groups
      start_idx = group_index * self._max_visible_faults
      text = ', '.join(
          faults[start_idx : start_idx + self._max_visible_faults])
    return text, stoplight


class FaultIsolationIndicator(BaseAllInvertersIndicator):

  def __init__(self, name):
    super(FaultIsolationIndicator, self).__init__(
        name, False, 'mean_common_mode_v')
    self._str_length_per_field = 6
    self._str_format = '%% %d.1f' % self._str_length_per_field

  def _AggregatedFilter(self, messages):
    blank = '--'.rjust(self._str_length_per_field)
    banks = [0, inverter_types.kNumInverters - 1]
    any_valid = False
    voltages = []
    for bank in banks:
      message = messages[bank]
      if message:
        any_valid = True
        value = self._str_format % getattr(message, self._field)
      else:
        value = blank
      voltages.append('I% 2d: %s' % (bank + 1, value))
    return ', '.join(voltages), (
        stoplights.STOPLIGHT_NORMAL if any_valid
        else stoplights.STOPLIGHT_UNAVAILABLE)
