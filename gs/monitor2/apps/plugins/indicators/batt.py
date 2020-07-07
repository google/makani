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

""""Monitor indicators from the ground station."""

import collections

from makani.analysis.checks import avionics_util
from makani.avionics.firmware.monitors import batt_types
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.gs.monitor2.apps.plugins.indicators import soc_calc
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree

BATT_ANALOG_VOLTAGE_HELPER = c_helpers.EnumHelper('BattAnalogVoltage',
                                                  batt_types)

BATT_LTC4151_MONITOR_HELPER = c_helpers.EnumHelper('BattLtc4151Monitor',
                                                   batt_types)

BATT_MCP342X_MONITOR_HELPER = c_helpers.EnumHelper('BattMcp342xMonitor',
                                                   batt_types)

BATT_MON_WARNING_HELPER = c_helpers.EnumHelper('BattMonitorWarning',
                                               batt_types)

BATT_MON_ERROR_HELPER = c_helpers.EnumHelper('BattMonitorError', batt_types)

BATT_MON_STATUS_HELPER = c_helpers.EnumHelper('BattMonitorStatus', batt_types)

BATT_HARDWARE_HELPER = c_helpers.EnumHelper('BattHardware', batt_types)

_SIGNIFICANT_CHARGE_CURRENT = 0.5

_BIG_REVS_LIST = [BATT_HARDWARE_HELPER.Value('BigCell18V1'),
                  BATT_HARDWARE_HELPER.Value('BigCell18Aa'),
                  BATT_HARDWARE_HELPER.Value('BigCell18Ab'),
                  BATT_HARDWARE_HELPER.Value('BigCell18Ac')]


class BatteryStatus(object):
  """The class to perform various checks for battery status."""

  def __init__(self, batts, aio_nodes, verbose_charging=False):
    self._batts = batts
    self._aio_nodes = aio_nodes
    self._verbose_charging = verbose_charging
    assert len(batts) == len(aio_nodes)
    self._i_hall_index = BATT_ANALOG_VOLTAGE_HELPER.Value('IHall')
    self._big_revs = _BIG_REVS_LIST

  def CheckConnection(self):
    """Check connection status."""
    connected = {}
    error = False
    error_msg = []

    for i, aio_node in enumerate(self._aio_nodes):
      status = self._batts[i].batt_mon.flags.status
      is_connected = status & BATT_MON_STATUS_HELPER.Value('Connected')
      connected[aio_node] = is_connected
      if not is_connected:
        error = True
        error_msg.append('%s Disconnected' % aio_node)
    return connected, error, error_msg

  def CheckErrorWarning(self):
    """Check status errors and warnings."""
    combined_warning = False
    combined_error = False
    warning_msg = []
    error_msg = []

    for i, aio_node in enumerate(self._aio_nodes):
      warning = self._batts[i].batt_mon.flags.warning
      error = self._batts[i].batt_mon.flags.error
      if warning:
        combined_warning = True
        warning_msg.append('%s Warning' % aio_node)
      if error:
        combined_error = True
        error_msg.append('%s Error' % aio_node)

    return combined_warning, combined_error, warning_msg, error_msg

  def CheckAnalogStatus(self):
    """Check analog statuses."""
    combined_warning = False
    combined_error = False
    warning_msg = []
    error_msg = []
    for i, aio_node in enumerate(self._aio_nodes):
      _, warning, error, _, _ = avionics.GetMonitorFields(
          self._batts[i], 'batt_mon', 'analog',
          BATT_ANALOG_VOLTAGE_HELPER,
          BATT_MON_WARNING_HELPER, BATT_MON_ERROR_HELPER,
          ['LvA', 'LvB', '12v', '5v', 'VLvOr', 'IHall'], aio_node, None)

      if warning:
        combined_warning = True
        warning_msg.append('%s Analog Warning' % aio_node)

      if error:
        combined_error = True
        error_msg.append('%s Analog Error' % aio_node)

    return combined_warning, combined_error, warning_msg, error_msg

  def CheckMcp342xStatus(self):
    """Check Mcp342x status fields."""
    combined_warning = False
    combined_error = False
    warning_msg = []
    error_msg = []
    for i, aio_node in enumerate(self._aio_nodes):
      _, warning, error, _, read_error = avionics.GetMonitorFields(
          self._batts[i], 'batt_mon', 'mcp342x',
          BATT_MCP342X_MONITOR_HELPER,
          BATT_MON_WARNING_HELPER, BATT_MON_ERROR_HELPER,
          ['Batteries1', 'Batteries2', 'HeatPlate1', 'HeatPlate2'],
          aio_node, 'TempReadErrors')

      if warning:
        combined_warning = True
        warning_msg.append('%s Temperature Warning' % aio_node)

      if error:
        combined_error = True
        error_msg.append('%s Temperature Error' % aio_node)

      if read_error:
        combined_error = True
        error_msg.append('%s Read Error' % aio_node)

    return combined_warning, combined_error, warning_msg, error_msg

  def CheckStateOfCharge(self, slow_statuses):
    """Check the charge levels."""
    assert len(slow_statuses) == len(self._aio_nodes)
    charger_socs = {}
    combined_warning = False
    combined_error = False
    warning_msg = []
    error_msg = []
    for i, aio_node in enumerate(self._aio_nodes):
      batt = self._batts[i]
      revision = slow_statuses[i].carrier_serial_params.hardware_revision
      stack_voltage = batt.batt_mon.ltc6804_data.stack_voltage
      i_hall = batt.batt_mon.analog_data[self._i_hall_index]

      if revision in self._big_revs:
        charger_socs[aio_node] = soc_calc.calculate_soc(
            stack_voltage, i_hall)

    if charger_socs:
      if max(charger_socs.values()) < 30.0:
        combined_error = True
        error_msg.append('Batt is low')
      elif max(charger_socs.values()) < 90.0:
        combined_warning = True
        warning_msg.append('Batt is not full')

    return combined_warning, combined_error, warning_msg, error_msg

  def CheckBattsDischarge(self, discharge_watchers):
    """Return true if batt powering bus for significant time period."""
    for i, aio_node in enumerate(self._aio_nodes):
      batt = self._batts[i]
      output_current = batt.batt_mon.analog_data[self._i_hall_index]
      if discharge_watchers[aio_node].Check(output_current):
        return True
    return False

  def CheckChargeStatus(self):
    status_msg = []
    for i, aio_node in enumerate(self._aio_nodes):
      batt = self._batts[i]
      if batt.batt_mon.charger_current > _SIGNIFICANT_CHARGE_CURRENT:
        status_msg.append(aio_node + ' is charging.')
    return status_msg

  def CheckAll(self, slow_statuses, discharge_watchers):
    """Perform all checks on the batteries."""
    batts_discharge = False  # True if batteries are powering LV bus.
    final_warning = False
    final_error = False
    final_warning_msg = []
    final_error_msg = []
    final_status_msg = []

    status_msg = self.CheckChargeStatus()
    if self._verbose_charging:
      final_status_msg += status_msg

    _, error, error_msg = self.CheckConnection()
    final_error |= error
    final_error_msg += error_msg
    if final_error:
      return (final_warning, final_error, final_warning_msg, final_error_msg,
              final_status_msg, batts_discharge)

    warning, error, warning_msg, error_msg = self.CheckErrorWarning()
    final_warning |= warning
    final_warning_msg += warning_msg
    final_error |= error
    final_error_msg += error_msg

    warning, error, warning_msg, error_msg = self.CheckAnalogStatus()
    final_warning |= warning
    final_warning_msg += warning_msg
    final_error |= error
    final_error_msg += error_msg

    warning, error, warning_msg, error_msg = self.CheckMcp342xStatus()
    final_warning |= warning
    final_warning_msg += warning_msg
    final_error |= error
    final_error_msg += error_msg

    warning, error, warning_msg, error_msg = self.CheckStateOfCharge(
        slow_statuses)
    final_warning |= warning
    final_warning_msg += warning_msg
    final_error |= error
    final_error_msg += error_msg

    batts_discharge = self.CheckBattsDischarge(discharge_watchers)

    return (final_warning, final_error, final_warning_msg, final_error_msg,
            final_status_msg, batts_discharge)


class BatterySummaryIndicator(indicator.BaseAttributeIndicator):
  """Indicator to summarize battery status."""

  def __init__(self, num_lines, enable_error, name='Batteries'):
    super(BatterySummaryIndicator, self).__init__(
        [('BatteryStatus', 'BattA'),
         ('BatteryStatus', 'BattB'),
         ('SlowStatus', 'BattA'),
         ('SlowStatus', 'BattB'),
        ], name)
    self._num_lines = num_lines
    # True if battery error will appear as red, yellow otherwise.
    self._enable_error = enable_error
    # Consider batteries to be powering bus if >1A output for >1s.
    self._discharge_watchers = {batt: common.DurationWatcher([1.0, 1.0e6], 1.0)
                                for batt in ['BattA', 'BattB']}

  def _Filter(self, batt_a, batt_b, slow_status_a, slow_status_b):
    aio_nodes = []
    batts = []
    slow_statuses = []

    if (struct_tree.IsValidElement(batt_a) and
        struct_tree.IsValidElement(slow_status_a)):
      aio_nodes.append('BattA')
      batts.append(batt_a)
      slow_statuses.append(slow_status_a)

    if (struct_tree.IsValidElement(batt_b) and
        struct_tree.IsValidElement(slow_status_b)):
      aio_nodes.append('BattB')
      batts.append(batt_b)
      slow_statuses.append(slow_status_b)

    if not aio_nodes:
      return (common.FillLines(['Unavailable'], self._num_lines),
              stoplights.STOPLIGHT_ERROR
              if self._enable_error else stoplights.STOPLIGHT_WARNING)

    status = BatteryStatus(batts, aio_nodes, verbose_charging=True)
    (warning, error, warning_msg, error_msg, status_msg,
     batts_discharge) = status.CheckAll(slow_statuses, self._discharge_watchers)
    if batts_discharge: status_msg += ['Batts powering bus.']

    if error:
      return (common.FillLines(error_msg, self._num_lines),
              stoplights.STOPLIGHT_ERROR
              if self._enable_error else stoplights.STOPLIGHT_WARNING)
    elif warning:
      return (common.FillLines(warning_msg + status_msg, self._num_lines),
              stoplights.STOPLIGHT_WARNING)
    else:
      return (common.FillLines(['Normal'] + status_msg, self._num_lines),
              stoplights.STOPLIGHT_NORMAL)


class BattConnectedIndicator(indicator.BaseIndicator):
  """Indicator for LV batteries' connectivity status."""

  def __init__(self, batts, name, verbose=True):
    self._short_names = batts
    self._verbose = verbose
    super(BattConnectedIndicator, self).__init__(name)

  def Filter(self, messages):
    if not messages:
      return ('\n'.join(['%s: --' % s for s in self._short_names]),
              stoplights.STOPLIGHT_UNAVAILABLE)

    any_reachable = False
    all_connected = True
    any_misconfigured = False
    statuses = []

    for batt in self._short_names:
      status = messages['BatteryStatus.Batt%s.batt_mon.flags.status' % batt]
      warnings = messages['BatteryStatus.Batt%s.batt_mon.flags.warning' % batt]
      if status is None:
        statuses.append('%s: --' % batt)
        continue
      any_reachable = True
      is_connected = status & BATT_MON_STATUS_HELPER.Value('Connected')
      if not is_connected:
        all_connected = False

      # Warn user if box is staying disconnected to protect other box.
      dual_big_box = ''
      if (warnings & batt_types.kBattMonitorWarningOCProtect
          and not is_connected
          and self._verbose):
        dual_big_box = '    Box won\'t connect until other box charges more.'

      statuses.append('%s: %s%s' % (batt, 'Connected' if is_connected
                                    else 'Disconnected', dual_big_box))

      # Warn user if box isn't configured for dual-big but should be.
      msc_msg = '\n    Warning: box improperly configured. Should be dual_big.'
      if warnings & batt_types.kBattMonitorWarningMisconfigured:
        any_misconfigured = True
        if self._verbose:
          statuses[-1] += msc_msg

    if not any_reachable:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    # One line for non-verbose flight layout; multi-line for verbose layout.
    if self._verbose:
      status_text = '\n'.join(statuses)
    else:
      status_text = '  '.join(statuses)

    if all_connected and not any_misconfigured:
      return status_text, stoplights.STOPLIGHT_NORMAL
    else:
      return status_text, stoplights.STOPLIGHT_WARNING


class BattChargerIndicator(indicator.BaseIndicator):
  """The base class for battery charger output voltage and current."""

  def __init__(self, name):
    self._short_names = ['A', 'B']
    self._i_hall_index = BATT_ANALOG_VOLTAGE_HELPER.Value('IHall')
    self._big_revs = _BIG_REVS_LIST
    super(BattChargerIndicator, self).__init__(name)

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    charger_voltages = collections.defaultdict(dict)
    charger_currents = collections.defaultdict(dict)
    charger_socs = collections.defaultdict(dict)
    batt_temps = collections.defaultdict(dict)
    any_value = False
    warning = False
    error_messages = []

    for batt in self._short_names:
      if 'BatteryStatus.Batt' + batt not in messages:
        continue
      any_value = True
      mcp342x_populated = messages[
          'BatteryStatus.Batt%s.batt_mon.mcp342x_populated' % batt]

      if mcp342x_populated:
        index_bt1 = BATT_MCP342X_MONITOR_HELPER.Value('Batteries1')
        index_bt2 = BATT_MCP342X_MONITOR_HELPER.Value('Batteries2')
        batt_temps[batt]['1'] = messages[
            'BatteryStatus.Batt%s.batt_mon.mcp342x_data[%d]' %
            (batt, index_bt1)]
        batt_temps[batt]['2'] = messages[
            'BatteryStatus.Batt%s.batt_mon.mcp342x_data[%d]' %
            (batt, index_bt2)]

      charger_voltages[batt] = messages[
          'BatteryStatus.Batt%s.batt_mon.ltc6804_data.stack_voltage'
          % (batt)]

      charger_currents[batt] = messages[
          'BatteryStatus.Batt%s.batt_mon.charger_current' % (batt)]

      # Calculate state-of-charge based on stack voltage and output current.
      revision = messages[
          'SlowStatus.Batt%s.carrier_serial_params.hardware_revision' % batt]
      stack_voltage = charger_voltages[batt]
      i_hall = messages['BatteryStatus.Batt%s.batt_mon.analog_data[%s]' %
                        (batt, self._i_hall_index)]
      if revision in self._big_revs:
        charger_socs[batt] = soc_calc.calculate_soc(stack_voltage, i_hall)
      else:
        # Small battery uses SOC estimation from BQ34Z100G1 coulomb counter.
        charger_socs[batt] = messages[
            'BatteryStatus.Batt%s.batt_mon.bq34z100_data[0].soc_per_cent' %
            batt]

    if error_messages:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    results = ['    Charger Status: voltage, current, state of charge']
    data_exists = False
    for batt in self._short_names:
      batt_text = batt + ':'
      if batt in charger_voltages:
        batt_text += ' {: 4.1f} [V] '.format(charger_voltages[batt])
        data_exists = True
      if batt in charger_currents:
        batt_text += ' {: 4.1f} [A] '.format(charger_currents[batt])
        data_exists = True
      if batt in charger_socs:
        batt_text += '    Charge remaining:{: 4.1f}%'.format(charger_socs[batt])
        data_exists = True
      if (batt in charger_currents and
          charger_currents[batt] > _SIGNIFICANT_CHARGE_CURRENT):
        batt_text += ' (Batteries are charging.)'
      if not data_exists:
        batt_text += ' --'.rjust(6)
      # Indicate if high or low temp is causing charger to shut off:
      if batt in batt_temps:
        if max(batt_temps[batt].values()) > 37.7:
          batt_text += ' (Too hot to charge.)'
        if min(batt_temps[batt].values()) < 10.0:
          batt_text += ' (Too cold to charge.)'
      results.append(batt_text)
    return '\n'.join(error_messages + results), stoplight


class BalancerIndicator(indicator.BaseIndicator):
  """The base class for displaying ltc6804 warnings, if any."""

  _ltc6804_name = 'Balancer'

  def __init__(self, batts):
    self._short_names = batts
    super(BalancerIndicator, self).__init__('Cell monitor')

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    ltc6804_min = collections.defaultdict(dict)
    ltc6804_max = collections.defaultdict(dict)
    read_error_count = {batt: 0 for batt in self._short_names}
    any_value = False
    warning = False
    error_messages = []

    for batt in self._short_names:
      if 'BatteryStatus.Batt' + batt not in messages:
        continue
      any_value = True

      # Guard against bad warning names.
      if self._ltc6804_name not in BATT_MON_WARNING_HELPER:
        error_messages.append('Batt %s: Invalid warning name (%s)' %
                              (batt, self._ltc6804_name))
        continue

      ltc6804_min[batt] = messages[
          'BatteryStatus.Batt%s.batt_mon.ltc6804_data.min_cell_v' % batt]

      ltc6804_max[batt] = messages[
          'BatteryStatus.Batt%s.batt_mon.ltc6804_data.max_cell_v' % batt]

      read_error_count[batt] = messages[
          'BatteryStatus.Batt%s.batt_mon.ltc6804_data.error_count' % batt]

      warning |= avionics_util.CheckWarning(
          messages['BatteryStatus.Batt%s.batt_mon.flags' % batt],
          BATT_MON_WARNING_HELPER.Value(self._ltc6804_name))

    if not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    elif error_messages:
      stoplight = stoplights.STOPLIGHT_ERROR
    elif warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    results = ['    '  + self._ltc6804_name]
    for batt in self._short_names:
      batt_text = batt + ':'
      if batt in ltc6804_min and batt in ltc6804_max:
        batt_text += ' {: 7.4f} [min cell V],'.format(ltc6804_min[batt])
        batt_text += ' {: 7.4f} [max cell V]'.format(ltc6804_max[batt])
        if ltc6804_max[batt] > 5.0:
          batt_text += ' (CHECK FOR DISCONNECTED BALANCE CABLE)\n'
        elif ltc6804_min[batt] < 2.0:
          batt_text += ' (CHECK FOR DISCONNECTED BALANCE CABLE OR DEAD CELLS)\n'
      else:
        batt_text += ' --'.rjust(6)
      if read_error_count[batt] > 0:
        batt_text += '    (CHIP UNRESPONSIVE - CHECK BALANCE CABLES)'
      results.append(batt_text)
    return '\n'.join(error_messages + results), stoplight


class BattMcp342xIndicator(avionics.BaseMonitorIndicator):

  def __init__(self):
    super(BattMcp342xIndicator, self).__init__(
        'BatteryStatus',
        'batt_mon',
        ['BattA', 'BattB'],
        'Temperature monitor',
        BATT_MON_ERROR_HELPER,
        BATT_MON_WARNING_HELPER,
        BATT_MCP342X_MONITOR_HELPER,
        'mcp342x',
        ['Batteries1', 'Batteries2', 'HeatPlate1', 'HeatPlate2'],
        'TempReadErrors')


class BattAnalogIndicator(avionics.BaseMonitorIndicator):

  def __init__(self):
    super(BattAnalogIndicator, self).__init__(
        'BatteryStatus',
        'batt_mon',
        ['BattA', 'BattB'],
        'Voltage and current monitors',
        BATT_MON_ERROR_HELPER,
        BATT_MON_WARNING_HELPER,
        BATT_ANALOG_VOLTAGE_HELPER,
        'analog',
        ['LvA', 'LvB', '12v', '5v', 'VLvOr', 'IHall'],
        None)


class BattErrorWarningIndicator(indicator.BaseIndicator):
  """Indicator explaining LV batteries' warnings or errors."""

  def __init__(self, batts, name):
    self._short_names = batts
    super(BattErrorWarningIndicator, self).__init__(name)

  def Filter(self, messages):
    if not messages:
      return ('\n'.join(['%s: --' % s for s in self._short_names]),
              stoplights.STOPLIGHT_UNAVAILABLE)

    any_reachable = False
    any_warnings = False
    any_errors = False
    alerts_present = {batt: [] for batt in self._short_names}
    statuses = []

    for batt in self._short_names:
      warnings = messages['BatteryStatus.Batt%s.batt_mon.flags.warning' % batt]
      errors = messages['BatteryStatus.Batt%s.batt_mon.flags.error' % batt]
      if warnings is None and errors is None:
        statuses.append('%s: --' % batt)
        continue
      any_reachable = True
      for warning in BATT_MON_WARNING_HELPER.Names():
        if warnings & BATT_MON_WARNING_HELPER.Value(warning):
          alerts_present[batt].append(warning)
          any_warnings = True
      for error in BATT_MON_ERROR_HELPER.Names():
        if errors & BATT_MON_ERROR_HELPER.Value(error):
          alerts_present[batt].append(error)
          any_errors = True

      chars_per_line = 1
      if alerts_present[batt]:
        chars_per_line += max([len(name) for name in alerts_present[batt]])
        statuses.append('%s: %s' %
                        (batt, common.TimeCycle(alerts_present[batt],
                                                chars_per_line, ',')))
      else: statuses.append('%s: No warnings or errors' % batt)

    status_text = '\n'.join(statuses)
    if not any_reachable:
      return status_text, stoplights.STOPLIGHT_UNAVAILABLE
    elif any_errors:
      return status_text, stoplights.STOPLIGHT_ERROR
    elif any_warnings:
      return status_text, stoplights.STOPLIGHT_WARNING
    else:
      return status_text, stoplights.STOPLIGHT_NORMAL


class StateOfChargeIndicator(indicator.BaseIndicator):
  """State of charge indicator for flight monitor."""

  def __init__(self, batts, name):
    self._short_names = batts
    self._i_hall_index = BATT_ANALOG_VOLTAGE_HELPER.Value('IHall')
    self._big_revs = _BIG_REVS_LIST
    super(StateOfChargeIndicator, self).__init__(name)

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    charger_socs = collections.defaultdict(dict)
    any_value = False
    soc_chip_warning = False

    for batt in self._short_names:
      if 'BatteryStatus.Batt' + batt not in messages:
        continue
      any_value = True

      # Calculate state-of-charge based on stack voltage and output current.
      revision = messages[
          'SlowStatus.Batt%s.carrier_serial_params.hardware_revision' % batt]
      stack_voltage = messages[
          'BatteryStatus.Batt%s.batt_mon.ltc6804_data.stack_voltage' % batt]
      i_hall = messages['BatteryStatus.Batt%s.batt_mon.analog_data[%s]' %
                        (batt, self._i_hall_index)]
      if revision in self._big_revs:
        charger_socs[batt] = soc_calc.calculate_soc(stack_voltage, i_hall)
      else:
        # Small battery uses SOC estimation from BQ34Z100G1 coulomb counter.
        charger_socs[batt] = messages[
            'BatteryStatus.Batt%s.batt_mon.bq34z100_data[0].soc_per_cent' %
            batt]

      # Check for warning flagged by state of charge chip.
      if (messages['BatteryStatus.Batt%s.batt_mon.flags.warning' % batt] &
          batt_types.kBattMonitorWarningLowCharge):
        soc_chip_warning = True

    if not any_value:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE
    if soc_chip_warning:
      stoplight = stoplights.STOPLIGHT_WARNING
    else:
      stoplight = stoplights.STOPLIGHT_NORMAL

    results = []
    data_exists = False
    for batt in self._short_names:
      batt_text = batt + ':'
      if batt in charger_socs:
        batt_text += '{: 3.0f}%'.format(charger_socs[batt])
        data_exists = True
      if not data_exists:
        batt_text += ' --'.rjust(6)
      results.append(batt_text)
    return '   '.join(results), stoplight


def _ShowLvBusSummary(monitors, any_value, error_messages, error, warning):
  """Show the summary of LV bus voltages from the standpoint of batteries."""

  bus_monitors = [
      # Big Battery A.
      [
          # Primary Bus.
          monitors['LvB']['BattA'] if 'BattA' in monitors['LvB'] else None,
          # Secondary Bus.
          monitors['LvA']['BattA'] if 'BattA' in monitors['LvA'] else None,
      ],
      # Small Battery B.
      [
          # Primary Bus.
          monitors['LvA']['BattB'] if 'BattB' in monitors['LvA'] else None,
          # Secondary Bus.
          monitors['LvB']['BattB'] if 'BattB' in monitors['LvB'] else None,
      ],
  ]

  if error_messages or error:
    stoplight = stoplights.STOPLIGHT_ERROR
  elif not any_value:
    stoplight = stoplights.STOPLIGHT_UNAVAILABLE
  elif warning:
    stoplight = stoplights.STOPLIGHT_WARNING
  else:
    stoplight = stoplights.STOPLIGHT_NORMAL

  # Show Values as:
  #            Primary    Secondary
  # Big/A     BattA.LvB   BattA.LvA
  # Small/B   BattB.LvA   BattB.LvB
  results = [' ' * 10 +
             ' '.join(v.rjust(6) for v in ['Primary', 'Secondary'])]
  text = '{:9}'.format('Big/A')
  text += (' {: 6.1f}'.format(bus_monitors[0][0])
           if bus_monitors[0][0] is not None else ' --'.rjust(7))
  text += (' {: 6.1f}'.format(bus_monitors[0][1])
           if bus_monitors[0][1] is not None else ' --'.rjust(7))
  results.append(text)

  text = '{:9}'.format('Small/B')
  text += (' {: 6.1f}'.format(bus_monitors[1][0])
           if bus_monitors[1][0] is not None else ' --'.rjust(7))
  text += (' {: 6.1f}'.format(bus_monitors[1][1])
           if bus_monitors[1][1] is not None else ' --'.rjust(7))
  results.append(text)

  return '\n'.join(error_messages + results), stoplight


class LvBusVoltageIndicator(avionics.BaseMonitorIndicator):
  """Indicator for the voltages on primary and secondary Lv buses."""

  def __init__(self):
    super(LvBusVoltageIndicator, self).__init__(
        'BatteryStatus',
        'batt_mon',
        ['BattA', 'BattB'],
        'LV Bus Voltages',
        BATT_MON_ERROR_HELPER,
        BATT_MON_WARNING_HELPER,
        BATT_ANALOG_VOLTAGE_HELPER,
        'analog',
        ['LvA', 'LvB'],
        None)

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    monitors, any_value, error_messages, error, warning = self._GetMonitors(
        messages)

    return _ShowLvBusSummary(monitors, any_value, error_messages,
                             error, warning)


class TetherDownLvBusVoltageIndicator(indicator.BaseAttributeIndicator):
  """Show Lv bus voltage summary using the TetherDown message."""

  def __init__(self, name='LV Bus Voltages'):
    super(TetherDownLvBusVoltageIndicator, self).__init__(
        [('filtered', 'merge_tether_down', 'tether_down.batt_a'),
         ('filtered', 'merge_tether_down', 'tether_down.batt_b'),
         ('filtered', 'merge_tether_down', 'valid'),
         ('filtered', 'merge_tether_down', 'timestamp_sec')], name)

  def _IsValidInput(self, *args):
    return args[2]

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, batt_a, batt_b, valid, timestamp_sec):
    any_value = True
    error_messages = []
    valid_a = (
        batt_a.no_update_count <= common.MAX_NO_UPDATE_COUNT_BATT_STATUS)
    valid_b = (
        batt_b.no_update_count <= common.MAX_NO_UPDATE_COUNT_BATT_STATUS)
    error = (valid_a and batt_a.error) or (valid_b and batt_b.error)
    warning = (valid_a and batt_a.warning) or (valid_b and batt_b.warning)
    monitors = {'LvA': {}, 'LvB': {}}
    if valid_a:
      monitors['LvA']['BattA'] = batt_a.lv_a
      monitors['LvB']['BattA'] = batt_a.lv_b
    if valid_b:
      monitors['LvA']['BattB'] = batt_b.lv_a
      monitors['LvB']['BattB'] = batt_b.lv_b
    return _ShowLvBusSummary(monitors, any_value, error_messages,
                             error, warning)
