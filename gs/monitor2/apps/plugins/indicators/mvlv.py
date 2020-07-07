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

from makani.avionics.firmware.monitors import mvlv_types
from makani.control import control_types
from makani.control import system_params
from makani.control import system_types
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.gs.monitor2.apps.plugins.indicators import batt
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree

FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)

MVLV_ANALOG_VOLTAGE_HELPER = c_helpers.EnumHelper('MvlvAnalogVoltage',
                                                  mvlv_types)

MVLV_MCP342X_MONITOR_HELPER = c_helpers.EnumHelper('MvlvMcp342xMonitor',
                                                   mvlv_types)

MVLV_MON_WARNING_HELPER = c_helpers.EnumHelper('MvlvMonitorWarning',
                                               mvlv_types)

MVLV_MON_ERROR_HELPER = c_helpers.EnumHelper('MvlvMonitorError', mvlv_types)

MVLV_MON_STATUS_HELPER = c_helpers.EnumHelper('MvlvMonitorStatus', mvlv_types)

_SYSTEM_PARAMS = system_params.GetSystemParams().contents


class MvlvStatus(object):
  """The class to perform various checks for MvLv status."""

  def __init__(self, mvlv_status_message):
    self._mvlv = mvlv_status_message

  def CheckStatus(self):
    """Check the status of MvLv."""
    status = self._mvlv.mvlv_mon.flags.status
    is_connected = status & MVLV_MON_STATUS_HELPER.Value('Connected')
    is_enabled = status & MVLV_MON_STATUS_HELPER.Value('Enabled')
    is_retry = status & MVLV_MON_STATUS_HELPER.Value('FaultRetry')
    warning_msg = []
    error_msg = []
    warning = False
    error = False
    msg = []
    if not is_enabled:
      msg.append('Disabled')
    if not is_connected:
      msg.append('Disconnected')
    if is_retry:
      msg.append('Retry')

    if msg:
      if not is_enabled:
        error = True
        error_msg = msg
      else:
        warning = True
        warning_msg = msg
    return (is_connected, is_enabled, is_retry,
            warning, error, warning_msg, error_msg)

  def CheckAnalogStatus(self):
    """Check analog sensor statuses."""
    voltages, warning, error, _, _ = avionics.GetMonitorFields(
        self._mvlv, 'mvlv_mon', 'analog',
        MVLV_ANALOG_VOLTAGE_HELPER,
        MVLV_MON_WARNING_HELPER, MVLV_MON_ERROR_HELPER,
        ['12v', '3v3', '5v', 'IHall', 'VExt', 'VLv', 'VLvOr', 'VLvPri',
         'VLvSec'], 'Mvlv', None)

    warning_msg = []
    error_msg = []
    if error:
      error_msg.append('Analog monitor error')
    elif warning:
      warning_msg.append('Analog monitor warning')

    vlv_pri = voltages['VLvPri']
    if 70.0 < vlv_pri < 75.0:
      pass
    elif 65.0 < vlv_pri < 80.0:
      warning = True
      warning_msg.append('MvLv VLvPri: %d' % int(round(vlv_pri)))
    else:
      # Errors should be already captured by avionics monitors in firmware.
      pass

    return voltages, warning, error, warning_msg, error_msg

  def CheckMcp342xStatus(self):
    """Check temperature sensor statuses."""
    valid_errors = []
    if _SYSTEM_PARAMS.wing_serial == system_types.kWingSerial01:
      # MVLV temperature sensor circuit picks up commom mode noise in
      # Gin configuration (SN01) which creates a big error (50C step).
      valid_errors = ['FilterCap', 'OutputSwitch', 'SyncRectMosfetSide',
                      'SyncRectPcb']
    else:
      valid_errors = ['EnclosureAir', 'FilterCap', 'HvResonantCap',
                      'Igbt', 'OutputSwitch', 'SyncRectMosfetSide',
                      'SyncRectMosfetTop', 'SyncRectPcb']
    temperatures, warning, error, _, error_reading = avionics.GetMonitorFields(
        self._mvlv, 'mvlv_mon', 'mcp342x',
        MVLV_MCP342X_MONITOR_HELPER,
        MVLV_MON_WARNING_HELPER, MVLV_MON_ERROR_HELPER,
        valid_errors, 'Mvlv', 'TempReadErrors')

    warning_msg = []
    error_msg = []
    if error_reading:
      # TODO: Fix Mvlv i2c problem and reinstate error checking.
      # error_msg.append('Mcp342x Reading error')
      warning = False
    elif error:
      error_msg.append('Temperature error')
    elif warning:
      warning_msg.append('Temperature warning')

    return temperatures, warning, error, warning_msg, error_msg

  def CheckAll(self):
    """Check all statuses."""
    final_warning = False
    final_error = False
    final_warning_msg = []
    final_error_msg = []

    _, _, _, warning, error, warning_msg, error_msg = self.CheckStatus()
    final_warning |= warning
    final_warning_msg += warning_msg
    final_error |= error
    final_error_msg += error_msg
    if final_error:
      return final_warning, final_error, final_warning_msg, final_error_msg

    _, warning, error, warning_msg, error_msg = self.CheckAnalogStatus()
    final_warning |= warning
    final_warning_msg += warning_msg
    final_error |= error
    final_error_msg += error_msg

    _, warning, error, warning_msg, error_msg = (
        self.CheckMcp342xStatus())
    final_warning |= warning
    final_warning_msg += warning_msg
    final_error |= error
    final_error_msg += error_msg

    return final_warning, final_error, final_warning_msg, final_error_msg


class MvLvSummaryIndicator(indicator.BaseAttributeIndicator):
  """Indicator to summarize MvLv statuses."""

  def __init__(self, num_lines, enable_error, name='Mvlv'):
    super(MvLvSummaryIndicator, self).__init__(
        [('MvlvStatus', None)], name)
    self._num_lines = num_lines
    # True if MvLv error will appear as red, yellow otherwise.
    self._enable_error = enable_error

  def _Filter(self, mvlv_status):
    if not struct_tree.IsValidElement(mvlv_status):
      return (common.FillLines(['Unavailable'], self._num_lines),
              stoplights.STOPLIGHT_ERROR
              if self._enable_error else stoplights.STOPLIGHT_WARNING)

    mvlv = MvlvStatus(mvlv_status)
    warning, error, warning_msg, error_msg = mvlv.CheckAll()

    if error:
      return (common.FillLines(error_msg, self._num_lines),
              stoplights.STOPLIGHT_ERROR
              if self._enable_error else stoplights.STOPLIGHT_WARNING)
    elif warning:
      return (common.FillLines(warning_msg, self._num_lines),
              stoplights.STOPLIGHT_WARNING)
    else:
      return (common.FillLines(['Normal'], self._num_lines),
              stoplights.STOPLIGHT_NORMAL)


class LvSummaryIndicator(indicator.BaseAttributeIndicator):
  """Indicator to summarize MvLv and batteries statuses."""

  def __init__(self, num_lines, name='Lv'):
    super(LvSummaryIndicator, self).__init__(
        [('MvlvStatus', None),
         ('BatteryStatus', 'BattA'),
         ('BatteryStatus', 'BattB'),
         ('SlowStatus', 'BattA'),
         ('SlowStatus', 'BattB'),
         ('ControlTelemetry', None),
        ], name)
    self._num_lines = num_lines
    # Consider batteries to be powering bus if >1A output for >1s.
    self._discharge_watchers = {batt: common.DurationWatcher([1.0, 1.0e6], 1.0)
                                for batt in ['BattA', 'BattB']}

  def _Filter(self, mvlv_status, batt_a, batt_b, slow_status_a, slow_status_b,
              control_telemetry):
    # TODO: Enable TetherDown message as well.
    mvlv_error = False
    mvlv_warning = False
    batt_error = False
    batt_warning = False
    mvlv_present = False
    flight_mode_perched = False
    batts_discharge = False

    error_msg = []
    warning_msg = []
    status_msg = []

    # TODO: Also allow indicator to use TetherDown.
    if struct_tree.IsValidElement(control_telemetry):
      if control_telemetry.flight_mode == FLIGHT_MODE_HELPER.Value('Perched'):
        flight_mode_perched = True

    if not struct_tree.IsValidElement(mvlv_status):
      mvlv_error = True
      error_msg.append('MvLv unavailable')
    else:
      mvlv_present = True
      mvlv_warning, mvlv_error, mvlv_warning_msg, mvlv_error_msg = MvlvStatus(
          mvlv_status).CheckAll()
      warning_msg += ['MvLv ' + m for m in mvlv_warning_msg]
      error_msg += ['MvLv ' + m for m in mvlv_error_msg]

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
      batt_error = True
      error_msg.append('Batteries unavailable')
    else:
      batt_status = batt.BatteryStatus(batts, aio_nodes)
      (batt_warning, batt_error,
       batt_warning_msg, batt_error_msg, batt_status_msg,
       batts_discharge) = batt_status.CheckAll(slow_statuses,
                                               self._discharge_watchers)
      warning_msg += batt_warning_msg
      error_msg += batt_error_msg
      status_msg += batt_status_msg

    # If running off batteries, add a message either as warning or just
    # status based on wing configuration and flight mode.
    discharge_msg = 'LV bus running off batts.'
    if batts_discharge:
      if mvlv_present or flight_mode_perched:
        warning_msg.append(discharge_msg)
        batt_warning = True
      else:
        # No MVLV (batteries only), so discharge during flight is expected.
        status_msg.append(discharge_msg)

    # Show red as long as battery has error, for the sake of glide landing.
    if batt_error:
      return (common.FillLines(error_msg + status_msg, self._num_lines),
              stoplights.STOPLIGHT_ERROR)
    elif not (mvlv_error or batt_warning or mvlv_warning):
      return (common.FillLines(['Normal'] + status_msg, self._num_lines),
              stoplights.STOPLIGHT_NORMAL)
    else:
      messages = error_msg + warning_msg + status_msg
      return (common.FillLines(messages, self._num_lines),
              stoplights.STOPLIGHT_WARNING)


class MvlvStatusIndicator(indicator.BaseIndicator):
  """Indicator for MVLV's output status."""

  def __init__(self, name):
    super(MvlvStatusIndicator, self).__init__(name)

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    status = messages['MvlvStatus.Mvlv.mvlv_mon.flags.status']
    if status is None:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
    else:
      is_connected = status & MVLV_MON_STATUS_HELPER.Value('Connected')
      is_enabled = status & MVLV_MON_STATUS_HELPER.Value('Enabled')
      is_retry = status & MVLV_MON_STATUS_HELPER.Value('FaultRetry')
      if is_enabled:
        status_msg = 'Enabled'
      else:
        status_msg = 'Disabled'
      if is_connected:
        status_msg += ' Connected'
      else:
        status_msg += ' Disconnected'
      if is_retry:
        status_msg += ' Retry'
      if is_connected:
        return status_msg, stoplights.STOPLIGHT_NORMAL
      else:
        return status_msg, stoplights.STOPLIGHT_WARNING


class MvlvMcp342xIndicator(avionics.BaseMonitorIndicator):

  def __init__(self):
    valid_errors = []
    if _SYSTEM_PARAMS.wing_serial == system_types.kWingSerial01:
      # MVLV temperature sensor circuit picks up commom mode noise in
      # Gin configuration (SN01) which creates a big error (50C step).
      valid_errors = ['FilterCap', 'OutputSwitch', 'SyncRectMosfetSide',
                      'SyncRectPcb']
    else:
      valid_errors = ['EnclosureAir', 'FilterCap', 'HvResonantCap',
                      'Igbt', 'OutputSwitch', 'SyncRectMosfetSide',
                      'SyncRectMosfetTop', 'SyncRectPcb']
    super(MvlvMcp342xIndicator, self).__init__(
        'MvlvStatus', 'mvlv_mon', ['Mvlv'], 'Temperature monitor',
        MVLV_MON_ERROR_HELPER, MVLV_MON_WARNING_HELPER,
        MVLV_MCP342X_MONITOR_HELPER, 'mcp342x',
        valid_errors, 'TempReadErrors')


class MvlvAnalogIndicator(avionics.BaseMonitorIndicator):

  def __init__(self):
    super(MvlvAnalogIndicator, self).__init__(
        'MvlvStatus', 'mvlv_mon', ['Mvlv'], 'Voltage and current monitors',
        MVLV_MON_ERROR_HELPER, MVLV_MON_WARNING_HELPER,
        MVLV_ANALOG_VOLTAGE_HELPER, 'analog',
        ['12v', '3v3', '5v', 'IHall', 'VExt', 'VLv', 'VLvOr', 'VLvPri',
         'VLvSec'], None)


class MvlvErrorWarningIndicator(indicator.BaseIndicator):
  """Indicator explaining Mvlv's warnings or errors."""

  def __init__(self, name, chars_per_line=15):
    super(MvlvErrorWarningIndicator, self).__init__(name)
    self._chars_per_line = chars_per_line
    self._errors_to_exclude = []
    if _SYSTEM_PARAMS.wing_serial == system_types.kWingSerial01:
      # Exclude 4 unreliable thermistor channels in Gin configuration (SN01)
      self._errors_to_exclude = ['SyncRectMosfetTop', 'HvResonantCap',
                                 'Igbt', 'EnclosureAir']

  def Filter(self, messages):
    if not messages:
      return ('--', stoplights.STOPLIGHT_UNAVAILABLE)

    reachable = True
    any_warnings = False
    any_errors = False
    alerts_present = []
    statuses = []

    warnings = messages['MvlvStatus.Mvlv.mvlv_mon.flags.warning']
    errors = messages['MvlvStatus.Mvlv.mvlv_mon.flags.error']
    if warnings is None and errors is None:
      statuses.append('--')
      reachable = False
    else:
      for warning in MVLV_MON_WARNING_HELPER.ShortNames():
        if warnings & MVLV_MON_WARNING_HELPER.Value(warning):
          alerts_present.append(warning)
          any_warnings = True
      for error in MVLV_MON_ERROR_HELPER.ShortNames():
        if self._errors_to_exclude and error in self._errors_to_exclude:
          continue
        if errors & MVLV_MON_ERROR_HELPER.Value(error):
          alerts_present.append(error)
          any_errors = True

      if alerts_present:
        statuses = common.TimeCycle(alerts_present, self._chars_per_line)
      else:
        statuses.append('Normal')

    if not reachable:
      return statuses, stoplights.STOPLIGHT_UNAVAILABLE
    elif any_errors:
      return statuses, stoplights.STOPLIGHT_ERROR
    elif any_warnings:
      return statuses, stoplights.STOPLIGHT_WARNING
    else:
      return statuses, stoplights.STOPLIGHT_NORMAL


