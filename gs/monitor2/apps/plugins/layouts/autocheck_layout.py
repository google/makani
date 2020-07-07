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

"""Layout for preflight autochecks."""
import collections
import json
import os

import jsmin
from makani.analysis.checks import base_check
from makani.analysis.checks import check_range
from makani.analysis.checks import gradebook
from makani.analysis.checks import gradebook_base_check
from makani.analysis.checks.collection import avionics_checks
from makani.analysis.checks.collection import fc_checks
from makani.analysis.checks.collection import gps_checks
from makani.analysis.checks.collection import motor_checks
from makani.analysis.checks.collection import servo_checks
from makani.avionics.common import gps_receiver
from makani.avionics.network import aio_labels
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.layout import widgets
from makani.gs.monitor2.apps.plugins.layouts import gps_util
from makani.gs.monitor2.project import settings
from makani.lib.python import c_helpers

_MONITOR_PARAMS = monitor_params.GetMonitorParams().contents
_SERVO_LABELS_HELPER = c_helpers.EnumHelper('ServoLabel', aio_labels,
                                            prefix='kServo')
_FC_LABELS_HELPER = c_helpers.EnumHelper('FcLabel', aio_labels,
                                         prefix='kFlightComputer')
# Path to the grade book for auto-checking.
_AUTOCHECK_GRADEBOOK = os.path.join(settings.MONITOR_PATH,
                                    'apps/plugins/layouts/gradebook.json')


class AutocheckLayout(base.BaseLayout):
  """Layout for auto checks."""

  # Name of the layout.
  _NAME = 'Preflight Autochecks'
  # The desired number of columns to place views.
  _DESIRED_VIEW_COLS = 6

  def Initialize(self):

    with open(_AUTOCHECK_GRADEBOOK, 'r') as fp:
      json_data = fp.read()

    try:
      # Use jsmin to allow comments in the gradebook JSON file.
      json_data = json.loads(jsmin.jsmin(json_data))
    except ValueError:
      raise ValueError('Invalid JSON format in "%s".' % _AUTOCHECK_GRADEBOOK)

    gradebook_to_load = gradebook.Gradebook(json_data, settings.NETWORK_YAML)

    # A dict of field indices to check, grouped by message type, source, and
    # the first-level attributes.
    # E.g., gradebook_fields[message_type][source][attribute] = [field_name]
    gradebook_fields = gradebook_to_load.GetFieldMap(
        '$message_type.$aio_node.')

    self._check_dict = collections.defaultdict(
        lambda: collections.defaultdict(  # pylint: disable=g-long-lambda
            lambda: collections.defaultdict(base_check.ListOfChecks)))
    for_log = False
    for message_name, sources in gradebook_fields.iteritems():
      for source, attributes in sources.iteritems():
        for _, fields in attributes.iteritems():
          for field in fields:
            # Get the referencing ranges to check from the gradebook.
            segments = field.split('.')
            criteria = gradebook_to_load.GetCriteria(segments)
            sub_field_name = '.'.join(segments[2:])
            field_checks = gradebook_base_check.GradebookChecks()
            field_checks.InitializeByField(
                for_log, message_name, source, sub_field_name, criteria, False)
            attribute_name = criteria.name if criteria.name else segments[-1]
            self._check_dict[message_name][source][attribute_name].Concatenate(
                field_checks)

    # Add checks for flight computers.
    default_temperature_limits = _MONITOR_PARAMS.thermal.aiomon_default
    message_name = 'FlightComputerSensor'
    for short_name in _FC_LABELS_HELPER.ShortNames():
      fc_name = 'Fc' + short_name
      fc_check_dict = self._check_dict[message_name][fc_name]
      fc_check_dict['FcMon Analog Voltages [V]'].Append(
          fc_checks.FcMonAnalogChecker(for_log, short_name))
      fc_check_dict['AioMon Analog Voltages [V]'].Append(
          avionics_checks.AioMonAnalogChecker(for_log, message_name, fc_name))
      fc_check_dict['AioMon Board Temperature [C]'].Append(
          avionics_checks.AioMonTemperatureChecker(
              for_log, message_name, fc_name,
              normal_ranges=check_range.Interval(
                  [default_temperature_limits.low,
                   default_temperature_limits.high]),
              warning_ranges=check_range.Interval(
                  [default_temperature_limits.very_low,
                   default_temperature_limits.very_high])))
      fc_check_dict['Bus Voltages [V]'].Append(
          avionics_checks.AioMonBusVoltageChecker(
              for_log, message_name, fc_name))
      fc_check_dict['Bus Current [A]'].Append(
          avionics_checks.AioMonBusCurrentChecker(
              for_log, message_name, fc_name))

    # Add checks for servos.
    message_name = 'ServoStatus'
    for short_name in _SERVO_LABELS_HELPER.ShortNames():
      servo_name = 'Servo' + short_name
      servo_check_dict = self._check_dict[message_name][servo_name]
      servo_check_dict['ServoMon Analog Voltages [V]'].Append(
          servo_checks.ServoMonAnalogChecker(for_log, short_name))
      servo_check_dict['ServoMon Temperature [C]'].Append(
          servo_checks.ServoMonTemperatureChecker(
              for_log, short_name,
              normal_ranges=check_range.Interval([0, 65]),
              warning_ranges=check_range.Interval([0, 80])))
      servo_check_dict['AioMon Analog Voltages [V]'].Append(
          avionics_checks.AioMonAnalogChecker(
              for_log, message_name, servo_name))
      servo_check_dict['AioMon Board Temperature [C]'].Append(
          avionics_checks.AioMonTemperatureChecker(
              for_log, message_name, servo_name,
              normal_ranges=check_range.Interval([0, 65]),
              warning_ranges=check_range.Interval([0, 80])))
      servo_check_dict['Bus Voltages [V]'].Append(
          avionics_checks.AioMonBusVoltageChecker(for_log, message_name,
                                                  servo_name))
      servo_check_dict['Bus Current [A]'].Append(
          avionics_checks.AioMonBusCurrentChecker(for_log, message_name,
                                                  servo_name))

    # Add checks for GPSes.
    gps_check_dict = self._check_dict['GpsSignalStatus']['Gps Receivers']
    gps_check_dict['Base NovAtel'].Append(
        gps_checks.NovAtelCn0Checker(for_log, 'GpsBaseStation',
                                     'GPS Base'))

    for gps_type, fc_name in gps_util.GpsSelector():
      if gps_type == gps_receiver.GpsReceiverType.NOV_ATEL.value:
        gps_check_dict[fc_name + ' NovAtel'].Append(
            gps_checks.NovAtelCn0Checker(for_log, fc_name, fc_name))
      elif gps_type == gps_receiver.GpsReceiverType.SEPTENTRIO.value:
        gps_check_dict[fc_name + ' Septentrio'].Append(
            gps_checks.SeptentrioCn0Checker(for_log, fc_name, fc_name))
      else:
        raise ValueError('Invalid GPS type: %d.' % gps_type)

    # Add checks for stacked motors.
    stacking_check_dict = self._check_dict['MotorStatus']['Stacking']
    stacking_check_dict['Pbi_Sti'].Append(
        motor_checks.MotorStackPairVoltageDiff(
            for_log, ['MotorPbi', 'MotorSti']))
    stacking_check_dict['Pbo_Sto'].Append(
        motor_checks.MotorStackPairVoltageDiff(
            for_log, ['MotorPbo', 'MotorSto']))
    stacking_check_dict['Pti_Sbi'].Append(
        motor_checks.MotorStackPairVoltageDiff(
            for_log, ['MotorPti', 'MotorSbi']))
    stacking_check_dict['Pto_Sbo'].Append(
        motor_checks.MotorStackPairVoltageDiff(
            for_log, ['MotorPto', 'MotorSbo']))

  def Filter(self, messages):
    """Select and compute data to show.

    Args:
      messages: A StructTree object, which is a nested dict that can be
          indexed using a string. Indexing returns None if field does not exist.

    Returns:
      A PlotData object storing the generated data.
    """
    data = self._GetPlotData()
    for message_name, section in self._check_dict.iteritems():
      for source, attributes in section.iteritems():
        for attribute, attribute_checks in attributes.iteritems():
          attribute_data = []
          attribute_stoplight = stoplights.STOPLIGHT_ANY
          is_any_field_available = False

          for check_item in attribute_checks.List():
            # Note a field can be a dict or list, each value will have to be
            # checked. So there may be multiple checks. The format is:
            # check_results = [{
            #     'name': ...,
            #     'value': {'stoplight': ..., 'value': ...},
            #     'std': {'stoplight': ..., 'value': ...}
            # }]
            check_item.Check(*check_item.Populate(messages))
            check_results = check_item.GetResults()
            if not check_results:
              continue

            for check in check_results:
              # Indicate the worst condition across all checks in this field.
              field_stoplight = check['stoplight']
              if field_stoplight is not None:
                attribute_stoplight = stoplights.MostSevereStoplight(
                    attribute_stoplight, field_stoplight)
                is_any_field_available = True
              else:
                field_stoplight = stoplights.STOPLIGHT_UNAVAILABLE
              attribute_data.append({
                  'name': None,
                  'fields': {check['name']: check},
                  'stoplight': field_stoplight
              })
              # Remove 'name' field because it is never used again.
              # This saves some traffic from the server to the client.
              del check['name']
          if not is_any_field_available:
            attribute_stoplight = stoplights.STOPLIGHT_UNAVAILABLE

          data[self._GetDataName(message_name, source, attribute)] = {
              'content': attribute_data,
              'stoplight': attribute_stoplight,
          }
    # TODO: Add voltage checks for servo, core switch,
    # and flight computers.

    # Data for additional indicators not defined in the gradebook.
    data.loadcell_aio_update_stoplight = stoplights.SetByAioUpdate(
        messages, 'Loadcell', None, stoplights.STOPLIGHT_NORMAL,
        stoplights.STOPLIGHT_ERROR, stoplights.STOPLIGHT_UNAVAILABLE)

    loadcells = ['StarboardA', 'PortA']
    for loadcell in loadcells:
      strains = []
      strains.append(messages['Loadcell.Loadcell' + loadcell
                              + '.loadcell_data.strain[0].value'])
      strains.append(messages['Loadcell.Loadcell' + loadcell
                              + '.loadcell_data.strain[1].value'])
      data['loadcell_strains_' + loadcell] = strains
      data['loadcell_timestamp_' + loadcell] = messages[
          'Loadcell.Loadcell' + loadcell + '.capture_info.timestamp']
    return data

  def Plot(self, data):
    """Defines a monitor layout.

    Args:
      data: A PlotData object, with fields defined in self.Filter.

    Returns:
      JSON representation of the AutoCheck layout.
    """
    views = collections.defaultdict(list)
    # Add autochecks defined by the gradebook.
    for message_name in sorted(self._check_dict):
      for source, attributes in self._check_dict[message_name].iteritems():
        widget_list = []
        for attr in sorted(attributes):
          widget_list.append(widgets.AttributesWidget(
              attr, data[self._GetDataName(message_name, source, attr)], 2))
        # Views are named after sources.
        views[source] += widget_list

    # Sort the views by their names.
    view_list = [(k, views[k], {}) for k in sorted(views)]

    # Add load cell charts that have a wider span.
    view_list.append((
        'Loadcell.StarboardA',
        [widgets.ListTrailsWidget('Force [N]',
                                  ['pin.x', 'pin.yz'],
                                  data.loadcell_timestamp_StarboardA,
                                  data.loadcell_strains_StarboardA,
                                  data.loadcell_aio_update_stoplight)],
        {'cols': 2},
    ))
    view_list.append((
        'Loadcell.PortA',
        [widgets.ListTrailsWidget('Force [N]',
                                  ['pin.x', 'pin.yz'],
                                  data.loadcell_timestamp_PortA,
                                  data.loadcell_strains_PortA,
                                  data.loadcell_aio_update_stoplight)],
        {'cols': 2},
    ))

    # Render the layout.
    return base.AssembleLayout(view_list, self._DESIRED_VIEW_COLS,
                               self._ORDER_HORIZONTALLY)

  def _GetDataName(self, message_name, source, attribute):
    """Get PlotData key of an attribute."""
    return '_'.join([message_name, source, attribute])
