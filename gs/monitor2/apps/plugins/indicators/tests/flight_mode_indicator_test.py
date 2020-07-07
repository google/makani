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

import unittest

from makani.avionics.linux.swig import aio_helper
from makani.control import control_types
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators.tests import base
from makani.gs.monitor2.apps.receiver import test_util
from makani.lib.python import c_helpers

_FLIGHT_MODE_HELPER = c_helpers.EnumHelper('FlightMode', control_types)


class FlightModeIndicatorTest(base.IndicatorTest):

  def _CreateIndicator(self, mode):
    return control.FlightModeIndicator(mode)

  def _SynthesizeMessages(
      self, full_source, init_state, tether_valid, flight_mode,
      no_update_count=0):
    messages = test_util.SynthesizeMessages(['ControlTelemetry'])
    data = messages.Data(convert_to_basic_types=False)
    filtered_data = aio_helper.GetFilteredData()
    data['filtered'] = filtered_data
    data['ControlTelemetry']['ControllerA'].init_state = init_state
    data['ControlTelemetry']['ControllerA'].flight_mode = flight_mode
    merge_tether_down = filtered_data.merge_tether_down
    merge_tether_down.tether_down.control_telemetry.flight_mode = flight_mode
    merge_tether_down.tether_down.control_telemetry.no_update_count = (
        no_update_count)
    merge_tether_down.valid = tether_valid
    merge_tether_down.timestamp_sec = 1234567
    return data

  def testEmpty(self):
    self._Run(
        common.FULL_COMMS_MODE, {}, ('--', stoplights.STOPLIGHT_UNAVAILABLE))

  def testFullModeInit(self):
    self._Run(
        common.FULL_COMMS_MODE,
        self._SynthesizeMessages('FcB', 0, False, 0),
        ('Init: FirstEntry', stoplights.STOPLIGHT_WARNING)
    )

  def testSparseModeInvalid(self):
    self._Run(
        common.SPARSE_COMMS_MODE,
        self._SynthesizeMessages('FcB', 0, False, 0),
        ('--', stoplights.STOPLIGHT_UNAVAILABLE)
    )

    self._Run(
        common.SPARSE_COMMS_MODE,
        self._SynthesizeMessages(
            'FcB', 0, True, 0,
            common.MAX_NO_UPDATE_COUNT_CONTROL_TELEMETRY + 1),
        ('--', stoplights.STOPLIGHT_UNAVAILABLE)
    )

  def testValidFlightMode(self):
    flight_mode = 2
    init_state = 2
    for node in ['FcA', 'FcB']:
      for mode in [common.SPARSE_COMMS_MODE, common.FULL_COMMS_MODE]:
        self._Run(
            mode, self._SynthesizeMessages(node, init_state, True, flight_mode),
            (_FLIGHT_MODE_HELPER.ShortName(flight_mode),
             stoplights.STOPLIGHT_NORMAL)
        )

  def testInvalidFlightMode(self):
    flight_mode = control_types.kNumFlightModes
    init_state = 2
    for mode in [common.SPARSE_COMMS_MODE, common.FULL_COMMS_MODE]:
      self._Run(
          mode, self._SynthesizeMessages('FcB', init_state, True, flight_mode),
          ('Invalid', stoplights.STOPLIGHT_ERROR)
      )


if __name__ == '__main__':
  unittest.main()
