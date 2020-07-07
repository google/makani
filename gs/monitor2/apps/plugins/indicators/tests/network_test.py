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

"""Test for network indicators."""

import unittest

from makani.avionics.common import cvt
from makani.avionics.linux.swig import aio_helper
from makani.avionics.linux.swig import aio_util
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins.indicators import network

from makani.lib.python import struct_tree


class TestIndicators(unittest.TestCase):

  @classmethod
  def setUp(cls):
    aio_util.InitFilters()

  def _SynthesizeTetherTelemetry(self):
    data = {}
    filtered = aio_helper.GetFilteredData()
    filtered.merge_tether_down.valid = True
    comms_status = filtered.merge_tether_down.comms_status
    comms_status_valid = filtered.merge_tether_down.comms_status_valid
    data['filtered'] = filtered
    messages = struct_tree.StructTree(data, readonly=False)
    return comms_status, comms_status_valid, messages

  def testTetherCommsStatusPoFIndicator(self):
    def SetLinkUp(comms_status, comms_status_valid, source):
      comms_status_valid[source] = True
      comms_status[source].no_update_count = 0
      comms_status[source].links_up |= cvt.kTetherCommsLinkPof

    def ResetLinkUp(comms_status, comms_status_valid, source):
      comms_status_valid[source] = True
      comms_status[source].no_update_count = 0
      comms_status[source].links_up = 0

    comms_status, comms_status_valid, messages = (
        self._SynthesizeTetherTelemetry())
    for source in [cvt.kTetherDownSourceCsGsA, cvt.kTetherDownSourceCsA,
                   cvt.kTetherDownSourceCsB]:
      ResetLinkUp(comms_status, comms_status_valid, source)

    indicator = network.TetherCommsStatusPoFIndicator('PoF')

    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, '    Link A     Link B\n      Down       Down')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ANY)

    SetLinkUp(comms_status, comms_status_valid, cvt.kTetherDownSourceCsGsA)
    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, '    Link A     Link B\n        Up       Down')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ANY)

    SetLinkUp(comms_status, comms_status_valid, cvt.kTetherDownSourceCsA)
    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, '    Link A     Link B\n        Up       Down')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ANY)

    SetLinkUp(comms_status, comms_status_valid, cvt.kTetherDownSourceCsB)
    text, stoplight = indicator.Filter(messages)
    self.assertEqual(text, '    Link A     Link B\n        Up         Up')
    self.assertEqual(stoplight, stoplights.STOPLIGHT_ANY)


if __name__ == '__main__':
  unittest.main()
