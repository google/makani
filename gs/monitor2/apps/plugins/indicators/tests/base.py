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

import json
import unittest

from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.lib.python import struct_tree


class IndicatorTest(unittest.TestCase):

  def setUp(self):
    self._stoplights = {
        'STOPLIGHT_ANY': stoplights.STOPLIGHT_ANY,
        'STOPLIGHT_NORMAL': stoplights.STOPLIGHT_NORMAL,
        'STOPLIGHT_WARNING': stoplights.STOPLIGHT_WARNING,
        'STOPLIGHT_ERROR': stoplights.STOPLIGHT_ERROR,
        'STOPLIGHT_UNAVAILABLE': stoplights.STOPLIGHT_UNAVAILABLE,
    }

    self._modes = {
        'FULL_COMMS_MODE': common.FULL_COMMS_MODE,
        'SPARSE_COMMS_MODE': common.SPARSE_COMMS_MODE,
    }

  def _CreateIndicator(self, data):
    raise NotImplementedError

  def _Run(self, monitor_params, messages, expected):
    messages_obj = struct_tree.StructTree(messages, readonly=False)
    indicator = self._CreateIndicator(monitor_params)
    plot_data = indicator.Filter(messages_obj)
    self.assertEqual(plot_data, tuple(expected))

  def _RunFromFile(self, indicator, config_file):
    """Run an indicator through test cases defined in a config file."""
    with open(config_file) as fp:
      content = fp.read()

    for name, value in self._stoplights:
      content = content.replace('"%s"' % name, value)

    for name, value in self._modes:
      content = content.replace('"%s"' % name, value)

    config = json.loads(content)
    for case in config:
      self._Run(indicator, case['messages'], case['expected'])
