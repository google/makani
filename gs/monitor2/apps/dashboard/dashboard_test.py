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

"""Test for web monitor layouts."""

import json
import unittest

from makani.gs.monitor2.apps.layout import loader
from makani.gs.monitor2.apps.receiver import test_util
import mock


def _FakedIsControlSystemRunning(unused_init_state):
  return True


class TestLayouts(unittest.TestCase):

  def setUp(self):
    self._layout_names = loader.LayoutLoader().ModuleNames()

  def _RunLayouts(self):
    num_messages = 10
    messages = test_util.SynthesizeMessages()
    for layout_name in self._layout_names:
      layout = loader.LayoutLoader().GetLayoutByModuleName(layout_name)
      layout.Initialize()
      for _ in range(num_messages):
        json.dumps(layout.Filter(messages).Json())
        assert not layout.ErrorReport()

  def testLayouts(self):
    self._RunLayouts()
    with mock.patch('makani.control.common.IsControlSystemRunning',
                    _FakedIsControlSystemRunning):
      self._RunLayouts()


if __name__ == '__main__':
  unittest.main()
