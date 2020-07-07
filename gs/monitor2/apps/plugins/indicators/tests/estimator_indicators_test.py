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

"""Tests for estimator indicators."""

import unittest

from makani.avionics.linux.swig import aio_util
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import estimator
from makani.gs.monitor2.apps.receiver import test_util
import mock


class TestPortedIndicatorTest(unittest.TestCase):

  @classmethod
  def setUp(cls):
    aio_util.InitFilters()

  def _SynthesizeControlTelemetry(self, sequence=0):
    return test_util.SynthesizeMessages(['ControlTelemetry'], sequence)

  def testGyroDiffIndicator_Trivial(self):
    """Trivial Test to check that GyroDiffIndicator _Filter can be run."""
    mode = common.FULL_COMMS_MODE
    indicator = estimator.EstimatorGyroDiffIndicator(mode)
    messages = self._SynthesizeControlTelemetry()

    with mock.patch('makani.control.common.IsControlSystemRunning',
                    lambda x: True):
      _, result_stoplight = indicator.Filter(messages)

    self.assertEqual(stoplights.STOPLIGHT_NORMAL, result_stoplight)

  def testGyroBiasIndicator_Trivial(self):
    """Trivial Test to check that GyroDiffIndicator _Filter can be run."""
    mode = common.FULL_COMMS_MODE
    indicator = estimator.EstimatorGyroBiasIndicator(mode)
    messages = self._SynthesizeControlTelemetry()
    ct = messages._seed['ControlTelemetry']['ControllerA']
    gyro_biases = ct.estimator.gyro_biases

    with mock.patch('makani.control.common.IsControlSystemRunning',
                    lambda x: True):
      _, result_stoplight = indicator.Filter(messages)
      self.assertEqual(stoplights.STOPLIGHT_NORMAL, result_stoplight)

      gyro_biases[0].x = 0.008
      _, result_stoplight = indicator.Filter(messages)
      self.assertEqual(stoplights.STOPLIGHT_WARNING, result_stoplight)

      gyro_biases[1].y = 0.011
      _, result_stoplight = indicator.Filter(messages)
      self.assertEqual(stoplights.STOPLIGHT_ERROR, result_stoplight)

if __name__ == '__main__':
  unittest.main()
