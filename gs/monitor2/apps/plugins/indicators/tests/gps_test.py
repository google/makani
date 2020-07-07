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

"""Test for web monitor indicators."""

import unittest

from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.plugins.indicators import gps
from makani.gs.monitor2.apps.receiver import test_util


class TestIndicators(unittest.TestCase):

  def setUp(self):
    self._messages = test_util.SynthesizeMessages(
        ['NovAtelObservations', 'SeptentrioObservations'], 0)
    # Convert data from ctypes object to Python dicts.
    data = self._messages.Data(convert_to_basic_types=False)
    self._novatel_obs = data['NovAtelObservations']['FcB']
    self._septentrio_obs = data['SeptentrioObservations']['FcA']
    # Values to qualify a good condition.
    for i in range(32):
      self._novatel_obs.range.cn0[i] = 51
      self._novatel_obs.range.status_bits[i] = 0
      self._septentrio_obs.meas_epoch.cn0[i] = 121
      self._septentrio_obs.meas_epoch.type[i] = 0

    self._novatel_obs.range.num_obs = 5
    self._septentrio_obs.meas_epoch.num_obs = 5
    self._novatel_cn0_indicator = gps.NovAtelObsCn0Indicator('FcB')
    self._septentrio_cn0_indicator = gps.SeptentrioObsCn0Indicator('FcA')

  def testGoodCn0(self):
    for indicator in [self._novatel_cn0_indicator,
                      self._septentrio_cn0_indicator]:
      self.assertEqual(indicator.Filter(self._messages)[1],
                       stoplights.STOPLIGHT_NORMAL)

  def testLowCn0(self):
    for i in range(32):
      self._novatel_obs.range.cn0[i] = 39
      self._septentrio_obs.meas_epoch.cn0[i] = 119
    for indicator in [self._novatel_cn0_indicator,
                      self._septentrio_cn0_indicator]:
      # Max Cn0 is too low.
      self.assertEqual(indicator.Filter(self._messages)[1],
                       stoplights.STOPLIGHT_WARNING)

  def testTooFewObs(self):
    self._novatel_obs.range.num_obs = 4
    self._septentrio_obs.meas_epoch.num_obs = 4
    for indicator in [self._novatel_cn0_indicator,
                      self._septentrio_cn0_indicator]:
      # Too few observatios.
      self.assertEqual(indicator.Filter(self._messages)[1],
                       stoplights.STOPLIGHT_WARNING)

  def testTooFewValidStatus(self):
    # For the first `num_obs` observations, only the last is valid.
    for i in range(4):
      self._novatel_obs.range.status_bits[i] = 0x03E70000
      self._septentrio_obs.meas_epoch.type[i] = 0x1F
    for i in range(4, 32):
      self._novatel_obs.range.status_bits[i] = 0
      self._septentrio_obs.meas_epoch.type[i] = 0

    for indicator in [self._novatel_cn0_indicator,
                      self._septentrio_cn0_indicator]:
      # Too few valid status.
      self.assertEqual(indicator.Filter(self._messages)[1],
                       stoplights.STOPLIGHT_WARNING)


if __name__ == '__main__':
  unittest.main()
