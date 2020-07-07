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

from makani.analysis.checks import tests
from makani.analysis.checks.collection import aiomon_checks
from makani.analysis.checks.collection import fc_checks
from makani.analysis.checks.collection import imu_checks
from makani.analysis.checks.collection import motor_checks
from makani.analysis.checks.collection import network_checks
from makani.analysis.checks.collection import q7_checks
from makani.analysis.checks.collection import servo_checks
from makani.gs.monitor2.apps.receiver import test_util


class TestListsOfChecks(unittest.TestCase):

  def testAioMonChecks(self):
    """Make sure that no checks crash when running against AIO messages."""
    self._DryrunAllListsOfChecks()
    with tests.PatchCheckers():
      self._DryrunAllListsOfChecks()

  def _DryrunAllListsOfChecks(self):
    self._DryrunChecks(network_checks.FrameUpdateRateChecks(for_log=False))
    self._DryrunChecks(aiomon_checks.AioMonChecks(for_log=False))
    self._DryrunChecks(fc_checks.FcChecks(for_log=False))
    self._DryrunChecks(imu_checks.ImuChecks(for_log=False))
    self._DryrunChecks(motor_checks.MotorChecks(for_log=False))
    self._DryrunChecks(q7_checks.Q7Checks(for_log=False))
    self._DryrunChecks(servo_checks.ServoChecks(for_log=False))
    self._DryrunChecks(network_checks.AggregatedLinkChecks(for_log=False))
    self._DryrunChecks(network_checks.TetherDownChecks(for_log=False))
    self._DryrunChecks(network_checks.TetherUpChecks(for_log=False))

  def _DryrunChecks(self, checks):
    num_messages = 10
    for n in range(num_messages):
      messages = test_util.SynthesizeMessages(sequence=n)
      for check in checks.List():
        # Check every element.
        args = check.Populate(messages)
        check.Check(*args)
    checks.GetSpecs()


if __name__ == '__main__':
  unittest.main()
