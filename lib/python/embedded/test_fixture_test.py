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

from makani.lib.python.embedded import test_fixture
import mock


class TestRelayManager(unittest.TestCase):

  def setUp(self):

    relay_class_name = 'MockedRelay'

    self.target_relays = {1: {'RelayModuleId': 1, 'RelayPort': 5}}

    # Patch the relay_device module with a mock class 'MockedRelay'.
    self._relay_device_patcher = mock.patch(
        target='test_fixture.relay_device.{}'.format(relay_class_name),
        create=True)

    self._fixture = test_fixture.TestFixture(database=':memory:')

    # Insert RelayModule.
    relay_config = {
        'Device': '192.168.1.200',
        'Channels': 20,
        'Type': relay_class_name}
    relay_module_id = self._fixture._db.Insert('RelayModules', relay_config)

    # Insert Target.
    self._fixture._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeControllerA',
        'BoardName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': self.target_relays[1]['RelayModuleId'],
        'RelayPort': self.target_relays[1]['RelayPort'],
    })
    self._relay_manager = test_fixture.RelayManager(self._fixture._db)

    # Start the mock.
    self._relay_device_patcher.start()
    self._relay_manager.AddRelay(relay_module_id, relay_config)

  def testPower(self):
    target = 1
    relay = self.target_relays[target]['RelayModuleId']
    relay_port = self.target_relays[target]['RelayPort']
    self._relay_manager.PowerOn(target)
    self._relay_manager._relays[relay].PowerOn.assert_called_with(relay_port)
    self._relay_manager.PowerOff(target)
    self._relay_manager._relays[relay].PowerOff.assert_called_with(relay_port)

  def tearDown(self):
    # Stop the mock.
    self._relay_device_patcher.start()


class TestTestFixture(unittest.TestCase):

  def setUp(self):
    # Define test fixture hardware.
    self._fixture = test_fixture.TestFixture(database=':memory:')
    self._fixture._relay = mock.Mock(test_fixture.RelayManager)

    # Insert RelayModule.
    relay_port = 1
    relay_config = {
        'Device': '192.168.1.200',
        'Channels': 20,
        'Type': 'MockedRelay'}
    relay_module_id = self._fixture._db.Insert('RelayModules', relay_config)

    # Insert Targets.
    self._fixture._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeControllerA',
        'BoardName': 'fc',
        'HardwareOptions': 'imu novatel',
        'RelayModuleId': relay_module_id,
        'RelayPort': relay_port,
    })
    relay_port += 1
    self._fixture._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcB',
        'AioNodeQ7': 'kAioNodeControllerB',
        'BoardName': 'fc',
        'HardwareOptions': 'imu septentrio',
        'RelayModuleId': relay_module_id,
        'RelayPort': relay_port,
    })
    relay_port += 1
    self._fixture._db.InsertTarget({
        'AioNodeTms570': 'kAioNodeFcC',
        'AioNodeQ7': 'kAioNodeControllerC',
        'BoardName': 'fc',
        'HardwareOptions': 'imu hemisphere',
        'RelayModuleId': relay_module_id,
        'RelayPort': relay_port,
    })
    relay_port += 1
    motors = ['Pbi', 'Pbo', 'Pti', 'Pto', 'Sbi', 'Sbo', 'Sti', 'Sto']
    for motor in motors:
      self._fixture._db.InsertTarget({
          'AioNodeTms570': 'kAioNodeMotor' + motor,
          'BoardName': 'motor',
          'RelayModuleId': relay_module_id,
          'RelayPort': relay_port,
      })
      relay_port += 1
    servos = ['A1', 'A2', 'A4', 'A5', 'A7', 'A8', 'E1', 'E2', 'R1', 'R2']
    for servo in servos:
      self._fixture._db.InsertTarget({
          'AioNodeTms570': 'kAioNodeServo' + servo,
          'BoardName': 'aio',
          'CarrierName': 'servo',
          'RelayModuleId': relay_module_id,
          'RelayPort': relay_port,
      })
      relay_port += 1

    # Define tests.
    # Default test node requirement.
    test_default = {
        'BoardName': None,
        'CarrierName': None,
        'HardwareOptions': None,
        'RelayModuleId': None,
        'RelayPort': None,
    }

    # Test 1 requires one flight computer and all motors.
    self._test1 = [dict(test_default, BoardName='fc')]
    for _ in motors:
      self._test1.append(dict(test_default, BoardName='motor'))
    self._fixture._db.InsertTest(self._test1, 1)

    # Test 2 requires one flight computer and one motor. It cannot run until
    # test 1 completes.
    self._test2 = [dict(test_default, BoardName='fc'),
                   dict(test_default, BoardName='motor')]
    self._fixture._db.InsertTest(self._test2, 2)

    # Test 3 requires one flight computer. It can run because it does not
    # prevent test 2 from running.
    self._test3 = [dict(test_default, BoardName='fc')]
    self._fixture._db.InsertTest(self._test3, 3)

  def testGetCandidateTargets(self):
    # Returns a list of lists that contains candidate nodes for each node in
    # the specified test. These results are independent of the currently
    # running tests.

    # Test 1 should return:
    test1_candidates = [
        [1, 2, 3],                   # Candidates for flight computer node.
        [4, 5, 6, 7, 8, 9, 10, 11],  # Candidates for motor node.
        [4, 5, 6, 7, 8, 9, 10, 11],  # Candidates for motor node.
        [4, 5, 6, 7, 8, 9, 10, 11],  # Candidates for motor node.
        [4, 5, 6, 7, 8, 9, 10, 11],  # Candidates for motor node.
        [4, 5, 6, 7, 8, 9, 10, 11],  # Candidates for motor node.
        [4, 5, 6, 7, 8, 9, 10, 11],  # Candidates for motor node.
        [4, 5, 6, 7, 8, 9, 10, 11],  # Candidates for motor node.
        [4, 5, 6, 7, 8, 9, 10, 11]]  # Candidates for motor node.
    c = self._fixture.GetCandidateTargets(self._test1)
    self.assertEqual(c, test1_candidates)

    # Test 2 should return:
    test2_candidates = [
        [1, 2, 3],
        [4, 5, 6, 7, 8, 9, 10, 11]]
    c = self._fixture.GetCandidateTargets(self._test2)
    self.assertEqual(c, test2_candidates)

    # Test 3 should return:
    test3_candidates = [[1, 2, 3]]
    c = self._fixture.GetCandidateTargets(self._test3)
    self.assertEqual(c, test3_candidates)

  def testGetPossibleTargets(self):
    avail_targets = [1, 2, 4, 5, 6]
    candidates = [
        [1, 2, 3],
        [4, 5, 6, 7, 8, 9, 10, 11]]
    p = self._fixture.GetPossibleTargets(avail_targets, candidates)
    self.assertEqual(p, [[1, 2], [4, 5, 6]])

  def testIsTestPossible(self):
    avail_targets = [1, 4, 5, 6]
    candidates = [
        [1, 2, 3],
        [4, 5, 6, 7, 8, 9, 10, 11]]
    self.assertTrue(self._fixture.IsTestPossible(avail_targets, candidates))
    avail_targets = [4, 5, 6]
    self.assertFalse(self._fixture.IsTestPossible(avail_targets, candidates))

  def testSelectTestTargets(self):
    # Test possible.
    avail_targets = [1, 2, 4, 5, 6]
    candidates = [
        [1, 2, 3],
        [4, 5, 6, 7, 8, 9, 10, 11]]
    s = self._fixture.SelectTestTargets(avail_targets, candidates)
    self.assertEqual(s, [1, 4])

    # Test impossible.
    avail_targets = [4, 5, 6]
    s = self._fixture.SelectTestTargets(avail_targets, candidates)
    self.assertEqual(s, [])

    # Test possible where we don't want to block another node. In this case,
    # the 'other' test can run on nodes 5 or 6 after we schedule our test.
    other = [[4, 5, 6, 7, 8, 9, 10, 11]]
    avail_targets = [1, 4, 5, 6]
    self.assertTrue(self._fixture.IsTestPossible(avail_targets, other))
    s = self._fixture.SelectTestTargets(avail_targets, candidates, other)
    self.assertEqual(s, [1, 4])

    # Test impossible. The 'other' test requres node 1, so we can't run.
    other = [[1, 2, 3]]
    avail_targets = [1, 4, 5, 6]
    self.assertTrue(self._fixture.IsTestPossible(avail_targets, other))
    s = self._fixture.SelectTestTargets(avail_targets, candidates, other)
    self.assertEqual(s, [])

  def testSchedulePendingTests(self):
    # See setUp(). We expect to schedule test 1 and test 3. Executing test 3
    # does not prevent test 2 from running.
    self._fixture.SchedulePendingTests()

    targets = self._fixture._db.SelectTargets(busy=True)
    self.assertEqual(len(targets), len(self._test1) + len(self._test3))

    # All busy nodes were powered on using mocked RelayManager.
    calls = [mock.call(target['TargetId']) for target in targets]
    self._fixture._relay.PowerOn.assert_has_calls(calls, any_order=True)

    # All nodes in test 1 scheduled.
    nodes = self._fixture._db.SelectTestNodes(test_id=1, running=True)
    self.assertEqual(len(nodes), len(self._test1))
    nodes = self._fixture._db.SelectTestNodes(test_id=1, running=False)

    # All nodes in test 2 not scheduled.
    self.assertEqual(len(nodes), 0)
    nodes = self._fixture._db.SelectTestNodes(test_id=2, running=True)
    self.assertEqual(len(nodes), 0)
    nodes = self._fixture._db.SelectTestNodes(test_id=2, running=False)
    self.assertEqual(len(nodes), len(self._test2))

    # All nodes in test 3 scheduled.
    nodes = self._fixture._db.SelectTestNodes(test_id=3, running=True)
    self.assertEqual(len(nodes), len(self._test3))
    nodes = self._fixture._db.SelectTestNodes(test_id=3, running=False)
    self.assertEqual(len(nodes), 0)


if __name__ == '__main__':
  unittest.main()
