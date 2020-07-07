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

import contextlib
import unittest

from makani.lib.python.embedded import test_fixture
from makani.lib.python.embedded import test_runner
import mock
import numpy


class _FakeChildProcess(object):
  """Simulate pexpect child process."""

  def __init__(self, expect_returns, exit_status=0):
    self.mock_expect = mock.Mock(side_effect=expect_returns)
    self.exitstatus = exit_status

  def expect(self, *args, **kwargs):  # pylint: disable=unused-argument, invalid-name
    return self.mock_expect()

  def close(self, *args, **kwargs):  # pylint: disable=unused-argument, invalid-name
    pass

  def isalive(self):
    return False


class _FakeChildSpawner(object):

  def __init__(self, expect_returns, exit_status=0):
    self._expect_returns = expect_returns
    self._exit_status = exit_status
    self._count = 0

  def next(self):  # pylint: disable=invalid-name
    if isinstance(self._exit_status, list):
      exit_status = self._exit_status[self._count % len(self._exit_status)]
    else:
      exit_status = self._exit_status
    self._count += 1
    return _FakeChildProcess(self._expect_returns, exit_status)


class TestCommandWorker(unittest.TestCase):
  """Test CommandWorker class."""

  def setUp(self):
    self._command_args = {
        'log_extra': {'TestId': 0, 'TestNodeId': 0, 'TargetId': 0},
        'command': 'echo',
        'timeout': 10,
    }

  def runTest(self, expect_returns, exit_status, clock):
    return_value = _FakeChildProcess(expect_returns, exit_status)
    with contextlib.nested(
        mock.patch.object(test_runner.time, 'time', side_effect=clock),
        mock.patch.object(test_runner.pexpect, 'spawn',
                          return_value=return_value)) as (_, mock_spawn):
      worker = test_runner.CommandWorker(**self._command_args)
      final_count = 0
      for count in range(len(expect_returns) + 1):
        if not worker.Iterate():
          final_count = count
          break
      self.assertEqual(final_count, len(expect_returns) - 1)
      self.assertEqual(worker.Close(), exit_status == 0)
      self.assertEqual(worker._child.mock_expect.call_count,
                       len(expect_returns))
      mock_spawn.assert_called_once_with(command=self._command_args['command'],
                                         logfile=worker._logfile,
                                         timeout=self._command_args['timeout'])

  def testNormal(self):
    self.runTest(expect_returns=[0, 0, 0, 1],
                 exit_status=0,
                 clock=[0.0] * 10)

  def testReturnFailure(self):
    self.runTest(expect_returns=[0, 0, 0, 1],
                 exit_status=1,
                 clock=[0.0] * 10)

  def testTimeout(self):
    clock = numpy.arange(0.0, self._command_args['timeout'] * 10.0, 1.0)
    self.runTest(expect_returns=[0] * self._command_args['timeout'],
                 exit_status=1,
                 clock=clock)


class TestJobWorker(unittest.TestCase):

  def setUp(self):
    self._command_args = {
        'log_extra': {'TestId': 0, 'TestNodeId': 0, 'TargetId': 0},
        'command': 'echo',
        'timeout': 10,
    }

  def testNormal(self):
    w1 = test_runner.CommandWorker(**self._command_args)
    w2 = test_runner.CommandWorker(**self._command_args)
    w1_iterate = [True, True, True, False]
    w2_iterate = [True, True, True, True, True, False]

    with contextlib.nested(
        mock.patch.object(test_runner.time, 'time', return_value=0),
        mock.patch.object(w1, 'Iterate', side_effect=w1_iterate),
        mock.patch.object(w1, 'Close', return_value=True),
        mock.patch.object(w2, 'Iterate', side_effect=w2_iterate),
        mock.patch.object(w2, 'Close', return_value=True)):
      worker = test_runner.JobWorker()
      worker.jobs.append(w1)
      worker.jobs.append(w2)
      final_count = 0
      for count in range(len(w1_iterate) + len(w2_iterate) + 1):
        if not worker.Iterate():
          final_count = count
          break
      self.assertEqual(final_count, len(w1_iterate) + len(w2_iterate) - 1)
      self.assertTrue(worker.Close())

  def testOneJobFailed(self):
    w1 = test_runner.CommandWorker(**self._command_args)
    w2 = test_runner.CommandWorker(**self._command_args)
    w1_iterate = [True, True, True, False]
    w2_iterate = [True, True, True, True, True, False]

    with contextlib.nested(
        mock.patch.object(test_runner.time, 'time', return_value=0),
        mock.patch.object(w1, 'Iterate', side_effect=w1_iterate),
        mock.patch.object(w1, 'Close', return_value=False),
        mock.patch.object(w2, 'Iterate', side_effect=w2_iterate),
        mock.patch.object(w2, 'Close', return_value=True)):
      worker = test_runner.JobWorker()
      worker.jobs.append(w1)
      worker.jobs.append(w2)
      final_count = 0
      for count in range(len(w1_iterate) + len(w2_iterate) + 1):
        if not worker.Iterate():
          final_count = count
          break
      self.assertEqual(final_count, len(w1_iterate) - 1)
      self.assertFalse(worker.Close())


class TestNodeWorker(unittest.TestCase):

  def setUp(self):
    self._paths = {
        'tms570_bin': '/path/to/tms570-bin',
        'q7_bin': '/path/to/q7-bin',
        'host_bin': '/path/to/host-bin'}
    self._config = {
        'FirmwareTms570': 'tms570_application.elf',
        'FirmwareQ7': 'q7_application',
        'CalibParams': 'calib_params.bin',
        'ConfigParams': 'config_params.bin',
        'HostApplication': None,
        'HostArguments': '',
        'Timeout': 10,
        'TestId': 0,
        'TestNodeId': 0,
        'TargetId': 0}
    self._target = {
        'AioNodeTms570': 'kAioNodeFcA',
        'AioNodeQ7': 'kAioNodeControllerA'}

  def testNormal(self):
    spawner = _FakeChildSpawner([0, 0, 0, 1])
    with contextlib.nested(
        mock.patch.object(test_runner.time, 'time', return_value=0),
        mock.patch.object(test_runner.pexpect, 'spawn', side_effect=spawner)):
      worker = test_runner.NodeWorker(self._config, self._target, self._paths)
      final_count = 0
      for count in range(100):
        if not worker.Iterate():
          final_count = count
          break
      self.assertGreater(final_count, 0)
      self.assertTrue(worker.Close())

  def testFailure(self):
    spawner = _FakeChildSpawner([0, 0, 0, 1], [0, 0, 1])
    with contextlib.nested(
        mock.patch.object(test_runner.time, 'time', return_value=0),
        mock.patch.object(test_runner.pexpect, 'spawn', side_effect=spawner)):
      worker = test_runner.NodeWorker(self._config, self._target, self._paths)
      final_count = 0
      for count in range(100):
        if not worker.Iterate():
          final_count = count
          break
      self.assertGreater(final_count, 0)
      self.assertFalse(worker.Close())


class TestTestRunner(unittest.TestCase):

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

    # Define test runner.
    paths = {
        'tms570_bin': '/path/to/tms570-bin',
        'q7_bin': '/path/to/q7-bin',
        'host_bin': '/path/to/host-bin'}
    self._runner = test_runner.TestRunner(paths=paths,
                                          database=self._fixture._db)

    # Insert tests into runner.
    test1_node1 = {
        'FirmwareTms570': 'tms570_application.elf',
        'FirmwareQ7': 'q7_application',
        'CalibParams': 'calib_params.bin',
        'ConfigParams': 'config_params.bin',
        'Timeout': 30,
        'HardwareOptions': 'novatel'}
    test1_node2 = {
        'FirmwareTms570': 'tms570_application.elf',
        'FirmwareQ7': 'q7_application',
        'ConfigParams': 'config_params.bin',
        'BoardName': 'fc',
        'Timeout': 30,
        'HardwareOptions': 'septentrio'}
    test2_node1 = {
        'FirmwareTms570': 'tms570_application.elf',
        'CalibParams': 'calib_params.bin',
        'Timeout': 30,
        'HardwareOptions': 'imu'}
    self._runner.InsertTest([test1_node1, test1_node2])
    self._runner.InsertTest([test2_node1])

  def runTest(self, spawner):
    with contextlib.nested(
        mock.patch.object(test_runner.time, 'time', return_value=0),
        mock.patch.object(test_runner.pexpect, 'spawn', side_effect=spawner)):
      final_count = 0
      for count in range(100):
        self._fixture.Iterate()
        if not self._runner.Iterate():
          final_count = count
          break
      return final_count

  def testNormal(self):
    final_count = self.runTest(_FakeChildSpawner([0, 0, 0, 1]))
    self.assertGreater(final_count, 0)
    self.assertFalse(self._runner.GetError())

  def testFailure(self):
    final_count = self.runTest(_FakeChildSpawner([0, 0, 0, 1], [0, 0, 0, 1]))
    self.assertGreater(final_count, 0)
    self.assertTrue(self._runner.GetError())

  def testInvalidHardware(self):
    test3_node1 = {
        'FirmwareTms570': 'tms570_application.elf',
        'Timeout': 30,
        'HardwareOptions': 'invalid'}
    self._runner.InsertTest([test3_node1])
    final_count = self.runTest(_FakeChildSpawner([0, 0, 0, 1]))
    self.assertGreater(final_count, 0)
    self.assertTrue(self._runner.GetError())

  def testHostApplication(self):
    test3_node1 = {
        'HostApplication': 'avionics/linux/test_command',
        'HostArguments': '--node=${AIO_NODE_TMS570} --timeout=${TIMEOUT}',
        'Timeout': 30}
    self._runner.InsertTest([test3_node1])
    final_count = self.runTest(_FakeChildSpawner([0, 0, 0, 1]))
    self.assertGreater(final_count, 0)
    self.assertFalse(self._runner.GetError())


if __name__ == '__main__':
  unittest.main()
