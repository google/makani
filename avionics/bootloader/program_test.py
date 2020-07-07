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

import os
import re
import shlex
from StringIO import StringIO
import unittest

from makani.avionics.bootloader import program
import mock


class ParseArgumentsTest(unittest.TestCase):

  def Run(self, arg_string):
    def _FakeFindUniqueFile(directory, variant, prefix, suffix, node_name):
      self.assertIsInstance(directory, str)
      self.assertIsInstance(variant, str)
      self.assertTrue(prefix is None or isinstance(prefix, str))
      self.assertIsInstance(suffix, str)
      self.assertTrue(node_name is None or isinstance(node_name, str))
      return os.path.join(directory, '%s_%s' % (variant, suffix))
    argv = ['program', '--tms570_bin=/abc/xyz/tms570-bin',
            '--bootloader_client=/abc/xyz/bootloader_client',
            '--print'] + shlex.split(arg_string)
    with mock.patch('sys.stdout', new_callable=StringIO) as output:
      with mock.patch('makani.avionics.bootloader.program._FindUniqueFile',
                      _FakeFindUniqueFile):
        program.Main(argv)
        return output.getvalue()

  def RunAssertArgs(self, args, num_ops, arg_pattern_pass=None,
                    arg_pattern_fail=None):
    """Runs the program and validates the output against constraints.

    Args:
      args: An argument string under test.
      num_ops: The number of operations expected to be performed.
      arg_pattern_pass: Regex expressions which are required to match in the
          returned operation.  Either a list of strings to apply to all
          operations or a list of lists of strings to apply to each operation
          in order.
      arg_pattern_fail: Regex expressions which are required to not match in the
          returned operation.  Either a list of strings to apply to all
          operations or a list of lists of strings to apply to each operation
          in order.
    """
    def _GetPatternList(patterns, index):
      if not patterns:
        return []
      if isinstance(patterns[0], str):
        return patterns
      return patterns[index]
    ops = [shlex.split(op.strip()) for op in self.Run(args).split('\n') if op]
    self.assertEqual(len(ops), num_ops)
    for i, op in enumerate(ops):
      for pattern in _GetPatternList(arg_pattern_pass, i):
        self.assertTrue(any(re.match(pattern + '$', arg) for arg in op))
      for pattern in _GetPatternList(arg_pattern_fail, i):
        self.assertFalse(any(re.match(pattern + '$', arg) for arg in op))

  def RunAssertException(self, args, exception=SystemExit):
    with self.assertRaises(exception):
      self.Run(args)

  def testBasic(self):
    fail = ['--bootloader', '--calib', '--config', '--override_target',
            '--force_hardware']
    self.RunAssertArgs('cs_a', 1, ['cs_a', '.*cs_application\\.elf'], fail)
    self.RunAssertArgs('servo_e1 servo_a1', 2,
                       ['servo_..', '.*servo_application\\.elf'], fail)
    self.RunAssertArgs('servo_e1 motor_pbo', 2,
                       [['servo_e1', '.*servo_application\\.elf'],
                        ['motor_pbo', '.*motor_application\\.elf']], fail)
    self.RunAssertArgs('--prefix servo', 10,
                       ['servo_..', '.*servo_application\\.elf'], fail)
    self.RunAssertArgs('--prefix cs cs_a cs_b', 2,
                       [['cs_.', '.*cs_application\\.elf']] * 2, fail)

  def testApplication(self):
    fail = ['--bootloader', '--calib', '--config', '--override_target',
            '--force_hardware']
    self.RunAssertArgs('servo_e1 motor_pbo --application', 2,
                       [['servo_e1', '.*servo_application\\.elf'],
                        ['motor_pbo', '.*motor_application\\.elf']], fail)
    self.RunAssertArgs('--prefix servo --application', 10,
                       ['servo_..', '.*servo_application\\.elf'], fail)

  def testInvalidTargets(self):
    self.RunAssertException('')
    self.RunAssertException('motor_e1')
    self.RunAssertException('cs_a cs_xyz')
    self.RunAssertException('--prefix xyz')
    self.RunAssertException('--prefix servo cs_a')

  def testConfig(self):
    fail = ['--bootloader', '--calib', '--override_target', '--force_hardware']
    self.RunAssertException('--config a')
    self.RunAssertException('cs_a --config')
    self.RunAssertArgs('cs_a --config a', 1,
                       ['--config', '.*a_config_params\\.bin'], fail)
    self.RunAssertArgs('cs_a cs_b --config a', 2,
                       ['--config', '.*a_config_params\\.bin'], fail)

  def testCalib(self):
    fail = ['--bootloader', '--config', '--override_target', '--force_hardware']
    self.RunAssertException('--calib a')
    self.RunAssertException('cs_a --calib')
    self.RunAssertArgs('cs_a --calib a', 1,
                       ['--calib', '.*a_calib_params\\.bin'], fail)
    self.RunAssertArgs('cs_a cs_b --calib a', 2,
                       ['--calib', '.*a_calib_params\\.bin'], fail)

  def testBootloader(self):
    fail = ['--calib', '--config', '--override_target', '--force_hardware']
    self.RunAssertException('--bootloader')
    self.RunAssertException('cs_a --bootloader a')
    self.RunAssertArgs('cs_a --bootloader', 1,
                       ['--bootloader', '.*bootloader\\.elf'], fail)
    self.RunAssertArgs('motor_sbo motor_pbo --bootloader', 2,
                       ['--bootloader', '.*bootloader\\.elf'], fail)

  def testBootloaderApplication(self):
    fail = ['--bootloader', '--calib', '--config', '--override_target',
            '--force_hardware']
    self.RunAssertException('--bootloader_application')
    self.RunAssertException('cs_a --bootloader_application a')
    self.RunAssertArgs('cs_a --bootloader_application', 1,
                       ['.*bootloader_application\\.elf'], fail)
    self.RunAssertArgs('motor_sbo motor_pbo --bootloader_application', 2,
                       ['.*bootloader_application\\.elf'], fail)

  def testUpgradeBootloader(self):
    fail = ['--calib', '--config', '--override_target', '--force_hardware']
    self.RunAssertException('--upgrade_bootloader')
    self.RunAssertException('cs_a --upgrade_bootloader a')
    self.RunAssertArgs('cs_a --upgrade_bootloader', 3,
                       [['.*bootloader_application\\.elf'],
                        ['--bootloader', '.*bootloader\\.elf'],
                        ['.*cs_application\\.elf']], fail)
    self.RunAssertArgs('motor_sbo motor_pbo --upgrade_bootloader', 6,
                       [['.*bootloader_application\\.elf'],
                        ['--bootloader', '.*bootloader\\.elf'],
                        ['.*motor_application\\.elf']] * 2, fail)

  def testRenameTo(self):
    fail = ['--calib', '--config', '--force_hardware']
    self.RunAssertException('--rename_to')
    self.RunAssertException('cs_a --rename_to')
    self.RunAssertException('cs_a --rename_to cs_a')
    self.RunAssertException('unknown --rename_to unknown')
    self.RunAssertArgs('cs_a --rename_to cs_b', 3,
                       [['.*bootloader_application\\.elf', 'cs_a'],
                        ['--bootloader', '.*bootloader\\.elf',
                         '--override_target', 'cs_b'],
                        ['.*cs_application\\.elf', 'cs_b']], fail)
    self.RunAssertException('motor_sbo motor_pbo --rename_to motor_pto')
    self.RunAssertArgs('unknown --rename_to cs_a', 3,
                       [['.*bootloader_application\\.elf', 'unknown'],
                        ['--bootloader', '.*bootloader\\.elf',
                         '--override_target', 'cs_a'],
                        ['.*cs_application\\.elf', 'cs_a']], fail)
    self.RunAssertArgs('cs_a --rename_to unknown', 3,
                       [['.*bootloader_application\\.elf', 'cs_a'],
                        ['--bootloader', '.*bootloader\\.elf',
                         '--override_target', 'unknown'],
                        ['.*bootloader_application\\.elf', 'unknown']], fail)

  def testForceHardware(self):
    fail = ['--calib', '--config']
    self.RunAssertException('cs_a --force_hardware cs')
    self.RunAssertException('cs_a --calib a --force_hardware cs')
    self.RunAssertArgs('cs_a --bootloader --force_hardware cs', 1,
                       ['--bootloader', '.*bootloader\\.elf',
                        '--force_hardware', 'cs'], fail)
    self.RunAssertArgs('motor_pbo --bootloader_application --force_hardware '
                       'motor', 1, ['.*bootloader_application\\.elf',
                                    '--force_hardware', 'motor'], fail)
    self.RunAssertArgs('cs_a --rename_to cs_b --force_hardware cs', 3,
                       [['.*bootloader_application\\.elf', 'cs_a',
                         '--force_hardware'],
                        ['--bootloader', '.*bootloader\\.elf',
                         '--override_target', 'cs_b', '--force_hardware',
                         'cs'],
                        ['.*cs_application\\.elf', 'cs_b']],
                       [fail, fail, fail + ['--force_hardware']])

  def testSerial(self):
    fail = ['--calib', '--config', '--force_hardware', '--.*application',
            '--bootloader', '--carrier_serial']
    self.RunAssertException('fc_a --serial aio rev_ab 123456')
    self.RunAssertArgs('unknown --serial aio rev_ab 123456', 2,
                       [['param_util', '--input', '.*aio_serial_params\\.yaml',
                         '--output', '.*_serial_params.bin', '--yaml_key',
                         'rev_ab', '--set_value', 'serial_number:123456'],
                        ['.*bootloader_client', 'unknown',
                         '.*_serial_params.bin', '--serial']], fail)

  def testCarrierSerial(self):
    fail = ['--calib', '--config', '--force_hardware', '--.*application',
            '--bootloader', '--serial']
    self.RunAssertException('fc_a --carrier_serial fc rev_ab 123456')
    self.RunAssertArgs('unknown --carrier_serial fc rev_ab 123456', 2,
                       [['param_util', '--input', '.*fc_serial_params\\.yaml',
                         '--output', '.*_carrier_serial_params.bin',
                         '--yaml_key',
                         'rev_ab', '--set_value', 'serial_number:123456'],
                        ['.*bootloader_client', 'unknown',
                         '.*_carrier_serial_params.bin', '--carrier_serial']],
                       fail)

  def testLocation(self):
    def _RunLocation(cmd, pass_targets, fail_targets):
      result = self.Run(cmd)
      self.assertTrue(all('--target %s' % node in result
                          for node in pass_targets))
      self.assertFalse(any('--target %s' % node in result
                           for node in fail_targets))
    self.RunAssertException('--location xyz')
    self.RunAssertException('--location wing cs_gs_a')
    self.RunAssertException('--location ground_station --prefix servo')
    wing_nodes = ['cs_a', 'cs_b', 'servo_r1', 'motor_pto',
                  'recorder_tms570_wing', 'fc_c']
    ground_nodes = ['cs_gs_a', 'cs_gs_b', 'platform_sensors_a', 'plc']
    remote_command_nodes = ['joystick']
    test_nodes = ['visualizer']
    _RunLocation('--location wing', wing_nodes,
                 ground_nodes + remote_command_nodes + test_nodes)
    _RunLocation('--location wing --prefix cs', ['cs_a', 'cs_b'],
                 ['cs_gs_a', 'cs_gs_b'])
    _RunLocation('cs_a --location wing', ['cs_a'],
                 ['cs_b', 'cs_gs_a', 'cs_gs_b'])
    _RunLocation('--location ground_station', ground_nodes,
                 wing_nodes + remote_command_nodes + test_nodes)
    _RunLocation('--location remote_command', remote_command_nodes,
                 wing_nodes + ground_nodes + test_nodes)
    _RunLocation('--location wing ground_station', ground_nodes + wing_nodes,
                 remote_command_nodes + test_nodes)

  def testBatch(self):
    self.RunAssertException('--batch cs_a cs_b --upgrade_bootloader')
    self.RunAssertException('--batch cs_a --calib a')
    self.RunAssertException('--batch cs_a --config a')
    self.RunAssertException('--batch cs_a --application')
    self.RunAssertException('--batch --prefix cs_a')
    self.RunAssertArgs('--batch cs_a', 1)
    self.RunAssertArgs('--batch cs_a motor_pbo', 2)
    self.RunAssertArgs('--batch "--prefix cs --location wing ground_station" '
                       '"--prefix servo"', 14)
    self.RunAssertArgs('--batch "--prefix cs" "cs_a --config a"', 7)
    self.RunAssertArgs('--batch "--prefix cs --upgrade_bootloader" '
                       '"--prefix servo"', 28)

  def testMixedOperations(self):
    self.RunAssertException('motor_pbi --config aa --application')
    self.RunAssertException('motor_pbi --config aa --calib')
    self.RunAssertException('motor_pbi --calib aa --application')
    self.RunAssertException('motor_pbi --bootloader --application')
    self.RunAssertException('motor_pbi --bootloader_application --application')
    self.RunAssertException('motor_pbi --bootloader_application --bootloader')
    self.RunAssertException('motor_pbi --calib aa --config bb')

  def testParallel(self):
    self.RunAssertArgs('servo_e1 motor_pbo --parallel', 2)
    self.RunAssertArgs('servo_e1 --config e1 --parallel', 1)
    self.RunAssertArgs('motor_pbo --calib xyz --parallel', 1)
    self.RunAssertArgs('--prefix servo --upgrade_bootloader --parallel', 30)
    self.RunAssertArgs('cs_a cs_b --bootloader --parallel', 2)
    self.RunAssertArgs('cs_a cs_b --bootloader_application --parallel', 2)
    self.RunAssertArgs('--batch "cs_a" "cs_b" "--prefix servo" '
                       '"cs_a --config x" --parallel', 13)
    self.RunAssertException('servo_e1 --rename_to servo_e2 --parallel')
    self.RunAssertException('--batch "motor_sbo --rename_to motor_pbo" '
                            '"servo_a1 --rename_to servo_r1" --parallel')

  def testJlink(self):
    self.RunAssertException('motor_sbo --jlink')
    self.RunAssertArgs('motor_sbo --jlink --force_hardware motor', 1)
    self.RunAssertArgs('motor_sbo --bootloader --jlink --force_hardware motor',
                       1)
    self.RunAssertArgs('motor_sbo --jlink --serial motor gin_a1 123456', 2)
    self.RunAssertArgs('unknown --jlink --serial aio rev_ab 123456', 2)
    self.RunAssertArgs('--jlink --serial aio rev_ab 123456', 2)
    self.RunAssertArgs('--jlink --force_hardware motor --batch '
                       '"motor_sbo --serial motor gin_a1 serial_num" '
                       '"motor_sbo --bootloader" '
                       '"motor_sbo --application"', 4)

if __name__ == '__main__':
  unittest.main()
