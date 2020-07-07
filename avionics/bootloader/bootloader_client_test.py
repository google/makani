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

import sys
import unittest

from makani.avionics.bootloader import bootloader_client
from makani.avionics.common import aio
from makani.avionics.firmware.identity import identity_types
from makani.avionics.network import aio_labels
from makani.avionics.network import aio_node
from makani.lib.python import c_helpers
import mock

hardware_type_helper = c_helpers.EnumHelper('HardwareType', identity_types)


class ParseArgumentsTest(unittest.TestCase):

  def RunParse(self, arg_string):
    argv = ['bootloader_client.py'] + arg_string.split()
    with mock.patch.object(sys, 'argv', argv):
      return bootloader_client.ParseArguments()

  def testBatt(self):
    parsed_args = self.RunParse('--target batt_a batt_application.elf')
    args = parsed_args['args']

    self.assertEqual(parsed_args['file'], 'batt_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeBattA))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeBattA))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kBattA)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testCoreSwitch(self):
    parsed_args = self.RunParse('--target cs_a cs_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'cs_a')
    self.assertEqual(parsed_args['file'], 'cs_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeCsA))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeCsA))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kCoreSwitchA)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testCoreSwitchGroundStation(self):
    parsed_args = self.RunParse('--target cs_gs_a cs_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'cs_gs_a')
    self.assertEqual(parsed_args['file'], 'cs_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeCsGsA))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeCsGsA))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kCoreSwitchGsA)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testDrum(self):
    parsed_args = self.RunParse('--target drum_sensors_a drum_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'drum_sensors_a')
    self.assertEqual(parsed_args['file'], 'drum_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(
        parsed_args['cur_ip'],
        aio.AioNodeToIpAddressString(aio_node.kAioNodeDrumSensorsA))
    self.assertEqual(
        parsed_args['new_ip'],
        aio.AioNodeToIpAddressString(aio_node.kAioNodeDrumSensorsA))
    self.assertEqual(parsed_args['cur_node_index'], 0)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testFlightComputer(self):
    parsed_args = self.RunParse('--target fc_b fc_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'fc_b')
    self.assertEqual(parsed_args['file'], 'fc_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeFcB))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeFcB))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kControllerB)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testGps(self):
    parsed_args = self.RunParse(
        '--target gps_base_station gps_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'gps_base_station')
    self.assertEqual(parsed_args['file'], 'gps_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(
                         aio_node.kAioNodeGpsBaseStation))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(
                         aio_node.kAioNodeGpsBaseStation))
    self.assertEqual(parsed_args['cur_node_index'], 0)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testMotor(self):
    parsed_args = self.RunParse('--target motor_pti motor_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'motor_pti')
    self.assertEqual(parsed_args['file'], 'motor_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeMotorPti))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeMotorPti))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kMotorPti)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testPlatform(self):
    parsed_args = self.RunParse(
        '--target platform_sensors_a platform_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'platform_sensors_a')
    self.assertEqual(parsed_args['file'], 'platform_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(
                         aio_node.kAioNodePlatformSensorsA))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(
                         aio_node.kAioNodePlatformSensorsA))
    self.assertEqual(parsed_args['cur_node_index'], 0)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testRecorderTms570(self):
    parsed_args = self.RunParse(
        '--target recorder_tms570_platform recorder_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'recorder_tms570_platform')
    self.assertEqual(parsed_args['file'], 'recorder_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(
        parsed_args['cur_ip'],
        aio.AioNodeToIpAddressString(aio_node.kAioNodeRecorderTms570Platform))
    self.assertEqual(
        parsed_args['new_ip'],
        aio.AioNodeToIpAddressString(aio_node.kAioNodeRecorderTms570Platform))
    self.assertEqual(parsed_args['cur_node_index'],
                     aio_labels.kRecorderTms570Platform)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testServo(self):
    parsed_args = self.RunParse(
        '--target servo_e2 servo_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'servo_e2')
    self.assertEqual(parsed_args['file'], 'servo_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeServoE2))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeServoE2))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kServoE2)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testDump(self):
    parsed_args = self.RunParse(
        '--target servo_e2 --dump_image servo_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'servo_e2')
    self.assertEqual(parsed_args['file'], 'servo_application.elf')
    self.assertEqual(args.force_hardware, None)

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeServoE2))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeServoE2))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kServoE2)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertTrue(args.dump_image)

  def testBadUpdateType(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target fc_c '
                    'tms570-bin/avionics/bootloader/firmware/bootloader.elf')

  def testFcConfigParamsSuccess(self):
    parsed_args = self.RunParse('--target fc_c --config fc_config_params.bin')
    args = parsed_args['args']

    self.assertEqual(args.target, 'fc_c')
    self.assertEqual(parsed_args['file'], 'fc_config_params.bin')

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeFcC))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeFcC))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kFlightComputerC)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'ConfigParams')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testFcCalibParamsSuccess(self):
    parsed_args = self.RunParse(
        '--target fc_c rev_a3_fc_calib_params.bin --calib')
    args = parsed_args['args']

    self.assertEqual(args.target, 'fc_c')
    self.assertEqual(parsed_args['file'], 'rev_a3_fc_calib_params.bin')

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeFcC))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeFcC))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kFlightComputerC)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'CalibParams')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testBootloaderSuccess(self):
    parsed_args = self.RunParse(
        '--target motor_sbo --bootloader bootloader.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'motor_sbo')
    self.assertEqual(parsed_args['file'], 'bootloader.elf')

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeMotorSbo))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeMotorSbo))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kMotorSbo)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Bootloader')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testIgnoreMismatch(self):
    parsed_args = self.RunParse(
        '--target motor_sbo --ignore_mismatch motor_application.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'motor_sbo')
    self.assertEqual(parsed_args['file'], 'motor_application.elf')

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeMotorSbo))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeMotorSbo))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kMotorSbo)
    self.assertEqual(parsed_args['new_node_index'],
                     parsed_args['cur_node_index'])
    self.assertEqual(parsed_args['update_type'], 'Application')
    self.assertFalse(args.dump_image)
    self.assertTrue(args.ignore_mismatch)

  def testBootloaderOverrideTarget(self):
    parsed_args = self.RunParse(
        '--target fc_a --bootloader --override_target gps_base_station'
        ' bootloader.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'fc_a')
    self.assertEqual(parsed_args['file'], 'bootloader.elf')

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeFcA))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(
                         aio_node.kAioNodeGpsBaseStation))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kFlightComputerA)
    self.assertEqual(parsed_args['new_node_index'], aio_labels.kGpsBaseStation)
    self.assertEqual(parsed_args['update_type'], 'Bootloader')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testOverrideTargetWithoutBootloaderFlag(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target fc_a fc_application.bin --override_target fc_c')

  def testBootloaderWithoutBootloaderFlag(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target motor_sbo bootloader.bin')

  def testParamsBadExtension(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target fc_c --config fc_params.elf')

  def testDumpWithoutElf(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target fc_c --dump_image fc_application.bin')

  def testCalibParamsWithoutCalibFlag(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target fc_c fc_calib_params.bin')

  def testConfigParamsWithCalibFlag(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target fc_c fc_config_params.bin --calib')

  def testNonParamsWithCalibFlag(self):
    with self.assertRaises(ValueError):
      self.RunParse('--target fc_c fc_application.elf --calib')

  def testBootloaderForceHardwareFc(self):
    parsed_args = self.RunParse(
        '--target cs_a --bootloader --force_hardware=fc cs_bootloader.elf')
    args = parsed_args['args']

    self.assertEqual(args.target, 'cs_a')
    self.assertEqual(parsed_args['file'], 'cs_bootloader.elf')
    self.assertEqual(args.force_hardware, 'fc')

    self.assertEqual(parsed_args['cur_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeCsA))
    self.assertEqual(parsed_args['new_ip'],
                     aio.AioNodeToIpAddressString(aio_node.kAioNodeCsA))
    self.assertEqual(parsed_args['cur_node_index'], aio_labels.kCoreSwitchA)
    self.assertEqual(parsed_args['new_node_index'], aio_labels.kCoreSwitchA)
    self.assertEqual(parsed_args['update_type'], 'Bootloader')
    self.assertFalse(args.dump_image)
    self.assertFalse(args.ignore_mismatch)

  def testBootloaderForceHardwareOld(self):
    with self.assertRaises(ValueError):
      self.RunParse(
          '--target cs_a cs_bootloader.elf --bootloader --force_hardware=old')


if __name__ == '__main__':
  unittest.main()
