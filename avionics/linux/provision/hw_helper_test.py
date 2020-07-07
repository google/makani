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
import unittest

from makani.avionics.linux.provision import hw_helper
from makani.lib.python import os_util
import mock


def MockPopenNet(interface):
  sub_mock = mock.Mock(spec=hw_helper.subprocess)
  def MockPopen(*args, **_):
    mock_popen = mock.Mock()
    mock_popen.communicate.return_value = ('', '')
    if args[0][2].split('/')[-1] == interface:
      mock_popen.communicate.return_value = (
          'P: /devices/pci0000:00/0000:00:19.0/net/{interface}\n'
          'E: ID_BUS=pci\n'
          'E: ID_MODEL_ID=0x1502\n'
          'E: ID_SERIAL_SHORT=<short_serial>\n'
          'E: DEVPATH=/devices/pci0000:00/0000:00:19.0/net/'
          '{interface}\n'.format(interface=interface), '')
    return mock_popen
  sub_mock.Popen = MockPopen
  return sub_mock


class MockSysCharInterface(os_util.TempDir):

  def __init__(self, device='ttyACM', device_id=2, uevent_line='uniquenewyork'):
    self.uevent_never_match = 'X' + uevent_line[::-1]
    self.device_basename = device
    self.device_id = str(device_id)
    self.uevent_line = uevent_line
    super(MockSysCharInterface, self).__init__()

  def __enter__(self):
    """Create a /sys/dev/char and /dev environment within a temporary directory.

    Description:
      Create sys/dev/char and dev subdirectories, create several subdirectories
      for various devices types including the one specified in the init. Create
      symlinks and directory structure to 5 devices in each device type. Create
      uevent files within a 'device' subdirectory with the string being hunted
      for in only one device.

    Returns:
      self
    """
    self.tmp_path = super(MockSysCharInterface, self).__enter__()
    self.sys_path = os.path.join(self.tmp_path, 'sys/dev/char')
    self.dev_path = os.path.join(self.tmp_path, 'dev')
    os.makedirs(self.sys_path)
    os.mkdir(self.dev_path)
    for device_type in set(['video', 'other', self.device_basename]):
      for device_id in [str(i) for i in range(5)]:
        path_sym_target = os.path.join(self.sys_path, '../../device',
                                       device_type, device_id,
                                       device_type + device_id)
        os.makedirs(os.path.join(path_sym_target, 'device'))
        os.symlink(path_sym_target, os.path.join(self.sys_path,
                                                 device_type+device_id))
        with open(os.path.join(path_sym_target, 'device/uevent'),
                  'w') as file_uevent:
          file_uevent.write(self.uevent_never_match)
          if (device_id == self.device_id
              and device_type == self.device_basename):
            file_uevent.truncate(0)
            file_uevent.write(self.uevent_line)
            with open(os.path.join(self.dev_path, self.device_basename
                                   + self.device_id), 'w'):
              pass
    return self


class TestHwHelper(unittest.TestCase):

  def testGetNetUdev(self):
    with mock.patch.object(hw_helper, 'subprocess', MockPopenNet('eth0')):
      self.assertEqual(hw_helper.GetNetUdev('eth0', 'ID_BUS='), 'pci')
      self.assertEqual(hw_helper.GetNetUdev('eth0', 'ID_MODEL_ID='), '0x1502')
      self.assertIsNone(hw_helper.GetNetUdev('eth0', 'INVALIDSTRING'))

  def testFindEth(self):
    mock_eth_list = mock.Mock()
    mock_eth_list.listdir.return_value = ['eth0', 'eth1', 'ethX']
    with mock.patch.object(hw_helper, 'subprocess', MockPopenNet('eth1')):
      with mock.patch.object(hw_helper, 'os', mock_eth_list):
        self.assertEqual(hw_helper.FindEthBySerial('<short_serial>'), 'eth1')
    with mock.patch.object(hw_helper, 'subprocess', MockPopenNet('ethX')):
      with mock.patch.object(hw_helper, 'os', mock_eth_list):
        self.assertEqual(hw_helper.FindEthBySerial('<short_serial>'), 'ethX')

  def testGetUdevDevice(self):
    with MockSysCharInterface(device='ttyEXAMPLE', device_id=2) as mock_sys:
      self.assertEqual(
          hw_helper.GetUdevDevice(
              'ttyEXAMPLE', 'uniquenewyork',
              path_sys_char=mock_sys.sys_path,
              path_dev=mock_sys.dev_path),
          os.path.join(mock_sys.tmp_path, 'dev', 'ttyEXAMPLE2'))
      self.assertIsNone(
          hw_helper.GetUdevDevice(
              'ttyEXAMPLE', 'uniquewrong',
              path_sys_char=mock_sys.sys_path,
              path_dev=mock_sys.dev_path))
      self.assertIsNone(
          hw_helper.GetUdevDevice(
              'ttyWRONG', 'uniquenewyork',
              path_sys_char=mock_sys.sys_path,
              path_dev=mock_sys.dev_path))

if __name__ == '__main__':
  unittest.main()
