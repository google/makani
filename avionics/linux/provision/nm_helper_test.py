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

from makani.avionics.linux.provision import nm_helper
from makani.lib.python import os_util
import mock


class MockSysInterface(os_util.TempDir):

  def __init__(self, subdir='ethX'):
    self.subdir = subdir
    super(MockSysInterface, self).__init__()

  def __enter__(self):
    self.path_tmp = super(MockSysInterface, self).__enter__()
    self.path_interface = os.path.join(self.path_tmp, self.subdir)
    os.mkdir(self.path_interface)
    self.tempfile_fh = open(os.path.join(self.path_interface, 'operstate'),
                            mode='w', buffering=False)
    self.Down()
    return self

  def __exit__(self, *args, **kwargs):
    self.tempfile_fh.close()
    return super(MockSysInterface, self).__exit__(*args, **kwargs)

  def Up(self):
    self.tempfile_fh.seek(0)
    self.tempfile_fh.truncate()
    self.tempfile_fh.write('up')

  def Down(self):
    self.tempfile_fh.seek(0)
    self.tempfile_fh.truncate()
    self.tempfile_fh.write('down')


class MockNmcli(object):

  def __init__(self):
    self.device_status = {'eth0': 'unavailable',
                          'eth1': 'unavailable',
                          'ethX': 'connected'}

  def CallNmcli(self, args):
    result = ''
    if args == ['-t', '-f', 'DEVICE,STATE', 'dev', 'status']:
      for dev, status in self.device_status.iteritems():
        result += '{}:{}\n'.format(dev, status)
    elif args[:3] == ['dev', 'disconnect', 'iface']:
      if args[3] in self.device_status:
        self.device_status[args[3]] = 'disconnected'
    elif args[:3] == ['con', 'up', 'id'] and len(args) >= 6:
      if args[5] in self.device_status:
        self.device_status[args[5]] = 'connected'
    else:
      raise SyntaxError(
          '{} does not support arguments: {}'.format(self.__class__, args))
    return result


class TestNetworkMonitor(unittest.TestCase):

  def setUp(self):
    self.mock_nmcli = MockNmcli()

  def testNmInterface(self):
    with MockSysInterface(subdir='ethX') as mock_interface:
      # Replace _CallNmcli method with mock call
      with mock.patch.object(nm_helper.NetworkMonitor, '_CallNmcli',
                             self.mock_nmcli.CallNmcli):
        network_monitor = nm_helper.NetworkMonitor(
            device='ethX',
            sys_operstate_path=mock_interface.tempfile_fh.name)
        mock_interface.Down()
        self.assertTrue(network_monitor.IsDown())
        self.assertFalse(network_monitor.IsUp())
        mock_interface.Up()
        self.assertFalse(network_monitor.IsDown())
        self.assertTrue(network_monitor.IsUp())

  def testNmUpDown(self):
    with MockSysInterface(subdir='ethX') as mock_interface:
      with mock.patch.object(nm_helper.NetworkMonitor, '_CallNmcli',
                             self.mock_nmcli.CallNmcli):
        network_monitor = nm_helper.NetworkMonitor(
            device='ethX',
            sys_operstate_path=mock_interface.tempfile_fh.name)
        network_monitor.Disconnect()
        self.assertFalse(network_monitor.IsConnected())
        self.assertTrue(network_monitor.IsDisconnected())
        network_monitor.Up()
        self.assertTrue(network_monitor.IsConnected())
        self.assertFalse(network_monitor.IsDisconnected())


if __name__ == '__main__':
  unittest.main()
