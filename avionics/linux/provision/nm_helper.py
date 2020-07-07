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

"""A wrapper for network-manager configuration and status checking."""

import logging
import subprocess
import time

# TODO: Add firewall checker.
# TODO: Move away from nmcli.
# TODO: Add timeouts to nmcli call and all subprocess calls.
# TODO: Check ethernet interface configuration.


class NetworkMonitor(object):
  """A network interface poller and controller.

  A class for polling a network interface's status and controlling Network
  Manager.
  """

  def __init__(self, device, sys_operstate_path='/sys/class/net/{}/operstate'):
    self.dev = device
    self._dev_fh = open(sys_operstate_path.format(self.dev), 'r')
    self.UpdateState()

  def UpdateState(self):
    self._dev_fh.seek(0)
    self.state = self._dev_fh.readline().strip('\n')
    self._dev_fh.flush()  # File contents are cached, must flush the cache.

  def Disconnect(self):
    self._CallNmcli(['dev', 'disconnect', 'iface', self.dev])

  def Up(self):
    self._CallNmcli(['con', 'up', 'id', '"AIO"', 'iface', self.dev])

  def GetStatus(self):
    """Get the status of this network interface from network manager.

    Returns:
      The string from STATE (e.g. "connected", "unavailable").
    """

    for line in self._CallNmcli(['-t', '-f', 'DEVICE,STATE', 'dev',
                                 'status']).split('\n'):
      device, _, state = line.partition(':')
      if device == self.dev:
        return state

  def _CallNmcli(self, args):
    proc_udev = subprocess.Popen(['nmcli'] + args,
                                 stdout=subprocess.PIPE)
    stdout, stderr = proc_udev.communicate()
    if proc_udev.returncode != 0:
      logging.warn('subprocess call "nmcli %s" bad return code: %i stderr: %s',
                   ' '.join(args), proc_udev.returncode, stderr)
    return stdout

  def IsConnected(self):
    return self.GetStatus() == 'connected'

  def IsDisconnected(self):
    return self.GetStatus() == 'disconnected'

  def BlockUntilConnected(self, timeout=None):
    self._BlockUntil(self.IsConnected, timeout)

  def BlockUntilDisconnected(self, timeout=None):
    self._BlockUntil(self.IsDisconnected, timeout)

  def IsUp(self):
    self.UpdateState()
    return self.state == 'up'

  def IsDown(self):
    return not self.IsUp()

  def _BlockUntil(self, func, timeout=None, poll_interval=.1):
    timer_start = time.time()
    while not func():
      self.UpdateState()
      if timeout and (time.time() - timer_start) > timeout:
        return False
      time.sleep(poll_interval)
    return True

  def BlockUntilUp(self, timeout=None):
    self._BlockUntil(self.IsUp, timeout)

  def BlockUntilDown(self, timeout=None):
    self._BlockUntil(self.is_down, timeout)

  def __del__(self):
    self._dev_fh.close()

