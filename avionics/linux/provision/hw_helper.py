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

"""Helpers for consistently getting a network interface from a serial number."""

import os
import subprocess


def GetNetUdev(interface, field_name):
  """Search udev info of interface for field_name and omit field_name in result.

  Args:
    interface: An ethernet device.
    field_name: A string to "grep" the results through (and remove).

  Returns:
    The first match of field_name from udevadm info command.

  Typical Usage:
    GetSerialDevice('eth0')
  """
  proc_udev = subprocess.Popen(['udevadm', 'info',
                                '/sys/class/net/{}'.format(interface)],
                               stdout=subprocess.PIPE)
  stdout, _ = proc_udev.communicate()
  for line in stdout.split('\n'):
    field_name_index = line.find(field_name)
    if field_name_index >= 0:
      return line[field_name_index+len(field_name):]


def FindEthBySerial(serial_num):
  """Return the network interface with ID_SERIAL_SHORT containing serial_num.

  Lenovo USB network interfaces are easily differentiated using their serial
  numbers, which are written on the bottom and identifiable through udev info.
  Though using MAC addresses is an obvious solution, Lenovo did not print MAC
  addresses on the enclosures.

  Args:
    serial_num: A string with the serial number of the interface.

  Returns:
    An ethernet interface string (e.g. 'eth0') or None.
  """
  for interface in os.listdir('/sys/class/net'):
    if interface.startswith('eth'):
      serial_num_cur = GetNetUdev(interface, ' ID_SERIAL_SHORT=')
      if serial_num_cur and serial_num in serial_num_cur:
        return interface


def GetUdevDevice(dev_basename_match, uevent_line_match,
                  path_sys_char='/sys/dev/char', path_dev='/dev'):
  """Searches for a udev device which matches the device name uevent line.

  This utility function can be used to find a device by idVendor and idProduct
  by matching a line like 'PRODUCT={idVendor}/{idProduct}' with
  uevent_line_match. dev_basename_match is provided to filter by the expected
  device name. Typical Usage: GetUdevDevice('ttyACM', 'PRODUCT=2a19/c06') or
  GetUdevDevice('video', 'PRODUCT=1871/7670').

  Args:
    dev_basename_match: A device name string (the {} of "/dev/{}*").
    uevent_line_match: A string to match on in the uevent file.
    path_sys_char: The path to the character devices in sysfs.
    path_dev: The path to system device nodes.

  Returns:
    None or string containing device name (i.e. /dev/ttyACM0).
  """
  for root, dirs, _ in os.walk(path_sys_char):
    for curdir in [os.path.join(root, subdir) for subdir in dirs]:
      link_name = os.readlink(curdir)
      if os.path.basename(link_name).startswith(dev_basename_match):
        path_uevent = os.path.join(root, link_name, 'device/uevent')
        if os.path.isfile(path_uevent):
          dev_name = os.path.join(path_dev, os.path.basename(link_name))
          with open(path_uevent) as fh:
            for line in fh:
              if uevent_line_match in line and os.path.exists(dev_name):
                return dev_name
  return None

