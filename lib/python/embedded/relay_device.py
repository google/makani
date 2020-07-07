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

"""Utilities for controlling relay board over USB and ethernet."""
import contextlib
import logging
import re
import socket
import time

import pyudev
import serial

# TODO: Rethink the args support, order is different!


class RelayDevice(object):
  """A base-class for interfacing to USB and ethernet relay boards."""

  def __init__(self, device, channels, one_indexed=False, invert=False,
               mask=None, default=0):
    self._one_indexed = one_indexed  # Flag indicates indexing starts at 1.
    self._device = device
    self._channels = channels
    self._invert = invert
    channel_mask = ((1 << self._channels) - 1) << self._one_indexed
    if mask:
      self._mask = mask & channel_mask
    else:
      self._mask = channel_mask
    self._shadow = default & channel_mask

  def _PrintChange(self, bitmask):
    change = self._shadow ^ bitmask
    for relay in range(0, self._channels + self._one_indexed):
      if change & (1 << relay):
        if bitmask & (1 << relay):
          logging.info('Power on relay %d.', relay)
        else:
          logging.info('Power off relay %d.', relay)

  def _CalcBitmask(self, bitmask):
    """Return a bitmask filtered by self._mask.

    Calculate and return bitmask after filtering through self._mask and
    retaining the states in self._shadow for any bits not set in self._mask.

    Args:
      bitmask: The bitmask to be filtered by self._mask.

    Returns:
      A combination of bitmask filtered through self._mask and self._shadow for
      bits unmasked by self._mask.
    """

    return (bitmask & self._mask) | (self._shadow & ~self._mask)

  def GetMask(self):
    """Query the device for the current bitmask, update self._shadow.

    The update to self._shadow is filtered through self._mask. Any bits which
    are not in self._mask will retain their state through this operation. This
    is useful for power-up states when self._shadow is set by default and does
    not match the actual state of the bits before running a SetMask.

    Returns:
      The bitmask state of the relays, except for masked bits which retain state
      from the initialized state defined by kwarg default.
    """

    self._shadow = self._CalcBitmask(self._GetMask())
    return self._shadow

  def SetMask(self, bitmask):
    """Set the device relay states according to bitmask.

    The bitmask argument is filtered through self._mask. This method calls
    self._SetMask to perform the actual hardware communication.

    Args:
      bitmask: The desired power state for each relay.

    Returns:
      True upon success, else False.
    """

    bitmask = self._CalcBitmask(bitmask)
    if self._shadow ^ bitmask:
      self._PrintChange(bitmask)
      if self._invert:
        bitmask = ~bitmask
      if self._SetMask(bitmask):
        self._shadow = bitmask
        if self._invert:
          self._shadow = ~bitmask
        return True
    return False

  def PowerOn(self, port):
    if port is not None:
      return self.SetMask(self._shadow | (1 << port))

  def PowerOff(self, port):
    if port is not None:
      return self.SetMask(self._shadow & ~(1 << port))

  def _SetMask(self, bitmask):
    """Communicates with hardware to update the relay states to bitmask."""
    raise NotImplementedError('Subclass should implement this method.')

  def _GetMask(self):
    """Communicates with hardware to retrieve the state of the relays."""
    raise NotImplementedError('Subclass should implement this method.')


@contextlib.contextmanager
def Socket(device, port, timeout=None):
  s1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s1.settimeout(timeout)
  s1.connect((device, port))
  yield s1
  s1.close()


class RobotElectronicsRelayDevice(RelayDevice):
  """Interface to Robot Electronics ETH8020 Ethernet Relay Module."""

  def __init__(self, port=17494, password='', timeout=3, *args, **kwargs):
    kwargs.update({'one_indexed': True})
    super(RobotElectronicsRelayDevice, self).__init__(*args, **kwargs)
    self._password = password
    self._port = port
    self._timeout = timeout
    self._retries = 2
    self._try_timeout = float(timeout) / (self._retries + 1)

  def _WithSocketRetry(self, function, *args, **kwargs):
    """Wrapper for making a socket connection to self._device and retrying.

    Connects to self._device, checks device type and authenticates before
    running function(*args, **kwargs). Will re-attempt the entire procedure if a
    socket.timeout occurs. This is 90% of a decorator, but pylint does not
    appreciate decorators which access private class members.

    Args:
      function: The method to run which uses self._sock.
      *args: Arguments are passed to function.
      **kwargs: Keyword arguments are passed to function.

    Returns:
      The result of function(*args, **kwargs).

    Raises:
      socket.timeout: Socket timeouts.
    """
    for retry_count in range(self._retries + 1):
      try:
        with Socket(self._device,
                    self._port,
                    self._try_timeout) as self._sock:
          assert self._GetModuleInfo()[0] == '\x15', 'Device is not ETH8020.'
          if not self._Unlock():
            raise RuntimeError('Authentication failed.')
          return function(*args, **kwargs)
      except socket.timeout:
        if retry_count < self._retries:
          logging.warn('Socket timeout, retrying %i more times.',
                       self._retries - retry_count)
        else:
          raise

  def _Unlock(self):
    self._BufferedWrite('\x7a')
    lock_status = self._BufferedRead(1)
    if lock_status == '\x00':
      if not self._password:
        logging.error('Password required!')
        return False
      logging.info('Authenticating with password.')
      self._BufferedWrite('\x79{}'.format(self._password))
      if self._AuthSuccess():
        return True
      else:
        logging.warn('Incorrect password.')
        return False
    elif lock_status == '\xFF':
      # Password is not enabled!
      return True
    else:
      logging.debug('Logout timer was at %ss, reset to 30s.', ord(lock_status))
      return True

  def _AuthSuccess(self):
    return self._BufferedRead(1) == '\x01'

  def _GetModuleInfo(self):
    self._BufferedWrite('\x10')
    return self._BufferedRead(3)

  def _BufferedWrite(self, buf):
    self._sock.sendall(buf)
    logging.debug('Wrote: %s', [hex(ord(b)) for b in buf])

  def _BufferedRead(self, read_len):
    buf = ''
    timer_start = time.time()
    while len(buf) < read_len:
      if (time.time() - timer_start) > self._try_timeout:
        raise socket.timeout('No data received.')
      buf += self._sock.recv(read_len - len(buf))
    logging.debug('Read: %s', [hex(ord(b)) for b in buf])
    return buf

  def _GetSetCommand(self, bitmask):
    assert not (bitmask & 1), 'Robot Electronics relays are 1 indexed.'
    bitmask >>= 1
    bitmask_string = '\x23'
    for _ in range(3):
      bitmask_string += chr(bitmask & 0xFF)
      bitmask >>= 8
    return bitmask_string

  def _GetMask(self):
    return self._WithSocketRetry(self.__GetMask)

  def __GetMask(self):
    self._BufferedWrite('\x24')
    bitmask = 0
    for i, char in enumerate(self._BufferedRead(3)):
      bitmask += ord(char) << (8 * i)
    return bitmask << 1

  def _SetMask(self, bitmask):
    return self._WithSocketRetry(self.__SetMask, bitmask=bitmask)

  def __SetMask(self, bitmask):
    self._BufferedWrite(self._GetSetCommand(bitmask))
    return self._BufferedRead(1) == '\x00'


def _NumatoOpenPort(device_name):
  return serial.Serial(device_name, 19200, timeout=0.1)


def _NumatoWrite(device_name, command):
  with _NumatoOpenPort(device_name) as f:
    f.write('\r' + command + '\r')
    f.flush()


def _NumatoRead(device_name, command):
  r"""Send a command, then wait for response.

  Command: id get
  Response: \n\r>id get\n\r00000000\n\r>

  Args:
    device_name: A fully qualified device descriptor path.
    command: A Numato command.

  Returns:
    The command response or None.
  """
  with _NumatoOpenPort(device_name) as f:
    f.write('\r' + command + '\r')
    f.flush()
    pattern = r'.*[\r\n]+(>' + command + r')[\r\n]+(.*)[\r\n]+>'
    match = re.match(pattern, f.readall())
    if match:
      return match.expand(r'\2')


def _NumatoUsbDetect():
  context = pyudev.Context()
  tty_devices = context.list_devices(subsystem='tty')
  for dev in tty_devices.match_property('ID_VENDOR_ID', '2a19'):
    numato_id = _NumatoRead(dev.device_node, 'id get')
    yield dev.device_node, numato_id


def _NumatoGetDeviceFromId(target_numato_id):
  for dev_path, numato_id in _NumatoUsbDetect():
    if target_numato_id == numato_id:
      return dev_path
  raise RuntimeError('No Numato device found with id: '
                     '{}'.format(target_numato_id))


class NumatoRelayDevice(RelayDevice):
  """Interface to Numato USB Relay Modules."""

  def __init__(self, device, *args, **kwargs):
    if not device.startswith('/dev/'):
      device = _NumatoGetDeviceFromId(device)
    super(NumatoRelayDevice, self).__init__(device=device, *args, **kwargs)

  def _GetSetCommand(self, bitmask):
    if self._invert:
      value = ~bitmask
    else:
      value = bitmask
    value &= (1 << self._channels) - 1
    hex_str = hex(value)[2:].zfill((self._channels + 3)/4)
    return 'relay writeall {:s}'.format(hex_str)

  def _GetMask(self):
    if self._device:
      mask = _NumatoRead(self._device, 'relay readall')
      return int(mask, 16)

  def _SetMask(self, bitmask):
    if self._device:
      _NumatoWrite(self._device, self._GetSetCommand(bitmask))
      time.sleep(0.1)  # Give time for relays to respond.
      return True


if __name__ == '__main__':
  b1 = RobotElectronicsRelayDevice(device='192.168.0.200', channels=20)
  b1.GetMask()
  b1.PowerOff(1)
  b1.PowerOn(1)
