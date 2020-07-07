#!/usr/bin/python
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


"""This is the tool used to burn new application images to TMS570s on the M600.

   Usage: bootloader_client.py --target $node_name $file_name
   Example: bootloader_client.py --target fc_b fc_application.elf
"""

import argparse
import errno
import logging
import os
import shutil
import socket
import struct
import sys
import tempfile

from makani.avionics.bootloader import generate_image
from makani.avionics.bootloader.firmware import update_server
from makani.avionics.firmware.identity import identity_types
from makani.avionics.network import network_config
from makani.lib.python import c_helpers

hardware_type_helper = c_helpers.EnumHelper('HardwareType', identity_types)
packet_id_helper = c_helpers.EnumHelper('PacketId', update_server)
update_type_helper = c_helpers.EnumHelper('UpdateType', update_server)

logging.basicConfig(level=logging.INFO,
                    format='%(levelname)s: %(message)s')


_RESPONSE_TIMEOUT = 0.1
_CHUNK_SIZE = 1024

# TODO: It may be worthwhile to pull these directly from the C code once
# the bootloader code gets cleaned up a bit.
_PROTO_MAJOR = 0
_PROTO_MINOR = 1
_BOOTLOADER_PORT = 40667
_RESET_PORT = 40668


# TODO: Don't use the file name to determine things about the file.
def GetInfoFromFileName(filename):
  """Returns the app type index and update type, given a filename."""
  basename, extension = filename.rsplit(os.extsep, 1)
  file_info = {}
  file_info['file_name'] = filename
  file_info['file_ext'] = extension
  tokens = basename.split('_')
  while tokens:
    file_info['file_type'] = tokens.pop().upper()
    if file_info['file_type'] in ['APPLICATION', 'BOOTLOADER', 'PARAMS']:
      break
  if not tokens and file_info['file_type'] != 'BOOTLOADER':
    raise ValueError('Unsupported update type.')
  if file_info['file_type'] == 'PARAMS' and extension.upper() != 'BIN':
    raise ValueError('Invalid extension for param file: "%s"' % extension)
  if file_info['file_type'] == 'PARAMS':
    file_info['param_type'] = tokens.pop().upper()

    # Carrier Serial Params take up two tokens
    next_tok = tokens.pop().upper()
    if next_tok == 'CARRIER':
      file_info['param_type'] = next_tok + file_info['param_type']
    else:
      tokens.append(next_tok)

    update_type = file_info['param_type'] + file_info['file_type']
    file_info['update_type'] = ParseUpdateType(update_type)
    if file_info['param_type'] not in ['CALIB', 'CONFIG', 'SERIAL',
                                       'CARRIERSERIAL']:
      raise ValueError('Unrecognized param type: "%s"'
                       % file_info['param_type'])
  else:
    file_info['update_type'] = ParseUpdateType(file_info['file_type'])
  return file_info


def FindShortNameCaseInsensitive(helper, name):
  for _, v in helper:
    short = helper.ShortName(v)
    if short.lower() == name.lower():
      return short
  raise ValueError('%s not found in %s.' % (name, helper.TypeName()))


def ParseHardwareType(name):
  return FindShortNameCaseInsensitive(hardware_type_helper, name)


def ParseUpdateType(name):
  return FindShortNameCaseInsensitive(update_type_helper, name)


def GetTargetInfo(name):
  """Gets info about our target board.

  Args:
    name: The target name (e.g., CS_A).

  Returns:
    {node_name: AioNode enum name.
     node_value: AioNode enum value.
     node_index: e.g. This is the return of IdentityGetIndex().
     ip_address: As packed binary.
    }

  Raises:
    ValueError: if the name of the target looks wrong.
  """
  config = network_config.NetworkConfig()
  node = config.GetAioNode(name.lower())
  return {
      'node_name': node.camel_name, 'node_value': node.enum_value,
      'node_index': node.label_value, 'ip_address': node.ip}


def GetBinarySize(file_name, update_type):
  """Get the size of the update binary.

  For APPLICATION and BOOTLOADER updates, we also check the binary size against
  what we expect it to be, raising an exception if they don't match.

  Args:
    file_name: name of the binary file.
    update_type: one of the shortnames of update_type_helper

  Returns:
    binary size in bytes.
  Raises:
    Exception: if expected binary size does not match actual size.
  """
  file_size = os.stat(file_name).st_size
  expected_size = None
  if update_type in ['Application', 'Bootloader']:
    expected_size = generate_image.GetExpectedBinarySize(update_type)
  if expected_size and expected_size != file_size:
    raise Exception('Unexpected file size %d; expected %d bytes.' %
                    (file_size, expected_size))
  return file_size


def GetUnsignedByte(buf, offset):
  """Get an unsigned byte from a buffer at a supplied offset.

  This returns the same value whether the input is a string or a bytes
  object, coercing the return value of socket.recv to the same type whether
  on python 2 or 3.

  Args:
    buf: A string or bytes object.
    offset: An offset into buf.

  Returns:
    an unsigned byte
  """
  return struct.unpack_from('B', buf, offset)[0]


def GetSignedByte(buf, offset):
  """Get a signed byte from a buffer at a supplied offset.

  This returns the same value whether the input is a string or a bytes
  object, coercing the return value of socket.recv to the same type whether
  on python 2 or 3.

  Args:
    buf: A string or bytes object.
    offset: An offset into buf.

  Returns:
    a signed byte
  """
  return struct.unpack_from('b', buf, offset)[0]


def GetUpdateMessageSet(update_type, proto_high, proto_low, size):
  """Get the update message and the checker that validates its response.

  Args:
    update_type: one of the shortnames of update_type_helper
    proto_high: The top byte of the bootloader protocol revision.
    proto_low: The bottom byte of the bootloader protocol revision.
    size: The number of bytes in the update.

  Returns:
    A tuple giving all the pieces we need to send the message initiating an
    update.

    (message: The update packet.
     checker: A function which, when given a response packet, tells you
              whether it's a correct response to the update packet.
    )
  """

  message = bytearray(1 + 1 + 1 + 1 + 4)
  message[0] = packet_id_helper.Value('kPacketIdStart')
  message[1] = proto_high
  message[2] = proto_low
  message[3] = update_type_helper.Value(update_type)

  struct.pack_into('!l', message, 4, size)
  ready_id = packet_id_helper.Value('kPacketIdReady')
  return (message, lambda x: (GetUnsignedByte(x, 0) == ready_id))


def GetChunkChecker(correct_bytes_received):
  """Creates a packet checker to validate received data-received packets.

  Mostly it will just reject duplicates to packets we've already
  seen acked.

  Args:
    correct_bytes_received: The number of bytes of payload we've sent.

  Returns:
    A function taking a packet and returning a boolean.
  """
  def Checker(data):
    data_received_id = packet_id_helper.Value('kPacketIdDataReceived')
    if GetUnsignedByte(data, 0) != data_received_id:
      return False
    (bytes_received,) = struct.unpack_from('!l', data, 1)
    if bytes_received != correct_bytes_received:
      return False
    return True
  return Checker


def GetChunkMessageSet(data_file, bytes_sent, size, is_last_chunk):
  """Creates a message set for a chunk of data.

  Args:
    data_file: An opened file from which to read the binary to send.
    bytes_sent: How many bytes of data_file we've sent so far.
    size: The overall size of data_file.
    is_last_chunk: Should this be the last chunk of the file?

  Returns:
    A tuple giving all the pieces we need to send a chunk of data:

    (message: The data packet.
     checker: A function which, when given a response packet, tells you
              whether it's a correct response to the supplied data packet.
     bytes_to_send: How many bytes of data went into the data packet.
    )
  """
  bytes_to_send = _CHUNK_SIZE
  if bytes_to_send >= size - bytes_sent:
    bytes_to_send = size - bytes_sent
    assert is_last_chunk
  else:
    assert not is_last_chunk
  message = bytearray(1 + 4 + 1 + bytes_to_send)
  message[0] = packet_id_helper.Value('kPacketIdData')
  struct.pack_into('!l', message, 1, bytes_sent)
  message[5] = is_last_chunk
  view = memoryview(message)
  data_file.readinto(view[6:])
  return (message, GetChunkChecker(bytes_sent + bytes_to_send),
          bytes_to_send)


def SendMessage(sock, message, response_checker, addr):
  while True:
    try:
      sock.sendto(message, addr)
    except socket.error as err:
      if err.errno == errno.EAGAIN:
        continue
      raise err
    sock.settimeout(_RESPONSE_TIMEOUT)
    queue_empty = False
    # Flush the rx queue of all stale messages before retransmitting.
    # This cuts down on the number of stale messages over time, instead of
    # sending one for each one we receive.
    while not queue_empty:
      try:
        data = sock.recv(1024)
        if response_checker(data):
          return data
        sock.settimeout(0)  # Don't block on subsequent recv calls.
      except socket.timeout:
        queue_empty = True
      except socket.error as err:
        if err.errno == errno.EAGAIN:
          queue_empty = True
        else:
          logging.error('Got error %s', str(err))


def SendResetRequest(sock, addr):
  """Send a reset request over sock to addr.

  This will cause the target system to reboot into the bootloader and wait for
  an update.  We send it 5 times just to make sure it gets through.

  Args:
    sock: The transmitting socket.
    addr: The target address.
  """
  reset_message = bytearray(4)
  struct.pack_into('!L', reset_message, 0, 0xde10fe1f)

  sock.sendto(reset_message, addr)
  sock.sendto(reset_message, addr)
  sock.sendto(reset_message, addr)
  sock.sendto(reset_message, addr)
  sock.sendto(reset_message, addr)


def SendStartMessage(sock, update_type, proto_high, proto_low, payload_size,
                     addr, ignore_mismatch):
  """Send a message to the target attempting to initiate an update.

  This assumes that the target system is in the bootloader.

  Args:
    sock: The transmitting socket.
    update_type: one of the shortnames of update_type_helper
    proto_high: The top byte of the bootloader protocol revision.
    proto_low: The bottom byte of the bootloader protocol revision.
    payload_size: The size of the update to be burned.
    addr: The address (ip, port) of the target.
    ignore_mismatch: Whether to ignore mismatches between what the target
                     reports and what we're expecting to find.

  Raises:
    Exception: on any error.

  Returns:
    The hardware type index [enum HardwareType] of the target.
  """
  logging.info('Binary size: %d bytes; target IP: %s.', payload_size, addr[0])
  # TODO: Remove this hack.  We're sending (0,0) and the update server
  # will send back (0,1) if it's a new one, (0,0) if it's an old one.
  (update_msg, update_checker) = GetUpdateMessageSet(
      update_type, 0, 0, payload_size)
  response = SendMessage(sock, update_msg, update_checker, addr)

  found_proto_high = GetUnsignedByte(response, 1)
  found_proto_low = GetUnsignedByte(response, 2)
  # Byte 3 unused (previously used as AppType).
  # Byte 4 unused (previously used as board index).

  if len(response) > 5:
    found_hardware_type = GetSignedByte(response, 5)
    try:
      hardware_type_name = hardware_type_helper.Name(found_hardware_type)
      logging.info('Target hardware type: %s.', hardware_type_name)
    except c_helpers.EnumError:
      logging.error('Target hardware not recognized: %d.', found_hardware_type)
  else:
    raise Exception('Unexpected length received. '
                    'Please update your bootloader.')

  if found_proto_high != proto_high:
    raise Exception(
        'proto_high mismatch: found "%s", expected "%s"' %
        (found_proto_high, proto_high))
  if found_proto_low != proto_low:
    if not ignore_mismatch:
      raise Exception(
          'proto_low mismatch: found "%s", expected "%s"' %
          (found_proto_low, proto_low))
  logging.info('Got an acknowledgement from target; starting upload.')
  return found_hardware_type


def SendFile(sock, file_to_send, file_size, addr):
  bytes_sent_successfully = 0
  while bytes_sent_successfully < file_size:
    last_chunk = int(file_size - bytes_sent_successfully <= _CHUNK_SIZE)
    (chunk_message, chunk_checker, chunk_size) = GetChunkMessageSet(
        file_to_send, bytes_sent_successfully, file_size, last_chunk)
    if (bytes_sent_successfully > 0 and
        bytes_sent_successfully % (1024 * 100) == 0):
      logging.info('Sent %d bytes...', bytes_sent_successfully)
    response = SendMessage(sock, chunk_message, chunk_checker, addr)
    assert GetUnsignedByte(response, 5) == last_chunk
    bytes_sent_successfully += chunk_size


def ParseArguments():
  """Parse command line arguments, validate them, and return them.

  Returns:
    A dict of:
      {'args': argparse arguments, see below,
       'cur_ip': the ip address of the target, as packed binary,
       'cur_node_index': the current node index of the target,
       'cur_node_name': the current node name of the target,
       'new_ip': the new ip address of the target, as packed binary,
       'new_node_index': the new node index of the target,
       'new_node_name': the new node name of the target,
       'update_type': the update type, a shortname of update_type_helper
      }

  Raises:
    RuntimeError: if run from outside the Makani workspace without specifying
      --tms570_bin.
    ValueError: if the binary the user supplied doesn't match the target type.
    ValueError: if user passes --dump_image without a .elf file.
    ValueError: if update is a param type, but the file doesn't end in '.bin'.
    ValueError: if the update type in the filename isn't recognized.
    ValueError: if the update type is 'CalibParams' but we don't see --calib.
    ValueError: if the update type is not 'CalibParams' but we see --calib.
    ValueError: if the update type is 'SerialParams' but we don't see --serial.
    ValueError: if the update type is not 'SerialParams' but we see --serial.
    ValueError: if the update type is 'CarrierSerialParams' but we don't
      see --serial.
    ValueError: if the update type is not 'CarrierSerialParams' but we
      see --serial.
  """
  parser = argparse.ArgumentParser(
      description='Burn an application or parameter set to a board.')
  parser.add_argument(
      '--target', help='board to burn, e.g. MOTOR_PBO or FC_A.', required=True)
  parser.add_argument('file',
                      help='binary to burn, e.g motor_application.elf '
                      'or servo_config_params.bin')
  parser.add_argument('--dump_image', action='store_true',
                      help='Output intermediate .bin file instead of'
                      ' sending it to the device.')
  parser.add_argument('--calib', action='store_true',
                      help='Add this flag to burn calibration parameters.')
  parser.add_argument('--serial', action='store_true',
                      help='Add this flag to burn serial parameters.')
  parser.add_argument('--carrier_serial', action='store_true',
                      help='Add this flag to burn carrier serial'
                      ' parameters.')
  parser.add_argument('--config', action='store_true',
                      help='Add this flag to burn config parameters.')
  parser.add_argument('--bootloader', action='store_true',
                      help='Add this flag to burn a bootloader.')
  parser.add_argument('--override_target',
                      help='Override target identity in bootloader image.')
  parser.add_argument('--force_hardware',
                      help='Burn e.g. an Fc board, rather than an Aio board.\n'
                      'use with argument "new" or "old".')
  parser.add_argument('--ignore_mismatch', action='store_true',
                      help='Ignore mismatch between binary and board app type, '
                      'ip address, etc.')
  # TODO: Allow override of IP address.
  args = parser.parse_args()
  args.application = not (args.calib or args.serial or args.carrier_serial
                          or args.config or args.bootloader)
  if (args.calib + args.serial + args.carrier_serial + args.config +
      args.bootloader + args.application) != 1:
    raise ValueError('Cannot specify more than one update type (calib, serial, '
                     'carrier_serial, config, or bootloader).')
  if args.force_hardware and not ParseHardwareType(args.force_hardware):
    raise ValueError('Unknown hardware type "%s"; please specify a valid '
                     'HardwareType.' % args.force_hardware)

  target_info = GetTargetInfo(args.target)
  file_info = GetInfoFromFileName(os.path.basename(args.file))
  if args.dump_image and not args.file.endswith('.elf'):
    raise ValueError('--dump_image requires an .elf file.')
  if args.calib and file_info['update_type'] != 'CalibParams':
    raise ValueError('That does not look like an calib param file to me.')
  if file_info['update_type'] == 'CalibParams' and not args.calib:
    raise ValueError('If you really want to burn calib params, pass --calib.')
  if args.serial and file_info['update_type'] != 'SerialParams':
    raise ValueError('That does not look like an serial param file to me.')
  if file_info['update_type'] == 'SerialParams' and not args.serial:
    raise ValueError('If you really want to burn serial params, pass --serial.')
  if args.carrier_serial and file_info['update_type'] != 'CarrierSerialParams':
    raise ValueError('That does not look like a carrier serial param'
                     ' file to me.')
  if (file_info['update_type'] == 'CarrierSerialParams'
      and not args.carrier_serial):
    raise ValueError('If you really want to burn carrier serial params,'
                     ' pass --carrier_serial.')
  if args.bootloader and file_info['update_type'] != 'Bootloader':
    raise ValueError('That does not look like a bootloader file to me.')
  if file_info['update_type'] == 'Bootloader' and not args.bootloader:
    raise ValueError(
        'If you really want to burn a bootloader, pass --bootloader.')
  if args.override_target and file_info['update_type'] != 'Bootloader':
    raise ValueError('--override_target only supported with --bootloader.')
  if args.override_target:
    new_target_info = GetTargetInfo(args.override_target)
  else:
    new_target_info = target_info
  logging.info('Attempting to flash %s segment on target %s [%s, index %d].',
               file_info['update_type'], target_info['node_name'],
               target_info['ip_address'], target_info['node_index'])
  logging.info('Flashing file %s.', args.file)
  return {'args': args,
          'cur_ip': target_info['ip_address'],
          'cur_node_index': target_info['node_index'],
          'cur_node_name': target_info['node_name'],
          'new_ip': new_target_info['ip_address'],
          'new_node_index': new_target_info['node_index'],
          'new_node_name': new_target_info['node_name'],
          'file': args.file,
          'update_type': file_info['update_type'],
         }


def GenerateImage(update_type, node_name, ip, hardware_type,
                  directory, elf_file):
  """Create a burnable image from an elf file.

  Args:
    update_type: 'Application' or 'Bootloader'.
    node_name: AioNode name to be assigned to the target.
    ip: Ip address to be assigned to the target.
    hardware_type: hardware type index to be assigned to the target.
    directory: A directory to use for temporary files.
    elf_file: The elf file.

  Returns:
    A tuple of:
      (the image file, open for reading,
       the size of the image in bytes
      )
  """
  config = {
      'aio_node': node_name,
      'ip_address': ip,
      'hardware_type': hardware_type_helper.Name(hardware_type),
  }
  bin_file = os.path.join(directory, 'image.bin')
  expected_size = generate_image.GetExpectedBinarySize(update_type)

  assert update_type in ['Application', 'Bootloader']
  if update_type == 'Application':
    generate_image.GenerateApplicationImage(config, elf_file, bin_file)
  elif update_type == 'Bootloader':
    generate_image.GenerateBootloaderImage(config, elf_file, bin_file)
  return (bin_file, expected_size)


def SetUpSocket(ip, bootloader_port, response_timeout):
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.settimeout(response_timeout)
  target_addr = (ip, bootloader_port)
  return (sock, target_addr)


def Main():
  """Update a TMS570 board with the supplied binary."""

  # Reopen standard output without buffering so that output gets to
  # the parallel bootloader expeditiously.  The "-u" shebang option to
  # Python is not available due to the bash stubs instituted by Bazel.
  sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)

  # Handle input arguments.
  parsed_args = ParseArguments()
  args = parsed_args['args']
  update_type = parsed_args['update_type']
  temp_dir = None
  file_name = parsed_args['file']
  if file_name.endswith('.bin'):
    binary_path = file_name
    binary_size = GetBinarySize(file_name, update_type)
  elif file_name.endswith('.elf'):
    binary_size = generate_image.GetExpectedBinarySize(update_type)
  else:
    raise ValueError('Unknown file type; expected a .bin or .elf file.')

  (sock, target_addr) = SetUpSocket(
      parsed_args['cur_ip'], _BOOTLOADER_PORT, _RESPONSE_TIMEOUT)

  # The bootloader app needs no reset AND doesn't want one.
  if update_type != 'Bootloader':
    SendResetRequest(sock, (parsed_args['cur_ip'], _RESET_PORT))

  hardware_type = SendStartMessage(sock, update_type, _PROTO_MAJOR,
                                   _PROTO_MINOR, binary_size, target_addr,
                                   args.ignore_mismatch)
  if args.force_hardware is not None:
    hardware_short_name = ParseHardwareType(args.force_hardware)
    hardware_type = hardware_type_helper.Value(hardware_short_name)
    logging.warning('Overriding target hardware type as: %s.',
                    hardware_type_helper.Name(hardware_type))
  elif hardware_type == hardware_type_helper.Value('kHardwareTypeUnknown'):
    raise ValueError('\nPlease specify the target hardware type with '
                     '--force_hardware=TYPE.')
  if file_name.endswith('.elf'):
    temp_dir = tempfile.mkdtemp()
    (binary_path, binary_size) = GenerateImage(update_type,
                                               parsed_args['new_node_name'],
                                               parsed_args['new_ip'],
                                               hardware_type,
                                               temp_dir,
                                               file_name)
  if args.dump_image:
    # Copy binary output file to allow debugging or programming without
    # objcopy.
    dump_path = file_name.rsplit(os.extsep, 1)[0] + '.bin'
    shutil.copyfile(binary_path, dump_path)
    logging.info('Wrote %s', dump_path)
  else:
    with open(binary_path, 'rb') as binary_file:
      SendFile(sock, binary_file, binary_size, target_addr)
    logging.info('Successfully transferred %d bytes; cleaning up.', binary_size)

  # Clean up.
  if temp_dir:
    shutil.rmtree(temp_dir)


if __name__ == '__main__':
  Main()
