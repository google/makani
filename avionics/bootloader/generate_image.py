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

"""Generate an application or bootloader image to program."""

import binascii
import ctypes
import os
import subprocess
import sys

import gflags
import makani
from makani.avionics.firmware.identity import identity_types
from makani.avionics.firmware.startup import pack_ldscript_types as ldscript
from makani.avionics.network import network_config
from makani.lib.python import c_helpers


hardware_type_helper = c_helpers.EnumHelper('HardwareType', identity_types)

_FLASH_SIZE = 0x140000
_APPLICATION_START = 0x40000
_BOOT_CONFIG_VERSION_FLAG = 0xDE100000
_BOOT_CONFIG_VERSION = 0x00000001


# Accessing the protected member _fields_ appears to be the only way of
# getting type information short of parsing the repr, which seems worse.
def _GetFieldInfo(obj):
  offset = 0
  info = {}
  for name, f in obj._fields_:  # pylint: disable=protected-access
    size = ctypes.sizeof(f)
    info[name] = {'size': size, 'offset': offset}
    offset += size
  return info


def _ConvertElfToBin(elf_name, bin_name):
  toolchain = os.path.join(
      makani.HOME, 'third_party_toolchains/gcc_arm_none_eabi')
  objcopy = toolchain + '/bin/arm-none-eabi-objcopy'
  subprocess.check_call([objcopy, '-O', 'binary', '-S', elf_name, bin_name])


def _WriteAppConfig(app_config, input_name, output_name):
  """Write AppConfig block to an application firmware image.

  Args:
    app_config: An object of type ldscript.AppConfig.
    input_name: Input binary file name (output of objdump).
    output_name: Output binary file name (may be the same as input_name).

  Raises:
    Exception: If the input binary file does not match the expected size.
  """
  block_size = ctypes.sizeof(ldscript.AppConfig)
  block_info = _GetFieldInfo(ldscript.AppConfig)
  expected_size = GetExpectedBinarySize('application')
  with open(input_name, 'rb') as f:
    input_data = f.read(expected_size + 1)
  if len(input_data) != expected_size:
    raise Exception(
        'Expected an application firmware image {} bytes long, '
        'found {} bytes.'.format(expected_size, len(input_data)))
  with open(output_name, 'wb') as f:
    f.write(input_data[:-block_size])
    # Write CRC32 of application only (this value should be consistent between
    # all nodes of the same type).
    crc = binascii.crc32(input_data[:-block_size])
    app_config.crc_app_only = crc
    # Write CRC32 of application + AppConfig (this value may not be consistent
    # between all nodes of the same type).
    crc_offset = block_info['crc_app_config']['offset']
    packed = bytearray(c_helpers.Pack(app_config))
    app_config.crc_app_config = binascii.crc32(packed[:crc_offset], crc)
    f.write(c_helpers.Pack(app_config))


def _WriteBootConfig(boot_config, input_name, output_name):
  """Write BootConfig block to a boot firmware image.

  Args:
    boot_config: An object of type ldscript.BootConfig.
    input_name: Input binary file name (output of objdump).
    output_name: Output binary file name (may be the same as input_name).

  Raises:
    Exception: If the input binary file does not match the expected size.
  """
  block_size = ctypes.sizeof(ldscript.BootConfig)
  block_info = _GetFieldInfo(ldscript.BootConfig)
  expected_size = GetExpectedBinarySize('bootloader')
  with open(input_name, 'rb') as f:
    input_data = f.read(expected_size + 1)
  if len(input_data) != expected_size:
    raise Exception(
        'Expected a bootloader firmware image {} bytes long, '
        'found {} bytes.'.format(expected_size, len(input_data)))
  with open(output_name, 'wb') as f:
    f.write(input_data[:-block_size])
    crc = binascii.crc32(input_data[:-block_size])
    crc_offset = block_info['crc']['offset']
    packed = bytearray(c_helpers.Pack(boot_config))
    boot_config.crc = binascii.crc32(packed[:crc_offset], crc)
    f.write(c_helpers.Pack(boot_config))


def GetExpectedBinarySize(file_type):
  """Get the expected binary size (output from objdump).

  Args:
    file_type: Specify 'application' or 'bootloader'.

  Returns:
    The expected binary size for the given file_type.

  Raises:
    ValueError: For an invalid file_type.
  """
  file_type = file_type.lower()
  if file_type == 'application':
    return _FLASH_SIZE - _APPLICATION_START
  elif file_type == 'bootloader':
    return _APPLICATION_START
  else:
    raise ValueError('Unknown file type: %s.', file_type)


def GenerateApplicationImage(app_info, elf_file, bin_file):
  """Generate an application image given an elf file.

  Args:
    app_info: A dictionary containing the application configuration. In the
        current implementation, the application configuration only requires
        the aio_node (specified as a full or short name).
    elf_file: Full path to input elf file.
    bin_file: Full path to output bin file.
  """
  aio_node = network_config.NetworkConfig().GetAioNode(app_info['aio_node'])
  _ConvertElfToBin(elf_file, bin_file)
  app_config = ldscript.AppConfig()
  app_config.aio_node = aio_node.enum_value
  app_config.node_index = aio_node.label_value
  _WriteAppConfig(app_config, bin_file, bin_file)


def GenerateBootloaderImage(boot_info, elf_file, bin_file):
  """Generate a bootloader image given an elf file.

  Args:
    boot_info: A dictionary containing the bootloader configuration. In the
        current implementation, possible options include
        'aio_node' (defaults to kAioNodeUnknown),
        'ip_address' (defaults the address corresponding to the kAioNode), and
        'hardware_type' (defaults to kHardwareTypeUnknown).
    elf_file: Full path to input elf file.
    bin_file: Full path to output bin file.
  """
  _ConvertElfToBin(elf_file, bin_file)

  # Determine IP address from the given information.
  if boot_info.get('ip_address'):
    ip = int(boot_info['ip_address'].split('.')[3])
  elif boot_info.get('aio_node'):
    aio_node = network_config.NetworkConfig().GetAioNode(boot_info['aio_node'])
    ip = aio_node.ip_octet
  else:
    ip = network_config.NetworkConfig().GetAioNode('unknown').ip_octet

  # Determine hardware type.
  if not boot_info.get('hardware_type'):
    boot_info['hardware_type'] = 'kHardwareTypeUnknown'
  hardware_type = hardware_type_helper.Value(boot_info['hardware_type'])

  boot_config = ldscript.BootConfig()
  boot_config.version = _BOOT_CONFIG_VERSION_FLAG | _BOOT_CONFIG_VERSION
  boot_config.ip_address[0] = 192
  boot_config.ip_address[1] = 168
  boot_config.ip_address[2] = 1
  boot_config.ip_address[3] = ip
  boot_config.mac_address[0] = 0x02
  boot_config.mac_address[1] = 0x00
  boot_config.mac_address[2] = 0x00
  boot_config.mac_address[3] = 0x00
  boot_config.mac_address[4] = 0x00
  boot_config.mac_address[5] = ip
  boot_config.hardware_type = hardware_type
  boot_config.unused = 0  # Previously used as AppType.
  boot_config.unused_2 = 0  # Previously used as AIO node index.
  _WriteBootConfig(boot_config, bin_file, bin_file)


def Main(argv):
  """Script main entry point."""

  gflags.DEFINE_string('bin_file', None,
                       'Full path to output binary file.')
  gflags.DEFINE_string('elf_file', None,
                       'Full path to input elf file.')
  gflags.DEFINE_string('aio_node', 'kAioNodeUnknown',
                       'AioNode enumeration name or short name of node to '
                       'encode in the bin file.')
  gflags.DEFINE_string('ip_address', None,
                       'IP address of node to encode in bin file.')
  gflags.DEFINE_string('hardware_type', 'kHardwareTypeUnknown',
                       'Hardware type of node to encode in bin file.')
  gflags.DEFINE_string('file_type', None,
                       'Specify "application" or "bootloader".')
  gflags.MarkFlagAsRequired('bin_file')
  gflags.MarkFlagAsRequired('elf_file')
  gflags.MarkFlagAsRequired('file_type')
  flags = gflags.FLAGS

  try:
    argv = flags(argv)
  except gflags.FlagsError, e:
    print '%s\nUsage: %s ARGS\n%s' % (e, sys.argv[0], flags)
    sys.exit(1)

  # Specify default configuration.
  config = {
      'aio_node': flags.aio_node,
      'hardware_type': flags.hardware_type,
  }

  # Specify IP address to override default mapping.
  if flags.ip_address:
    if flags.ip_address.startswith('192.168.1.'):
      config['ip_address'] = flags.ip_address
    else:
      raise ValueError('Invalid AIO network IP address: %s', flags.ip_address)

  if flags.file_type.lower() == 'application':
    GenerateApplicationImage(config, flags.elf_file, flags.bin_file)
  elif flags.file_type.lower() == 'bootloader':
    GenerateBootloaderImage(config, flags.elf_file, flags.bin_file)
  else:
    raise ValueError('Unknown file type: %s.', flags.file_type)


if __name__ == '__main__':
  Main(sys.argv)
