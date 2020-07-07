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


"""A command-line tool for using J-Link JTAG programmers."""

import logging
import os
import re
import sys
import tempfile

import gflags
import makani
from makani.avionics.bootloader import generate_image
from makani.avionics.common import aio
from makani.avionics.linux.provision import process_helper
from makani.lib.python import c_helpers

APPLICATION_LOCATION = '0x40000'
BOOTLOADER_LOCATION = '0x0'
CALIB_LOCATION = '0xF0204000'
CONFIG_LOCATION = '0xF0200000'
SERIAL_LOCATION = '0xF020C000'


def _JlinkCheck(func):
  def Wrapper(*args, **kwargs):
    try:
      return func(*args, **kwargs)
    except OSError:
      logging.error('JLinkExe error: %s', sys.exc_info()[1])
      return (-1, '', 'JLinkExe not installed or not in system path!\n'
                      'Go to https://www.segger.com/jlink-software.html and'
                      ' download the "Software and documentation pack."')
  return Wrapper


def _Jlink(command, speed='4000'):
  """A wrapper for running commands in JlinkExe.

  JlinkExe can take commands directly through an interactive prompt, or can run
  them from a pipe. JlinkExe will always return 0, so this wrapper provides an
  output processing mechanism for overriding the return code.

  Args:
    command: A string of commands separated by newlines to run at the JlinkExe
        command prompt.
    speed: Target interface speed kHz for JLinkExe. Supports 'auto' and
        'adaptive'.

  Returns:
    A tuple of the return code, the stdout string, and the stderr string.
  """
  settings_file = os.path.join(makani.HOME, 'avionics', 'bootloader',
                               'jlink_tms570_settings.txt')
  with tempfile.TemporaryFile() as stdin:
    stdin.write(command)
    stdin.seek(0)
    logging.info('Running JLinkExe with commands: %s',
                 command.encode('string_escape'))
    jlink_command = ['JLinkExe', '-SettingsFile', settings_file, '-Device',
                     'tms570ls1227', '-JTAGConf', '-1,-1', '-Si', 'JTAG',
                     '-Speed', str(speed)]
    logging.debug('Running %s', ' '.join(jlink_command).encode('string_escape'))
    jlink_runner = _JlinkCheck(process_helper.RunProcess)
    return_code, stdout, stderr = jlink_runner(jlink_command, stdin_pipe=stdin)
  logging.info('JLinkExe returned %i', return_code)
  return return_code, stdout, stderr


# pylint: disable=unused-argument
def _FlashSuccess(return_code, stdout, stderr):
  new_return_code = 1
  if (re.search('\nProgramming flash[^\n]*100%] Done.\n', stdout) or
      re.search('Flash contents already match\n', stdout)):
    new_return_code = 0
  logging.info('Overridding return code %i -> %i', return_code, new_return_code)
  return new_return_code, stdout, stderr


# pylint: disable=unused-argument
def _FlashEraseSuccess(return_code, stdout, stderr):
  new_return_code = 1
  if re.search('\nErasing flash[^\n]*100%] Done.\n', stdout):
    new_return_code = 0
  logging.info('Overridding return code %i -> %i', return_code, new_return_code)
  return new_return_code, stdout, stderr


def JlinkErase(speed=4000):
  """Erase all flash memory using Jlink."""
  return _FlashEraseSuccess(*_Jlink(command='erase\nexit\n', speed=speed))


def JlinkProgram(filename, location, speed=4000):
  """Program filename into memory location "location" using Jlink."""
  assert os.path.isfile(filename)

  result = _FlashSuccess(*_Jlink(
      command='loadfile {} {}\nexit\n'.format(filename, location),
      speed=speed))
  return result


def JlinkProgramApplicationBin(bin_filename, speed=4000):
  return JlinkProgram(bin_filename, location=APPLICATION_LOCATION, speed=speed)


def JlinkProgramApplicationElf(target_snake, hardware_type_snake, elf_file,
                               speed=4000):
  """Program an elf_file application on a node of hardware_type using Jlink."""
  target_camel = c_helpers.SnakeToCamel(target_snake)
  hardware_type_camel = c_helpers.SnakeToCamel(hardware_type_snake)

  with tempfile.NamedTemporaryFile(suffix='.bin') as bin_file:
    node_info = {
        'aio_node': target_camel,
        'hardware_type':
            generate_image.hardware_type_helper.Name(hardware_type_camel)}
    assert (node_info['hardware_type'] in
            generate_image.hardware_type_helper.Names())
    generate_image.GenerateApplicationImage(node_info, elf_file,
                                            bin_file.name)
    return JlinkProgramApplicationBin(bin_file.name, speed)


def JlinkProgramBootloaderBin(bin_filename, speed=4000):
  return JlinkProgram(bin_filename, location=BOOTLOADER_LOCATION, speed=speed)


def JlinkProgramBootloaderElf(target_snake, hardware_type_snake, elf_file,
                              speed=4000):
  """Program an elf_file bootloader on a node of hardware_type using Jlink."""
  target_camel = c_helpers.SnakeToCamel(target_snake)
  hardware_type_camel = c_helpers.SnakeToCamel(hardware_type_snake)

  with tempfile.NamedTemporaryFile(suffix='.bin') as bin_file:
    node_info = {
        'aio_node': target_camel,
        'hardware_type': generate_image.hardware_type_helper.Name(
            hardware_type_camel)
    }
    assert (node_info['hardware_type'] in
            generate_image.hardware_type_helper.Names())
    generate_image.GenerateBootloaderImage(node_info, elf_file, bin_file.name)
    return JlinkProgramBootloaderBin(bin_file.name, speed)


def JlinkProgramSerialBin(bin_filename, speed=4000):
  return JlinkProgram(bin_filename, location=SERIAL_LOCATION, speed=speed)


def JlinkProgramCalibBin(calib_filename, speed=4000):
  return JlinkProgram(calib_filename, location=CALIB_LOCATION, speed=speed)


def JlinkProgramConfigBin(config_filename, speed=4000):
  return JlinkProgram(config_filename, location=CONFIG_LOCATION, speed=speed)


def _ValidFileArg(filename, file_extension=''):
  return filename.endswith(file_extension) and os.path.isfile(filename)


def ParseArguments(argv):
  """Parse the arguments and do sanity checks."""

  targets = [c_helpers.CamelToSnake(node) for node in
             aio.aio_node_helper.ShortNames()]
  hardware_types = ['aio', 'motor', 'cs']

  flags = gflags.FLAGS

  gflags.DEFINE_bool('application', False, 'Flash an application bin.')
  gflags.DEFINE_bool('bootloader', False, 'Flash a bootloader bin.')
  gflags.DEFINE_bool('calib', False, 'Flash calibration parameters.')
  gflags.DEFINE_bool('config', False, 'Flash configuration parameters.')
  gflags.DEFINE_bool('erase', False, 'Erase all flash memory.')
  gflags.DEFINE_bool('serial', False, 'Flash a serial bin file.')
  gflags.DEFINE_integer('speed', 4000, 'JTAG Speed (kHz).')
  gflags.DEFINE_enum('hardware_type', None, hardware_types, 'Hardware type.')
  gflags.DEFINE_enum('target', None, targets, 'Target node to program')
  gflags.DEFINE_integer('retry', 3,
                        'How many retries to attempt during failures.')
  gflags.DEFINE_string('file', None, 'The file.')
  try:
    argv = flags(argv)
    if (flags.application or flags.bootloader or flags.calib or flags.config or
        flags.serial):
      if not flags.file:
        raise gflags.FlagsError('You must specify --file with --application, '
                                '--bootloader,  --calib, --config or --serial.')
      if _ValidFileArg(flags.file, '.elf'):
        file_type = 'elf'
      elif _ValidFileArg(flags.file, '.bin'):
        file_type = 'bin'
      else:
        raise gflags.FlagsError(
            'Unknown file type: "{}", expected .bin or .elf'.format(flags.file))
    elif not flags.erase:
      raise gflags.FlagsError('Invalid argument combination! Pick one of: '
                              '--application, --bootloader, --calib, --config '
                              '--erase or --serial')
    if flags.bootloader or flags.application:
      if file_type == 'elf':
        if not flags.target:
          raise gflags.FlagsError('You must specify --target when using '
                                  '--bootloader or --application with .elf '
                                  'files.')
        if not flags.hardware_type:
          raise gflags.FlagsError('You must specify a hardware_type.')
    elif flags.calib or flags.config or flags.serial:
      if file_type != 'bin':
        raise gflags.FlagsError('You must specify a bin file for this action.')

  except gflags.FlagsError, e:
    print e, flags
    sys.exit(1)

  # pylint: disable=invalid-name
  RetryThrice = process_helper.RunProcessRetry(flags.retry)
  try:
    if flags.application:
      if file_type == 'elf':
        RetryThrice(JlinkProgramApplicationElf)(
            flags.target,
            flags.hardware_type,
            flags.file,
            speed=flags.speed)
      elif file_type == 'bin':
        RetryThrice(JlinkProgramApplicationBin)(flags.file)
    elif flags.bootloader:
      if file_type == 'elf':
        RetryThrice(JlinkProgramBootloaderElf)(
            flags.target,
            flags.hardware_type,
            flags.file,
            speed=flags.speed)
      elif file_type == 'bin':
        RetryThrice(JlinkProgramBootloaderBin)(flags.file, speed=flags.speed)
    elif flags.calib:
      RetryThrice(JlinkProgramCalibBin)(flags.file, speed=flags.speed)
    elif flags.config:
      RetryThrice(JlinkProgramConfigBin)(flags.file, speed=flags.speed)
    elif flags.serial:
      RetryThrice(JlinkProgramSerialBin)(flags.file, speed=flags.speed)
    elif flags.erase:
      RetryThrice(JlinkErase)()
  except RuntimeError:
    sys.exit(1)


if __name__ == '__main__':
  logging.basicConfig(
      format='%(levelname)s:%(name)s:%(message)s',
      datefmt='%Y-%m-%d %H:%M:%S',
      level=logging.INFO)
  ParseArguments(sys.argv)

