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


"""Handle details of most programming tasks."""

import argparse
import logging
import os
import subprocess
import sys
import tempfile

import makani
from makani.avionics.bootloader import jlink_client
from makani.avionics.bootloader import parallel_bootloader
from makani.avionics.bootloader import system_config
from makani.avionics.firmware.params import codec
from makani.avionics.network import aio_node
from makani.avionics.network import network_config
from makani.avionics.network import node_locations
from makani.lib.bazel import bazel_util
from makani.lib.python import c_helpers

aio_node_helper = c_helpers.EnumHelper('AioNode', aio_node)

_NETWORK_CONFIG = network_config.NetworkConfig()
SERIAL_PARAM_FILE_TEMPLATE = os.path.join(makani.HOME, 'avionics', 'firmware',
                                          'serial', '{}_serial_params.yaml')


def _Find(directory, name):
  matches = []
  for dirpath, _, filenames in os.walk(directory, followlinks=True):
    for f in filenames:
      if f == name:
        matches.append(os.path.join(dirpath, f))
  return matches


def _FindUniqueFile(directory, variant, prefix, suffix, node_name):
  """Find a unique file matching constraints in the specified directory.

  Files are expected to be of the form variant_suffix, or
  variant_prefix_suffix, with provided prefix or a prefix generated
  from node_name.

  Returns:
    A unique file if exactly one matching file exists.

  Args:
    directory: Search base directory.
    variant: Required start of the file name.
    prefix: Node prefix optionally used for convenient matching.
    suffix: Required end of the file name.
    node_name: Name of the node being serviced.  Used in lieu of the prefix.

  Raises:
    RuntimeError: Unable to search or there was not one unique file.
  """
  names = set(['_'.join([variant, suffix])])
  if prefix:
    names.add('_'.join([variant, prefix, suffix]))
  elif node_name:
    # If no explicit prefix, attempt to be clever and use the beginnings of
    # node name as prefix.
    tokens = node_name.split('_')
    for i in range(len(tokens)):
      names.add('_'.join([variant] + tokens[:i + 1] + [suffix]))
  files = set()
  for name in names:
    files.update(_Find(directory, name))
  if len(files) > 1:
    sys.stderr.write('Pattern matches the following files:\n%s\n\n' %
                     '\n'.join(['  * %s' % os.path.relpath(f, directory)
                                for f in files]))
    raise RuntimeError('Found multiple bin files for patterns %s.'
                       % ', '.join(names))
  if not files:
    raise RuntimeError('No match for patterns %s.' % ', '.join(names))
  return files.pop()


def _GetProgramElf(directory, node, segment):
  """Returns the elf file for this node for the specified memory segment."""

  bin_file = None
  if segment == 'bootloader':
    bin_file = node.bootloader_path
  elif segment == 'bootloader_application':
    bin_file = node.bootloader_application_path
  elif segment == 'application':
    bin_file = node.application_path
  else:
    raise ValueError('Invalid segment %s.' % segment)
  return os.path.join(directory, 'avionics', bin_file)


def _GetTargetSelection(args, explicit_targets=None):
  """Find all TMS570 nodes matching constraints."""
  target_list = explicit_targets
  if not target_list:
    target_list = args.target

  available_nodes = set(n for n in _NETWORK_CONFIG.aio_nodes
                        if n.tms570_node or n.snake_name == 'unknown')
  selected_nodes = set()

  if args.location:
    selected_nodes = set()
    nodes_by_location = node_locations.GetNodeLocations(_NETWORK_CONFIG)
    for loc in args.location:
      if loc in nodes_by_location.keys():
        selected_nodes.update(nodes_by_location[loc])
      else:
        raise ValueError('Invalid node location %s.' % loc)
    selected_nodes.intersection_update(available_nodes)
    available_nodes = selected_nodes

  if args.prefix:
    selected_nodes = set()
    for node in available_nodes:
      if node.tms570_node and node.snake_name.startswith(args.prefix):
        selected_nodes.add(node)
    if not selected_nodes:
      raise ValueError('Found no nodes for prefix %s.' % args.prefix)
    available_nodes = selected_nodes

  if target_list:
    # Retain input ordering for target list.
    selected_nodes = []
    for target in target_list:
      if args.prefix and not target.startswith(args.prefix):
        target = '%s_%s' % (args.prefix, target)
      node = _NETWORK_CONFIG.GetAioNode(target)
      if not node:
        raise ValueError('Unable to locate node with name %s.' % target)
      elif node not in available_nodes:
        raise ValueError('Node %s not in selected locations or prefix.'
                         % target)
      else:
        selected_nodes.append(node)

  if not selected_nodes:
    raise ValueError('No nodes selected.  Must specify node targets or '
                     '--prefix or --location.')
  return list(selected_nodes)


class JlinkTask(object):
  """Represents an operation with jlink_client.

  Args:
    binary: A .bin or .elf file to be flashed.
    task: A string matching 'bootloader', 'application', 'config', 'param',
        'calib'.
    target: The target node type in snake format (e.g. motor_sbo).
    hardware_type: The target hardware type (e.g. motor or aio).
  """

  def __init__(self, binary, task, target=None, hardware_type=None):
    self.binary = binary
    self.target = target
    self.hardware_type = hardware_type
    self.task = task
    self._jlink_method = None
    self._jlink_args = []
    self._jlink_kwargs = {}

  def GetArgs(self):
    self._PrepareForExecution()
    return ['Python call to jlink_client.{} with arguments: {} and '
            'keyword-arguments: {}'.format(self._jlink_method.__name__,
                                           self._jlink_args,
                                           self._jlink_kwargs)]

  def ExecuteTask(self):
    """Performs the J-link programming task.

    Returns:
      A tuple of the return_code, stdout string, stderr string.

    Raises:
      CalledProcessError: Non-zero return code from jlink_client.
    """
    self._PrepareForExecution()
    return_code, stdout, stderr = self._jlink_method(*self._jlink_args,
                                                     **self._jlink_kwargs)
    if return_code != 0:
      error_logger = logging.getLogger('stdout')
      for line in stdout.split('\n'):
        error_logger.warn(line)
      error_logger = logging.getLogger('stderr')
      for line in stderr.split('\n'):
        error_logger.warn(line)
    else:
      logger = logging.getLogger('stdout')
      for line in stdout.split('\n'):
        if line.startswith('J-Link: '):
          logger.info(line)

    if return_code != 0:
      raise subprocess.CalledProcessError(return_code, stdout + stderr)
    return return_code, stdout, stderr

  def _PrepareForExecution(self):
    """Sets _jlink_method, self._jlink_args and self._jlink_kwargs.

    Raises:
      NameError: Invalid combinations of arguments.
      ValueError: Invalid argument values.
    """
    if self.binary.endswith('.elf'):
      if not self.target or not self.hardware_type:
        raise ValueError('target and hardware_type must be specified for '
                         'elf files.')
      if self.task == 'bootloader':
        self._jlink_method = jlink_client.JlinkProgramBootloaderElf
        self._jlink_args = (self.target, self.hardware_type, self.binary)
      elif self.task == 'application':
        self._jlink_method = jlink_client.JlinkProgramApplicationElf
        self._jlink_args = (self.target, self.hardware_type, self.binary)
      else:
        raise NameError('Invalid argument task: {}'.format(self.task))
    elif self.binary.endswith('.bin'):
      if self.task == 'bootloader':
        self._jlink_method = jlink_client.JlinkProgramBootloaderBin
        self._jlink_args = (self.binary,)
      elif self.task == 'application':
        self._jlink_method = jlink_client.JlinkProgramApplicationBin
        self._jlink_args = (self.binary,)
      elif self.task == 'config':
        self._jlink_method = jlink_client.JlinkProgramConfigBin
        self._jlink_args = (self.binary,)
      elif self.task == 'param':
        self._jlink_method = jlink_client.JlinkProgramSerialBin
        self._jlink_args = (self.binary,)
      elif self.task == 'calib':
        self._jlink_method = jlink_client.JlinkProgramCalibBin
        self._jlink_args = (self.binary,)
      else:
        raise NameError('Invalid argument task: {}'.format(self.task))
    else:
      raise ValueError('Invalid file: {}'.format(self.binary))

  def GetDependentTask(self):
    return None


class BinTask(object):
  """Represents an operation with bootloader_client.

  Raises:
    subprocess.CalledProcessError for non-zero return codes.
  """

  def __init__(self, bootloader_client, target, binary, extra_args):
    self.bootloader_client = bootloader_client
    self.target = target
    self.binary = binary
    self.extra_args = extra_args

  def GetArgs(self):
    bootloader_client = self.bootloader_client
    if not bootloader_client:
      bootloader_client = 'bootloader_client'
    return [bootloader_client, '--target', self.target,
            self.binary] + list(self.extra_args)

  def ExecuteTask(self):
    subprocess.check_call(self.GetArgs())

  def GetDependentTask(self):
    return None


class ParamTask(object):
  """Represents an operation with param_util."""

  def __init__(self, input_file, yaml_key, set_values, temp_file_suffix):
    self.input_file = input_file
    self.yaml_key = yaml_key
    self.set_values = set_values
    self.temp_file_suffix = temp_file_suffix
    self.temp_file = 'TEMP' + self.temp_file_suffix
    self.temp_handle = None

  def GetArgs(self):
    cmd_args = ['param_util', '--input', self.input_file, '--output',
                self.temp_file, '--yaml_key', self.yaml_key]
    for k, v in self.set_values.iteritems():
      cmd_args += ['--set_value', '%s:%s' % (k, v)]
    return cmd_args

  def GetDependentTask(self):
    return None

  def ExecuteTask(self):
    self._DeleteTempFile()
    self.temp_handle, self.temp_file = tempfile.mkstemp(self.temp_file_suffix)
    with open(self.input_file) as input_file:
      with open(self.temp_file, 'w') as output_file:
        param = codec.DecodeYaml(input_file.read(), self.yaml_key)
        for k, v in self.set_values.iteritems():
          param.SetField(k, v)
        output_file.write(codec.EncodeBin(param))

  def _DeleteTempFile(self):
    if self.temp_handle is not None:
      try:
        os.unlink(self.temp_file)
      except IOError:
        sys.stderr.write('Error removing temp file %s.\n', self.temp_file)

  def __del__(self):
    self._DeleteTempFile()


class ParamTaskBootloaderClient(ParamTask):
  """Represents an operation with param_util and bootloader_client."""

  def __init__(self, bootloader_client, target, input_file, yaml_key,
               set_values, extra_args, temp_file_suffix):
    self.bootloader_client = bootloader_client
    self.target = target
    self.extra_args = extra_args
    super(ParamTaskBootloaderClient, self).__init__(input_file, yaml_key,
                                                    set_values,
                                                    temp_file_suffix)

  def GetDependentTask(self):
    return BinTask(self.bootloader_client, self.target, self.temp_file,
                   self.extra_args)


class ParamTaskJlink(ParamTask):
  """Represents an operation with param_util and jlink_client."""

  def __init__(self, input_file, yaml_key, set_values,
               temp_file_suffix):
    super(ParamTaskJlink, self).__init__(input_file, yaml_key,
                                         set_values,
                                         temp_file_suffix)

  def GetDependentTask(self):
    return JlinkTask(self.temp_file, 'param')


def _CheckGlobalOptions(args):
  """Handles options that apply to all operations."""
  if not args.bootloader_client and not args.print_args:
    args.bootloader_client = os.path.join(
        makani.HOME, 'avionics/bootloader/bootloader_client')
  if not args.tms570_bin:
    args.tms570_bin = bazel_util.GetTms570BinDirectory()
  if args.batch and args.system_config:
    raise ValueError('System config and batch mode are not allowed together.')
  # --batch does not allow for the following global arguments, howerver
  # --force_hardware is allowed as a global only in combination with --jlink.
  operation_args = (args.prefix or args.target or args.calib or args.config or
                    args.bootloader or args.bootloader_application or
                    args.application or args.upgrade_bootloader or
                    args.rename_to or args.serial or args.carrier_serial or
                    (not args.jlink and args.force_hardware))
  if args.batch and operation_args:
    raise ValueError('In batch mode, operations can only be completed as '
                     'batch arguments.')
  if args.system_config and operation_args:
    raise ValueError('In system config mode, operations cannot be specified.')
  if not args.batch and not args.system_config:
    return _CheckOperationOptions(args, args.jlink, False)
  if args.jlink and args.parallel:
    raise ValueError('In parallel mode, --jlink is impossible.')
  return args


def _CheckOperationOptions(args, global_jlink=False, reject_global_args=True):
  """Handles options that apply to a specific operation."""
  if reject_global_args and (args.tms570_bin or args.bootloader_client
                             or args.parallel or args.batch
                             or args.system_config or args.print_args
                             or args.jlink):
    raise ValueError('Got global arguments in batch mode operation.')
  args.application |= not (args.calib or args.config or args.bootloader
                           or args.bootloader_application
                           or args.upgrade_bootloader or args.rename_to
                           or args.serial or args.carrier_serial)
  if (sum([1 for x in [args.calib, args.config, args.bootloader,
                       args.bootloader_application, args.application,
                       args.upgrade_bootloader, args.rename_to,
                       args.serial, args.carrier_serial] if x]) != 1):
    raise ValueError('Cannot specify more than one update type (calib, serial, '
                     'carrier_serial, config, bootloader, '
                     'or bootloader_application).')
  if (args.force_hardware and not args.jlink and not (
      args.bootloader or args.bootloader_application or args.upgrade_bootloader
      or args.rename_to)):
    raise ValueError('Cannot specify force_hardware unless writing the '
                     'bootloader or bootloader app (including renaming).')

  # Jlink does not need a target for serial programming, override with unknown
  # to satisfy target parsing checks.
  if (args.jlink or global_jlink) and args.serial:
    args.target = ['unknown']
  args.target = _GetTargetSelection(args)
  if len(args.target) != 1 and (args.rename_to or args.serial
                                or args.carrier_serial):
    raise ValueError('Only one node can be renamed, or programmed with '
                     'serial params at a time.')
  unknown_node = _NETWORK_CONFIG.GetAioNode('unknown')
  if unknown_node not in args.target and (args.serial or args.carrier_serial):
    raise ValueError('Can only program carrier or serial params on unknown '
                     'node.')
  if unknown_node in args.target and not (args.rename_to or args.serial or
                                          args.carrier_serial or args.jlink):
    raise ValueError('Unknown node can only be renamed, or programmed with '
                     'serial params.')
  return args


def _GetOperationCommands(global_args, operation_args):
  """Get the bootloader client arguments for this operation."""
  cmd_list = []
  def _AddCommand(target, binary, *extra_args):
    cmd_list.append(BinTask(global_args.bootloader_client, target.snake_name,
                            binary, extra_args))
  def _AddJlinkCommand(task, target, elf_file):
    hardware_type = operation_args.force_hardware or global_args.force_hardware
    cmd_list.append(
        JlinkTask(elf_file, task, target.camel_name, hardware_type))
  def _GetForceHardwareArgs():
    if operation_args.force_hardware:
      return ['--force_hardware', operation_args.force_hardware]
    elif global_args.force_hardware:
      return ['--force_hardware', global_args.force_hardware]
    else:
      return []
  def _GetKnownProgramBinary(target, other, segment):
    """Returns the elf binary of target or other if target is unknown."""
    if target.snake_name != 'unknown' or global_args.jlink:
      return _GetProgramElf(global_args.tms570_bin, target, segment)
    if other.snake_name != 'unknown':
      # Programming application on unknown requires bootloader_application.
      if segment == 'application':
        segment = 'bootloader_application'
      return _GetProgramElf(global_args.tms570_bin, other, segment)
    raise ValueError('Cannot locate binary for unknown nodes.')
  def _ProgramBootloader(source, dest, *extra_args):
    if dest != source:
      if global_args.parallel:
        raise ValueError('Parallel mode not supported for rename operations.')
      extra_args = ['--override_target', dest.snake_name] + list(extra_args)
    if global_args.jlink:
      _AddJlinkCommand(
          'bootloader', source,
          _GetKnownProgramBinary(dest, source, 'bootloader'))
    else:
      _AddCommand(source, _GetKnownProgramBinary(dest, source, 'bootloader'),
                  '--bootloader', *(_GetForceHardwareArgs() + list(extra_args)))
  def _RenameNode(source, dest):
    _AddCommand(source, _GetKnownProgramBinary(source, dest,
                                               'bootloader_application'),
                *_GetForceHardwareArgs())
    _ProgramBootloader(source, dest)
    _AddCommand(dest, _GetKnownProgramBinary(dest, source, 'application'))
  def _SerialParam(target, template, revision, serial, carrier=False):
    input_file = SERIAL_PARAM_FILE_TEMPLATE.format(template)
    param_type_name = 'carrier_serial' if carrier else 'serial'
    extra_args = ['--%s' % param_type_name]
    temp_file_suffix = '_%s_params.bin' % param_type_name
    if global_args.jlink:
      cmd_list.append(ParamTaskJlink(
          input_file=input_file, yaml_key=revision,
          set_values={'serial_number': serial},
          temp_file_suffix=temp_file_suffix,))
    else:
      cmd_list.append(ParamTaskBootloaderClient(
          bootloader_client=global_args.bootloader_client,
          target=target.snake_name, input_file=input_file, yaml_key=revision,
          temp_file_suffix=temp_file_suffix,
          set_values={'serial_number': serial}, extra_args=extra_args))

  if global_args.jlink:
    if not (operation_args.bootloader or operation_args.application or
            operation_args.bootloader_application or operation_args.serial or
            operation_args.config or operation_args.calib):
      raise ValueError('Cannot specify --jlink unless writing the bootloader, '
                       'bootloader application, application, serial, config, '
                       'or calib.')
    if ((operation_args.bootloader or operation_args.application or
         operation_args.bootloader_application) and not
        (global_args.force_hardware or operation_args.force_hardware)):
      raise ValueError('You must specify --force_hardware with --jlink when '
                       'writing bootloader or bootloader_application or '
                       'application.')
    if len(operation_args.target) != 1:
      raise ValueError('Only one node can be specified when using --jlink.')

  for target in operation_args.target:
    if operation_args.calib:
      calib_file = _FindUniqueFile(global_args.tms570_bin, operation_args.calib,
                                   operation_args.prefix,
                                   'calib_params.bin', target.snake_name)
      if global_args.jlink:
        _AddJlinkCommand('calib', target, calib_file)
      else:
        _AddCommand(target, calib_file, '--calib')
    elif operation_args.config:
      config_file = _FindUniqueFile(
          global_args.tms570_bin, operation_args.config,
          operation_args.prefix, 'config_params.bin', target.snake_name)
      if global_args.jlink:
        _AddJlinkCommand('config', target, config_file)
      else:
        _AddCommand(target, config_file, '--config')
    elif operation_args.bootloader:
      _ProgramBootloader(target, target)
    elif operation_args.bootloader_application:
      if global_args.jlink:
        _AddJlinkCommand(
            'application', target,
            _GetProgramElf(global_args.tms570_bin, target,
                           'bootloader_application'))
      else:
        _AddCommand(target, _GetProgramElf(global_args.tms570_bin, target,
                                           'bootloader_application'),
                    *(_GetForceHardwareArgs()))
    elif operation_args.application:
      if global_args.jlink:
        _AddJlinkCommand(
            'application', target,
            _GetProgramElf(global_args.tms570_bin, target, 'application'))
      else:
        _AddCommand(target, _GetProgramElf(global_args.tms570_bin, target,
                                           'application'))
    elif operation_args.upgrade_bootloader:
      _RenameNode(target, target)
    elif operation_args.rename_to:
      dest = _GetTargetSelection(operation_args, [operation_args.rename_to])[0]
      if dest == target:
        raise ValueError('Cannot rename node to itself!')
      _RenameNode(target, dest)
    elif operation_args.carrier_serial:
      _SerialParam(target, *operation_args.carrier_serial, carrier=True)
    elif operation_args.serial:
      _SerialParam(target, *operation_args.serial, carrier=False)
    else:
      assert False, 'Invalid operation.'

  return cmd_list


def Main(argv):
  """Main function for the application."""
  parser = argparse.ArgumentParser(
      description='Program multiple nodes in a sane manner.  Guess what the '
      'user wants to do.  The application is programmed by default.')
  operation = parser.add_argument_group('operation')
  operation.add_argument('target', nargs='*', help='List of target nodes.')
  operation.add_argument('--prefix',
                         help='Prefix used for matching nodes or files.  Will '
                         'program all nodes with this prefix unless a subset '
                         'is specified.')
  operation.add_argument('--location', nargs='+', help='Select nodes from a '
                         'specific location (wing, ground_station, '
                         'remote_command).')
  operation.add_argument('--force_hardware', help='Force the hardware type for '
                         'a bootloader operation.')
  segment = operation.add_mutually_exclusive_group()
  segment.add_argument('--application', action='store_true',
                       help='Program the application segment (default).')
  segment.add_argument('--calib', help='Program calib params.')
  segment.add_argument('--config', help='Program config params.')
  segment.add_argument('--bootloader', action='store_true',
                       help='Program the bootloader segment.')
  segment.add_argument('--bootloader_application', action='store_true',
                       help='Program the bootloader application.')
  segment.add_argument('--upgrade_bootloader', action='store_true',
                       help='Upgrade the bootloader for a node.')
  segment.add_argument('--rename_to', help='Rename target node.')
  segment.add_argument('--serial', nargs=3,
                       help='Program the serial information.',
                       metavar=('SERIAL_TYPE', 'REVISION', 'SERIAL_NUMBER'))
  segment.add_argument('--carrier_serial', nargs=3,
                       help='Program the carrier serial information.',
                       metavar=('SERIAL_TYPE', 'REVISION', 'SERIAL_NUMBER'))
  parser.add_argument('--batch', nargs='+', help='Perform a sequence of '
                      'operations.')
  parser.add_argument('--system_config', nargs='+', help='Use a designated '
                      'system configuration (or multiple configurations).  '
                      'Options are: %s.' % ', '.join(
                          system_config.SystemConfig.GetAllSystemNames()))
  parser.add_argument('--print_args', action='store_true',
                      help='Print arguments rather than running commands.')
  parser.add_argument('--bootloader_client', help='Override the default '
                      'bootloader client found by bazel.')
  parser.add_argument('--tms570_bin', help='Override the TMS570 binary '
                      'directory found by bazel.')
  parser.add_argument('--parallel', action='store_true',
                      help='Invoke the parallel bootloader.')
  parser.add_argument('--jlink', action='store_true',
                      help='Uses J-Link JTAG device for operations.')
  try:
    args = _CheckGlobalOptions(parser.parse_args(argv[1:]))
    cmd_list = []
    def _CreateAndAddCommand(cmd_args):
      op_args = _CheckOperationOptions(parser.parse_args(cmd_args),
                                       args.jlink)
      cmd_list.extend(_GetOperationCommands(args, op_args))
    if args.batch:
      for batch_step in args.batch:
        _CreateAndAddCommand(batch_step.split())
    elif args.system_config:
      for config_name in args.system_config:
        sys_config = system_config.SystemConfig(config_name,
                                                net_config=_NETWORK_CONFIG)
        for node in sys_config.nodes:
          _CreateAndAddCommand([node.snake_name])
          if node in sys_config.config:
            _CreateAndAddCommand([node.snake_name, '--config',
                                  sys_config.config[node]])
          if node in sys_config.calib:
            _CreateAndAddCommand([node.snake_name, '--calib',
                                  sys_config.calib[node]])
    else:
      cmd_list += _GetOperationCommands(args, args)
  except (ValueError, KeyError) as e:
    sys.stderr.write('ERROR: %s\n\n' % e)
    parser.print_usage()
    sys.exit(-1)

  parallel_args = ['--curses']
  for cmd in cmd_list:
    while cmd:
      if args.print_args:
        print ' '.join(cmd.GetArgs())
      elif args.parallel and isinstance(cmd, BinTask):
        parallel_args.append(' '.join(cmd.GetArgs()))
      else:
        try:
          cmd.ExecuteTask()
        except subprocess.CalledProcessError:
          sys.exit(-1)
      cmd = cmd.GetDependentTask()

  if args.parallel and not args.print_args:
    parallel_bootloader.Main(parallel_args)


if __name__ == '__main__':
  logging.basicConfig(
      format='%(levelname)s:%(name)s:%(message)s',
      level=logging.INFO)
  Main(sys.argv)
