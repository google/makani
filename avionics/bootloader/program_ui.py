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

"""A user-friendly interface for program.py."""
import collections
import logging
import os
import socket
import sys
import tempfile
import time
import gflags

import makani
from makani.avionics.bootloader import generate_image
from makani.avionics.common import aio
from makani.avionics.common import aio_version
from makani.avionics.linux.provision import process_helper
from makani.avionics.linux.provision import ui_helper
from makani.avionics.network import network_config
from makani.lib.bazel import bazel_util
from makani.lib.python import c_helpers
import program

_NETWORK_CONFIG = network_config.NetworkConfig()


def IpToAioNode(ip):
  assert ip.startswith('192.168.1.'), ('Ip {} not in AIO range '
                                       '192.168.1.0/24.'.format(ip))
  final_octet = int(ip.split('.')[-1])
  node_num = aio.aio_node_to_ip_address.FinalOctetToAioNode(final_octet)
  return _NETWORK_CONFIG.aio_nodes[node_num]


def DetectAio(timeout=1.1):
  """Detect AIO nodes on the network, present all options if none detected."""
  sources = aio.aio_node_helper.Names()
  types = aio.message_type_helper.Names()
  client = aio.AioClient(types, timeout=0.1, allowed_sources=sources)
  ip_list = []
  version_list = []
  timer_start = time.time()
  while time.time() - timer_start < timeout:
    try:
      ip, header, _ = client.Recv(accept_invalid=True)
      ip_list.append(ip)
      version_list.append(header.version)
    except socket.error:
      pass
  client.Close()
  if ip_list and version_list:
    # De-duplication using set conversion.
    ip_tuple, version_tuple = zip(*set(zip(ip_list, version_list)))
    return tuple([IpToAioNode(ip) for ip in ip_tuple]), version_tuple
  return tuple(), tuple()


def Tms570NodeDict():
  my_dict = collections.defaultdict(list)
  for node in _NETWORK_CONFIG.aio_nodes:
    if node.tms570_node or node.label_name == 'unknown':
      my_dict[node.label_name].append(node)
  return my_dict


def NodeSelectMenu(dialog_ui, dialog_kwargs=None):
  """Ask the user to select and AIO node using a menu and sub-menu."""
  kwargs = dialog_kwargs or {}
  aio_node_tree = Tms570NodeDict()
  top_menu = sorted([(key, key) for key in aio_node_tree])
  sub_ok, select = dialog_ui.Menu('Select AIO node label:', top_menu, **kwargs)
  if not sub_ok:
    return None
  sub_menu = [(str(val.enum_value), val.snake_name) for val in
              aio_node_tree[select]]
  sub_ok, select = dialog_ui.Menu('Select AIO node:', sub_menu, **kwargs)
  if not sub_ok:
    return None
  return select


def GetSerialRevisions(serial_type='aio'):
  yaml_file = program.SERIAL_PARAM_FILE_TEMPLATE.format(serial_type)
  return program.codec.DecodeYaml(open(yaml_file).read()).keys()


def GetSerialTypes():
  serial_type = []
  filename_suffix = os.path.basename(
      program.SERIAL_PARAM_FILE_TEMPLATE.format(''))
  filename_dirname = os.path.dirname(program.SERIAL_PARAM_FILE_TEMPLATE)
  for filename in os.listdir(filename_dirname):
    if filename.endswith(filename_suffix):
      serial_type.append(filename.replace(filename_suffix, ''))
  return serial_type


class MenuFunction(object):
  """A mapping dialog menu entries to functions."""

  def __init__(self, dialog_ui, title=None):
    self.dialog_ui = dialog_ui
    self.title = title
    self.tag = []
    self.name = []
    self.func = []
    self.args = []

  def Register(self, name, func, func_args=None, tag=None):
    """Creates a mapping from a dialog menu entry to func."""
    if not tag:
      tag = str(len(self.tag))
    assert isinstance(tag, str), 'Invalid parameter: tag must be a str.'
    assert isinstance(name, str), 'Invalid parameter: name must be a str.'
    assert hasattr(func, '__call__'), ('Invalid parameter: func must have '
                                       '__call__ attribute.')
    self.tag.append(tag)
    self.name.append(name)
    self.func.append(func)
    self.args.append(func_args or [])

  def Run(self, menu_text, dialog_kwargs=None):
    """Presents a dialog menu and runs the user-selected entry."""
    kwargs = dialog_kwargs or {}
    if self.title:
      kwargs['title'] = self.title
    assert isinstance(menu_text, str), ('Invalid parameter: menu_text must be '
                                        'a str.')
    sub_ok, tag = self.dialog_ui.Menu(menu_text, zip(self.tag, self.name),
                                      **kwargs)
    if sub_ok:
      index = self.tag.index(tag)
      return sub_ok, self.func[index](*self.args[index])
    return sub_ok, None


class ProgramWrapper(object):
  """A wrapper for program.py operations which provides a UI using dialog."""
  program_py_command = [
      'python',
      os.path.join(makani.HOME, 'avionics', 'bootloader', 'program.py')]

  def __init__(self):
    self.dialog_ui = ui_helper.UserInterface()
    self.AutoDetect()

  def AutoDetect(self):
    timeout = 1.1
    while True:
      if self.DetectNodeAndVersion(detection_timeout=timeout):
        break
      else:
        ok = self.dialog_ui.YesNo('Please select a node from the list.',
                                  title='Node Scanner',
                                  yes_label='OK', no_label='Quit')
        if not ok:
          break
        if self.SelectNode():
          break
        # User selected cancel in SelectNode menu, try harder to detect.
        else:
          timeout = 3.3

  def DetectNodeAndVersion(self, detection_timeout=1.1):
    """Detect an AIO nodes, offer selection if more than one is found."""
    ok = False
    while not ok:
      self.dialog_ui.Info('Scanning for AIO nodes...')
      try:
        detected_nodes, versions = DetectAio(timeout=detection_timeout)
        if detected_nodes:
          select_node_menu = [(str(node.enum_value), str(node.camel_name)) for
                              node in detected_nodes]

          sentinel = str(detected_nodes[-1].enum_value + 1)
          select_node_menu.append((sentinel, 'Select from complete list'))
          ok, node_ind = self.dialog_ui.Menu('Please select an AIO node:',
                                             options=select_node_menu,
                                             cancel_label='Rescan')
          if node_ind == sentinel:
            return False
        else:
          return False
      except socket.error, e:
        self.dialog_ui.Message('Socket error: {}'.format(e),
                               title='Node Scanner')
        return False
    self.select_node = detected_nodes[int(node_ind)]
    self.version = versions[int(node_ind)]
    return detected_nodes and ok

  def SelectNode(self):
    node_ind = NodeSelectMenu(self.dialog_ui, dialog_kwargs={'cancel_label':
                                                             'Re-scan'})
    if node_ind:
      self.select_node = _NETWORK_CONFIG.aio_nodes[int(node_ind)]
      self.version = None
      return True
    else:
      return False

  def HasCarrierBoard(self):
    node_prefix = self.select_node.snake_name.partition('_')[0]
    return node_prefix not in ('cs', 'motor')

  def SerialParamSelect(self, hardware_type):
    """Present menu to select serial parameter revision, and serial_number."""
    rev_list = sorted(GetSerialRevisions(hardware_type))
    if 'common' in rev_list:
      rev_list.remove('common')  # 'common' is not to be used on hardware.
    sub_ok, rev = self.dialog_ui.Menu('Select the hardware revision:',
                                      zip(rev_list, rev_list))
    if not sub_ok:
      return None, None
    sub_ok, serial_number = self.dialog_ui.Input('Please enter the'
                                                 ' serial number:')
    if not sub_ok:
      return None, None
    return rev, serial_number

  def RunDialogProcess(self, process, title):
    """Run process in an dialog progress-box with munged stdout & stderr."""
    with tempfile.NamedTemporaryFile() as temp_file:
      def WriteAndDisplay(line):
        temp_file.write(line)
        temp_file.flush()
        self.dialog_ui.dialog_instance.progressbox(file_path=temp_file.name,
                                                   title=title, width=100,
                                                   height=30)
      return process_helper.RunProcess(process, parse_stdout=WriteAndDisplay,
                                       parse_stderr=WriteAndDisplay)

  def GetFirmwareFiles(self, file_suffix):
    """Returns a list of firmware files with file_suffix removed."""
    assert isinstance(self.select_node, network_config.AioNode)
    firmware_path = os.path.join(
        bazel_util.GetTms570BinDirectory(), 'avionics',
        os.path.dirname(self.select_node.application_path))
    matching_files = []
    for filename in os.listdir(firmware_path):
      if filename.endswith(file_suffix):
        matching_files.append(filename.replace(file_suffix, ''))
    return matching_files

  def GetConfigNames(self):
    """Returns a list of config file short-names compiled for select_node."""
    return self.GetFirmwareFiles('_config_params.bin')

  def GetCalibNames(self):
    """Returns a list of calib file short-names compiled for select_node."""
    return self.GetFirmwareFiles('_calib_params.bin')

  def RenameNode(self):
    """Rename the node from node to a user-selected node type."""
    rename_to = NodeSelectMenu(self.dialog_ui)
    if rename_to:
      rename_to = _NETWORK_CONFIG.aio_nodes[int(rename_to)]
      rc, stdout, stderr = self.RunDialogProcess(
          self.program_py_command + [self.select_node.snake_name,
                                     '--rename_to', rename_to.snake_name],
          title='Renaming node...')
      if rc == 0:
        self.select_node = rename_to
        self.UpdateVersion(timeout=3)
      return rc, stdout, stderr
    return -1, '', 'User cancelled.'

  def ProgramApplication(self):
    """Program the application on select_node."""
    rc, stdout, stderr = self.RunDialogProcess(
        self.program_py_command + [self.select_node.snake_name],
        title='Programming application...')
    return rc, stdout, stderr

  def ProgramBootloader(self):
    """Program the bootloader on select_node."""
    rc, stdout, stderr = self.RunDialogProcess(
        self.program_py_command + [self.select_node.snake_name, '--bootloader'],
        title='Programming bootloader...')
    return rc, stdout, stderr

  def ProgramSerial(self):
    """Program the serial number into select_node."""
    hardware_type = self.SerialTypeMenu(is_carrier=False)
    if not hardware_type:
      return -1, '', 'User cancelled'
    rev, serial_number = self.SerialParamSelect(hardware_type)
    if not rev or not serial_number:
      return -1, '', 'User cancelled.'
    rc, stdout, stderr = self.RunDialogProcess(
        self.program_py_command + [self.select_node.snake_name, '--serial',
                                   hardware_type, rev, serial_number],
        title='Programming serial...')
    return rc, stdout, stderr

  def ProgramCarrierSerial(self):
    """Program the carrier serial board onto select_node."""
    node_prefix = self.select_node.snake_name.partition('_')[0]
    if node_prefix in ['cs', 'motor']:
      self.dialog_ui.Message('{} type nodes do not have carriers.'.format(
          node_prefix), title='Carrier Serial Programmer')
    else:
      carrier_type = self.SerialTypeMenu(is_carrier=True)
      if not carrier_type:
        return -1, '', 'User cancelled.'
      rev, serial_number = self.SerialParamSelect(carrier_type)
      if not rev or not serial_number:
        return -1, '', 'User cancelled.'
      rc, stdout, stderr = self.RunDialogProcess(
          self.program_py_command + [self.select_node.snake_name,
                                     '--carrier_serial', carrier_type, rev,
                                     serial_number],
          title='Programming carrier serial...')
    return rc, stdout, stderr

  def ProgramCalib(self):
    """Program the calib params into select_node."""
    calibs = self.GetCalibNames()
    if not calibs:
      return (-1, '', 'No valid calibrations for '
              '{}.'.format(self.select_node.snake_name))
    sub_ok, calib_name = self.dialog_ui.Menu('Select calibration:',
                                             zip(calibs, calibs))
    if sub_ok:
      return self.RunDialogProcess(
          self.program_py_command + [self.select_node.snake_name, '--calib',
                                     calib_name],
          title='Programming calib...')
    return -1, '', 'User cancelled.'

  def ProgramConfig(self):
    """Program the config params into select_node."""
    configs = self.GetConfigNames()
    if not configs:
      return (-1, '',
              'No valid configs for {}.'.format(self.select_node.snake_name))
    sub_ok, config_name = self.dialog_ui.Menu('Select config:',
                                              zip(configs, configs))
    if sub_ok:
      return self.RunDialogProcess(
          self.program_py_command + [self.select_node.snake_name, '--config',
                                     config_name],
          title='Programming config...')
    return -1, '', 'User cancelled.'

  def UpgradeBootloader(self):
    """Upgrade the bootloader on select_node."""
    return self.RunDialogProcess(
        self.program_py_command + [self.select_node.snake_name,
                                   '--upgrade_bootloader'],
        title='Upgrading bootloader...')

  def CheckConsole(self):
    """Display the Stdio and SelfTest messages from select_node using dialog."""
    sources = [self.select_node.enum_name]
    types = ['kMessageTypeStdio', 'kMessageTypeSelfTest']
    self.dialog_ui.Info('Watching for Stdio messages...(3s)',
                        title='Console Checker')
    try:
      client = aio.AioClient(types, timeout=3, allowed_sources=sources)
      _, _, message = client.Recv(accept_invalid=True)
      self.dialog_ui.Message(getattr(message, 'text', repr(message)),
                             title='Console Message')
    except socket.timeout, e:
      return -1, '', 'No Stdio messages found.'
    except socket.error, e:
      if str(e) == 'timed out':
        return -1, '', 'No Stdio messages found.'
      raise
    finally:
      client.Close()
    return 0, '', ''

  def UpdateVersion(self, timeout=1):
    """Get AIO version from header of any packet from select_node."""
    sources = [self.select_node.enum_name]
    types = aio.message_type_helper.Names()
    self.dialog_ui.Info('Watching for messages...({}s)'.format(timeout),
                        title='Version Checker')
    try:
      client = aio.AioClient(types, timeout=timeout, allowed_sources=sources)
      _, header, _ = client.Recv(accept_invalid=True)
    except socket.timeout as e:
      self.version = None
      return -1, '', str(e)
    except socket.error as e:
      if str(e) == 'timed out':
        self.version = None
        return -1, '', str(e)
      else:
        raise
    else:
      self.version = str(header.version)
    finally:
      client.Close()
    return 0, '', ''

  def SerialTypeMenu(self, is_carrier=False):
    """Ask the user to select a hardware type of select_node.

    The menu options are derived from serial_params files.

    Args:
      is_carrier: Set to True if flashing serial params to a carrier board.

    Returns:
      Selected hardware type or None if the user cancelled.
    """
    default = 'aio'
    serial_types = GetSerialTypes()
    node_prefix = self.select_node.snake_name.partition('_')[0]
    if is_carrier:
      menu_text = 'Select carrier hardware type:'
      serial_types.remove('aio')  # Carrier cannot be AIO node.
      serial_types.remove('motor')
      serial_types.remove('cs')
      if node_prefix in serial_types:
        default = node_prefix
    else:
      if (node_prefix in serial_types and not
          self.HasCarrierBoard()):
        default = node_prefix
      menu_text = 'Select hardware type:'
    serial_types.sort()
    sub_ok, serial_type = self.dialog_ui.Menu(menu_text, zip(serial_types,
                                                             serial_types),
                                              default_item=default)
    if sub_ok:
      return serial_type

  def HardwareIdentityMenu(self):
    """Ask the user to select a hardware identity of select_node.

    The menu options are derived from identity_types.h.

    Returns:
      Selected hardware identity or None if the user cancelled.
    """
    default = 'aio'
    node_prefix = self.select_node.snake_name.partition('_')[0]
    hardware_types = [c_helpers.CamelToSnake(camel) for camel in
                      generate_image.hardware_type_helper.ShortNames()]
    hardware_types.remove('unknown')
    if (node_prefix in hardware_types and not
        self.HasCarrierBoard()):
      default = node_prefix
    rc, hardware_type = self.dialog_ui.Menu(
        'Select hardware type:', zip(hardware_types, hardware_types),
        default_item=default)
    if rc:
      return hardware_type

  def ToggleJlink(self):
    """Toggles the --jlink flag for program.py, creates a new menu."""
    if '--jlink' not in self.program_py_command:
      hardware_type = self.HardwareIdentityMenu()
      if hardware_type:
        self.program_py_command.append('--jlink')
        self.program_py_command += ['--force_hardware', hardware_type]
    else:
      self.program_py_command = [
          'python',
          os.path.join(makani.HOME, 'avionics', 'bootloader', 'program.py')]
    return 0, '', ''


def CreateMenu(program_wrapper):
  """Creates a MenuFunction instance and returns it."""
  jlink_mode = '--jlink' in program_wrapper.program_py_command
  new_menu = MenuFunction(program_wrapper.dialog_ui, 'Program.py Menu')
  new_menu.Register('Program Application', program_wrapper.ProgramApplication)
  new_menu.Register('Program Bootloader', program_wrapper.ProgramBootloader)
  if not jlink_mode:
    new_menu.Register('Rename Node', program_wrapper.RenameNode)
  new_menu.Register('Program Serial', program_wrapper.ProgramSerial)
  if not jlink_mode and program_wrapper.HasCarrierBoard():
    new_menu.Register('Program Carrier Serial',
                      program_wrapper.ProgramCarrierSerial)
  new_menu.Register('Program Config', program_wrapper.ProgramConfig)
  new_menu.Register('Program Calib', program_wrapper.ProgramCalib)
  if not jlink_mode:
    new_menu.Register('Upgrade Bootloader', program_wrapper.UpgradeBootloader)
    new_menu.Register('Check Console', program_wrapper.CheckConsole)
    new_menu.Register('Check Version', program_wrapper.UpdateVersion)
  new_menu.Register('Toggle Jlink', program_wrapper.ToggleJlink)
  return new_menu


def main():
  logging.basicConfig(level=logging.WARNING)
  flags = gflags.FLAGS
  try:
    _ = flags(sys.argv)
  except gflags.FlagsError, e:
    print ('%s\nUsage: %s ARGS\n%s'
           % (e, sys.argv[0], flags))
    sys.exit(1)

  program_py = ProgramWrapper()
  assert hasattr(program_py, 'select_node'), 'No node was selected.'

  ok = True
  while ok:
    menu_title = 'Node: {}\n'.format(program_py.select_node.snake_name)
    if program_py.version:
      menu_title += 'Detected AIO Version: {}\n'.format(program_py.version)
    menu_title += 'Compiled AIO Version: {}\n'.format(aio_version.AIO_VERSION)
    menu_title += 'Using jlink: {}'.format('--jlink' in
                                           program_py.program_py_command)

    ok, result = CreateMenu(program_py).Run(menu_title,
                                            dialog_kwargs={'cancel_label':
                                                           'Quit'})
    if ok and result[0] != 0:
      # Replace \\n with spaces, or dialog will assume that all newlines are
      # escaped twice and the output won't have newlines where it should.
      program_py.dialog_ui.Message(('Failure (process returned {}):\n'
                                    '{}').format(result[0],
                                                 result[2].replace('\\n', ' ')),
                                   title='Error')


if __name__ == '__main__':
  main()
