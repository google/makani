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

"""Generate common monitoring code."""

import copy
import importlib
import os
import re
import sys
import textwrap

import gflags
import makani
from makani.lib.python import c_helpers


class DeviceConfigBase(object):
  """Base class for monitor driver configuration.

  This class attempts to handle all common operations for each monitor driver
  configuration. Each monitor should inherit this class and implement the
  GetConfigAsString() function at minimum.
  """

  def __init__(self, config, expected_parameters):
    """Initialize the base class.

    Args:
      config: Specify a dict containing the per-device configuration.
      expected_parameters: Specify a list of keys to expect in config.
    """
    self._config = copy.deepcopy(config)

    # Validate configuration.
    self._CheckExpectedParameters(self._config, expected_parameters)
    self._config['camel_name'] = c_helpers.SnakeToCamel(config.get('name'))
    self.ComputeParameters(self._config)
    self.CheckParameterValues(self._config)

  def _CheckExpectedParameters(self, config, expected_parameters):
    """Check configuration for expected parameters.

    Args:
      config: Specify a dict containing the per-device configuration.
      expected_parameters: Specify a list of keys to expect in config.

    Raises:
      ValueError: If config does not contain the expected parameters.
    """
    name = config.get('name')
    expected = set(expected_parameters)
    actual = set(config.keys())
    missing = expected - actual
    if missing:
      print 'Missing config parameters not specified for {}: {}.'.format(
          name, list(missing))
    extra = actual - expected & actual
    if extra:
      print 'Extra config parameters specified for {}: {}.'.format(
          name, list(extra))
    if expected != actual:
      raise ValueError('Invalid configuration for {}.'.format(name))

  def CheckParameterValues(self, config):  # pylint: disable=unused-argument
    """Check configuration parameter values.

    The monitor driver class can implement this function to verify the
    configuration parameter values.

    Args:
      config: The configuration dict from the python configuration file.
    """
    pass

  def ComputeParameters(self, config):  # pylint: disable=unused-argument
    """Compute derived configuration parameter values.

    This monitor driver class can implement this function to compute any
    derived parameter values.

    Args:
      config: The configuration dict from the python configuration file.
    """
    pass

  def GetConfigAsString(self, config, enum_values, index):  # pylint: disable=unused-argument
    """Output a C source code to initialize the device configuration.

    This function should return a string that initializes the device
    configuration structure. The string format should be similar to:

    [0] = {
      .monitor = kMotorMcp342xMonitorController1,
      .addr = 0x68,
      .config = {
        .channel = kMcp342xChannel1,
        .polarity = kMcp342xPolarityPositive,
        .mode = kMcp342xModeSingle,
        .sps = kMcp342xSps15,
        .gain = kMcp342xGain1X}},

    Args:
      config: The configuration dict from the python configuration file.
      enum_values: A dict mapping each enum group to an enumeration value.
      index: The array index.

    Returns:
      A string that initialized the device configuration structure.
    """
    raise NotImplementedError

  def GetHeaderFiles(self):
    return []

  def GetName(self):
    return self._config.get('name')

  def GetAddress(self):
    return self._config.get('address')

  def GetConfig(self):
    return self._config


class GenerateMonitorConfig(object):
  """Generate the monitor driver configuration in C.

  The motivation for this class is to generate a collection of monitor
  configurations for each hardware revisions.
  """

  def __init__(self, type_prefix, name_prefix, device_class,
               common_group='monitor', get_revision_fields=None):
    """Initialize the generator.

    Args:
      type_prefix: A snake_case string to prefix each monitor structure type.
      name_prefix: A snake_case string to prefix each enum and function name.
      device_class: A device class that inherits DeviceConfigBase.
      common_group: Name of group containing all monitors.
      get_revision_fields: A function that returns an initialization string
                           for extra fields in the revision map. Used by
                           the analog monitors to initialize channel_map.
    """
    self._type_prefix = c_helpers.SnakeToCamel(type_prefix)
    self._name_prefix = c_helpers.SnakeToCamel(name_prefix)
    self._device_class = device_class
    self._get_revision_fields = get_revision_fields
    self._groups = {}
    self._includes = []

    return_true = lambda x: True
    self.CreateGroup(name=common_group, is_member_function=return_true)

  def LoadConfig(self, revision_tuple, multiple_configs_per_device=False):
    """Load the python monitor driver revision configuration.

    This function processes the revision_tuple recursively, where
    revision_tuple specifies an enumeration and dict of configurations for
    each enumation value. Each configuration may specify a list of device
    configurations or refer to another revision_tuple for a multi-level
    configuration. In the following example, mcp342x_config represents the
    top-level revision_tuple, and tuples {protean|yasa}_config represent
    the second level revision_tuples. The generator uses this information
    to create a [MotorType][MotorHardware] configuration map.

    controller = [
        dict(ch1_pos, name='controller_1', address=0x68),
        dict(ch4_pos, name='controller_2', address=0x68),
        dict(ch3_pos, name='controller_3', address=0x68),
    ]

    protean = [
        dict(ch1_neg, name='stator_1', address=0x6C),
        dict(ch2_neg, name='stator_2', address=0x6C),
        dict(ch2_pos, name='stator_3', address=0x68),
    ]

    yasa = [
        dict(ch3_neg, name='rotor', address=0x6C),
        dict(ch1_neg, name='stator_1', address=0x6C),
        dict(ch2_neg, name='stator_2', address=0x6C),
        dict(ch2_pos, name='pylon_ambient', address=0x68),
    ]

    protean_config = (rev.MotorHardware, {
        rev.MotorHardware.GIN_A1: controller + protean,
        rev.MotorHardware.GIN_A2: controller + protean,
        rev.MotorHardware.GIN_A3: controller + protean,
    })

    yasa_config = (rev.MotorHardware, {
        rev.MotorHardware.GIN_A1: controller + yasa,
        rev.MotorHardware.GIN_A2: controller + yasa,
        rev.MotorHardware.GIN_A3: controller + yasa,
    })

    mcp342x_config = (config_params.MotorType, {
      config_params.MotorType.PROTEAN: protean_config,
      config_params.MotorType.YASA: yasa_config,
    })

    Args:
      revision_tuple: A tuple of (revision_enum, revision_config), where
                      revision_enum specifies the enumeration helper for
                      which each configuration corresponds to, and
                      revision_config specifies a dict mapping each
                      enumeration value to a configuration list.
      multiple_configs_per_device: Allow multiple configs per device.
    """
    self._enum_path = []
    self._multiple_configs_per_device = multiple_configs_per_device
    self._revision_by_path = {}
    self._LoadRevisionRecursive([], revision_tuple)

  def _LoadRevisionRecursive(self, path, revision_tuple):
    """Recursively load a revision tuple.

    Args:
      path: A list of pack2 enum names describing the configuration type.
      revision_tuple: A tuple of (revision_enum, revision_config), where
                      revision_enum specifies the enumeration helper for
                      which each configuration corresponds to, and
                      revision_config specifies a dict mapping each
                      enumeration value to a configuration list.
    """
    revision_enum, revision_config_by_enum = revision_tuple
    self._includes += [revision_enum.HeaderFile()]
    self._CheckRevisionEnum(path, revision_enum, revision_config_by_enum.keys())
    if len(self._enum_path) <= len(path):
      self._enum_path.append(revision_enum)

    for name, revision_config in revision_config_by_enum.iteritems():
      sub_path = path + [name]
      if isinstance(revision_config, tuple):
        self._LoadRevisionRecursive(sub_path, revision_config)
      else:
        self._CheckRevisionPath(sub_path)
        self._CheckDeviceNames(sub_path, revision_config)
        self._CheckDeviceAddresses(sub_path, revision_config)
        revision = self._LoadRevisionConfig(revision_config)
        self._revision_by_path[tuple(sub_path)] = revision

  def _LoadRevisionConfig(self, revision_config):
    """Load the configuration for one revision (a list of devices).

    Args:
      revision_config: A list of dicts that describe the configuration for
                       each device.

    Returns:
      A dict describing the text formatted revision configuration.
    """
    # Sort configurations such that all builds produce identical code.
    revision_config = sorted(revision_config, key=lambda c: c['name'])

    # In some implementations (e.g., MCP342x, ADS7828), we specify multiple
    # configurations per device. Since the I2C monitoring code performs one
    # I2C transaction per configuration, rather than waiting for one
    # device to complete its measurement cycle, we group configurations
    # per device. The monitor code then executes one I2C transaction per
    # device. When the associated I2C driver completes a measurement
    # cycle, the monitoring code executes the next configuration.
    devices = []
    devices_by_address = {}
    for device in revision_config:
      device = self._device_class(device)
      address = device.GetAddress()
      if address in devices_by_address:
        devices_by_address[address].append(device)
      else:
        devices_by_address[address] = [device]
      devices.append(device)

    # Generate the source code to initialize the per device configurations.
    text_by_address = {}
    for address, device_list in devices_by_address.iteritems():
      string = ''
      for index, device in enumerate(device_list):
        enum_values = self.GetGroupEnumValues(device)
        string += device.GetConfigAsString(device.GetConfig(),
                                           enum_values, index)
      text_by_address[address] = string

    # Generate the source code to initialize all devices (for when only
    # one configuration exists per device).
    text = ''
    for index, device in enumerate(revision_config):
      device = self._device_class(device)
      enum_values = self.GetGroupEnumValues(device)
      text += device.GetConfigAsString(device.GetConfig(),
                                       enum_values, index)
      self._includes += device.GetHeaderFiles()

    # Output dict.
    revision = {
        'text': text,
        'text_by_address': text_by_address,
        'devices': devices,
        'devices_by_address': devices_by_address,
    }
    return revision

  def _CheckRevisionPath(self, path):
    """Check consistency between revision paths."""
    # Some boards have a multi-level revision path. Here, we want to ensure
    # a consistent type between all nodes at the same depth in the path. So,
    # for the motors, where we have [MotorType, MotorHardware], we want to
    # verify that all names at the MotorHardware depth have enumeration type
    # MotorHardware.
    for i, p in enumerate(path):
      if self._enum_path[i].TypeName() != p.TypeName():
        a_types = [a.TypeName() for a in self._enum_path[:i + 1]]
        b_types = [b.TypeName() for b in path[:i + 1]]
        raise ValueError('Inconsistent revision path: '
                         '{} vs {}.'.format(a_types, b_types))

  def _CheckRevisionEnum(self, path, revision_enum, enum_names):
    """Check for a configuration for all revisions."""
    # This function is particularly helpful when we add a new board revision.
    expected = set(n for n, v in revision_enum.iteritems())
    actual = set(str(n) for n in enum_names)
    missing = expected - actual
    if missing:
      print '{} value {} not specified.'.format(
          revision_enum.TypeName(), list(missing))
    extra = actual - expected & actual
    if extra:
      print '{} does not contain value {}.'.format(
          revision_enum.TypeName(), list(extra))
    if expected != actual:
      raise ValueError('Configuration must specify all values in '
                       '{} for {}.'.format(path, revision_enum.TypeName()))

  def _CheckDeviceNames(self, path, revision_config):
    """Check for duplicate device names within one revision."""
    names = [c['name'] for c in revision_config]
    dups = list(set(n for n in names if names.count(n) > 1))
    if dups:
      raise ValueError('Duplicate device name found in revision {}: '
                       '{}.'.format(path, dups))

  def _CheckDeviceAddresses(self, path, revision_config):
    """Check for duplicate or invalid I2C addresses within one revision."""
    addresses = [c['address'] for c in revision_config
                 if 'address' in c]
    dups = list(set(hex(a) for a in addresses if addresses.count(a) > 1))
    invalid = list(set(hex(a) for a in addresses if a < 0 or 127 < a))

    if not self._multiple_configs_per_device and dups:
      raise ValueError('Duplicate address found in revision {}: '
                       '{}.'.format(path, dups))
    if invalid:
      raise ValueError('Invalid I2C address found in revision {}: '
                       '{}.'.format(path, invalid))

  def CreateGroup(self, name, is_member_function):
    """Create an enumeration group.

    This function allows the creation of sub-group enumerations of the
    monitors. See the 'voltage' sub-group of the analog monitors as an
    example.

    Args:
      name: The group name in snake case (corresponds to enum prefix).
      is_member_function: A function with prototype f(config) to determine if
                          a device is part of the group.
    """
    self._groups[name] = {
        'is_member_function': is_member_function
    }

  def IsDeviceInGroup(self, group, device):
    return group['is_member_function'](device.GetConfig())

  def GetMembersInGroup(self, group):
    """Get all members in group as a sorted list."""
    members = set()
    for revision in self._revision_by_path.values():
      for device in revision['devices']:
        if self.IsDeviceInGroup(group, device):
          members.add(device.GetName())
    return sorted(list(members))

  def GetGroupEnumValues(self, device):
    """Get group enumeration values for a particular device.

    See the analog monitors. The analog monitors defines groups 'input' and
    'voltage'. This function creates a dict mapping 'input' and 'voltage' to
    the device's respective enumeration value for each type.

    Args:
      device: A device object that inherits DeviceConfigBase.

    Returns:
      A dict mapping each group name to an enumeration value.
    """
    enum_values = {}
    for name, group in self._groups.iteritems():
      prefix = 'k' + self._GetEnumPrefix(name)
      if self.IsDeviceInGroup(group, device):
        enum_values[name] = prefix + c_helpers.SnakeToCamel(device.GetName())
      else:
        enum_values[name] = prefix + 'ForceSigned'  # Not supported.
    return enum_values

  def GenerateMap(self):
    """Generate the map from revision to monitor configuration."""
    # Parts in output.
    parts = []

    # Store device reference (i.e., variable name) as a map from the  generated
    # source code to the device reference. We use this mapping to eliminate
    # redundent configurations.
    device_ref_by_text = {}

    # Relate revision path to device reference for generating the revision map.
    device_ref_by_path = {}

    # Loop for all revisions in sorted order.
    path_revision = [(p, r) for p, r in self._revision_by_path.iteritems()]
    for path, revision in sorted(path_revision):
      # If device_text exists in device_ref_by_text, then device_ref will
      # reference an existing device. If not, device_ref_by_text will be
      # updated with device_text and a new reference.
      num_devices = len(revision['devices'])
      device_text = revision['text']
      device_ref, device_parts = self._GenerateDeviceReference(
          'Monitor', path, num_devices, device_text, device_ref_by_text)
      parts.extend(device_parts)

      # Used to compute the revision map.
      device_ref_by_path[tuple(path)] = device_ref

    # Generate kRevisionMap.
    parts.append(self._GenerateRevisionMap(device_ref_by_path, 'devices'))
    return '\n'.join(parts)

  def GenerateMapByAddress(self):
    """Generate the map from revision to per-device monitor configuration."""
    # Parts in output.
    parts = []

    # Store config/device reference (i.e., variable name) as a map from the
    # generated source code to the config/device reference. We use this
    # mapping to eliminate redundent configurations.
    config_ref_by_text = {}
    device_ref_by_text = {}

    # Relate revision path to device reference for generating the revision map.
    device_ref_by_path = {}

    # Loop for all revisions in sorted order.
    path_revision = [(p, r) for p, r in self._revision_by_path.iteritems()]
    for path, revision in sorted(path_revision):
      # If config_text exists in config_ref_by_text, then config_parts will
      # be empty. If not, config_ref_by_text will be updated with config_text
      # and a new reference.
      config_parts, device_parts = self._GenerateConfigDeviceText(
          path, revision, config_ref_by_text)
      parts.extend(config_parts)

      # If device_text exists in device_ref_by_text, then device_ref will
      # reference an existing device. If not, device_ref_by_text will be
      # updated with device_text and a new reference.
      num_devices = len(revision['devices_by_address'])
      device_text = '\n'.join(device_parts)
      device_ref, device_parts = self._GenerateDeviceReference(
          'MonitorDevice', path, num_devices, device_text, device_ref_by_text)
      parts.extend(device_parts)

      # Used to compute the revision map.
      device_ref_by_path[tuple(path)] = device_ref

    # Generate kRevisionMap.
    parts.append(self._GenerateRevisionMap(device_ref_by_path,
                                           'devices_by_address'))
    return '\n'.join(parts)

  def _GenerateConfigDeviceText(self, path, revision, config_ref_by_text):
    """Helper function for GenerateMapByAddress.

    Args:
      path: A list of pack2 enum names describing the configuration type.
      revision: A dict describing the source code associated with this revision.
      config_ref_by_text: A dict mapping the source code initialization to
                          variable reference. This function updates this map.

    Returns:
      A tuple containing the source code initialization for config and device
      structures.
    """
    # Parts in output.
    config_parts = []
    device_parts = []

    # Generate config structure for each device.
    addresses = revision['text_by_address'].keys()
    for config_index, address in enumerate(addresses):
      num_configs = len(revision['devices_by_address'][address])

      # If config_text exists in config_ref_by_text, do not duplicate.
      config_text = revision['text_by_address'][address]
      if config_text in config_ref_by_text:
        config_ref = config_ref_by_text[config_text]
      else:
        config_ref = 'kConfig{}Address0x{:02X}'.format(
            self._PathToString(path), address)
        config_ref_by_text[config_text] = config_ref
        config_parts.append(textwrap.dedent("""\
            static const {type_prefix}MonitorConfig {ref}[{num}] = {{
            {text}}};
            """).format(type_prefix=self._type_prefix, ref=config_ref,
                        text=c_helpers.Indent(config_text),
                        num=num_configs))

      # Generate device text using remapped config_ref such to eliminate
      # redundent device structures.
      device_parts.append(textwrap.dedent("""\
          [{config_index}] = {{
            .config = {config_ref},
            .num_configs = {num_configs}}},
          """).format(config_index=config_index,
                      config_ref=config_ref,
                      num_configs=num_configs))

    return (config_parts, device_parts)

  def _GenerateDeviceReference(self, type_name, path, num_devices, device_text,
                               device_ref_by_text):
    """Helper function for GenerateMap and GenerateMapByAddress.

    Args:
      type_name: Specify 'Monitor' or 'MonitorDevice'.
      path: A list of pack2 enum names describing the configuration type.
      num_devices: Total number of devices in list.
      device_text: Source code initialization of device corresponding to path.
      device_ref_by_text: A dict mapping the source code initialization to
                          variable reference. This function updates this map.

    Returns:
      A tuple containing the device reference and a list of source code outputs.
    """
    # Parts in output.
    device_parts = []

    # If device_text exists in device_ref_by_text, do not duplicate.
    if device_text in device_ref_by_text:
      device_ref = device_ref_by_text[device_text]
    elif num_devices > 0:
      device_ref = 'kDevice{}'.format(self._PathToString(path))
      device_ref_by_text[device_text] = device_ref
      device_parts.append(textwrap.dedent("""\
          static const {type_prefix}{type_name} {device_ref}[{num}] = {{
          {text}}};
          """).format(type_prefix=self._type_prefix,
                      type_name=type_name,
                      device_ref=device_ref,
                      text=c_helpers.Indent(device_text),
                      num=num_devices))
    else:
      device_ref = 'NULL'
      device_ref_by_text[device_text] = device_ref

    return (device_ref, device_parts)

  def _GenerateRevisionMap(self, device_ref_by_path, revision_key):
    dim = ''.join(['[{}]'.format(p.max_value + 1) for p in self._enum_path])
    text = self._GenerateRevisionMapRecursive(
        [], device_ref_by_path, revision_key)
    return textwrap.dedent("""\
        static const {type_prefix}Monitors kRevisionMap{dim} = {text};
        """).format(type_prefix=self._type_prefix, text=text, dim=dim)

  def _GenerateRevisionMapRecursive(self, path, device_ref_by_path,
                                    revision_key):
    """Generate map recursively for all revision paths."""
    parts = ['{']
    if len(path) == len(self._enum_path):
      revision = self._revision_by_path.get(tuple(path))
      if revision:
        populated = self._GetPopulated(revision)
        device_ref = device_ref_by_path[tuple(path)]
        num_devices = len(revision[revision_key])
        devices = revision['devices']
      else:
        populated = 0
        device_ref = 'NULL'
        num_devices = 0
        devices = []
      if self._get_revision_fields:
        parts.append(c_helpers.Indent(self._get_revision_fields(devices)))
      parts.append(c_helpers.Indent(textwrap.dedent("""\
          .populated = 0x{populated:08X},
          .device = {device_ref},
          .num_devices = {num_devices}}}
          """))[:-1].format(populated=populated,
                            device_ref=device_ref,
                            num_devices=num_devices))
    else:
      for name in self._enum_path[len(path)].Names():
        value = self._GenerateRevisionMapRecursive(
            path + [name], device_ref_by_path, revision_key)
        parts.append('  [{name}] = {value},'.format(
            name=name.CName(), value=c_helpers.Indent(value)[2:]))
      parts.append('}')
    return '\n'.join(parts)

  def _GetPopulated(self, revision):
    """Helper function to compute the bitmask of populated monitors."""
    populated = 0x0
    for index in range(len(revision['devices'])):
      populated |= 1 << index
    return populated

  def _PathToString(self, path):
    """Helper function to convert a path of pack2 types to a string."""
    return ''.join([str(s) for s in path])

  def GenerateIncludes(self):
    includes = sorted('#include "{}"'.format(i) for i in set(self._includes))
    return '\n'.join(includes)

  def GenerateDefines(self):
    """Generate helpful #defines for monitoring C code."""
    max_devices = max([len(r['devices_by_address'].keys())
                       for r in self._revision_by_path.values()])
    return textwrap.dedent("""\
        #define MAX_{name_upper}_DEVICES {max_devices}
        """).format(
            name_upper=c_helpers.CamelToSnake(self._name_prefix).upper(),
            max_devices=max_devices)

  def GenerateEnums(self):
    """Generate common enum that describes the monitor outputs."""
    parts = []
    for name, group in self._groups.iteritems():
      parts.append(self._GenerateEnum(name, self.GetMembersInGroup(group)))
    return '\n'.join(parts)

  def _GenerateEnum(self, name, members):
    """Generate common enum that describes the monitor outputs."""
    enum_prefix = self._GetEnumPrefix(name)
    parts = [textwrap.dedent("""\
        typedef enum {{
          k{enum_prefix}ForceSigned = -1,""").format(enum_prefix=enum_prefix)]
    for value, name in enumerate(sorted(members)):
      parts.append('  k{enum_prefix}{name} = {value},'.format(
          enum_prefix=enum_prefix,
          name=c_helpers.SnakeToCamel(name),
          value=value))
    parts.append(textwrap.dedent("""\
          kNum{enum_prefix}s = {num}
        }} {enum_prefix};
        """).format(enum_prefix=enum_prefix, num=len(members)))
    return '\n'.join(parts)

  def _GetEnumPrefix(self, name):
    return self._name_prefix + c_helpers.SnakeToCamel(name)

  def GenerateGetConfigPrototype(self):
    """Generate GetConfig function prototype."""
    f = lambda p: p.TypeName() + ' ' + c_helpers.CamelToSnake(p.TypeName())
    args = ', '.join(f(p) for p in self._enum_path)
    return 'const {name}Monitors *{prefix}GetConfig({args})'.format(
        name=self._type_prefix,
        prefix=self._name_prefix,
        args=args)

  def GenerateGetConfigFunction(self):
    """Generate GetConfig function source."""
    f = lambda p: '{}'.format(c_helpers.CamelToSnake(p.TypeName()))
    path = self._enum_path
    variables = ['int32_t i_{0} = (int32_t){0};'.format(f(p)) for p in path]
    checks = ['{min} <= i_{name} && i_{name} <= {max}'.format(
        min=p.min_value, max=p.max_value, name=f(p)) for p in path]
    deref = ''.join('[i_{}]'.format(f(p)) for p in path)

    return textwrap.dedent("""\
        {prototype} {{
          // Avoid "always true" compiler warnings.
          {variables}
          if ({checks}) {{
            return &kRevisionMap{deref};
          }}
          return NULL;
        }}
        """).format(prototype=self.GenerateGetConfigPrototype(),
                    variables='\n  '.join(variables),
                    checks='\n      && '.join(checks),
                    deref=deref)

  def GenerateHeader(self, header_file):
    """Generate header file source."""
    guard = re.sub('[/.]', '_', header_file).upper() + '_'
    string = textwrap.dedent("""\
        #ifndef {guard}
        #define {guard}

        #include <stdbool.h>

        {includes}

        {defines}
        {enums}
        {get_config};

        #endif  // {guard}
        """).format(guard=guard,
                    includes=self.GenerateIncludes(),
                    defines=self.GenerateDefines(),
                    enums=self.GenerateEnums(),
                    get_config=self.GenerateGetConfigPrototype())
    return string

  def GenerateSource(self, header_file):
    """Generate source file source."""

    if self._multiple_configs_per_device:
      config_map = self.GenerateMapByAddress()
    else:
      config_map = self.GenerateMap()

    string = textwrap.dedent("""\
        #include "{header_file}"

        #include <stdbool.h>
        #include <stddef.h>
        #include <stdint.h>

        {includes}

        {config_map}
        {get_config}
        """)[:-1].format(header_file=header_file,
                         includes=self.GenerateIncludes(),
                         config_map=config_map,
                         get_config=self.GenerateGetConfigFunction())
    return string


def ParseFlags(argv):
  """Parse common command line arguments."""
  gflags.DEFINE_string('config_file', None,
                       'Full path to Python config file.')
  gflags.DEFINE_string('prefix', None,
                       'Function and enumeration prefix name (snake case).')
  gflags.DEFINE_string('source_file', None,
                       'Full path to output source file.')
  gflags.DEFINE_string('header_file', None,
                       'Full path to output header file.')
  gflags.DEFINE_string('autogen_root', makani.HOME,
                       'Root of the source tree for the output files.')
  gflags.MarkFlagAsRequired('prefix')
  gflags.MarkFlagAsRequired('config_file')

  try:
    argv = gflags.FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], gflags.FLAGS)
    sys.exit(1)

  return gflags.FLAGS


def GetConfigModule(flags):
  """Get python configuration module from config_file command line argument."""
  config_modpath = flags.config_file.rsplit(os.path.extsep, 1)[0]
  config_modpath = 'makani.' + re.sub('[/]', '.', config_modpath)
  return importlib.import_module(config_modpath)


def WriteOutputFiles(flags, generator):
  """Write output files."""
  header = ''
  if flags.header_file:
    header = os.path.relpath(flags.header_file, start=flags.autogen_root)
    with open(flags.header_file, 'w') as f:
      f.write(generator.GenerateHeader(header))

  if flags.source_file:
    with open(flags.source_file, 'w') as f:
      f.write(generator.GenerateSource(header))
