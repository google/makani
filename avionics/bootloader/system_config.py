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

"""Wrapper for a wing configuration file."""

import os

import makani
from makani.avionics.network import network_config
from makani.control import system_types
from makani.lib.python import c_helpers
import yaml


_WING_SERIAL_HELPER = c_helpers.EnumHelper('WingSerial', system_types,
                                           prefix='kWingSerial')


class SystemConfig(object):
  """Wrapper for a wing configuration YAML file."""

  @staticmethod
  def _LoadSystemYaml(yaml_file=None):
    if not yaml_file:
      yaml_file = os.path.join(makani.HOME,
                               'avionics/bootloader/system_config.yaml')
    with open(yaml_file, 'r') as f:
      y = yaml.full_load(f)
    for k, v in y.items():
      y[k.lower()] = v
    return y

  @staticmethod
  def GetAllSystemNames(yaml_file=None):
    y = SystemConfig._LoadSystemYaml(yaml_file)
    return y.keys()

  @staticmethod
  def GetSystemConfigBySerial(serial, yaml_file=None):
    """Returns the configuration corresponding to the given wing serial."""
    system_config = None
    y = SystemConfig._LoadSystemYaml(yaml_file)
    for name, system_yaml in y.iteritems():
      if 'serial' in system_yaml:
        if _WING_SERIAL_HELPER.Value(system_yaml['serial']) == serial:
          if system_config is None:
            system_config = SystemConfig(name, yaml_file)
          else:
            raise ValueError('Multiple configs found for given wing serial.')
    if system_config is None:
      raise ValueError('Could not find configuration for given wing serial.')
    else:
      return system_config

  def _ProcessNodeList(self, node_list):
    """Returns the set of nodes specified in node_list."""
    nodes = set()
    for n in node_list:
      if n.startswith('label:'):
        label_nodes = self._net_config.GetAioNodesByLabel(n[len('label:'):])
        if not label_nodes:
          raise ValueError('Node label %s from system config not found.' % n)
        nodes.update(label_nodes)
      else:
        nodes.add(self._net_config.GetAioNode(n))
    return nodes

  def _LoadSystem(self, y, system_name):
    """Recursively load a system definition from the yaml file."""
    system = y[system_name.lower()]

    if 'base' in system:
      nodes, config, calib = self._LoadSystem(y, system['base'])
    else:
      nodes = set()
      config = {}
      calib = {}

    if 'nodes' in system:
      nodes |= self._ProcessNodeList(system['nodes'])
    if 'exclude_nodes' in system:
      nodes -= self._ProcessNodeList(system['exclude_nodes'])

    if 'config' in system:
      for node_name, conf in system['config'].iteritems():
        if 'config' in conf:
          config[self._net_config.GetAioNode(node_name)] = conf['config']
        if 'calib' in conf:
          calib[self._net_config.GetAioNode(node_name)] = conf['calib']

    return nodes, config, calib

  def __init__(self, system_name, yaml_file=None, net_config=None):
    if net_config:
      self._net_config = net_config
    else:
      self._net_config = network_config.NetworkConfig()

    y = SystemConfig._LoadSystemYaml(yaml_file)

    nodes, self.config, self.calib = self._LoadSystem(y, system_name)
    self.nodes = sorted(nodes, key=lambda x: x.snake_name)
