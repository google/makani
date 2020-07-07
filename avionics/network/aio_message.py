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

"""Collect all message type information into one common module."""

import collections
import os

import makani
from makani.avionics.common import pack_avionics_messages
from makani.avionics.network import network_config
from makani.control import pack_control_telemetry
from makani.control import pack_ground_telemetry
from makani.sim import pack_sim_messages
from makani.sim import pack_sim_telemetry

_PACK_MODULES = [
    pack_avionics_messages,
    pack_control_telemetry,
    pack_sim_messages,
    pack_sim_telemetry,
    pack_ground_telemetry]


def GetInfoByModule(module, struct_name):
  """Returns a namedtuple of useful type info."""
  Info = collections.namedtuple(  # pylint: disable=invalid-name
      'Info', ['module', 'struct_name', 'ctype', 'pack_size', 'unpack_size',
               'pack_func', 'unpack_func', 'header_file'])
  ctype = getattr(module, struct_name)
  return Info(module=module,
              struct_name=struct_name,
              ctype=ctype,
              pack_size='PACK_{}_SIZE'.format(struct_name.upper()),
              unpack_size='sizeof({})'.format(struct_name),
              pack_func='Pack{}'.format(struct_name),
              unpack_func='Unpack{}'.format(struct_name),
              header_file=module.H2PY_HEADER_FILE)


def GetInfoByName(name):
  """Returns a namedtuple of useful type info."""
  for module in _PACK_MODULES:
    struct_name = name + 'Message'
    if struct_name in module.__dict__:
      return GetInfoByModule(module, struct_name)
    if name in module.__dict__:
      return GetInfoByModule(module, name)


def GetMessageInfoMapByNetworkConfig(config):
  """Returns a map from MessageType to type info."""
  return {m: GetInfoByName(m.name) for m in config.aio_messages}


def GetMessageInfoMapByNetworkFile(
    network_file=os.path.join(makani.HOME, 'avionics/network/network.yaml')):
  """Returns a map from MessageType to type info."""
  config = network_config.NetworkConfig(network_file)
  return GetMessageInfoMapByNetworkConfig(config)


def GetHeaderFilesFromMessageInfoMap(message_info_map):
  """Returns a list of header files required for each message."""
  return list(set(i.header_file for i in message_info_map.values() if i))
