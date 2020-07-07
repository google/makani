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

"""Parameters used to configure the network.

The naming scheme for the UDP ports is:

  <struct name>_<type>_port

where <struct name> is a slight abbreviation of the structure name and
<type> is either wing, remote, sensor, or hitl depending on whether
the packet is transmitted on the wing network (192.168.2.*), remote
network (192.168.4.*), sensor network (192.168.3.*), or only during
HITL.
"""

from makani.avionics.common import network_config
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  """Create udpio params."""
  udpio = {
      'aio_telemetry_remote_addr': '127.255.255.255',
      'aio_telemetry_1_remote_port': 27203,
      'aio_telemetry_2_remote_port': 27204,
      'aio_telemetry_3_remote_port': 27205,
      'flight_gear_remote_addr': '127.0.0.1',
      'flight_gear_remote_port': 40022,
      'joystick_input_remote_port': 30003,
  }
  # Check that all the ports are unique.
  port_list = [port for port in udpio.itervalues() if isinstance(port, int)]
  assert len(port_list) == len(set(port_list))

  return {
      'udpio': udpio,

      # Port on which AIO communication occurs.
      'aio_port': network_config.UDP_PORT_AIO,
  }
