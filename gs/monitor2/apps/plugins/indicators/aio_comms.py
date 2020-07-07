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

"""Monitor indicators to check AIO update status."""

from makani.analysis.checks import avionics_util
from makani.avionics.common import pack_avionics_messages
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.lib.python import c_helpers


class AioCommIndicator(indicator.BaseIndicator):
  """Base class to check AIO communications.

  Attributes:
    _message_type: Short name of the message type.
    _aio_nodes: Short names of AIO nodes from where messages should be received.
  """

  def __init__(self, label, message_type, aio_nodes, normal_if_any=False):
    """Initialize an AIO Update Indicator.

    Show green if the message is sent from all specified AIO nodes. If only a
    subset of the AIO nodes are sending the message, show either green or
    yellow, depending on the value of `normal_if_any`. If none of the nodes is
    sending the message, show red.

    Args:
      label: Label of the indicator.
      message_type: Name of the message type.
      aio_nodes: AIO nodes that send the message type. It can be a dict of
          {aio_node_name: verbose_name} or a list of AIO node names.
      normal_if_any: If True, the indicator is green if any of the AIO nodes
          is sending the message. Otherwise, all AIO nodes must be sending
          the message or the indicator will show yellow.
    """
    super(AioCommIndicator, self).__init__(label)
    self._message_type = message_type
    if isinstance(aio_nodes, dict):
      self._aio_nodes = aio_nodes
    elif isinstance(aio_nodes, list):
      self._aio_nodes = {x: x for x in aio_nodes}
    else:
      self._aio_nodes = {aio_nodes: aio_nodes}
    self._normal_if_any = normal_if_any

  def Filter(self, messages):
    """Check whether a message type is received from the specified AIO nodes.

    Args:
      messages: StructTree of all received objects.

    Returns:
      A list of sender AIO nodes, sorted by short names.
      The stoplight to use.
    """

    expected_aio_nodes = set(self._aio_nodes.keys())
    nodes = messages.Skeleton(self._message_type, 1)
    updated_aio_nodes = set(nodes.keys()) if nodes else set()
    updated_aio_nodes &= expected_aio_nodes

    if expected_aio_nodes == updated_aio_nodes:
      stoplight = stoplights.STOPLIGHT_NORMAL
    elif updated_aio_nodes:
      stoplight = (stoplights.STOPLIGHT_NORMAL if self._normal_if_any
                   else stoplights.STOPLIGHT_WARNING)
    elif messages:
      stoplight = stoplights.STOPLIGHT_ERROR
    else:
      stoplight = stoplights.STOPLIGHT_UNAVAILABLE

    return sorted([self._aio_nodes[x] for x in updated_aio_nodes]), stoplight


class GsCoreSwitchAioUpdateIndicator(AioCommIndicator):

  def __init__(self):
    super(GsCoreSwitchAioUpdateIndicator, self).__init__(
        'Gs Core Switch', 'CoreSwitchStatus', ['CsGsA', 'CsGsB'])


class WingCoreSwitchAioUpdateIndicator(AioCommIndicator):

  def __init__(self):
    super(WingCoreSwitchAioUpdateIndicator, self).__init__(
        'Wing Core Switch', 'CoreSwitchStatus', ['CsA', 'CsB'])


class CoreSwitchAioUpdateIndicator(AioCommIndicator):

  def __init__(self):
    super(CoreSwitchAioUpdateIndicator, self).__init__(
        'Core Switch', 'CoreSwitchStatus', ['CsA', 'CsB', 'CsGsA', 'CsGsB'])


class ControllerAioUpdateIndicator(AioCommIndicator):

  def __init__(self):
    super(ControllerAioUpdateIndicator, self).__init__(
        'Controllers', 'ControlTelemetry',
        {'ControllerA': 'A', 'ControllerB': 'B', 'ControllerC': 'C'},
        normal_if_any=True)


class FlightComputerAioUpdateIndicator(AioCommIndicator):

  def __init__(self):
    super(FlightComputerAioUpdateIndicator, self).__init__(
        'Flight Computers', 'FlightComputerSensor', ['FcA', 'FcB', 'FcC'])


class JoystickAioUpdateIndicator(indicator.BaseIndicator):

  _MASK = c_helpers.EnumHelper(
      'JoystickWarning', pack_avionics_messages).Value('NotPresent')

  def __init__(self):
    super(JoystickAioUpdateIndicator, self).__init__('Joystick')

  def Filter(self, messages):

    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    status = messages['JoystickStatus.JoystickA.status']
    if status is None:
      return 'No Update', stoplights.STOPLIGHT_ERROR
    elif avionics_util.CheckWarning(status, self._MASK):
      return 'Not Present', stoplights.STOPLIGHT_ERROR
    else:
      return 'Up', stoplights.STOPLIGHT_NORMAL


class GsGpsAioUpdateIndicator(indicator.BaseIndicator):

  def __init__(self):
    super(GsGpsAioUpdateIndicator, self).__init__('GS GPS')

  def Filter(self, messages):
    if not messages:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE

    if ('NovAtelCompass.GpsBaseStation' in messages and
        'NovAtelSolution.GpsBaseStation' in messages):
      return 'Up', stoplights.STOPLIGHT_NORMAL
    else:
      return '--', stoplights.STOPLIGHT_ERROR


class PlatformSensorsAioUpdateIndicator(AioCommIndicator):

  def __init__(self):
    super(PlatformSensorsAioUpdateIndicator, self).__init__(
        'Platform Sensors', 'PlatformSensors',
        {'PlatformSensorsA': 'A', 'PlatformSensorsB': 'B'})


class WinchPlcAioUpdateIndicator(AioCommIndicator):

  def __init__(self):
    super(WinchPlcAioUpdateIndicator, self).__init__(
        'Winch PLC', 'GroundStationWinchStatus', ['Plc'])
