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

"""Monitor indicators for the short stack."""

from makani.avionics.firmware.monitors import short_stack_types
from makani.gs.monitor2.apps.layout import indicator
from makani.gs.monitor2.apps.layout import stoplights
from makani.lib.python import c_helpers
from makani.lib.python import struct_tree

SHORT_STACK_GPIO_HELPER = c_helpers.EnumHelper('ShortStackGpioInputPin',
                                               short_stack_types)

SHORT_STACK_STATUS_HELPER = c_helpers.EnumHelper('ShortStackStatus',
                                                 short_stack_types)


class ShortStackFlightIndicator(indicator.BaseAttributeIndicator):
  """A compact indicator for flight page to show short stack state."""

  def __init__(self):
    super(ShortStackFlightIndicator, self).__init__([
        ('ShortStackStatus', None, 'short_stack_mon.flags.status'),
        ('ShortStackStatus', None, 'short_stack_mon.gpio_inputs'),
    ], 'Short stack')

  def _IsValidInput(self, status_flag, gpio_inputs):
    return status_flag is not None and gpio_inputs is not None

  @indicator.ReturnIfInputInvalid('--', stoplights.STOPLIGHT_UNAVAILABLE)
  def _Filter(self, status_flag, gpio_inputs):
    # Error stoplight if a level is tripped.
    if status_flag & SHORT_STACK_STATUS_HELPER.Value('TrippedB0'):
      return 'B0 tripped (Sbo/Pto)', stoplights.STOPLIGHT_ERROR
    elif status_flag & SHORT_STACK_STATUS_HELPER.Value('TrippedB1'):
      return 'B1 tripped (Sbi/Pti)', stoplights.STOPLIGHT_ERROR
    elif status_flag & SHORT_STACK_STATUS_HELPER.Value('TrippedB2'):
      return 'B2 tripped (Pbi/Sti)', stoplights.STOPLIGHT_ERROR
    elif status_flag & SHORT_STACK_STATUS_HELPER.Value('TrippedB3'):
      return 'B3 tripped (Pbo/Sto)', stoplights.STOPLIGHT_ERROR
    # Warning status if in force-no-trips mode.
    elif status_flag & SHORT_STACK_STATUS_HELPER.Value('ForceNoTrips'):
      return 'force-no-trips mode', stoplights.STOPLIGHT_WARNING
    # Green if in default mode with no trips. Notify if armed.
    elif gpio_inputs & (1 << SHORT_STACK_GPIO_HELPER.Value('XArmed')):
      return 'Normal, Armed', stoplights.STOPLIGHT_NORMAL
    return 'Normal, Unarmed', stoplights.STOPLIGHT_NORMAL


class ShortStackGpioIndicator(indicator.BaseAttributeIndicator):
  """Indicator to display short stack GPIO pin status."""

  def __init__(self, name='GPIO status'):
    super(ShortStackGpioIndicator, self).__init__(
        [('ShortStackStatus', 'ShortStack')], name)

  def _Filter(self, short_stack_status):
    if struct_tree.IsValidElement(short_stack_status):
      gpio_readout = []
      for gpio_pin in SHORT_STACK_GPIO_HELPER.Names():
        pin_value = (1 if (short_stack_status.short_stack_mon.gpio_inputs
                           & (1 << SHORT_STACK_GPIO_HELPER.Value(gpio_pin)))
                     else 0)
        gpio_readout.append(SHORT_STACK_GPIO_HELPER.ShortName(gpio_pin)
                            + ': ' + str(pin_value))
      return ('\n'.join(gpio_readout), stoplights.STOPLIGHT_NORMAL)
    else:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE


class ShortStackStatusIndicator(indicator.BaseAttributeIndicator):
  """Indicator to display short stack StatusFlags status."""

  def __init__(self, name='Flags status'):
    super(ShortStackStatusIndicator, self).__init__(
        [('ShortStackStatus', 'ShortStack')], name)

  def _Filter(self, short_stack_status):
    if struct_tree.IsValidElement(short_stack_status):
      status_readout = []
      for status_name in SHORT_STACK_STATUS_HELPER.Names():
        status_val = (1 if (short_stack_status.short_stack_mon.flags.status &
                            SHORT_STACK_STATUS_HELPER.Value(status_name))
                      else 0)
        status_readout.append(SHORT_STACK_STATUS_HELPER.ShortName(status_name)
                              + ': ' + str(status_val))
      return ('\n'.join(status_readout), stoplights.STOPLIGHT_NORMAL)
    else:
      return '--', stoplights.STOPLIGHT_UNAVAILABLE
