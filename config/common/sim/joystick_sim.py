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

"""Programmed joystick schedule."""

import sys

from makani.config import mconfig
from makani.sim import sim_types as m


@mconfig.Config(deps={
    'system': mconfig.WING_MODEL + '.system_params',
})
def MakeParams(params):
  throttle_settings = [0.0] * m.kNumSimJoystickThrottles
  throttle_settings[m.kSimJoystickThrottleOff] = 0.0
  throttle_settings[m.kSimJoystickThrottleEnterCrosswind] = 0.81
  throttle_settings[m.kSimJoystickThrottleRemainInHover] = 0.49
  throttle_settings[m.kSimJoystickThrottleReturnToPerch] = 0.39

  updates = []

  if params['system']['flight_plan'] == m.kFlightPlanDisengageEngage:
    updates = [{
        't_update': 0.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleManual,
        'value': 0.5,
    }, {
        't_update': 0.0,
        'type': m.kSimJoystickUpdateSwitchUp,
        'value': 0.0
    }, {
        't_update': 0.1,
        'type': m.kSimJoystickUpdateSwitchMiddle,
        'value': 0.0
    }, {
        't_update': 6.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleEnterCrosswind
    }, {
        't_update': 250.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleReturnToPerch
    }]
  elif params['system']['flight_plan'] == m.kFlightPlanHighHover:
    updates = [{
        't_update': 0.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleOff
    }, {
        't_update': 0.0,
        'type': m.kSimJoystickUpdateSwitchUp,
        'value': 0.0
    }, {
        't_update': 10.1,
        'type': m.kSimJoystickUpdateSwitchMiddle,
        'value': 0.0
    }, {
        't_update': 11.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleEnterCrosswind
    }, {
        't_update': 500.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleReturnToPerch
    }]

  elif params['system']['flight_plan'] in [m.kFlightPlanHoverInPlace]:
    updates = [{
        't_update': 0.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleManual,
        'value': 0.71
    }]
  elif params['system']['flight_plan'] == m.kFlightPlanLaunchPerch:
    updates = [{
        't_update': 0.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleOff
    }, {
        't_update': 0.0,
        'type': m.kSimJoystickUpdateSwitchUp,
        'value': 0.0
    }, {
        't_update': 5.1,
        'type': m.kSimJoystickUpdateSwitchMiddle,
        'value': 0.0
    }, {
        't_update': 6.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleEnterCrosswind
    }, {
        't_update': 120.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleReturnToPerch
    }]
  elif params['system']['flight_plan'] in [m.kFlightPlanTurnKey]:
    updates = [{
        't_update': 0.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleOff
    }, {
        't_update': 0.0,
        'type': m.kSimJoystickUpdateSwitchUp,
        'value': 0.0
    }, {
        't_update': 10.1,
        'type': m.kSimJoystickUpdateSwitchMiddle,
        'value': 0.0
    }, {
        't_update': 11.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleEnterCrosswind
    }, {
        't_update': 820.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleReturnToPerch
    }]
  elif params['system']['flight_plan'] == m.kFlightPlanStartDownwind:
    updates = [{
        't_update': 0.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleOff
    }, {
        't_update': 15.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleEnterCrosswind
    }, {
        't_update': 200.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleRemainInHover
    }, {
        't_update': 210.0,
        'type': m.kSimJoystickUpdateThrottle,
        'enum_value': m.kSimJoystickThrottleReturnToPerch
    }]

  no_update = {
      't_update': sys.float_info.max,
      'type': m.kSimJoystickUpdateNone,
      'value': 0.0
  }

  updates_padded = updates + [
      no_update.copy() for _ in range(m.MAX_JOYSTICK_UPDATES - len(updates))]

  # TODO: Consider using overrides_util._PreprocessJoystickSim.
  for update in updates_padded:
    if update['type'] == m.kSimJoystickUpdateThrottle:
      if update['enum_value'] != m.kSimJoystickThrottleManual:
        assert 'value' not in update, (
            'Value should not be specified for kSimJoystickUpdateThrottle '
            'unless it is a manual enum_value: %s.' % str(update))
        update['value'] = -1.0
    else:
      assert 'enum_value' not in update, (
          'Enum_value should not be specified for a joystick update unless it '
          'is kSimJoystickUpdateThrottle: %s.' % str(update))
      update['enum_value'] = 0

  return {
      'joystick_type': m.kSimJoystickTypeProgrammed,
      'num_updates': len(updates),
      'updates': updates_padded,
      'throttle_settings': throttle_settings
  }
