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

"""Report the various flap limits applied in the system."""

import collections
import math
import os

import makani
from makani.avionics.bootloader import system_config
from makani.avionics.firmware.params import codec
from makani.avionics.network import network_config
from makani.config import mconfig
from makani.control import system_types

_CONFIG = mconfig.MakeParams('common.all_params')
_SERVOS = ['A' + f for f in '124578'] + ['E1', 'E2', 'R1', 'R2']
_FLAPS = _SERVOS[0:6] + ['Elevator', 'Rudder']


def GetSimLimits():
  servo_limits = [_CONFIG['sim']['servos_sim'][i]['servo_drive']
                  for i in range(len(_SERVOS))]
  return {_SERVOS[i]: (servo_limits[i]['ref_model_min_position_limit'],
                       servo_limits[i]['ref_model_max_position_limit'])
          for i in range(len(_SERVOS))}


def GetControlLimits():
  return {_FLAPS[i]: (_CONFIG['control']['control_output']['flaps_min'][i],
                      _CONFIG['control']['control_output']['flaps_max'][i])
          for i in range(len(_FLAPS))}


def _GetControlOutputLimits(output, lower_limit='lower_flap_limits',
                            upper_limit='upper_flap_limits'):
  return {_FLAPS[i]: (output[lower_limit][i], output[upper_limit][i])
          for i in range(len(_FLAPS))}


def GetControlCrosswindLimits():
  return _GetControlOutputLimits(_CONFIG['control']['crosswind']['output'])


def GetControlCrosswindFlareLimits():
  return _GetControlOutputLimits(_CONFIG['control']['crosswind']['output'],
                                 lower_limit='lower_flap_limits_flare')


def GetControlTransInLimits():
  return _GetControlOutputLimits(_CONFIG['control']['trans_in']['output'])


def GetControlHoverLimits():
  return _GetControlOutputLimits(_CONFIG['control']['hover']['output'])


def GetControlManualLimits():
  return _GetControlOutputLimits(_CONFIG['control']['manual']['output'])


def GetAvionicsServoLimits():
  """Get the avionics servo mechanical limits for the current system."""
  sys_conf = system_config.SystemConfig.GetSystemConfigBySerial(
      _CONFIG['system']['wing_serial'])
  config_file = os.path.join(makani.HOME,
                             'avionics/servo/firmware/config_params.yaml')
  net_conf = network_config.NetworkConfig()

  yaml_keys = [sys_conf.config[net_conf.GetAioNode('servo_%s' % s.lower())]
               for s in _SERVOS]
  limits = [codec.DecodeYamlFile(config_file, key) for key in yaml_keys]
  return {_SERVOS[i]: (limits[i].servo_min_limit, limits[i].servo_max_limit)
          for i in range(len(_SERVOS))}


def _RudderServoToFlap(servo_angle):
  servo_config = _CONFIG['system']['servos'][system_types.kServoR1]
  return servo_config['nonlinear_servo_to_flap_ratio'] * math.sin(servo_angle)


def _RudderFlapToServo(flap_angle):
  servo_config = _CONFIG['system']['servos'][system_types.kServoR1]
  return math.asin(flap_angle / servo_config['nonlinear_servo_to_flap_ratio'])


def FlapsToServos(flaps):
  """Convert flap limits to servo limits."""
  servos = {}
  for flap, flap_range in flaps.iteritems():
    if flap.startswith('A'):
      servos[flap] = flap_range
    elif flap == 'Elevator':
      servos['E1'] = flap_range
      servos['E2'] = flap_range
    elif flap == 'Rudder':
      servos['R1'] = (_RudderFlapToServo(flap_range[0]),
                      _RudderFlapToServo(flap_range[1]))
      servos['R2'] = servos['R1']
    else:
      raise ValueError('Invalid flap %s' % flap)
  return servos


def ServosToFlaps(servos):
  """Convert servo limits to flap limits."""
  flaps = {}
  for servo, servo_range in servos.iteritems():
    if servo.startswith('A'):
      flaps[servo] = servo_range
  flaps['Elevator'] = (0.5 * (servos['E1'][0] + servos['E2'][0]),
                       0.5 * (servos['E1'][1] + servos['E2'][1]))
  flaps['Rudder'] = (
      _RudderServoToFlap(0.5 * (servos['R1'][0] + servos['R2'][0])),
      _RudderServoToFlap(0.5 * (servos['R1'][1] + servos['R2'][1])))
  return flaps


def _PrintFlaps(name, limits, print_header):
  if print_header:
    print '%15s         A1    A2    A4    A5    A7    A8   Ele   Rud' % ''
  print ('%15s min:' + ' %5.1f' * 8) % (
      (name,) + tuple([math.degrees(limits[_FLAPS[i]][0])
                       for i in range(len(_FLAPS))]))
  print ('%15s max:' + ' %5.1f' * 8) % (
      ('',) + tuple([math.degrees(limits[_FLAPS[i]][1])
                     for i in range(len(_FLAPS))]))


def _PrintServos(name, limits, print_header):
  if print_header:
    print ('%15s         A1    A2    A4    A5    A7    A8'
           '    E1    E2    R1    R2' % '')
  print ('%15s min:' + ' %5.1f' * 10) % (
      (name,) + tuple([math.degrees(limits[_SERVOS[i]][0])
                       for i in range(len(_SERVOS))]))
  print ('%15s max:' + ' %5.1f' * 10) % (
      ('',) + tuple([math.degrees(limits[_SERVOS[i]][1])
                     for i in range(len(_SERVOS))]))


Limit = collections.namedtuple('Limit', ['name', 'limits', 'is_flap_limit'])


_LIMITS = [
    Limit('Controller', GetControlLimits(), True),
    Limit('Cont. Crosswind', GetControlCrosswindLimits(), True),
    Limit('Cont. Flare', GetControlCrosswindFlareLimits(), True),
    Limit('Cont. Trans In', GetControlTransInLimits(), True),
    Limit('Cont. Hover', GetControlHoverLimits(), True),
    Limit('Cont. Manual', GetControlManualLimits(), True),
    Limit('Servos', GetAvionicsServoLimits(), False),
    Limit('Simulator', GetSimLimits(), False),
]


def GetServoLimits():
  return [(l.name, FlapsToServos(l.limits) if l.is_flap_limit else l.limits)
          for l in _LIMITS]


def GetFlapLimits():
  return [(l.name, l.limits if l.is_flap_limit else ServosToFlaps(l.limits))
          for l in _LIMITS]


def Main():
  print '\nFlap limits:'
  print_header = True
  for name, flaps in GetFlapLimits():
    _PrintFlaps(name, flaps, print_header)
    print_header = False
  print '\nServo limits:'
  print_header = True
  for name, servos in GetServoLimits():
    _PrintServos(name, servos, print_header)
    print_header = False


if __name__ == '__main__':
  Main()
