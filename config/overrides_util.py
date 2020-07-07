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


r"""Preprocesses overrides passed to the run_sim script.

This utility allows for more natural specification of certain
overrides parameters.

Overrides for sim_opt are passed as an array of strings giving the
names of SimOption values which are combined using bitwise or.

Overrides for sim.faults_sim are passed as an array of fault events with
num_fault_events inferred.  For each fault event:
  - t_end can be omitted indicating that the fault persists,
  - parameters does not need to be padded (num_parameters is inferred),
  - type is a string giving the name of a SimFaultType value.

Overrides for sim.joystick_sim are passed as an array of updates with
num_updates inferred.  For each update:
  - value can be omitted indicating a zero value,
  - type is a string giving the name of a SimJoystickUpdateType value.

E.g.:
  ./config/overrides_util.py \
    '{
         "sim": {
             "sim_opt": ["kSimOptImperfectSensors"],
             "faults_sim": [{
                 "t_start": 100.0,
                 "component": "IMU[0]/acc",
                 "type": "kSimFaultMeasurementNoiseRescale",
                 "parameters": [10.0, 10.0, 10.0]
             }],
             "joystick_sim": [{
                 "t_update": 5.0,
                 "type": "kSimJoystickUpdateSwitchMiddle"
             }, {
                 "t_update": 5.0,
                 "type": "kSimJoystickUpdateThrottle",
                 "value": 0.81
            }]
         }
     }'

"""

import copy
import functools
import json
import sys

import gflags
from makani.lib.python import c_helpers
from makani.sim import sim_types
import numpy as np

FLAGS = gflags.FLAGS

flight_plan_helper = c_helpers.EnumHelper('FlightPlan', sim_types)
test_site_helper = c_helpers.EnumHelper('TestSite', sim_types)
sim_joystick_type_helper = c_helpers.EnumHelper('SimJoystickType', sim_types)
sim_joystick_update_helper = c_helpers.EnumHelper('SimJoystickUpdate',
                                                  sim_types)
sim_joystick_throttle_helper = c_helpers.EnumHelper('SimJoystickThrottle',
                                                    sim_types)
sim_fault_helper = c_helpers.EnumHelper('SimFault', sim_types)
sim_opt_helper = c_helpers.EnumHelper('SimOpt', sim_types)
wind_model_helper = c_helpers.EnumHelper('WindModel', sim_types)


class InvalidOverrideUpdate(Exception):
  pass


class InvalidArrayLength(Exception):
  pass


class InvalidArgument(Exception):
  pass


class InvalidSimOptValue(Exception):
  pass


def _GetEnumValue(helper, name_or_value):
  """Maps an enum name to its value. Checks that a value is valid."""
  if isinstance(name_or_value, int):
    helper.Name(name_or_value)  # Raises EnumError if not a valid value.
    return name_or_value
  else:
    return helper.Value(name_or_value)


def _PreprocessSimOpt(sim_opts):
  if isinstance(sim_opts, int):
    return sim_opts
  elif isinstance(sim_opts, list):
    sim_opt = 0
    for opt in sim_opts:
      sim_opt |= _GetEnumValue(sim_opt_helper, opt)
    return sim_opt
  else:
    raise InvalidSimOptValue('Value of sim_opts should be an integer or list.')


def _PreprocessFaultsSim(fault_events):
  """Process overrides for sim.faults_sim."""

  if len(fault_events) > sim_types.MAX_FAULT_EVENTS:
    raise InvalidArrayLength('Too many fault events (%d).  The maximum is %d.'
                             % (len(fault_events), sim_types.MAX_FAULT_EVENTS))

  faults = copy.deepcopy(fault_events)
  for i in range(len(faults)):
    # String values are interpreted as enumerations.
    faults[i]['type'] = _GetEnumValue(sim_fault_helper, faults[i]['type'])

    if 't_end' not in faults[i]:
      faults[i]['t_end'] = sys.float_info.max

    if 'parameters' not in faults[i]:
      faults[i]['parameters'] = []

    faults[i]['num_parameters'] = len(faults[i]['parameters'])

    if len(faults[i]['parameters']) > sim_types.MAX_FAULT_EVENT_PARAMETERS:
      raise InvalidArrayLength('Too many parameters (%d) for fault event '
                               '%d.  The maximum is %d.'
                               % (len(faults[i]['parameters']), i,
                                  sim_types.MAX_FAULT_EVENT_PARAMETERS))

    # Pad parameters with zeros.
    faults[i]['parameters'] += [0.0] * (sim_types.MAX_FAULT_EVENT_PARAMETERS
                                        - faults[i]['num_parameters'])

  none_event = {
      't_start': 0.0,
      't_end': 0.0,
      'component': '',
      'type': sim_types.kSimFaultNoFault,
      'num_parameters': 0,
      'parameters': [0.0] * sim_types.MAX_FAULT_EVENT_PARAMETERS,
  }

  faults_sim_overrides = {
      'num_fault_events': len(faults),

      # Pad fault list with no fault events.
      'fault_events': faults + [copy.deepcopy(none_event)
                                for _ in range(sim_types.MAX_FAULT_EVENTS
                                               - len(faults))]
  }

  return faults_sim_overrides


def _PreprocessJoystickSim(joystick_sim):
  """Process overrides for sim.joystick_sim."""

  new_joystick_sim = copy.deepcopy(joystick_sim)

  if 'joystick_type' in joystick_sim:
    new_joystick_sim['joystick_type'] = _GetEnumValue(
        sim_joystick_type_helper, joystick_sim['joystick_type'])

  if 'updates' in joystick_sim:
    raw_updates = joystick_sim['updates']
    if len(raw_updates) > sim_types.MAX_JOYSTICK_UPDATES:
      raise InvalidArrayLength('Too many joystick updates (%d). The maximum is '
                               '%d.' % (len(raw_updates),
                                        sim_types.MAX_JOYSTICK_UPDATES))

    updates = copy.deepcopy(raw_updates)
    for update in updates:
      # String values are interpreted as enumerations.
      update['type'] = _GetEnumValue(sim_joystick_update_helper, update['type'])
      if 'enum_value' in update:
        update['enum_value'] = _GetEnumValue(sim_joystick_throttle_helper,
                                             update['enum_value'])

    new_joystick_sim['num_updates'] = len(updates)

    # Pad update list with no updates.
    no_update = {
        't_update': sys.float_info.max,
        'type': sim_types.kSimJoystickUpdateNone,
        'value': 0.0
    }
    updates_padded = updates + [
        copy.deepcopy(no_update)
        for _ in range(sim_types.MAX_JOYSTICK_UPDATES - len(updates))]

    for update in updates_padded:
      if update['type'] == sim_types.kSimJoystickUpdateThrottle:
        if update['enum_value'] != sim_types.kSimJoystickThrottleManual:
          assert update.get('value', -1.0) < 0.0, (
              'Value should not be specified for kSimJoystickUpdateThrottle '
              'unless it is a manual enum_value: %s.' % str(update))
          update['value'] = -1.0
      else:
        update['enum_value'] = 0
        if 'value' not in update:
          update['value'] = 0.0

    new_joystick_sim['updates'] = updates_padded

  return new_joystick_sim


def _PreprocessWingSimXg0Opt(xg0):
  if isinstance(xg0, list) and len(xg0) == 3:
    return xg0
  elif (isinstance(xg0, dict)
        and set(xg0.keys()) == {'azi', 'ele', 'r'}):
    return [-xg0['r'] * np.cos(xg0['ele']) * np.cos(xg0['azi']),
            -xg0['r'] * np.cos(xg0['ele']) * np.sin(xg0['azi']),
            -xg0['r'] * np.sin(xg0['ele'])]
  else:
    raise InvalidArgument('Xg_0 must be a list of Cartesian coordinates '
                          'or a dict with keys ("azi", "ele", "r").')


def PreprocessWindSpeedUpdates(wind_speed_updates):
  """Process overrides for sim.phys_sym.wind_speed_update."""

  if isinstance(wind_speed_updates, dict):
    return wind_speed_updates
  else:
    num_updates = len(wind_speed_updates)
    if num_updates > sim_types.MAX_WIND_SPEED_UPDATES:
      raise InvalidArrayLength(
          'Too many wind speed updates (%d). The maximum is '
          '%d.' % (num_updates, sim_types.MAX_WIND_SPEED_UPDATES))
    # Pad update list with no updates.
    no_update = {
        't_update': sys.float_info.max,
        'offset': 0.0,
    }
    return {
        'num_updates': num_updates,
        'offsets': wind_speed_updates + [
            no_update.copy()
            for _ in range(sim_types.MAX_WIND_SPEED_UPDATES - num_updates)]}


def _RecursivelyPreprocessOverrides(preprocessors, overrides):
  """Visit each override and apply preprocessors as appropriate."""
  overrides_out = {}
  for k in overrides:
    if k in preprocessors:
      if isinstance(preprocessors[k], dict):
        overrides_out[k] = _RecursivelyPreprocessOverrides(preprocessors[k],
                                                           overrides[k])
      else:
        overrides_out[k] = preprocessors[k](overrides[k])
    else:
      overrides_out[k] = overrides[k]
  return overrides_out


def _RecursiveUnicodeToStr(overrides):
  """Recursively convert values of type unicode to type str."""
  if isinstance(overrides, unicode):
    return str(overrides)
  elif isinstance(overrides, dict):
    return {k: _RecursiveUnicodeToStr(v) for (k, v) in overrides.items()}
  elif isinstance(overrides, list):
    return [_RecursiveUnicodeToStr(v) for v in overrides]
  else:
    return overrides


def _EnumPreprocessor(helper):
  """Returns a preprocessor function from an EnumHelper."""
  def Preprocess(arg):
    if isinstance(arg, int):
      return arg
    elif isinstance(arg, (str, unicode)):
      return helper.Value(arg)
    else:
      raise ValueError('Only integer or string values supported for enum '
                       'lookup.')
  return Preprocess


def PreprocessOverrides(overrides):
  """Preprocess a structure of overrides.

  Args:
    overrides: Structure of unprocessed overrides.

  Returns:
    A structure of overrides appropriate for mconfig.MakeParams.

  Raises:
    InvalidArraylength: If an array is given that exceeds fixed limits.
  """
  overrides = _RecursiveUnicodeToStr(overrides)

  preprocessors = {
      'system': {
          'flight_plan': functools.partial(_GetEnumValue, flight_plan_helper),
          'test_site': functools.partial(_GetEnumValue, test_site_helper),
      },
      'sim': {
          'sim_opt': _PreprocessSimOpt,
          'faults_sim': _PreprocessFaultsSim,
          'joystick_sim': _PreprocessJoystickSim,
          'phys_sim': {
              'wind_model': _EnumPreprocessor(wind_model_helper),
              'wind_speed_update': PreprocessWindSpeedUpdates,
          },
          'wing_sim': {'Xg_0': _PreprocessWingSimXg0Opt},
      }
  }
  return _RecursivelyPreprocessOverrides(preprocessors, overrides)


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  overrides = _RecursiveUnicodeToStr(json.loads(argv[1]))

  print PreprocessOverrides(overrides)


if __name__ == '__main__':
  main(sys.argv)
