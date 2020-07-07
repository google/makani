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

"""Tests for the overrides_util library."""

import unittest

from makani.config import overrides_util
from makani.lib.python import c_helpers
from makani.sim import sim_types
import numpy


class OverridesUtilTest(unittest.TestCase):

  def testSimOpt(self):
    overrides_struct = {
        'sim': {
            'sim_opt': ['kSimOptImperfectSensors', 1]
        }
    }
    overrides = overrides_util.PreprocessOverrides(overrides_struct)
    self.assertIn('sim', overrides)
    self.assertIn('sim_opt', overrides['sim'])
    self.assertEqual(1 | sim_types.kSimOptImperfectSensors,
                     overrides['sim']['sim_opt'])

  def testFaultsSim(self):
    faults_in = [{
        't_start': 100.0,
        't_end': 200.0,
        'component': 'Pitot/actual_P_dyn',
        'type': 'kSimFaultMeasurementRescale',
        'parameters': [1.0, 2.0, 3.0],
    }, {
        't_start': 100.0,
        'component': 'Pitot/actual_P_dyn',
        'type': 'kSimFaultMeasurementRescale',
    }]

    overrides_struct = {'sim': {'faults_sim': faults_in}}
    overrides = overrides_util.PreprocessOverrides(overrides_struct)

    # Check the overall structure of the created fields.
    self.assertTrue('sim' in overrides and 'faults_sim' in overrides['sim']
                    and 'num_fault_events' in overrides['sim']['faults_sim']
                    and 'fault_events' in overrides['sim']['faults_sim'])
    self.assertEqual(2, overrides['sim']['faults_sim']['num_fault_events'])

    faults_out = overrides['sim']['faults_sim']['fault_events']
    self.assertEqual(sim_types.MAX_FAULT_EVENTS, len(faults_out))

    # Check the fault events that we specified.
    for (fault_in, fault_out) in zip(faults_in, faults_out[:len(faults_in)]):
      self.assertEqual(fault_in['component'], fault_out['component'])
      self.assertEqual(sim_types.kSimFaultMeasurementRescale,
                       fault_out['type'])

      if 'parameters' in fault_in:
        params_in = fault_in['parameters']
        params_out = fault_out['parameters']
        self.assertEqual(sim_types.MAX_FAULT_EVENT_PARAMETERS,
                         len(params_out))
        self.assertEqual(params_in, params_out[:len(params_in)])
      else:
        self.assertEqual([0.0] * sim_types.MAX_FAULT_EVENT_PARAMETERS,
                         fault_out['parameters'])

    # Check basic reasonableness of the unused faults.
    for fault in faults_out[len(faults_in):]:
      self.assertEqual(sim_types.kSimFaultNoFault, fault['type'])
      self.assertEqual(sim_types.MAX_FAULT_EVENT_PARAMETERS,
                       len(fault['parameters']))

  def testWingSimXg0(self):
    # Test Cartesian coordinates.
    with self.assertRaises(overrides_util.InvalidArgument):
      overrides_util.PreprocessOverrides({
          'sim': {'wing_sim': {'Xg_0': [22.0, 22.0]}}
      })

    with self.assertRaises(overrides_util.InvalidArgument):
      overrides_util.PreprocessOverrides({
          'sim': {'wing_sim': {'Xg_0': [22.0, 22.0, 22.0, 22.0]}}
      })

    overrides = overrides_util.PreprocessOverrides({
        'sim': {'wing_sim': {'Xg_0': [22.0, 22.0, 22.0]}}
    })
    self.assertTrue('sim' in overrides
                    and 'wing_sim' in overrides['sim']
                    and 'Xg_0' in overrides['sim']['wing_sim'])
    self.assertEqual([22.0, 22.0, 22.0],
                     overrides['sim']['wing_sim']['Xg_0'])

    # Test spherical coordinates.
    with self.assertRaises(overrides_util.InvalidArgument):
      overrides_util.PreprocessOverrides({
          'sim': {'wing_sim': {'Xg_0': {
              'ele': 22.0, 'azi': 0.0, 'r': 0.0, 'bad': 0.0
          }}}})

    with self.assertRaises(overrides_util.InvalidArgument):
      overrides_util.PreprocessOverrides({
          'sim': {'wing_sim': {'Xg_0': {
              'ele': 22.0, 'azi': 0.0
          }}}})

    overrides = overrides_util.PreprocessOverrides({
        'sim': {'wing_sim': {'Xg_0': {
            'ele': numpy.pi/3, 'azi': 0.0, 'r': 1.0
        }}}})
    self.assertTrue('sim' in overrides
                    and 'wing_sim' in overrides['sim']
                    and 'Xg_0' in overrides['sim']['wing_sim'])
    expected = [-0.5, 0.0, -numpy.sqrt(3.0)/2.0]
    for i in range(3):
      self.assertAlmostEqual(expected[i],
                             overrides['sim']['wing_sim']['Xg_0'][i])

  def testBadEnumValues(self):
    # Test overriding the flight_plan.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({
          'system': {'flight_plan': 'kFlightPlanEit'}
      })

    # Test a name that doesn't start with FLIGHT_PLAN.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({
          'system': {'flight_plan': 'kSimOptPerch'}
      })

    # Test a name that doesn't exist.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({'sim': {'sim_opt': ['kSimOptEit']}})

    # Test a name that doesn't start with SIM_OPT.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({
          'sim': {'sim_opt': ['kFlightPlanTurnKey']}
      })

    # Test a name that doesn't exist.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({
          'sim': {'faults_sim': [{'type': 'kSimFaultUpdateEit'}]}
      })

    # Test a name that doesn't start with SIM_FAULT.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({
          'sim': {'faults_sim': [{'type': 'kSimOptPerch'}]}
      })

    # Test a name that doesn't exist.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({
          'sim': {'joystick_sim': {
              'updates': [{'type': 'kSimJoystickUpdateEit'}]
          }}
      })

    # Test a name that doesn't start with SIM_JOYSTICK_UPDATE.
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides({
          'sim': {'joystick_sim': {
              'updates': [{'type': 'kSimJoystickTypeProgrammed'}]
          }}
      })

  def testFaultsSimError(self):
    overrides_in = {
        'sim': {
            'faults_sim': [{
                't_start': 100.0,
                'component': 'Pitot/actual_P_dyn',
                'type': 'kSimFaultMeasurementRescale',
            }] * (1 + sim_types.MAX_FAULT_EVENTS)
        }
    }
    with self.assertRaises(overrides_util.InvalidArrayLength):
      overrides_util.PreprocessOverrides(overrides_in)

    overrides_in = {
        'sim': {
            'faults_sim': [{
                't_start': 100.0,
                'component': 'Pitot/actual_P_dyn',
                'type': 'kSimFaultMeasurementRescale',
                'parameters': [0.0] * (1 + sim_types.MAX_FAULT_EVENT_PARAMETERS)
            }]
        }
    }
    with self.assertRaises(overrides_util.InvalidArrayLength):
      overrides_util.PreprocessOverrides(overrides_in)

  def testJoystickSimJoystickType(self):
    overrides_struct = {'sim': {'joystick_sim': {'joystick_type': 'Hardware'}}}
    overrides = overrides_util.PreprocessOverrides(overrides_struct)

    self.assertTrue('sim' in overrides and 'joystick_sim' in overrides['sim']
                    and 'joystick_type' in overrides['sim']['joystick_sim'])
    self.assertEqual(sim_types.kSimJoystickTypeHardware,
                     overrides['sim']['joystick_sim']['joystick_type'])

  def testJoystickSimUpdates(self):
    updates_in = [{
        't_update': 100.0,
        'type': 'kSimJoystickUpdateSwitchMiddle'
    }, {
        't_update': 100.0,
        'type': 'kSimJoystickUpdateThrottle',
        'enum_value': 'kSimJoystickThrottleManual',
        'value': 22.0
    }, {
        't_update': 200.0,
        'type': 'kSimJoystickUpdateThrottle',
        'enum_value': 'kSimJoystickThrottleCrosswindNormal'
    }, {
        't_update': 200.0,
        'type': 'kSimJoystickUpdateThrottle',
        'enum_value': 'kSimJoystickThrottleCrosswindNormal',
        'value': -1.0
    }, {
        't_update': 200.0,
        'type': 'kSimJoystickUpdateSwitchMiddle',
        'enum_value': 0
    }]

    overrides_struct = {'sim': {'joystick_sim': {'updates': updates_in}}}
    overrides = overrides_util.PreprocessOverrides(overrides_struct)

    self.assertTrue('sim' in overrides and 'joystick_sim' in overrides['sim']
                    and 'num_updates' in overrides['sim']['joystick_sim']
                    and 'updates' in overrides['sim']['joystick_sim'])
    self.assertEqual(5, overrides['sim']['joystick_sim']['num_updates'])
    self.assertEqual(sim_types.MAX_JOYSTICK_UPDATES,
                     len(overrides['sim']['joystick_sim']['updates']))

    self.assertEqual(0.0,
                     overrides['sim']['joystick_sim']['updates'][0]['value'])

    for update in overrides['sim']['joystick_sim']['updates'][len(updates_in):]:
      self.assertEqual(sim_types.kSimJoystickUpdateNone, update['type'])

  def testJoystickSimError(self):
    overrides_in = {
        'sim': {
            'joystick_sim': {
                'updates': [{
                    't_update': 100.0,
                    'type': 'kSimJoystickUpdateSwitchMiddle'
                }] * (1 + sim_types.MAX_JOYSTICK_UPDATES)
            }
        }
    }
    with self.assertRaises(overrides_util.InvalidArrayLength):
      overrides_util.PreprocessOverrides(overrides_in)

  def testWindModel(self):
    overrides_in = {
        'sim': {
            'phys_sim': {
                'wind_model': sim_types.kWindModelDrydenTurbulence,
            },
        },
    }
    overrides_out = overrides_util.PreprocessOverrides(overrides_in)
    self.assertEqual(sim_types.kWindModelDrydenTurbulence,
                     overrides_out['sim']['phys_sim']['wind_model'])

    overrides_in = {
        'sim': {
            'phys_sim': {
                'wind_model': 'DrydenTurbulence',
            },
        },
    }
    overrides_out = overrides_util.PreprocessOverrides(overrides_in)
    self.assertEqual(sim_types.kWindModelDrydenTurbulence,
                     overrides_out['sim']['phys_sim']['wind_model'])

    overrides_in = {
        'sim': {
            'phys_sim': {
                'wind_model': 'NotAWindModel',
            },
        },
    }
    with self.assertRaises(c_helpers.EnumError):
      overrides_util.PreprocessOverrides(overrides_in)

  def testWindSpeedUpdates(self):
    updates_in = [{
        't_update': 20.0,
        'offset': -5.0
    }, {
        't_update': 40.0,
        'offset': 10.0
    }]

    overrides_struct = {'sim': {'phys_sim': {'wind_speed_update': updates_in}}}
    overrides = overrides_util.PreprocessOverrides(overrides_struct)

    self.assertTrue(
        'num_updates' in overrides['sim']['phys_sim']['wind_speed_update'] and
        'offsets' in overrides['sim']['phys_sim']['wind_speed_update'])
    self.assertEqual(
        2, overrides['sim']['phys_sim']['wind_speed_update']['num_updates'])
    self.assertEqual(
        sim_types.MAX_WIND_SPEED_UPDATES,
        len(overrides['sim']['phys_sim']['wind_speed_update']['offsets']))


if __name__ == '__main__':
  unittest.main()
