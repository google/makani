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

import math
import unittest

from makani.analysis.control import flap_limits
from makani.config import mconfig
from makani.control import system_types

# This test asserts these desired constraints:
# - Avionics servo limits strictly wider than control limits with additional
#   small margin for elevator and rudder.
# - Control limits strictly wider than control mode limits.
# - Sim limits strictly wider than control limits.
# - Sim and avionics paired servo limits approximately equal.

_FLOAT_DELTA = math.radians(0.01)


class FlapLimitsTest(unittest.TestCase):

  def _AssertSingleLimitEqual(self, a, b):
    """Test if single 'a' limit is approximately equal to 'b' limit."""
    self.assertAlmostEqual(a[0], b[0], delta=_FLOAT_DELTA)
    self.assertAlmostEqual(a[1], b[1], delta=_FLOAT_DELTA)

  def _AssertLimitsSane(self, limits):
    for limit in limits.values():
      self.assertLess(limit[0], limit[1])

  def _AssertLimitsEqual(self, a, b):
    """Test if 'a' limits are approximately equal to 'b' limits."""
    self.assertItemsEqual(a.keys(), b.keys())
    for name in a.keys():
      self._AssertSingleLimitEqual(a[name], b[name])

  def _AssertLimitsContain(self, a, b):
    """Test if 'a' limits fully encompass 'b' limits."""
    self.assertItemsEqual(a.keys(), b.keys())
    for name in a.keys():
      # Assert that 'a' limits encompass 'b' limits.
      self.assertTrue(a[name][0] - _FLOAT_DELTA <= b[name][0],
                      'Fails for %s lower limits' % name)
      self.assertTrue(a[name][1] + _FLOAT_DELTA >= b[name][1],
                      'Fails for %s upper limits' % name)

  def testFlapLimitsFunctions(self):
    self.assertIsNotNone(flap_limits.GetFlapLimits())
    self.assertIsNotNone(flap_limits.GetAvionicsServoLimits())

  def testAllLimitsSane(self):
    for limit in flap_limits.GetFlapLimits():
      self._AssertLimitsSane(limit[1])
    for limit in flap_limits.GetServoLimits():
      self._AssertLimitsSane(limit[1])

  def testControlCrosswindLimitsInsideControlLimits(self):
    self._AssertLimitsContain(flap_limits.GetControlLimits(),
                              flap_limits.GetControlCrosswindLimits())

  def testControlCrosswindFlareLimitsInsideControlLimits(self):
    self._AssertLimitsContain(flap_limits.GetControlLimits(),
                              flap_limits.GetControlCrosswindFlareLimits())

  def testControlTransInLimitsInsideControlLimits(self):
    self._AssertLimitsContain(flap_limits.GetControlLimits(),
                              flap_limits.GetControlTransInLimits())

  def testControlHoverLimitsInsideControlLimits(self):
    self._AssertLimitsContain(flap_limits.GetControlLimits(),
                              flap_limits.GetControlHoverLimits())

  def testControlManualLimitsInsideControlLimits(self):
    self._AssertLimitsContain(flap_limits.GetControlLimits(),
                              flap_limits.GetControlManualLimits())

  def testControlLimitsInsideServoLimits(self):
    self._AssertLimitsContain(flap_limits.GetAvionicsServoLimits(),
                              flap_limits.FlapsToServos(
                                  flap_limits.GetControlLimits()))

  def testControlLimitsInsideSimLimits(self):
    self._AssertLimitsContain(flap_limits.GetSimLimits(),
                              flap_limits.FlapsToServos(
                                  flap_limits.GetControlLimits()))

  def testDualServoMargin(self):
    control_limits = flap_limits.FlapsToServos(flap_limits.GetControlLimits())
    servo_limits = flap_limits.GetAvionicsServoLimits()
    half_degree = math.radians(0.5)
    # Shrink servo limits by half a degree on each side.
    for servo in ['E1', 'E2', 'R1', 'R2']:
      servo_limits[servo] = (servo_limits[servo][0] + half_degree,
                             servo_limits[servo][1] - half_degree)
    self._AssertLimitsContain(servo_limits, control_limits)

  def testDualServoMatchAvionics(self):
    servo_limits = flap_limits.GetAvionicsServoLimits()
    self._AssertSingleLimitEqual(servo_limits['E1'], servo_limits['E2'])
    self._AssertSingleLimitEqual(servo_limits['R1'], servo_limits['R2'])

  def testDualServoMatchSim(self):
    servo_limits = flap_limits.GetSimLimits()
    self._AssertSingleLimitEqual(servo_limits['E1'], servo_limits['E2'])
    self._AssertSingleLimitEqual(servo_limits['R1'], servo_limits['R2'])

  def testFlapsServosConversion(self):
    all_servo_limits = flap_limits.GetServoLimits()
    all_flap_limits = flap_limits.GetFlapLimits()
    for servo, flap in zip(all_servo_limits, all_flap_limits):
      # Verify names match.
      self.assertEqual(servo[0], flap[0])
      self._AssertLimitsEqual(servo[1], flap_limits.FlapsToServos(flap[1]))
      self._AssertLimitsEqual(flap_limits.ServosToFlaps(servo[1]), flap[1])

  def testSystemServoFlapRatios(self):
    """Sanity check servo/flap ratios in system config (used for rudder)."""
    servo_config = mconfig.MakeParams('common.all_params')['system']['servos']

    for servo in range(system_types.kNumServos):
      if servo == system_types.kServoR1 or servo == system_types.kServoR2:
        self.assertEqual(servo_config[servo]['linear_servo_to_flap_ratio'], 0.0)
      else:
        self.assertAlmostEqual(
            servo_config[servo]['linear_servo_to_flap_ratio'], 1.0,
            delta=_FLOAT_DELTA)
        self.assertEqual(
            servo_config[servo]['nonlinear_servo_to_flap_ratio'], 0.0)

    self.assertAlmostEqual(
        servo_config[system_types.kServoR1]['nonlinear_servo_to_flap_ratio'],
        servo_config[system_types.kServoR2]['nonlinear_servo_to_flap_ratio'],
        delta=_FLOAT_DELTA)


if __name__ == '__main__':
  unittest.main()
