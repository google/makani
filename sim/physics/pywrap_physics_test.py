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

"""Tests for the physics library Swig wrapper."""

import unittest

from makani.control import system_types
from makani.sim.physics import physics


class PhysicsTest(unittest.TestCase):

  def testStateToVector(self):
    """Simple test of calling the Aero class."""
    aero = physics.Aero(physics.GetAeroSimParams())

    omega_hat = physics.Vec3()
    omega_hat.x = 0.0
    omega_hat.y = 0.0
    omega_hat.z = 0.0
    flaps = physics.VecWrapper(system_types.kNumFlaps)
    for i in range(system_types.kNumFlaps):
      flaps.SetValue(i, 0.0)
    force_moment = physics.ForceMoment()
    thrust_coeff = 0.0
    aero.CalcForceMomentCoeff(0.0, 0.0, omega_hat, flaps.GetVec(), 0.0,
                              force_moment, thrust_coeff)


if __name__ == '__main__':
  unittest.main()
