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

"""Servo parameters."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config
def MakeParams():
  # The ailerons and elevator are direct-drive, while the rudder has
  # a nonlinear linkage.
  #
  # The rudder moves about 40% of the servo shaft rotation for
  # rudder angles < 10 deg, and progressively less than that at
  # larger rudder angles.  The sine function is used to fit the
  # servo shaft rotation to the rudder rotation.  The scale factor
  # below is the multiplier on the sine function.  The error between
  # the measured rudder deflection and the sine approximation is a few
  # tenths of a degree until the rudder angle is < -20 deg.  The error is
  # about 1 deg when the rudder angle is in the range -20 to -24 deg.
  servos = [{'linear_servo_to_flap_ratio': 1.0,
             'nonlinear_servo_to_flap_ratio': 0.0}
            for _ in range(system_types.kNumServos)]
  servos[system_types.kServoR1]['linear_servo_to_flap_ratio'] = 0.0
  servos[system_types.kServoR2]['linear_servo_to_flap_ratio'] = 0.0
  servos[system_types.kServoR1]['nonlinear_servo_to_flap_ratio'] = 0.401
  servos[system_types.kServoR2]['nonlinear_servo_to_flap_ratio'] = 0.401

  return servos
