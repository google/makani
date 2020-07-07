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

"""Rotor sensing parameters."""

from makani.config import mconfig
from makani.control import system_types as m


@mconfig.Config
def MakeParams():
  common_rotor_sensor = {
      # Calibration for rotor speed and torque.  This applies both to
      # the speed sensed by and commanded to the motor controllers.
      #
      # TODO: The sign convention for rotor
      # velocity should be reversed both here and in the motor
      # controller.
      'omega_cal': {'scale': -1.0, 'bias': 0.0, 'bias_count': 0},
      'torque_cal': {'scale': -1.0, 'bias': 0.0, 'bias_count': 0},
  }

  return [common_rotor_sensor for _ in range(m.kNumMotors)]
