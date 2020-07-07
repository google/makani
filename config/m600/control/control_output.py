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

"""Controller output parameters."""
import os

import makani
from makani.avionics.firmware.params import codec
from makani.config import mconfig
import numpy as np


@mconfig.Config
def MakeParams():
  config_file = os.path.join(makani.HOME,
                             'avionics/servo/firmware/config_params.yaml')
  keys = ['aileron_a' + f for f in '124578'] + ['elevator', 'rudder']
  servos = {key: codec.DecodeYamlFile(config_file, key) for key in keys}

  one_degree = np.deg2rad(1.0)

  return {
      # Minimum and maximum mechanical flap limits [rad].
      # The elevator and rudder limits include a one degree margin off of
      # the servo limits of the control surfaces. See go/makanienvelope
      # for details.

      # TODO: Consider deprecating these limits or derive them in a
      # different way.
      'flaps_min': [servos['aileron_a1'].servo_min_limit,
                    servos['aileron_a2'].servo_min_limit,
                    servos['aileron_a4'].servo_min_limit,
                    servos['aileron_a5'].servo_min_limit,
                    servos['aileron_a7'].servo_min_limit,
                    servos['aileron_a8'].servo_min_limit,
                    servos['elevator'].servo_min_limit + one_degree,
                    np.deg2rad(23.0) *
                    np.sin(servos['rudder'].servo_min_limit)
                    + one_degree / 2.0],

      'flaps_max': [servos['aileron_a1'].servo_max_limit,
                    servos['aileron_a2'].servo_max_limit,
                    servos['aileron_a4'].servo_max_limit,
                    servos['aileron_a5'].servo_max_limit,
                    servos['aileron_a7'].servo_max_limit,
                    servos['aileron_a8'].servo_max_limit,
                    servos['elevator'].servo_max_limit - one_degree,
                    np.deg2rad(23.0) *
                    np.sin(servos['rudder'].servo_max_limit)
                    - one_degree / 2.0],

      # Maximum flap rate limit [rad/s].
      'rate_limit': 6.0
  }
