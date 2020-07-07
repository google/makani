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

"""Parameters for command of dyno motors from the simulator."""

from makani.config import mconfig
from makani.control import system_types


@mconfig.Config
def MakeParams():
  # Scale factors [#] for commanded torques for each of the motors.
  torque_scales = [1.0 for _  in range(system_types.kNumMotors)]

  return {
      # Maximum speed [rad/s] for the dynos.
      'max_speed': 140.0,

      # Maximum torque [N-m] to command to the dyno controllers.
      'max_torque': 600.0,

      # See comment above.
      'torque_scales': torque_scales
  }
