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

"""Simulated winch parameters."""

from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # Proportional gain [N-m/rad] for the winch controller.
      'k_theta': 1.2e6,

      # Derivative gain [N-m/(rad/s)] for the winch controller.
      'k_omega': 4.9e5,

      # Maximum torque [N-m] that the drive can output.
      'max_torque': 3.0e5,
  }
