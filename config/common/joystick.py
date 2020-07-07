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

"""Joystick parameters."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      'cal': {
          'throttle': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
          'roll': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
          'pitch': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0},
          'yaw': {'scale': 1.0, 'bias': 0.0, 'bias_count': 0}
      }
  }
