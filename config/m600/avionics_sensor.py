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

"""Avionics sensing parameters."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      # TODO: These values are out-of-date.
      'v_avionics_cal': {'scale': 25.0 / 4096.0, 'bias': 0.0, 'bias_count': 0},
      'v_servo_cal': {'scale': 25.0 / 4096.0, 'bias': 0.0, 'bias_count': 0}
  }
