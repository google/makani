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

"""Power sensing parameters."""
from makani.config import mconfig


@mconfig.Config
def MakeParams():
  return {
      'v_bus_cal': {'scale': 5.0 * 5000.0 / (4096.0 * 18.2),
                    'bias': 0.0, 'bias_count': 0},
      'i_bus_cal': {'scale': 5.0 * 1000.0 / (4096.0 * 49.9),
                    'bias': -2.5 * 1000.0 / 49.9, 'bias_count': 0},
      'v_380_cal': {'scale': 5.0 * 2000.0 / (4096.0 * 18.2),
                    'bias': 0.0, 'bias_count': 0},
      'v_batt_48_cal': {'scale': 5.0 * 750.0 / (4096.0 * 49.9),
                        'bias': 0.0, 'bias_count': 0},
      'v_release_cal': {'scale': 25.0 / 1024.0,
                        'bias': 0.0, 'bias_count': 0},
      'i_release_cal': {'scale': -312.5 / 1024.0,
                        'bias': 156.25, 'bias_count': 0},
      'temperature_cal': {'scale': 0.25,
                          'bias': 0.0, 'bias_count': 0}
  }
