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

""""Calculate state of charge for big LV battery box."""

import numpy as np


def calculate_soc(stack_voltage, current):
  """Calculate big-box state of charge given stack voltage & output current."""

  # Calculate voltage discharge curve at 0A, so that for any current,
  # we can subtract (current * stack_ohms) from measured voltage to
  # compare to true_voltage and thus estimate state-of-charge (SOC).
  stack_ohms = 0.115  # Typical resistance as measured in lab tests in ALM.
  drop_at_8a = stack_ohms * 8.0
  discharge_8a = np.array([73.134003, 72.022003, 71.796005, 71.608002,
                           71.442001, 71.266006, 71.084, 70.920006,
                           70.766006, 70.624001, 70.498001, 70.354004,
                           70.218002, 70.076004, 69.944, 69.812004,
                           69.669998, 69.529999, 69.402008, 69.258003,
                           69.122002, 68.988007, 68.850006, 68.704002,
                           68.582001, 68.444, 68.326004, 68.210007,
                           68.102005, 68.001999, 67.902, 67.798004,
                           67.706001, 67.626007, 67.534004, 67.462006,
                           67.374001, 67.298004, 67.234001, 67.160004,
                           67.096001, 67.040001, 66.976006, 66.917999,
                           66.874008, 66.830002, 66.784004, 66.738007,
                           66.708008, 66.674004, 66.648003, 66.638,
                           66.613998, 66.590004, 66.570007, 66.556,
                           66.536003, 66.503998, 66.484001, 66.454002,
                           66.434006, 66.404007, 66.362, 66.318008,
                           66.278, 66.222, 66.152008, 66.082001,
                           65.998001, 65.895996, 65.780006, 65.641998,
                           65.490005, 65.328003, 65.162003, 64.976006,
                           64.784004, 64.592003, 64.398003, 64.204002,
                           64.0, 63.808006, 63.624004, 63.456001,
                           63.276001, 63.086002, 62.908005, 62.736004,
                           62.568001, 62.414001, 62.258003, 62.100002,
                           61.918003, 61.694004, 61.348003, 60.82,
                           60.094002, 59.334003, 58.236004, 56.844002,
                           54.858002])
  true_voltage_curve = discharge_8a + drop_at_8a

  # Calculate state of charge by comparing to 0A discharge curve.
  current = max(current, 0.0)  # No current less than 0.
  true_voltage = stack_voltage + (current * stack_ohms)
  for i in range(len(true_voltage_curve)):
    if true_voltage > true_voltage_curve[i]:
      return 100 - i
  return 0
