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

"""Hover controller parameters for experiments."""
import copy
import numpy as np


def GetExperiments():
  """Returns a dict of experiments with their experiment cases."""
  experiments = {
      # Experiment 1: Elevator steps.
      # Maps to kExperimentTypeHoverElevator.
      'elevator_rate_limit': np.deg2rad(3.0),
      'hover_elevator': [
          # Experiment case 0
          {'elevator': np.deg2rad(15.0)},

          # Experiment case 1
          {'elevator': np.deg2rad(5.0)},

          # Experiment case 2
          {'elevator': np.deg2rad(-5.0)},

          # Experiment case 3
          {'elevator': np.deg2rad(-15.0)},

          # Experiment case 4
          {'elevator': np.deg2rad(-25.0)},

          # Experiment case 5
          {'elevator': np.deg2rad(-35.0)},

          # Experiment case 6
          {'elevator': np.deg2rad(-45.0)},

          # Experiment case 7
          {'elevator': np.deg2rad(-55.0)},

          # Experiment case 8
          {'elevator': np.deg2rad(-65.)},

          # Experiment case 9
          {'elevator': np.deg2rad(-75.0)},

          # Experiment case 10
          {'elevator': np.deg2rad(-85.0)},

          # Experiment case 11
          {'elevator': np.deg2rad(-90.0)},
      ],
  }

  return copy.deepcopy(experiments)
