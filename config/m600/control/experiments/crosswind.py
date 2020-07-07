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

"""Configs for crosswind experiments."""
import copy
import numpy as np


def GetExperiments():
  """Returns a dict of experiments with their test cases."""
  experiments = {
      'crosswind_spoiler': [
          # Test case 0
          {
              'target': np.deg2rad(-2.5),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
          # Test case 1
          {
              'target': np.deg2rad(-5.0),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
          # Test case 2
          {
              'target': np.deg2rad(-7.5),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
          # Test case 3
          {
              'target': np.deg2rad(-10.0),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
          # Test case 4
          {
              'target': np.deg2rad(-12.5),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
          # Test case 5
          {
              'target': np.deg2rad(-15.0),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
          # Test case 6
          {
              'target': np.deg2rad(-17.5),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
          # Test case 7
          {
              'target': np.deg2rad(-20.0),
              'start_loop_angle': 0.0,
              'end_loop_angle': 0.0,
              'always_on': True,
          },
      ]
  }

  return copy.deepcopy(experiments)
