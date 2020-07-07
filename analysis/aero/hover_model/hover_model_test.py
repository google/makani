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

"""Tests for makani.analysis.aero.hover_model.hover_model."""

import copy
import unittest

from makani.analysis.aero.hover_model import hover_model
import numpy as np


class HoverModelTest(unittest.TestCase):

  PARAMS = hover_model.GetParams('M600a', '04Hover')

  def testGetPanelSamplingPoints(self):
    # Check that all combinations of x, y, and z appear in the
    # sampling points for a panel aligned with body coordinates.
    panel = {
        'pos_b': [0.0, 0.0, 0.0],
        'chord': 1.0,
        'span': 2.0,
        'dcm_b2s': np.eye(3)
    }
    panel_points_b = hover_model._GetPanelSamplingPoints(
        panel, thickness_ratio=0.5, num_points=(3, 2, 2))
    for x in (-0.75, -0.25, 0.25):
      for y in (-1.0, 1.0):
        for z in (-0.25, 0.25):
          found = False
          for v in panel_points_b:
            if (abs(v[0] - x) < 1e-9 and abs(v[1] - y) < 1e-9
                and abs(v[2] - z) < 1e-9):
              found = True
          self.assertTrue(found)

    # Check that all combinations of x, y, and z appear in the
    # sampling points for a vertical panel.
    panel = {
        'pos_b': [1.0, 2.0, 3.0],
        'chord': 1.0,
        'span': 2.0,
        'dcm_b2s': np.array([[1.0, 0.0, 0.0],
                             [0.0, 0.0, -1.0],
                             [0.0, 1.0, 0.0]])
    }
    panel_points_b = hover_model._GetPanelSamplingPoints(
        panel, thickness_ratio=0.5, num_points=(3, 2, 2))
    for x in (-0.75, -0.25, 0.25):
      for y in (-0.25, 0.25):
        for z in (-1.0, 1.0):
          found = False
          for v in panel_points_b:
            if (abs(v[0] - (x + panel['pos_b'][0])) < 1e-9
                and abs(v[1] - (y + panel['pos_b'][1])) < 1e-9
                and abs(v[2] - (z + panel['pos_b'][2])) < 1e-9):
              found = True
          self.assertTrue(found)

  def testCalcLocalApparentWind(self):
    # Check simple stationary case without propwash.
    apparent_wind_b = hover_model._CalcLocalApparentWind(
        [0.0, 10.0, 0.0], [0.0, 0.0, 0.0], 5.0, np.array([[np.pi / 2.0]]),
        np.array([[0.0]]), True, self.PARAMS)
    self.assertAlmostEqual(apparent_wind_b[0, 0][0], 0.0)
    self.assertAlmostEqual(apparent_wind_b[0, 0][1], 0.0)
    self.assertAlmostEqual(apparent_wind_b[0, 0][2], -5.0)

    # Check simple rotating case without propwash.
    apparent_wind_b = hover_model._CalcLocalApparentWind(
        [0.0, 10.0, 0.0], [2.0, 0.0, 0.0], 0.0, np.array([[0.0]]),
        np.array([[0.0]]), True, self.PARAMS)
    self.assertAlmostEqual(apparent_wind_b[0, 0][0], 0.0)
    self.assertAlmostEqual(apparent_wind_b[0, 0][1], 0.0)
    self.assertAlmostEqual(apparent_wind_b[0, 0][2], -20.0)

    # Check simple stationary case with only propwash.  Compare the
    # velocity to what is expected from basic momentum theory.  Note
    # that, even though we evaluate this function at a small axial
    # position, we use the far downstream velocity because this wake
    # model deals with this difference by contracting the radius near
    # the source.
    no_rotation_params = copy.copy(self.PARAMS)
    no_rotation_params['rotor_pitch'] = 0.0
    point_b = copy.copy(self.PARAMS['rotors'][0]['pos'])
    point_b[0] -= 0.01
    apparent_wind_b = hover_model._CalcLocalApparentWind(
        point_b, [0.0, 0.0, 0.0], 0.0, np.array([[0.0]]), np.array([[0.0]]),
        True, no_rotation_params)
    wake_vel = 2.0 * np.sqrt(
        self.PARAMS['rotors'][0]['thrust']
        / (2.0 * self.PARAMS['phys']['rho']
           * np.pi * self.PARAMS['rotors'][0]['radius']**2.0))
    self.assertAlmostEqual(apparent_wind_b[0, 0][0], -wake_vel, delta=1e-2)
    self.assertAlmostEqual(apparent_wind_b[0, 0][1], 0.0)
    self.assertAlmostEqual(apparent_wind_b[0, 0][2], 0.0)


if __name__ == '__main__':
  unittest.main()
