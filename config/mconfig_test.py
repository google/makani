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

"""Tests for the Makani configuration library."""

import os
import unittest

import makani
from makani.config import mconfig
from makani.lib.python import test_util
from makani.sim import sim_types
from makani.system import labels


class MConfigTest(unittest.TestCase):

  def setUp(self):
    # Default parameters created when MakeParams is run on the
    # all_params.py file without overrides.
    self._default_params = mconfig.MakeParams('common.all_params')

    # Set flight plan to something that is not the default flight
    # plan.  This makes the harmless assumption that there are at
    # least two flight plans 0 and 1.
    if self._default_params['system']['flight_plan'] == 0:
      self._changed_flight_plan = 1
    else:
      self._changed_flight_plan = 0

  def testMakeParamsSimpleOverrides(self):
    params = mconfig.MakeParams(
        'common.all_params',
        overrides={'control': {'hover':
                               {'path':
                                {'reel_azimuth_offset': -0.2222,
                                 'accel_start_elevation': 0.2222}}},
                   'sim': {'ground_frame_sim': {
                       'pos_ecef': [1e3, 1e2, 1e1]}},
                   'system': {'flight_plan': self._changed_flight_plan}},
        override_method='simple')

    # Check that wind speed is overridden as expected.
    self.assertEqual(
        params['sim']['ground_frame_sim']['pos_ecef'], [1e3, 1e2, 1e1])

    # Check that multiple parameters in the same module are overridden.
    hover_path_params = params['control']['hover']['path']
    self.assertEqual(hover_path_params['reel_azimuth_offset'], -0.2222)
    self.assertEqual(hover_path_params['accel_start_elevation'], 0.2222)

    # Check that the simple override overrides the main flight plan
    # but not the derived flight plan used in the controller.
    self.assertEqual(params['system']['flight_plan'], self._changed_flight_plan)
    self.assertNotEqual(params['control']['flight_plan'],
                        self._changed_flight_plan)

  def testMakeParamsDerivedOverrides(self):
    params = mconfig.MakeParams(
        'common.all_params',
        overrides={'control': {'hover':
                               {'path':
                                {'reel_azimuth_offset': -0.2222,
                                 'accel_start_elevation': 0.2222}}},
                   'sim': {'ground_frame_sim': {
                       'pos_ecef': [1e3, 1e2, 1e1]}},
                   'system': {'flight_plan': self._changed_flight_plan}},
        override_method='derived')

    # Check that wind speed is overridden as expected.
    self.assertEqual(
        params['sim']['ground_frame_sim']['pos_ecef'], [1e3, 1e2, 1e1])

    # Check that multiple parameters in the same module are overridden.
    hover_path_params = params['control']['hover']['path']
    self.assertEqual(hover_path_params['reel_azimuth_offset'], -0.2222)
    self.assertEqual(hover_path_params['accel_start_elevation'], 0.2222)

    # Check that the derived override overrides the main flight plan
    # and the derived flight plan used in the controller.
    self.assertEqual(params['system']['flight_plan'], self._changed_flight_plan)
    self.assertEqual(params['control']['flight_plan'],
                     self._changed_flight_plan)

  def testTypeChecks(self):
    stdout = test_util.StdoutPatch()

    # Double array.
    with stdout, self.assertRaises(AssertionError):
      mconfig.MakeParams(
          'common.all_params',
          overrides={
              'control': {'control_output': {
                  'flaps_min': [0] + [0.0 for _ in range(labels.kNumFlaps - 1)]
              }}
          },
          override_method='derived')
    self.assertRegexpMatches(
        stdout.Read(),
        r'(?s).*control_output\.flaps_min\[0\] must be a float.*')

    # Vec3.
    with stdout, self.assertRaises(AssertionError):
      mconfig.MakeParams(
          'common.all_params',
          overrides={
              'sim': {'ground_frame_sim': {'pos_ecef': [1e3, 1, 1e1]}}},
          override_method='derived')
    self.assertRegexpMatches(
        stdout.Read(),
        r'(?s).*ground_frame_sim\.pos_ecef\[1\] must be a float.*')

    # Integer array.
    with stdout, self.assertRaises(AssertionError):
      mconfig.MakeParams(
          'common.all_params',
          overrides={
              'control': {
                  'fault_detection': {'gsg': {
                      'no_update_counts_limit': [0, 3.14]}}}},
          override_method='derived')
    self.assertRegexpMatches(
        stdout.Read(),
        (r'(?s).*fault_detection\.gsg\.no_update_counts_limit\[1\]'
         ' must be an int.*'))

  def testMakeParamsIntegerIndexedOverrides(self):
    overrides = {
        'sim': {
            'aero_sim': {
                'force_coeff_w_scale_factors': {
                    'flap_derivatives': {
                        0: {'1': 22.0},
                        1: [2.0, 3.0, 4.0]
                    }
                }
            }
        }
    }
    params = mconfig.MakeParams(
        'common.all_params',
        overrides=overrides,
        override_method='derived')
    factors = params['sim']['aero_sim']['force_coeff_w_scale_factors']
    for i in range(sim_types.kNumFlaps):
      if i == 1:
        self.assertEqual(2.0, factors['flap_derivatives'][i][0])
        self.assertEqual(3.0, factors['flap_derivatives'][i][1])
        self.assertEqual(4.0, factors['flap_derivatives'][i][2])
      else:
        for j in range(3):
          if i == 0 and j == 1:
            self.assertEqual(22.0, factors['flap_derivatives'][i][j])
          else:
            self.assertEqual(1.0, factors['flap_derivatives'][i][j])

  def testMakeParamsArrayOverrides(self):
    with self.assertRaises(mconfig.InvalidOverrideException):
      # There are either one or three IMUs so this call should always fail.
      params = mconfig.MakeParams(
          'common.all_params',
          overrides={'sim': {'wing_imus_sim': [{'delay': 0.22}, {}]}},
          override_method='derived')

    with self.assertRaises(mconfig.InvalidOverrideException):
      overrides = {
          'sim': {
              'aero_sim': {
                  'force_coeff_w_scale_factors': {
                      'flap_derivatives': {
                          's': {'1': 22.0},
                      }
                  }
              }
          }
      }
      params = mconfig.MakeParams(
          'common.all_params',
          overrides=overrides,
          override_method='derived')

    with self.assertRaises(mconfig.InvalidOverrideException):
      overrides = {
          'sim': {
              'aero_sim': {
                  'force_coeff_w_scale_factors': {
                      'flap_derivatives': {
                          0: {'3': 22.0},
                      }
                  }
              }
          }
      }
      params = mconfig.MakeParams(
          'common.all_params',
          overrides=overrides,
          override_method='derived')

    if sim_types.kNumWingImus == 1:
      params = mconfig.MakeParams(
          'common.all_params',
          overrides={'sim': {'wing_imus_sim': [{'delay': 0.22}]}},
          override_method='derived')

      # Check that delay is overridden as expected.
      self.assertEqual(params['sim']['wing_imus_sim'][0]['delay'], 0.22)
    else:
      params = mconfig.MakeParams(
          'common.all_params',
          overrides={
              'sim': {'wing_imus_sim': [{'delay': 0.2}, {}, {'delay': 2.0}]}},
          override_method='derived')

      # Check that delays are overridden as expected.
      self.assertEqual(params['sim']['wing_imus_sim'][0]['delay'], 0.2)
      self.assertEqual(params['sim']['wing_imus_sim'][2]['delay'], 2.0)
      # This delay should still have the default value.
      self.assertEqual(params['sim']['wing_imus_sim'][1]['delay'],
                       self._default_params['sim']['wing_imus_sim'][1]['delay'])

  def testFileDoesNotExistInAConfigDirectoryAndCommonDirectory(self):
    """Protects against redundant common configuration files.

    The configuration system does not allow configuration files with
    the same name to exist both in the common directory and in another
    configuration directory.
    """
    config_dir = os.path.join(makani.HOME, 'config')
    common_dir = os.path.join(config_dir, 'common')

    other_models = [fname for fname in os.listdir(config_dir)
                    if os.path.isdir(os.path.join(config_dir, fname))]
    other_models.remove('common')

    for model in other_models:
      model_dir = os.path.join(config_dir, model)
      for root, _, files in os.walk(model_dir):
        for f in files:
          if f != '__init__.py' and f.endswith('.py'):
            rel_path = os.path.relpath(os.path.join(root, f), model_dir)
            self.assertFalse(os.path.isfile(os.path.join(common_dir, rel_path)),
                             msg='common/%s shared file: %s' % (model,
                                                                rel_path))


if __name__ == '__main__':
  unittest.main()
