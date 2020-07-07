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

import unittest

from makani.analysis.system_design import site


class SiteTest(unittest.TestCase):

  def testDefaultCreateSite(self):
    default_site = site.Site()
    self.assertEqual(default_site.site_name, 'Default Site Name')
    self.assertEqual(default_site.velocity_distribution, {
        'velocity': [0.0, 10.0, 20.0],
        'probability': [0.2, 0.6, 0.2]})
    self.assertEqual(default_site.shear_distribution, {
        'shear_coefficient': [0.0, 0.1, 0.2],
        'probability': [0.0, 0.9, 0.1]})
    self.assertEqual(default_site.capital_costs, 1.0)

  def testModifiedCreateSite(self):
    velocity_distribution_input = {
        'velocity': [0.0, 8.0, 50.0],
        'probability': [0.0, 1.0, 0.0]
    }
    shear_distribution_input = {
        'shear_coefficient': [0.0, 0.14, 0.51],
        'probability': [0.0, 1.0, 0.0]
    }
    modified_site = site.Site('Unique Name', velocity_distribution_input,
                              shear_distribution_input, 5.0)
    self.assertEqual(modified_site.site_name, 'Unique Name')
    self.assertEqual(modified_site.velocity_distribution, {
        'velocity': [0.0, 8.0, 50.0],
        'probability': [0.0, 1.0, 0.0]})
    self.assertEqual(modified_site.shear_distribution, {
        'shear_coefficient': [0.0, 0.14, 0.51],
        'probability': [0.0, 1.0, 0.0]})
    self.assertEqual(modified_site.capital_costs, 5.0)


if __name__ == '__main__':
  unittest.main()
