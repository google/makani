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

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import unittest

from makani.analysis.util.simdata_analysis import risk
from makani.analysis.util.simdata_analysis import simdata
from makani.analysis.util.simdata_analysis.data_import import BatchSimDataImporter


class TestRisk(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    super(TestRisk, self).__init__(*args, **kwargs)
    self.simdata = simdata.SimData(
        'analysis/util/simdata_analysis/test/data/1531/db.h5')

  def test_risk(self):
    this_risk = risk.evaluate_risk(probability=0.8, severity=5)
    self.assertEqual(this_risk['likelihood'], 5)
    self.assertEqual(this_risk['rank'], 1)

    this_risk = risk.evaluate_risk(probability=0, severity=5)
    self.assertEqual(this_risk['likelihood'], 1)
    self.assertEqual(this_risk['rank'], 16)

    this_risk = risk.evaluate_risk(probability=0, severity=1)
    self.assertEqual(this_risk['likelihood'], 1)
    self.assertEqual(this_risk['rank'], 25)

    this_risk = risk.evaluate_risk(probability=0.3, severity=2)
    self.assertEqual(this_risk['likelihood'], 3)
    self.assertEqual(this_risk['rank'], 14)
    self.assertEqual(this_risk['row'], 2)
    self.assertEqual(this_risk['col'], 1)

  def test_risk_table(self):
    table = 7
    risk_table_without, _ = risk.get_risk_table(self.simdata, table,
                                                skip_experimental_scores=True)

    risk_table_with, _ = risk.get_risk_table(self.simdata, table,
                                             skip_experimental_scores=False)

    sum2d_array = lambda x: sum(map(sum, x))
    self.assertLess(sum2d_array(risk_table_without),
                    sum2d_array(risk_table_with))

if __name__ == '__main__':
  bsdi = BatchSimDataImporter('analysis/util/simdata_analysis/test/data/'
                              'Crosswind_sweeps_monte_carlo_nightly_1531',
                              process_inputs=False)
  bsdi.create_database('analysis/util/simdata_analysis/test/data/1531/db.h5')

  unittest.main()
