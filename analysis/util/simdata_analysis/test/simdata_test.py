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

import json
import random
import unittest

from makani.analysis.util.simdata_analysis import vardata
from makani.analysis.util.simdata_analysis.data_import import BatchSimDataImporter
from makani.analysis.util.simdata_analysis.simdata import SimData
import numpy as np
import pandas as pd


class TestSimData(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    super(TestSimData, self).__init__(*args, **kwargs)

    self.simdata = SimData(
        'analysis/util/simdata_analysis/test/data/1531/db.h5')

    fn = ('analysis/util/simdata_analysis/test/data/'
          'Crosswind_sweeps_monte_carlo_nightly_1531/overview_data.json')
    fp = open(fn, 'r')
    self.raw_data = json.load(fp)
    fp.close()

  def test_metadata(self):
    metadata = self.simdata.get_metadata()

    self.assertEqual(metadata['commit'], self.raw_data['commit'])
    self.assertEqual(metadata['title'], self.raw_data['title'])

  def test_scorelist(self):
    scorelist = self.simdata.get_score_list()
    self.assertEqual(scorelist, [str(score['name']) for score in
                                 self.raw_data['metrics']] +
                     ['Crash indicator'])

  def test_find_score(self):
    score_name = '-Crash- Max Servo Moment [N.m]'
    score_candidate = self.simdata.find_score(score_name, verbose=False)
    self.assertEqual(score_name, score_candidate)
    score_name = 'Servo Moment'
    score_candidate = self.simdata.find_score(score_name, verbose=False)
    self.assertEqual('-Crash- Max Servo Moment [N.m]', score_candidate)
    score_candidate = self.simdata.find_score('xyz', verbose=False)
    self.assertIsNone(score_candidate)

  def test_score_info(self):
    score_index = random.randint(0, self.simdata.num_scores - 2)
    score_name = self.simdata.get_score_list()[score_index]
    scoredata = self.simdata.get_score_info(score_name, verbose=False)
    sev = self.raw_data['metrics'][scoredata['index']]['severity']
    self.assertEqual(scoredata['severity'], int(sev))
    score_raw_data = self.raw_data['metrics'][scoredata['index']]
    experimental_flag = bool('experimental' in score_raw_data['system_labels'])
    self.assertEqual(experimental_flag,
                     self.simdata.get_score_info(scoredata['index'],
                                                 verbose=False)['experimental'])
    score_index = random.randint(0, self.simdata.num_scores - 2)
    self.assertEqual(self.simdata.get_score_info(score_index,
                                                 verbose=False)['severity'],
                     int(self.raw_data['metrics'][score_index]['severity']))

  def test_find_table(self):
    table_name = 'wind speed = 11 shear exponent = 0.1'
    table_candidate = self.simdata.find_table(table_name, verbose=False)
    self.assertEqual(table_candidate, 'Monte Carlo Wind Speed [m/s] '
                                      '@ 21 [m] AGL = 11, shear exponent = 0.1')
    self.assertIsNone(self.simdata.find_table('xyz', verbose=False))

  def test_table_info(self):
    table_index = random.randint(0, self.simdata.num_tables - 1)
    table_title = self.simdata.get_table_list()[table_index]
    tabledata = self.simdata.get_table_info(table_index, verbose=False)
    self.assertEqual(table_title, tabledata['title'])

  def test_get_score_data(self):
    score = random.randint(0, self.simdata.num_scores - 2)
    table = random.randint(0, self.simdata.num_tables - 1)
    scoredata = self.simdata.get_score_data(score, table, verbose=False)

    scores_df = pd.read_hdf(('analysis/util/simdata_analysis/test/data/'
                             '1531/db.h5'),
                            'tables/table_{0}'.format(table))
    scores = scores_df['score_{0}'.format(score)].tolist()
    self.assertEqual(scoredata.data[~np.isnan(scoredata.data)].tolist(),
                     np.array(scores)[~np.isnan(scores)].tolist())

  def test_get_all_score_data(self):
    table = random.randint(0, self.simdata.num_tables - 1)
    df_dict = self.simdata.get_all_score_data(table)
    self.assertEqual(len(df_dict), self.simdata.num_scores)
    for df in df_dict:
      self.assertIsInstance(df_dict[df], vardata.ScoreData)


if __name__ == '__main__':
  bsdi = BatchSimDataImporter('analysis/util/simdata_analysis/test/data/'
                              'Crosswind_sweeps_monte_carlo_nightly_1531',
                              process_inputs=False)
  bsdi.create_database('analysis/util/simdata_analysis/test/data/1531/db.h5')

  unittest.main()
