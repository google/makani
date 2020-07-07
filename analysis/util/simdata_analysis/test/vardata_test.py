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
import os
import random
import unittest

from makani.analysis.util.simdata_analysis import simdata
from makani.analysis.util.simdata_analysis.data_import import BatchSimDataImporter
import numpy as np


class TestVarData(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    super(TestVarData, self).__init__(*args, **kwargs)
    self.simdata = simdata.SimData(
        'analysis/util/simdata_analysis/test/data//1531/db.h5')

  def test_init(self):
    score = random.sample(range(self.simdata.num_scores - 1), 1)[0]
    table = random.sample(range(self.simdata.num_tables), 1)[0]

    vardata = self.simdata.get_score_data(score, table, verbose=False)
    self.assertEqual(vardata.variable_info['name'],
                     self.simdata.get_score_info(score)['name'])
    self.assertEqual(vardata.table_info['title'],
                     self.simdata.get_table_info(table)['title'])

    job_ids = vardata.var_df['job_id'].tolist()
    job_id = random.sample(job_ids, 1)[0]
    fn = os.path.join(vardata.var_df.loc[
        vardata.var_df['job_id'] == job_id]['folder'].values[0],
                      BatchSimDataImporter.outputdata_filename)
    fp = open(fn, 'r')
    rawdata = json.load(fp)
    fp.close()
    raw_table_data = rawdata['table_data'][table]['job_data']
    raw_job_ids = [jobdata['job_id'] for jobdata in raw_table_data]
    raw_index = raw_job_ids.index(job_id)
    raw_score = raw_table_data[raw_index]['scores'][score]
    if raw_score is None:
      self.assertTrue(np.isnan(vardata.var_df.loc[
          vardata.var_df['job_id'] == job_id]['score'].values[0]))
    else:
      self.assertEqual(vardata.var_df.loc[
          vardata.var_df['job_id'] == job_id]['score'].values[0], raw_score)

  def test_resample(self):
    score = random.sample(range(self.simdata.num_scores - 1), 1)[0]
    table = random.sample(range(self.simdata.num_tables), 1)[0]
    vardata = self.simdata.get_score_data(score, table, verbose=False)

    self.assertEqual(vardata.bootstrapped_data.shape, (vardata.n_resamples,
                                                       vardata.n_samples))
    vardata.resample()
    self.assertEqual(vardata.bootstrapped_data.shape, (vardata.n_resamples,
                                                       vardata.n_samples))

    n_samples = random.randint(1, 1000)
    n_resamples = random.randint(1, 1000)
    vardata.resample(n_samples)
    self.assertEqual(vardata.bootstrapped_data.shape, (vardata.n_resamples,
                                                       n_samples))
    vardata.resample(n_samples, n_resamples)
    self.assertEqual(vardata.bootstrapped_data.shape, (n_resamples, n_samples))

  def test_stats(self):
    score = random.sample(range(self.simdata.num_scores - 1), 1)[0]
    table = random.sample(range(self.simdata.num_tables), 1)[0]
    vardata = self.simdata.get_score_data(score, table, verbose=False)
    self.assertAlmostEqual((vardata.prob_above(1.).mean +
                            vardata.prob_below(1.).mean), 1., delta=1e-2)


if __name__ == '__main__':
  bsdi = BatchSimDataImporter('analysis/util/simdata_analysis/test/data/'
                              'Crosswind_sweeps_monte_carlo_nightly_1531',
                              process_inputs=False)
  bsdi.create_database('analysis/util/simdata_analysis/test/data//1531/db.h5')

  unittest.main()
