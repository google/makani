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

from makani.analysis.util.simdata_analysis.data_import import BatchSimDataImporter
import numpy as np
import pandas as pd


class TestBatchSimDataImporter(unittest.TestCase):

  def test_folder(self):
    infiles = ('analysis/util/simdata_analysis/test/data/'
               'Crosswind_sweeps_monte_carlo_nightly_1531')
    bsdi = BatchSimDataImporter(infiles, process_inputs=False)
    self.assertIsNotNone(bsdi.input_files)

  def test_output_data(self):
    infiles = ('analysis/util/simdata_analysis/test/data/'
               'Crosswind_sweeps_monte_carlo_nightly_1531')
    bsdi = BatchSimDataImporter(infiles, process_inputs=False)
    self.assertEqual(len(bsdi.scorelist), 116)
    self.assertEqual(len(bsdi.scorelist), len(bsdi.severitylist))

  def test_database_creation(self):
    infiles = ('analysis/util/simdata_analysis/test/data/'
               'Crosswind_sweeps_monte_carlo_nightly_1531')
    bsdi = BatchSimDataImporter(infiles, process_inputs=False)
    bsdi.create_database('analysis/util/simdata_analysis/test/data/1531/db.h5')
    self.assertTrue(os.path.isfile('analysis/util/simdata_analysis/'
                                   'test/data/1531/db.h5'))

  def test_dataset_data(self):
    infiles = ('analysis/util/simdata_analysis/test/data/'
               'Crosswind_sweeps_monte_carlo_nightly_1531')
    bsdi = BatchSimDataImporter(infiles, process_inputs=False)
    bsdi.create_database('analysis/util/simdata_analysis/test/data/1531/db.h5')
    infile = open(bsdi.input_files[0], 'r')
    self.assertEqual(pd.read_hdf(('analysis/util/simdata_analysis/'
                                  'test/data/1531/db.h5'),
                                 'metadata')['commit'][0],
                     json.load(infile)['commit'])
    infile.close()

    num_tests = 10
    for (score, table) in zip(random.sample(range(bsdi.num_scores - 1),
                                            num_tests),
                              random.sample(range(bsdi.num_tables), num_tests)):

      run_id = random.randint(1, bsdi.num_table_samples[table]) - 1
      infile = open(bsdi.input_files[0], 'r')
      scorename = pd.read_hdf(('analysis/util/simdata_analysis/'
                               'test/data/1531/db.h5'),
                              'scoredata')['name'][score]
      score_list = json.load(infile)['metrics']
      self.assertEqual(scorename, score_list[score]['name'])
      infile.close()

      fp = open(bsdi.input_files[0], 'r')
      raw_data = json.load(fp)
      fp.close()
      scores = [job_data['scores'][score] for job_data in
                raw_data['table_data'][table]['job_data'] if
                job_data['sim_success']]
      if scores[run_id] is not None:
        in_data = pd.read_hdf(('analysis/util/simdata_analysis/'
                               'test/data/1531/db.h5'),
                              'tables/table_{0}'.format(table))
        self.assertEqual(in_data['score_{0}'.format(score)][run_id],
                         scores[run_id])

  def test_experimental_scores(self):
    infiles = ('analysis/util/simdata_analysis/test/data/'
               'Crosswind_sweeps_monte_carlo_nightly_1531')
    bsdi = BatchSimDataImporter(infiles, process_inputs=False)
    bsdi.create_database('analysis/util/simdata_analysis/test/data/1531/db.h5')
    infile = open(bsdi.input_files[0], 'r')
    raw_data = json.load(infile)
    infile.close()
    experimental_idxs = [idx for idx in range(len(raw_data['metrics'])) if
                         'experimental' in
                         raw_data['metrics'][idx]['system_labels']]
    score_experimental = pd.read_hdf(('analysis/util/simdata_analysis/'
                                      'test/data/1531/db.h5'),
                                     'scoredata')['experimental']
    self.assertListEqual([i for i in range(len(score_experimental)) if
                          score_experimental[i]], experimental_idxs)

  def test_multiple_dataset_data(self):
    infiles = [('analysis/util/simdata_analysis/test/data/'
                'Crosswind_sweeps_monte_carlo_nightly_1531'),
               ('analysis/util/simdata_analysis/test/data/'
                'Crosswind_sweeps_monte_carlo_nightly_1531')]
    bsdi = BatchSimDataImporter(infiles, process_inputs=False)
    bsdi.create_database('analysis/util/simdata_analysis/test/data/1531/db.h5')

    num_tests = 10
    for (score, table) in zip(random.sample(range(bsdi.num_scores - 1),
                                            num_tests),
                              random.sample(range(bsdi.num_tables), num_tests)):

      run_id = random.randint(1, bsdi.num_table_samples[table]) - 1
      infile = open(bsdi.input_files[0], 'r')
      scorename = pd.read_hdf(('analysis/util/simdata_analysis/'
                               'test/data/1531/db.h5'),
                              'scoredata')['name'][score]
      score_list = json.load(infile)['metrics']
      self.assertEqual(scorename, score_list[score]['name'])
      infile.close()

      scores = []
      for input_fn in bsdi.input_files:
        fp = open(input_fn, 'r')
        raw_data = json.load(fp)
        fp.close()
        scores.extend([job_data['scores'][score] for job_data in
                       raw_data['table_data'][table]['job_data'] if
                       job_data['sim_success']])

      if scores[run_id] is not None:
        in_data = pd.read_hdf(('analysis/util/simdata_analysis/'
                               'test/data/1531/db.h5'),
                              'tables/table_{0}'.format(table))
        self.assertEqual(in_data['score_{0}'.format(score)][run_id],
                         scores[run_id])

  def test_input_data(self):
    infiles = [('analysis/util/simdata_analysis/test/data/'
                'manual_crosswind_sweeps_3859')]
    bsdi = BatchSimDataImporter(infiles, process_inputs=True)
    bsdi.create_database('analysis/util/simdata_analysis/test/data/'
                         'manual_crosswind_sweeps_3859/db.h5')
    # Read input table.
    z_inertia_error_str = ('sim.wing_sim.mass_prop_uncertainties.'
                           'moment_of_inertia_scale.2')
    z_inertia_error_df = pd.read_hdf(
        ('analysis/util/simdata_analysis/test/data/'
         'manual_crosswind_sweeps_3859/db.h5'),
        'inputs/' + z_inertia_error_str)

    # Check the type is correct.
    z_inertia_error = z_inertia_error_df['value'].values
    self.assertIsInstance(z_inertia_error, np.ndarray)

    # Check the file stored in each entry is correct.
    folder_list = z_inertia_error_df['folder'].tolist()
    self.assertEqual(len(set(folder_list)), 1)

    # Check some values.
    z_inertia_error_jobid_11 = z_inertia_error_df[
        z_inertia_error_df['job_id'] == 11]['value'].values[0]
    self.assertAlmostEqual(z_inertia_error_jobid_11, 0.980477179416295, 12)
    z_inertia_error_jobid_77 = z_inertia_error_df[
        z_inertia_error_df['job_id'] == 77]['value'].values[0]
    self.assertAlmostEqual(z_inertia_error_jobid_77, 1.0222892978321174, 12)
    z_inertia_error_jobid_0 = z_inertia_error_df[
        z_inertia_error_df['job_id'] == 0]['value'].values[0]
    self.assertAlmostEqual(z_inertia_error_jobid_0, 1.0000000000000000, 12)
    z_inertia_error_jobid_215 = z_inertia_error_df[
        z_inertia_error_df['job_id'] == 215]['value'].values[0]
    self.assertAlmostEqual(z_inertia_error_jobid_215, 0.9622778751299995, 12)

    # Check that the outputs map correctly to the inputs.
    job_id_pick = 104
    score_pick = 'score_51'  # '-Crash- Gsg Termination Range [deg]'
    score_table_df = pd.read_hdf(
        ('analysis/util/simdata_analysis/test/data/'
         'manual_crosswind_sweeps_3859/db.h5'),
        'tables/table_11')
    score = score_table_df[
        score_table_df['job_id'] == job_id_pick][score_pick].values[0]
    self.assertAlmostEqual(score, -4.9845969028882, 12)
    z_inertia_error_jobid_104 = z_inertia_error_df[
        z_inertia_error_df['job_id'] == job_id_pick]['value'].values[0]
    self.assertAlmostEqual(z_inertia_error_jobid_104, 0.9993414627740633, 12)


if __name__ == '__main__':
  unittest.main()
