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

import os
import unittest

from makani.analysis.util.simdata_analysis import compare_simdata
from makani.analysis.util.simdata_analysis.data_import import BatchSimDataImporter
from makani.analysis.util.simdata_analysis.simdata import SimData


class TestCompare(unittest.TestCase):

  def test_compare(self):
    output_folder = 'analysis/util/simdata_analysis/test/data/compare_1524_1531'

    print('Creating prior DB...')
    folderlist_prior = [('analysis/util/simdata_analysis/test/data/'
                         'Crosswind_sweeps_monte_carlo_nightly_1524')]
    prior_db_fn = os.path.join(output_folder, 'prior.h5')
    BatchSimDataImporter(
        folderlist_prior, process_inputs=False).create_database(prior_db_fn)
    simdata_prior = SimData(prior_db_fn)
    print('...done.')

    print('Creating post DB...')
    folderlist_post = [('analysis/util/simdata_analysis/test/data/'
                        'Crosswind_sweeps_monte_carlo_nightly_1531')]
    post_db_fn = os.path.join(output_folder, 'post.h5')
    BatchSimDataImporter(
        folderlist_post, process_inputs=False).create_database(post_db_fn)
    simdata_post = SimData(post_db_fn)
    print('...done.')

    # Score threshold not set exactly to 1, so the torque ratio scores
    # (which are exactly 100 when bad) get captured as bad.
    score_thr = 0.999

    compare_simdata.compare_simulation_data(simdata_prior,
                                            simdata_post,
                                            os.path.join(output_folder,
                                                         'comp.csv'),
                                            score_thr,
                                            use_concurrency=True,
                                            ncores=None)


if __name__ == '__main__':
  unittest.main()
