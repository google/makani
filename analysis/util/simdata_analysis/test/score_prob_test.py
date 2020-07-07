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

from makani.analysis.util.simdata_analysis import score_prob


class TestScoreProb(unittest.TestCase):

  def test_score_prob(self):
    folderlist = [('analysis/util/simdata_analysis/test/data/'
                   'Crosswind_sweeps_monte_carlo_nightly_1531')]
    output_folder = 'analysis/util/simdata_analysis/test/data/1531'
    # Not 1 so the torque ratio scores (which are exactly 100 when bad)
    # get captured as bad.
    score_thr = 0.999

    score_prob.process_score_probabilities(folderlist, output_folder,
                                           score_thr=score_thr,
                                           use_concurrency=True, ncores=None)


if __name__ == '__main__':
  unittest.main()
