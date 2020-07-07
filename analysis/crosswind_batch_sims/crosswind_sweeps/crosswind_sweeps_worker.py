#!/usr/bin/python
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

"""Worker for a crosswind sweeps batch simulation."""

import sys

import gflags
from makani.analysis.crosswind_batch_sims import batch_sim_params
from makani.lib.python import wing_flag
from makani.lib.python.batch_sim import worker as worker_base
from makani.lib.python.batch_sim.scoring_functions import worker as score_worker

FLAGS = gflags.FLAGS


class CrosswindSweepsSimWorker(score_worker.ScoringFunctionsSimWorker):
  """Worker for a crosswind sweeps batch simulation."""

  def __init__(self, *args, **kwargs):
    wing_model = wing_flag.FlagToWingModelName(FLAGS.wing_model)
    # TODO: Support scoring based on different flight plans and
    # test sites.
    super(CrosswindSweepsSimWorker, self).__init__(
        batch_sim_params.CrosswindSweepsParameters(
            offshore=True, wing_model=wing_model),
        *args, **kwargs)


def main(argv):
  worker_base.InitMain(argv)
  worker = CrosswindSweepsSimWorker()
  worker.Run()


if __name__ == '__main__':
  main(sys.argv)
