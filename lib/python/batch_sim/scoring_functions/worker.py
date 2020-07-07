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

"""Abstract classes for batch_sim workers who handle scoring functions."""

import json
import os
import h5py

from makani.analysis.checks import autocheck
from makani.analysis.crosswind_batch_sims import crosswind_sweep_checks
from makani.lib.python import json_util
from makani.lib.python import struct_tree
from makani.lib.python.batch_sim import batch_sim_util
from makani.lib.python.batch_sim import worker as worker_base


class ScoringFunctionsSimWorker(worker_base.BatchSimWorker):
  """Abstract class for scoring function based batch simulation."""

  def __init__(self, params, *args, **kwargs):
    super(ScoringFunctionsSimWorker, self).__init__(*args, **kwargs)
    self._params = params

  def _ProcessSimOutput(self, unused_config_id, sim_success, log_file_name):
    """Processes the simulator output.

    Args:
      unused_config_id: ID of the parameter file used.
      sim_success: Whether the sim was successful.
      log_file_name: Name of the log file to be parsed.

    Returns:
      See parent class.
    """

    # TODO: Improve the following attempt to capture simulator
    # output such that it only captures the simulator output (i.e. via
    # some use of "tee"), works when run locally, etc.
    # TODO: Figure out a way to get rid of duplicated log entries, or
    # move the log capture to the batch_sim worker base class.
    sim_error_message = ''
    if not sim_success:
      sim_error_message = batch_sim_util.Tail('/var/log/syslog', 150)

    full_output = {
        'sim_success': sim_success,
        'sim_error_message': sim_error_message
    }

    # Only try to read the log file if the simulation succeeded.
    if sim_success:
      log_file = h5py.File(log_file_name, 'r')
      params = log_file['parameters']
      messages = log_file['messages']
      sim = messages['kAioNodeSimulator']['kMessageTypeSimTelemetry']['message']
      control = (messages['kAioNodeControllerA']['kMessageTypeControlDebug']
                 ['message'])

      wing_model = int(params['system_params']['wing_model'][0])

      for scoring_function in self._params.scoring_functions:
        assert scoring_function.GetName() not in full_output
        full_output[scoring_function.GetName()] = (
            scoring_function.GetOutput(
                scoring_function.GetTimeSeries(params, sim, control)))
      log_file.close()

      if self._scoring_events:
        checklist = crosswind_sweep_checks.CrosswindSweepChecks(
            True, wing_model)
        checklist.SetMinGap(10)
        # TODO: Clean up the description which can be different from
        # the original scoring functions (i.e. % of saturation --> saturation).
        check_results = autocheck.RunFromLocal(
            os.path.dirname(log_file_name),
            os.path.basename(log_file_name),
            checklist, None, False)
        full_output['events'] = struct_tree.StructTree(
            autocheck.GatherResultsFromMultipleFiles(check_results),
            True).Data()

    return json.dumps(full_output, indent=2,
                      separators=(',', ': '),
                      cls=json_util.JsonNumpyEncoder)
