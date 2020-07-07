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

"""Worker for an IEC cases batch simulation."""

import json
import sys

import h5py
from makani.lib.python.batch_sim import worker as worker_base


class IecCasesSimWorker(worker_base.BatchSimWorker):
  """Worker for an IEC cases batch simulation."""

  def __init__(self, *args, **kwargs):
    super(IecCasesSimWorker, self).__init__(*args, **kwargs)

  def _ProcessSimOutput(self, unused_config_id, sim_success, log_file_name):
    # An unsuccessful sim is handled gracefully below, but we'll accept a
    # crash if there wasn't even a log file.
    log_file = h5py.File(log_file_name, 'r')

    simulator = log_file['messages']['kAioNodeSimulator']
    sim_telem = simulator['kMessageTypeSimTelemetry']['message']

    output = {
        'sim_successful': sim_success,
        'tether_tension': (
            sim_telem['wing']['tether_force_b']['tension'].tolist()),
        'time': sim_telem['time'].tolist(),
        'wing_Xg': {x: sim_telem['wing']['Xg'][x].tolist()
                    for x in ('x', 'y', 'z')}
    }

    log_file.close()

    return json.dumps(output, indent=2, separators=(',', ': '))


def main(argv):
  worker_base.InitMain(argv)
  worker = IecCasesSimWorker()
  worker.Run()


if __name__ == '__main__':
  main(sys.argv)
