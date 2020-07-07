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

"""Tests for the batch sim client and worker modules.

Testing either component independently doesn't offer much benefit since you end
up having to mock or fake the behavior of the other.
"""

import contextlib
import json
import os
import unittest

import gflags
import h5py
import makani
from makani.lib.python import os_util
from makani.lib.python import test_util
from makani.lib.python.batch_sim import client as client_module
from makani.lib.python.batch_sim import testing_fakes
from makani.lib.python.batch_sim import worker as worker_module
import mock
import numpy as np

FLAGS = gflags.FLAGS

_SIM_TIME = 120.0


class TestSimClient(client_module.BatchSimClient):

  _BASE_PATH = 'path/to/base_target'

  def __init__(self, **kwargs):
    super(TestSimClient, self).__init__('fake_worker_main.py', **kwargs)
    self.total_sim_time = 0.0

  def _GenerateConfigs(self):
    for i in xrange(10):
      yield {
          'config_index': i,
          'sim': {'phys_sim': {'wind_database': {'name': ''}},
                  'sim_time': _SIM_TIME}
          }

  @client_module.JsonReducer
  def _ReduceWorkerOutput(self, outputs):
    for output in outputs:
      self.total_sim_time += output['final_time']


class FakeBinaries(testing_fakes.FakeBinaries):

  def Controller(self, args):
    self._config_file = self._ParseArg(args, '--all_params')
    # Simulate being terminated with SIGINT.
    return -2

  def GroundEstimator(self, args):
    self._config_file = self._ParseArg(args, '--all_params')
    # Simulate being terminated with SIGINT.
    return -2

  def Simulator(self, args):
    self._config = json.load(open(self._config_file, 'r'))
    return 0

  def PcapToHdf5(self, args):
    input_filename = self._ParsePositionalArg(args, 0)
    if not os.path.isfile(input_filename):
      raise ValueError('First positional arg to pcap_to_hdf5 (%s) is '
                       'not a file.' % input_filename)

    self._output_file = self._ParseArg(args, '--output_file')
    num_samples = 10  # Arbitrary
    telem_time = np.linspace(0.0, self._config['sim']['sim_time'], num_samples)

    with contextlib.closing(test_util.CreateSampleHDF5File(
        self._output_file, num_samples)) as log_file:
      sim_node = log_file['messages']['kAioNodeSimulator']
      with test_util.H5DatasetWriter(sim_node['kMessageTypeSimTelemetry']) as s:
        s['message']['time'] = telem_time

      control_node = log_file['messages']['kAioNodeControllerA']
      with test_util.H5DatasetWriter(
          control_node['kMessageTypeControlDebug']) as c:
        c['message']['time'] = telem_time

    return 0


class GoodWorker(worker_module.BatchSimWorker):
  """This worker performs a simple transformation of the log file."""

  def _ProcessSimOutput(self, unused_config_id, sim_success, log_file_name):
    assert sim_success
    with contextlib.closing(h5py.File(log_file_name, 'r')) as log_file:
      sim_time = (log_file['messages']['kAioNodeSimulator']
                  ['kMessageTypeSimTelemetry']['message']['time'])
      return json.dumps({'final_time': sim_time[-1]})


class BadWorker(worker_module.BatchSimWorker):
  """This worker dies horribly so we can test the client's error-handling."""

  def _ProcessSimOutput(self, unused_config_id, unused_sim_success,
                        log_file_name):
    raise testing_fakes._FakeError('Oh no!')


class BatchSimClientTest(unittest.TestCase):

  SIM_NAME = 'test_sim'
  NUM_WORKERS = 3

  BINARY_MAP = {os.path.join(makani.HOME, k): v for k, v in [
      ('control/sim_controller', 'Controller'),
      ('lib/pcap_to_hdf5/pcap_to_hdf5', 'PcapToHdf5'),
      ('sim/sim', 'Simulator'),
      ('control/sim_ground_estimator', 'GroundEstimator')]}

  def testSuccess(self):
    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP, binaries=FakeBinaries(),
        worker_factory=GoodWorker, client_class=TestSimClient)

    with patch:
      client = TestSimClient(sim_name=self.SIM_NAME,
                             num_workers=self.NUM_WORKERS)
      client.Run()

    self.assertAlmostEqual(10 * _SIM_TIME, client.total_sim_time)

  @test_util.FlagValueSaver()
  def testMaxJobsPerWorker(self):
    for max_jobs, expected_num_workers in zip([1, 2, 3, 4, 5],
                                              [10, 5, 4, 3, 2]):
      FLAGS.max_jobs_per_worker = max_jobs

      patch = testing_fakes.PatchAllFakes(
          binary_map=self.BINARY_MAP, binaries=FakeBinaries(),
          worker_factory=GoodWorker, client_class=TestSimClient)

      with patch:
        client = TestSimClient(sim_name=self.SIM_NAME)
        client.Run()

      self.assertAlmostEqual(10 * _SIM_TIME, client.total_sim_time)
      self.assertEqual(expected_num_workers, client._num_workers)

  @test_util.FlagValueSaver()
  def testReduceOnly(self):
    with os_util.TempDir() as temp_dir:
      FLAGS.local_output_dir = temp_dir

      # Run a normal batch sim to generate local worker outputs.
      with testing_fakes.PatchAllFakes(
          binary_map=self.BINARY_MAP, binaries=FakeBinaries(),
          worker_factory=GoodWorker, client_class=TestSimClient):
        client = TestSimClient(sim_name=self.SIM_NAME,
                               num_workers=self.NUM_WORKERS)
        client.Run()
      self.assertAlmostEqual(10 * _SIM_TIME, client.total_sim_time)

      # Run a reduce-only client with patches that will make it crash horribly
      # if it does anything other than reducing.
      FLAGS.reduce_only = True
      with testing_fakes.PatchAllFakes(binary_map=None,
                                       binaries=None,
                                       worker_factory=None,
                                       client_class=TestSimClient):
        reducer_client = TestSimClient(sim_name=self.SIM_NAME,
                                       num_workers=self.NUM_WORKERS)
        reducer_client.Run()
      self.assertAlmostEqual(10 * _SIM_TIME, client.total_sim_time)

  def testError(self):
    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP, binaries=FakeBinaries(),
        worker_factory=BadWorker, client_class=TestSimClient)

    with patch, mock.patch(
        'makani.lib.python.batch_sim.client.logging') as mock_log:
      client = TestSimClient(sim_name=self.SIM_NAME,
                             num_workers=self.NUM_WORKERS)
      client.Run()

    self.assertTrue(mock_log.error.called)
    self.assertEqual(0.0, client.total_sim_time)
    self.assertEqual({}, client._gcompute.metadata_by_instance_name)

  def testNoDeleteWorkersOnError(self):
    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP, binaries=FakeBinaries(),
        worker_factory=BadWorker, client_class=TestSimClient)

    with patch, mock.patch(
        'makani.lib.python.batch_sim.client.logging') as mock_log:
      with test_util.FlagValueSaver():
        FLAGS.delete_workers_on_error = False
        client = TestSimClient(sim_name=self.SIM_NAME,
                               num_workers=self.NUM_WORKERS)
        client.Run()

    self.assertTrue(mock_log.error.called)
    self.assertEqual(0.0, client.total_sim_time)
    self.assertEqual(self.NUM_WORKERS,
                     len(client._gcompute.metadata_by_instance_name))


if __name__ == '__main__':
  unittest.main()
