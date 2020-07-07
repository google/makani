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

"""Tests for IEC cases batch sim client and worker."""

import json
import logging
import os
import re
import unittest

import gflags
import makani
from makani.analysis.iec_cases import batch_sim_client
from makani.analysis.iec_cases import batch_sim_worker
from makani.lib.python import os_util
from makani.lib.python import test_util
from makani.lib.python.batch_sim import testing_fakes
from makani.sim import sim_types
import numpy

FLAGS = gflags.FLAGS


def MakeFakeLogFile(file_name, num_samples):
  """Makes a fake log file for a worker to consume.

  Args:
    file_name: Name of the log file.
    num_samples: Number of data points for the log file.
  """
  log_file = test_util.CreateSampleHDF5File(file_name, num_samples)

  telem_time = numpy.linspace(0.0, 150.0, num_samples)

  simulator = log_file['messages']['kAioNodeSimulator']
  with test_util.H5DatasetWriter(simulator['kMessageTypeSimTelemetry']) as sim:
    sim['message']['time'] = telem_time
    sim['message']['wing']['tether_force_b']['tension'][:] = 2000.0
    for i, x in enumerate(('x', 'y', 'z')):
      sim['message']['wing']['Xg'][x] = float(i)

  controller = log_file['messages']['kAioNodeControllerA']
  with test_util.H5DatasetWriter(
      controller['kMessageTypeControlDebug']) as c:
    c['message']['time'] = telem_time

  log_file.close()


class IecCasesTest(unittest.TestCase):

  SIM_NAME = 'iec_cases_test'
  NUM_WORKERS = 3
  BINARY_MAP = {os.path.join(makani.HOME, k): v for k, v in [
      ('control/sim_controller', 'Controller'),
      ('lib/pcap_to_hdf5/pcap_to_hdf5', 'PcapToHdf5'),
      ('sim/sim', 'Simulator'),
      ('control/sim_ground_estimator', 'GroundEstimator')]}

  @test_util.FlagValueSaver()
  def testClientAndWorker(self):
    class FakeBinaries(testing_fakes.FakeBinaries):

      _NUM_SAMPLES = 100

      def Controller(self, args):
        return 0

      def GroundEstimator(self, args):
        return 0

      def Simulator(self, args):
        return 0

      def PcapToHdf5(self, args):
        log_file_name = self._ParseArg(args, '--output_file')
        MakeFakeLogFile(log_file_name, self._NUM_SAMPLES)
        return 0

    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP,
        binaries=FakeBinaries(),
        worker_factory=batch_sim_worker.IecCasesSimWorker,
        client_class=batch_sim_client.IecCasesSimClient)

    with os_util.TempDir() as temp_dir:
      with patch:
        gflags.FLAGS.output_dir = temp_dir
        client = batch_sim_client.IecCasesSimClient(
            num_workers=self.NUM_WORKERS, sim_name=self.SIM_NAME)
        client.Run()

        # Check that all the plots and HTML files were created.
        image_files = [f for f in os.listdir(temp_dir)
                       if re.match(r'.+\.png', f)]
        self.assertEqual(len(batch_sim_client.IEC_CASES), len(image_files))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'index.html')))

  @test_util.FlagValueSaver()
  @test_util.LogDisabler(logging.WARNING)
  def testNonzeroSimReturnCode(self):
    class FakeBinaries(testing_fakes.FakeBinaries):

      _NUM_SAMPLES = 100

      def Controller(self, args):
        with open(self._ParseArg(args, '--all_params'), 'r') as config_file:
          config = json.load(config_file)
        self._iec_case = config['sim']['iec_sim']['load_case']
        # Simulate being terminated with SIGINT.
        return -2

      def GroundEstimator(self, args):
        with open(self._ParseArg(args, '--all_params'), 'r') as config_file:
          config = json.load(config_file)
        self._iec_case = config['sim']['iec_sim']['load_case']
        # Simulate being terminated with SIGINT.
        return -2

      def Simulator(self, args):
        if (self._iec_case
            == sim_types.kIecCaseExtremeCoherentGustWithDirectionChange):
          return -1
        else:
          return 0

      def PcapToHdf5(self, args):
        log_file_name = self._ParseArg(args, '--output_file')
        MakeFakeLogFile(log_file_name, self._NUM_SAMPLES)
        return 0

    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP,
        binaries=FakeBinaries(),
        worker_factory=batch_sim_worker.IecCasesSimWorker,
        client_class=batch_sim_client.IecCasesSimClient)

    with os_util.TempDir() as temp_dir:
      with patch:
        gflags.FLAGS.cases = ['1.1', '1.3', '1.4b']
        gflags.FLAGS.output_dir = temp_dir
        gflags.FLAGS.local_output_dir = temp_dir
        client = batch_sim_client.IecCasesSimClient(
            num_workers=self.NUM_WORKERS, sim_name=self.SIM_NAME)
        client.Run()

        filenames = [os.path.join(temp_dir, '%d.json' % i) for i in range(3)]
        for filename in filenames:
          self.assertTrue(os.path.isfile(filename))

        sim_successes = []
        for filename in filenames:
          with open(filename, 'r') as f:
            config = json.load(f)
          sim_successes.append(config['sim_successful'])
        self.assertEqual([True, True, False], sim_successes)


if __name__ == '__main__':
  unittest.main()
