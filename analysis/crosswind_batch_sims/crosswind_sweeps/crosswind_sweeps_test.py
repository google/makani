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

"""Tests for crosswind sweeps batch sim client and worker."""

import json
import os
import unittest

import gflags

import makani
from makani.analysis.crosswind_batch_sims.crosswind_sweeps import crosswind_sweeps_client
from makani.analysis.crosswind_batch_sims.crosswind_sweeps import crosswind_sweeps_worker
from makani.control import control_types
from makani.lib.python import os_util
from makani.lib.python import test_util
from makani.lib.python.batch_sim import testing_fakes
import numpy

FLAGS = gflags.FLAGS


def MakeFakeLogFile(ts, sim_time, wind_speed, file_name):
  """Makes a fake log file for a worker to consume.

  Args:
    ts: Time step.
    sim_time: Simulation duration.
    wind_speed: Wind speed.
    file_name: Name of the log file.
  """
  test_ts = 20 * ts  # To keep the test fast we sub-sample.
  num_samples = 10
  ground_z = 15.0
  sim_time = numpy.array(range(num_samples)) * test_ts
  control_time = sim_time

  log_file = test_util.CreateSampleHDF5File(file_name, num_samples)

  with test_util.H5DatasetWriter(
      log_file['parameters']['system_params']) as system_params:
    system_params['ground_frame']['ground_z'] = ground_z

  with test_util.H5DatasetWriter(
      log_file['parameters']['sim_params']) as sim_params:
    # Wind speed.
    sim_params['phys_sim']['wind_speed'] = wind_speed

  simulator = log_file['messages']['kAioNodeSimulator']
  with test_util.H5DatasetWriter(simulator['kMessageTypeSimTelemetry']) as sim:
    sim['message']['time'] = sim_time

    sim['message']['wing']['Xg']['z'][:] = wind_speed

  controller = log_file['messages']['kAioNodeControllerA']
  with test_util.H5DatasetWriter(
      controller['kMessageTypeControlDebug']) as ct:
    ct['message']['time'] = control_time
    ct['message']['flight_mode'][:] = control_types.kFlightModeCrosswindNormal
    ct['message']['flight_mode'][0] = control_types.kFlightModeHoverFullLength
    ct['message']['flight_mode'][1] = control_types.kFlightModeHoverAccel
    ct['message']['flight_mode'][2] = control_types.kFlightModeTransIn
    ct['message']['crosswind']['loop_angle'][-1] = numpy.pi * 3.0 / 2.0

  log_file.close()


class FakeBinaries(testing_fakes.FakeBinaries):

  def Controller(self, args):
    with open(self._ParseArg(args, '--all_params'), 'r') as config_file:
      config = json.load(config_file)
    self._ts = config['system']['ts']
    self._sim_time = config['sim']['sim_time']
    self._wind_speed = config['sim']['phys_sim']['wind_speed']
    self._faults_sim = config['sim']['faults_sim']
    # Simulate being terminated with SIGINT.
    return -2

  def GroundEstimator(self, args):
    with open(self._ParseArg(args, '--all_params'), 'r') as config_file:
      config = json.load(config_file)
    self._ts = config['system']['ts']
    self._sim_time = config['sim']['sim_time']
    self._wind_speed = config['sim']['phys_sim']['wind_speed']
    self._faults_sim = config['sim']['faults_sim']
    # Simulate being terminated with SIGINT.
    return -2

  def Simulator(self, args):
    # TODO(b/126956582): Fix this test.  The fake log data breaks the scoring
    # functions.  For now, simulate failed sim to replicate previous behavior.
    return -1

  def PcapToHdf5(self, args):
    log_file_name = self._ParseArg(args, '--output_file')
    MakeFakeLogFile(self._ts, self._sim_time, self._wind_speed,
                    log_file_name)
    return 0


class CrosswindSweepsTest(unittest.TestCase):

  SIM_NAME = 'crosswind_sweeps'
  NUM_WORKERS = 3
  BINARY_MAP = {os.path.join(makani.HOME, k): v for k, v in [
      ('control/sim_controller', 'Controller'),
      ('lib/pcap_to_hdf5/pcap_to_hdf5', 'PcapToHdf5'),
      ('sim/sim', 'Simulator'),
      ('control/sim_ground_estimator', 'GroundEstimator')]}

  @test_util.FlagValueSaver()
  def testClientAndWorker(self):
    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP,
        binaries=FakeBinaries(),
        worker_factory=crosswind_sweeps_worker.CrosswindSweepsSimWorker,
        client_class=crosswind_sweeps_client.CrosswindSweepsSimClient)

    # FLAGS are not automatically reset between tests.
    with os_util.TempDir() as temp_dir:
      with patch:
        gflags.FLAGS.output_dir = temp_dir
        gflags.FLAGS.wind_speeds = [5.0]
        client = crosswind_sweeps_client.CrosswindSweepsSimClient(
            num_workers=self.NUM_WORKERS, sim_name=self.SIM_NAME)
        client.Run()

        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'style.css')))
        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'index.html')))

    with os_util.TempDir() as temp_dir:
      with patch:
        gflags.FLAGS.output_dir = temp_dir
        gflags.FLAGS.custom_sweep = (
            '[{"name": "Mass Scale",'
            '"path": ["sim", "wing_sim", "mass_prop_uncertainties",'
            '"mass_scale"],'
            '"distribution": {"lower_bound": 0.9, "upper_bound": 1.1,'
            '"type": "uniform"},'
            '"values": [0.9, 1.0, 1.1]}]')
        gflags.FLAGS.only_custom_sweep = True
        client = crosswind_sweeps_client.CrosswindSweepsSimClient(
            num_workers=self.NUM_WORKERS, sim_name=self.SIM_NAME)
        client.Run()

        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'style.css')))
        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'index.html')))

        # Reset these flags to default before next test.
        gflags.FLAGS.custom_sweep = '[]'
        gflags.FLAGS.only_custom_sweep = False

    with os_util.TempDir() as temp_dir:
      with patch:
        gflags.FLAGS.output_dir = temp_dir
        gflags.FLAGS.monte_carlo = True
        gflags.FLAGS.monte_carlo_rows = 2
        gflags.FLAGS.monte_carlo_cols = 2
        gflags.FLAGS.wind_speeds = [5.0]
        client = crosswind_sweeps_client.CrosswindSweepsSimClient(
            num_workers=self.NUM_WORKERS, sim_name=self.SIM_NAME)
        client.Run()

        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'style.css')))
        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'index.html')))


if __name__ == '__main__':
  unittest.main()
