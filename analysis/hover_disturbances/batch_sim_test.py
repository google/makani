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

"""Tests for hover disturbances batch sim client and worker."""

import json
import os
import sys
import unittest

import gflags
import makani
from makani.analysis.hover_disturbances import batch_sim_client
from makani.analysis.hover_disturbances import batch_sim_worker
from makani.lib.python import os_util
from makani.lib.python import test_util
from makani.lib.python.batch_sim import testing_fakes
import numpy

FLAGS = gflags.FLAGS


def MakeFakeLogFile(ts, sim_time, wind_speed, faults_sim, file_name):
  """Makes a fake log file for a worker to consume.

  Args:
    ts: Time step.
    sim_time: Simulation duration.
    wind_speed: Wind speed.
    faults_sim: Simulated fault parameters.
    file_name: Name of the log file.
  """
  test_ts = 5 * ts  # To keep the test fast we sub-sample.
  num_samples = int(sim_time / test_ts)
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

    # Copy over faults.
    sim_params['faults_sim']['num_fault_events'] = (
        faults_sim['num_fault_events'])
    for i in range(faults_sim['num_fault_events']):
      sim_params['faults_sim']['fault_events'][0, i]['parameters'][:] = (
          faults_sim['fault_events'][i]['parameters'])
      for key in ('t_start', 't_end'):
        sim_params['faults_sim']['fault_events'][0, i][key] = (
            faults_sim['fault_events'][i][key])

  simulator = log_file['messages']['kAioNodeSimulator']
  with test_util.H5DatasetWriter(simulator['kMessageTypeSimTelemetry']) as sim:
    sim['message']['time'] = sim_time

    num_faults = faults_sim['num_fault_events']
    faults = faults_sim['fault_events'][0:num_faults]
    max_amplitude = numpy.linalg.norm(faults[-1]['parameters'][0:3])
    sim['message']['wing']['Xg']['z'][:] = wind_speed
    for fault in faults:
      amplitude = numpy.linalg.norm(fault['parameters'][0:3])
      msk = numpy.logical_and(sim_time >= fault['t_start'],
                              sim_time <= fault['t_end'])
      sim['message']['wing']['Xg']['z'][msk] = (
          wind_speed * (1.0 - amplitude / max_amplitude))

  controller = log_file['messages']['kAioNodeControllerA']
  with test_util.H5DatasetWriter(
      controller['kMessageTypeControlDebug']) as ct:
    ct['message']['time'] = control_time
    ct['message']['hover']['wing_pos_g_cmd']['z'][:] = wind_speed

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
    return 0

  def PcapToHdf5(self, args):
    log_file_name = self._ParseArg(args, '--output_file')
    MakeFakeLogFile(self._ts, self._sim_time, self._wind_speed,
                    self._faults_sim, log_file_name)
    return 0


class HoverDisturbancesTest(unittest.TestCase):

  SIM_NAME = 'hover_disturbances'
  NUM_WORKERS = 3
  BINARY_MAP = {os.path.join(makani.HOME, k): v for k, v in [
      ('control/sim_controller', 'Controller'),
      ('lib/pcap_to_hdf5/pcap_to_hdf5', 'PcapToHdf5'),
      ('sim/sim', 'Simulator'),
      ('control/sim_ground_estimator', 'GroundEstimator')]}

  @test_util.FlagValueSaver()
  def testClientAndWorker(self):
    FLAGS.wind_speeds = numpy.linspace(3.0, 15.0, 3)
    FLAGS.num_amplitudes = 2
    FLAGS.show_events = False

    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP,
        binaries=FakeBinaries(),
        worker_factory=batch_sim_worker.HoverDisturbancesSimWorker,
        client_class=batch_sim_client.HoverDisturbancesSimClient)

    with os_util.TempDir() as temp_dir:
      with patch:
        gflags.FLAGS.output_dir = temp_dir
        client = batch_sim_client.HoverDisturbancesSimClient(
            num_workers=self.NUM_WORKERS, sim_name=self.SIM_NAME)
        client.Run()

        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'style.css')))
        self.assertTrue(os.path.isfile(os.path.join(temp_dir, 'index.html')))


if __name__ == '__main__':
  FLAGS(sys.argv)
  unittest.main()
