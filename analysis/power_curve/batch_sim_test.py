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

"""Tests for power curve batch sim client and worker."""

import json
import os
import unittest

import gflags
import h5py
import makani
from makani.analysis.power_curve import batch_sim_client
from makani.analysis.power_curve import batch_sim_worker
from makani.control import control_types
from makani.lib.python import os_util
from makani.lib.python import test_util
from makani.lib.python.batch_sim import testing_fakes
import numpy

FLAGS = gflags.FLAGS


def MakeFakeLogFile(wind_speed, file_name, num_samples):
  """Makes a fake log file for a worker to consume.

  Args:
    wind_speed: List of wind speed values.
    file_name: Name of the log file.
    num_samples: Number of data points for the log file.
  """
  log_file = test_util.CreateSampleHDF5File(file_name, num_samples)
  telem_time = numpy.linspace(0.0, FLAGS.sim_time, num_samples)

  with test_util.H5DatasetWriter(
      log_file['parameters']['sim_params']) as sim_params:
    sim_params['phys_sim']['wind_speed'] = wind_speed
    sim_params['joystick_sim']['updates'][0, 0]['value'] = 0.5

  simulator = log_file['messages']['kAioNodeSimulator']
  controller_a = log_file['messages']['kAioNodeControllerA']
  with test_util.H5DatasetWriter(simulator['kMessageTypeSimTelemetry']) as s:
    s['message']['time'] = telem_time
    power = 1000.0 * numpy.linalg.norm(wind_speed)**3
    s['message']['rotors']['aero_power'][:][:] = power / 8.0

  with test_util.H5DatasetWriter(
      controller_a['kMessageTypeControlDebug']) as c:
    c['message']['flight_mode'][:] = control_types.kFlightModeCrosswindNormal
    c['message']['time'] = telem_time

  log_file.close()


class FakeBinaries(testing_fakes.FakeBinaries):

  _NUM_SAMPLES = 100

  def Controller(self, args):
    with open(self._ParseArg(args, '--all_params'), 'r') as config_file:
      config = json.load(config_file)
    self._wind_speed = config['sim']['phys_sim']['wind_speed']

  def GroundEstimator(self, args):
    with open(self._ParseArg(args, '--all_params'), 'r') as config_file:
      config = json.load(config_file)
    self._wind_speed = config['sim']['phys_sim']['wind_speed']

  def Simulator(self, args):
    with open(self._ParseArg(args, '--all_params'), 'r') as config_file:
      config = json.load(config_file)
    wind_speed = config['sim']['phys_sim']['wind_speed']
    wind_norm = numpy.linalg.norm(wind_speed)
    # Pretend the simulator fails for certain wind speeds.
    sim_success = wind_norm < 4.0 or wind_norm > 6.0
    return 0 if sim_success else 1

  def PcapToHdf5(self, args):
    log_file_name = self._ParseArg(args, '--output_file')
    MakeFakeLogFile(self._wind_speed, log_file_name, self._NUM_SAMPLES)


class PowerCurveTest(unittest.TestCase):

  SIM_NAME = 'power_curve'
  NUM_WORKERS = 3
  BINARY_MAP = {os.path.join(makani.HOME, k): v for k, v in [
      ('control/sim_controller', 'Controller'),
      ('lib/pcap_to_hdf5/pcap_to_hdf5', 'PcapToHdf5'),
      ('sim/sim', 'Simulator'),
      ('control/sim_ground_estimator', 'GroundEstimator')]}

  @test_util.FlagValueSaver()
  def testClientAndWorker(self):
    FLAGS.wind_speeds = numpy.linspace(3.0, 15.0, 10)

    patch = testing_fakes.PatchAllFakes(
        binary_map=self.BINARY_MAP,
        binaries=FakeBinaries(),
        worker_factory=batch_sim_worker.PowerCurveSimWorker,
        client_class=batch_sim_client.PowerCurveSimClient)

    with os_util.TempDir() as temp_dir:
      with patch, test_util.DisableWarnings():
        gflags.FLAGS.output_dir = temp_dir
        client = batch_sim_client.PowerCurveSimClient(
            num_workers=self.NUM_WORKERS, sim_name=self.SIM_NAME)
        client.Run()

        # Check that a data file containing the power curve was created.
        data_file = h5py.File(os.path.join(temp_dir, 'data.h5'))
        wind_speeds = data_file['wind_speed'][:]
        powers = data_file['crosswind_power'][:]
        sim_successes = data_file['sim_success'][:]
        for w, p, s in zip(wind_speeds, powers, sim_successes):
          self.assertAlmostEqual(1000.0 * w**3, p, delta=1e-8)
          self.assertEqual(s, w < 4.0 or w > 6.0)
        # Check that all the plots and HTML files were created.
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'mean_flap_deflections.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'standard_deviation_flap_deflections.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'faults_vs_wind_speed.png')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'power_curve_by_throttle.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'power_and_tension_curve.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'angles_vs_wind_speed.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'angle_errors_vs_wind_speed.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'radius_vs_wind_speed.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'curvature_errors_vs_wind_speed.svg')))
        self.assertTrue(os.path.isfile(
            os.path.join(temp_dir, 'index.html')))

if __name__ == '__main__':
  unittest.main()
