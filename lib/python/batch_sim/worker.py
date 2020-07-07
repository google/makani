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

"""Base definition for a batch worker and batch simulation worker.

See the docstring of client.py in this directory for an overview, and
BatchSimWorker below for details on defining workers.
"""

import contextlib
import gc
import io
import json
import logging
import os
import re
import shutil
import signal
import tempfile
import time

import gflags
import h5py
import makani
from makani.lib.python import gsutil
from makani.lib.python import os_util
from makani.lib.python import shell_interfaces
from makani.lib.python.batch_sim import batch_sim_util
from makani.lib.python.batch_sim import gcloud_util
from makani.lib.python.h5_utils import h5_io

gflags.DEFINE_string('config_dir', os.path.join(makani.HOME, 'gce_config'),
                     'Directory containing JSON config files.')

gflags.DEFINE_list('config_range', None,
                   'Inclusive integer range indicating the configuration files '
                   'that this worker should process.')

gflags.DEFINE_boolean('scoring_events', False,
                      'Analyze scoring function events.')

FLAGS = gflags.FLAGS

DEFAULT_BUCKET = 'makani'


def _MakeSparseLog(dense_path, sparse_path, downsampling=10):
  """Makes a sparse log file for quicker downloading and plotting.

  The sparse log file contains only SimTelemetry and ControlDebugMessage, stored
  at the same paths as in the original log, and downsampled according to the
  downsampling parameter.

  Args:
    dense_path: Path to the dense (original) log file.
    sparse_path: Path to the sparse (output) log file.
    downsampling: Factor by which to downsample timeseries in the sparse log.
  """
  dense_log = h5py.File(dense_path, 'r')

  # These datasets will be copied, downsampled, to the sparse log.
  #
  # Note that each dataset in the "parameters" group is length-1, so the
  # downsampling will have no effect on it.
  datasets = [
      ('messages', 'kAioNodeSimulator', 'kMessageTypeSimTelemetry'),
      ('messages', 'kAioNodeControllerA', 'kMessageTypeControlDebug'),
  ] + [('parameters', key) for key in dense_log['parameters'].keys()]

  sparse_data = {}
  for ref_chain in datasets:
    # Grab the dense dataset.
    dense_dataset = dense_log
    for ref in ref_chain:
      dense_dataset = dense_dataset[ref]

    # Populate the sparse dataset.
    ref = sparse_data
    for group in ref_chain[:-1]:
      if group not in ref:
        ref[group] = {}
      ref = ref[group]
    ref[ref_chain[-1]] = dense_dataset[::downsampling]

  h5_io.H5Dump(sparse_path, sparse_data, compression_level=5)


class BatchJobWorker(object):
  """Base class for a worker of a batch job.

  A proper subclass must define the Run() method.

  The main function need only initialize flags (a helper InitMain() is provided
  below), instantiate the worker, and call its Run() method.  E.g.
      def main(argv):
        worker_module.InitMain(argv)  # From this module.
        worker = MyWorkerSubclass()
        worker.Run()
  """

  # Timeout simulations after 20 minutes.
  _SIM_TIMEOUT_SEC = 60.0 * 20.0

  # Wait time before starting sim.
  _SIM_STARTUP_WAIT_SEC = 5.0

  def __init__(self, config_dir=None, config_range=None):
    """Initialize the BatchSimWorker.

    Args:
      config_dir: Directory that contains config files.  File names must be of
          the form '%d.json' % file_number.  --config_dir is used by default.
      config_range: Pair of integers or indicating the inclusive range of
          configs to be processed.  --config_range is used by default.
    """
    if config_dir is None:
      config_dir = FLAGS.config_dir
    self._config_dir = config_dir

    if config_range is None:
      config_range = (int(FLAGS.config_range[0]), int(FLAGS.config_range[1]))

    first, last = config_range  # pylint: disable=unpacking-non-sequence
    self._config_ids = range(first, last + 1)
    self._scoring_events = FLAGS.scoring_events

  def _ConfigFileName(self, config_id):
    return os.path.join(self._config_dir, '%d.json' % config_id)

  def Run(self):
    raise NotImplementedError()


class BatchSimWorker(BatchJobWorker):
  """Base class for a batch simulation worker.

  A proper subclass must define the _ProcessSimOutput method.  This method maps
  the return code and log file from an individual simulation to the output
  format that will be consumed by the client's _ReduceWorkerOuptut().
  """

  def _ProcessSimOutput(self, config_id, sim_success, log_file_name):
    """Parses simulation output and returns contents of an output file.

    This method must be defined on a per-application basis.

    Args:
      config_id: ID of the parameter file used.
      sim_success: Whether the sim was successful.
      log_file_name: Name of the log file.

    Returns:
      Blob of results, to be passed to the client via GCS.

    Raises:
      SimWorkerError: Something failed, e.g. the log file couldn't be parsed.
    """
    raise NotImplementedError()

  # Helper function to check exit codes of spawned processes.
  def _LogProcessReturnCode(self, name, process, expected_returncode):
    if process.returncode != expected_returncode:
      if process.returncode is None:
        logging.warning('%s is not yet terminated!', name)
      else:
        logging.warning('%s exited with unexpected return value %d',
                        name, process.returncode)

  def _CheckTelemetryTimes(self, config_file_name, log_file_name):
    """Checks whether sim and control telemetries reach the configured time."""
    with open(config_file_name, 'r') as config_file:
      config_json = json.load(config_file)
    expected_time = config_json['sim']['sim_time']

    with contextlib.closing(h5py.File(log_file_name, 'r')) as log_file:
      last_sim_time = (log_file['messages']['kAioNodeSimulator']
                       ['kMessageTypeSimTelemetry']['message']['time'])[-1]
      last_control_time = (log_file['messages']['kAioNodeControllerA']
                           ['kMessageTypeControlDebug']['message']
                           ['time'])[-1]

    if (abs(last_sim_time - expected_time) > 1.0
        or abs(last_control_time - expected_time) > 1.0):
      logging.warning('Disagreement between configured sim time (%g) and last '
                      '(sim, controller) time (%g, %g).', expected_time,
                      last_sim_time, last_control_time)
      return False
    return True

  def _RunSimulation(self, config_id, log_dir):
    """Run a simulation defined by the given config.

    Args:
      config_id: Integer identifying this configuration.
      log_dir: Directory in which to store the h5 log file.

    Returns:
      Filename of the h5 log resulting from the simulation, and the output
      of _ProcessSimOutput().
    """

    config_file_name = self._ConfigFileName(config_id)
    logging.info('Running simulation with config file %s.', config_file_name)
    log_file_name = os.path.join(log_dir, 'log_file.h5')
    if os.path.exists(log_file_name):
      os.remove(log_file_name)

    # Blargh. AppArmor won't let tcpdump write a file in any subdirectory of
    # ${HOME} that begins with a dot. This prevents a local worker from running
    # under Bazel, as all the Bazel directoriess are beneath ${HOME}/.cache.  To
    # hack around that limitation, we use a temporary file for the output and
    # then move that file to the intended path.
    with tempfile.NamedTemporaryFile(suffix='.pcap', delete=False) as f:
      temp_pcap_path = f.name
    pcap_path = os.path.join(log_dir, 'last.pcap')

    # Prepare the turbsim file, as needed.
    # TODO: This is mostly copied from run_sim - move to a new
    # module (lib/python/turbsim_util.py) that both can use once this and
    # parent CL are merged.
    with open(config_file_name, 'r') as params_path:
      all_params = json.load(params_path)
    cloud_path = all_params['sim']['phys_sim']['wind_database']['name']
    local_path = os.path.basename(cloud_path)
    turbsim_name_re = re.compile(r'\d\d\d\d[0-1]\d[0-3]\d'  # date
                                 r'-[0-2]\d[0-5]\d[0-5]\d'  # time
                                 r'-\d\d\d'  # online folder identifier
                                 r'-\d\d'    # iteration number from start seed
                                 r'_\d\dmps_\d\dshear\.h5')  # wind conditions
    # Check that the name includes the path to the online folder
    # and that the database name follows the TurbSim naming convention.
    if (cloud_path.startswith('gs://makani_turbsim_databases') and
        turbsim_name_re.match(local_path)):
      gsutil_api = gsutil.GsutilApi()
      gsutil_api.Copy(cloud_path, local_path, False)
      # Update wind database path in config file to downloaded location.
      all_params['sim']['phys_sim']['wind_database']['name'] = (
          os.path.abspath(local_path))
      with open(config_file_name, 'w') as params_path:
        json.dump(all_params, params_path)

    # Ensure that the unpacked worker runfiles dir is passed down.
    makani.SetRunfilesDirFromBinaryPath()

    # Run the controller, recorder, and simulator, blocking on the simulator's
    # completion.
    executor = shell_interfaces.Executor()
    os.chmod(os.path.join(makani.HOME, 'lib/scripts/sim_tcpdump.sh'), 0o777)
    tcpdump_process = executor.RunInBackground(
        [os.path.join(makani.HOME, 'lib/scripts/sim_tcpdump.sh'),
         temp_pcap_path])
    os.chmod(os.path.join(makani.HOME, 'control/sim_controller'), 0o777)
    controller_process = executor.RunInBackground(
        [os.path.join(makani.HOME, 'control/sim_controller'),
         '--all_params=' + config_file_name])
    os.chmod(os.path.join(makani.HOME, 'control/sim_ground_estimator'), 0o777)
    estimator_process = executor.RunInBackground(
        [os.path.join(makani.HOME, 'control/sim_ground_estimator'),
         '--all_params=' + config_file_name])

    # Wait before starting the sim to give enough time to the controller and the
    # estimator to start and be able to receive messages.
    time.sleep(self._SIM_STARTUP_WAIT_SEC)
    os.chmod(os.path.join(makani.HOME, 'sim/sim'), 0o777)
    try:
      sim_process = executor.Run(
          [os.path.join(makani.HOME, 'sim/sim'),
           '--all_params=' + config_file_name],
          timeout_sec=self._SIM_TIMEOUT_SEC)
    except shell_interfaces.TimeoutError as e:
      sim_timed_out = True
      logging.error(e)
    else:
      sim_timed_out = False
      self._LogProcessReturnCode('Simulator', sim_process, 0)

    logging.info('Sim done.')

    # The controller should terminate naturally. If it doesn't get there on its
    # own, kill it.
    if controller_process.poll() is None:
      time.sleep(1)
      if controller_process.poll() is None:
        controller_process.send_signal(signal.SIGINT)

    if estimator_process.poll() is None:
      time.sleep(1)
      if estimator_process.poll() is None:
        estimator_process.send_signal(signal.SIGINT)
    # After the controller has been killed, sleep for 1 second before
    # terminating the logger to make sure the entire log file is
    # captured.
    time.sleep(1)

    # Ask tcpdump to terminate.
    tcpdump_process.send_signal(signal.SIGINT)

    # Wait for the background processes to terminate.
    controller_process.wait()
    tcpdump_process.wait()

    # Check expected return codes.
    self._LogProcessReturnCode('Controller', controller_process, 0)
    self._LogProcessReturnCode('TcpDump', tcpdump_process, 0)

    shutil.move(temp_pcap_path, pcap_path)
    os.chmod(os.path.join(makani.HOME, 'lib/pcap_to_hdf5/pcap_to_hdf5'), 0o777)
    pcap_converter_process = executor.Run(
        [os.path.join(makani.HOME, 'lib/pcap_to_hdf5/pcap_to_hdf5'),
         '--all_params=' + config_file_name,
         '--output_file=' + log_file_name, pcap_path])

    self._LogProcessReturnCode('PcapToHdf5', pcap_converter_process, 0)

    sim_successful = (not sim_timed_out and sim_process.returncode == 0)
    if sim_successful:
      self._CheckTelemetryTimes(config_file_name, log_file_name)

    return (log_file_name, sim_successful)

  def Run(self):
    """Runs a bach of simulations."""
    self._gstorage = gcloud_util.CloudStorageApi(DEFAULT_BUCKET)

    with os_util.TempDir('logs') as log_dir:
      for config_id in self._config_ids:

        with open(self._ConfigFileName(config_id), 'r') as config_file:
          sim_config = json.load(config_file)
        output_file_path = sim_config['output_file_path']
        h5_log_file_path = sim_config['h5_log_file_path']
        h5_keep_sparse_log_only = sim_config['h5_keep_sparse_log_only']

        # Run the simulation.
        local_h5_log_path, success = self._RunSimulation(config_id, log_dir)

        # Archive the h5 log (if it exists) if a filename is given.
        # Do this before processing the sim output, so that the h5 log
        # is still archived even if there is a fatal error in
        # processing the h5 log.
        sparse_local_log_path = local_h5_log_path.replace('.h5', '_sparse.h5')
        sparse_log_path = h5_log_file_path.replace('.h5', '_sparse.h5')
        try:
          if h5_log_file_path and os.path.isfile(local_h5_log_path):
            _MakeSparseLog(local_h5_log_path, sparse_local_log_path)
            if h5_log_file_path.startswith('gs://makani'):
              if not h5_keep_sparse_log_only:
                self._gstorage.UploadFile(local_h5_log_path,
                                          re.sub('^gs://makani/', '',
                                                 h5_log_file_path))
              self._gstorage.UploadFile(sparse_local_log_path,
                                        re.sub('^gs://makani/', '',
                                               sparse_log_path))
            else:
              if not h5_keep_sparse_log_only:
                shutil.copyfile(local_h5_log_path, h5_log_file_path)
              shutil.copyfile(sparse_local_log_path, sparse_log_path)
        except IOError:
          logging.warning('Unable to archive h5 log file.')

        # Process the output.
        sim_output = self._ProcessSimOutput(config_id, success,
                                            local_h5_log_path)

        # Store the processed simulation output, to be consumed by the
        # client.  This must be done last, as the client will
        # forcefully delete all worker instances once it sees all
        # expected output files.
        if output_file_path.startswith('gs://makani'):
          self._gstorage.UploadStream(io.BytesIO(sim_output),
                                      re.sub('^gs://makani/', '',
                                             output_file_path))
        else:
          with open(output_file_path, 'w') as f:
            f.write(sim_output)
        del sim_output
        gc.collect()


def InitMain(argv):
  """Helper for initializing a worker's main()."""

  def ValidateConfigRange(config_range):
    if len(config_range) != 2:
      return False
    for i in config_range:
      try:
        int(i)
      except ValueError:
        return False
    return True

  gflags.RegisterValidator(
      'config_range',
      ValidateConfigRange,
      message=('--config_range must be of the form <first>,<last>, where '
               '<first> and <last> are valid integers.'))

  gcloud_util.InitializeFlagsWithOAuth2(argv)

  batch_sim_util.SetUpLogging()
