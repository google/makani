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

"""Base definition for for a batch simulation client.

A batch simulation is a MapReduce-style computation consisting of the following
steps:
  1. A local client process creates configuration dictionaries.
  2. The client brings up worker instances on Compute Engine (GCE).
  3. Worker instances map each configuration dictionary to an output.
  4. Once all workers are complete, the client reduces all worker outputs to a
      final, user-determined form.

In order to use the batch simulation infrastructure, a user must provide:
  - A subclass of BatchSimClient with a main() script.
  - A subclass of BatchSimWorker with a main() script.

See the documentation for BatchSimClient (below) and BatchSimWorker (in
worker.py) for details.
"""
# TODO: Support running individual parts of the client.

import functools
import io
import json
import logging
import os
import re
import shutil
import subprocess
import sys
import tarfile
import time

import gflags
from googleapiclient import errors as gapi_errors
import makani
from makani.lib.python import os_util
from makani.lib.python import shell_interfaces
from makani.lib.python import wing_flag
from makani.lib.python.batch_sim import batch_sim_util
from makani.lib.python.batch_sim import gcloud_constants
from makani.lib.python.batch_sim import gcloud_util

wing_flag.AppeaseLintWhenImportingFlagOnly()

gflags.DEFINE_boolean('upload_worker_package', True,
                      'Upload worker package.')

gflags.DEFINE_integer('num_workers', None, 'Number of worker instances.')

gflags.DEFINE_integer('max_jobs_per_worker', None,
                      'Set number of workers to assign at most this many '
                      'configs per worker.')
gflags.RegisterValidator('max_jobs_per_worker', lambda m: m is None or m > 0)

gflags.DEFINE_bool('delete_old_workers', False,
                   'Delete workers left from a previous run. Do not run '
                   'a new batch sim.')

gflags.DEFINE_string('sim_name', None,
                     'Name of the batch simulation.  This must not match the '
                     'name of any currently-running batch simulation, as it '
                     'identifies file paths on Cloud Storage and worker '
                     'instances on Compute Engine.')

gflags.DEFINE_string('local_output_dir', None,
                     'Directory in which to store local output.  If '
                     'unspecified, a temp directory will be used.')

gflags.DEFINE_string('local_h5_logs_dir', None,
                     'Directory in which to locally archive h5 logs when '
                     'running with a local worker and the keep_h5_logs option.')

gflags.DEFINE_enum('worker_machine_type', 'n1-standard-2',
                   gcloud_constants.MACHINE_TYPES,
                   'Machine type for the Compute Engine workers.')

gflags.DEFINE_enum('worker_zone', 'us-central1-f', gcloud_constants.ZONES,
                   'Zone in which to run Compute Engine workers.')

gflags.DEFINE_string('worker_image', '',
                     'Name of disk image for Compute Engine workers.')

gflags.DEFINE_string('worker_image_project', 'google.com:makani',
                     'Project that owns the image specified by --worker_image.')

gflags.DEFINE_string('max_sim_time', 24 * 60 * 60,  # 1 day
                     'Maximum sim time [seconds] that may be specified in a '
                     'config.  This is primarily a guard against using the '
                     'default value of sys.float_info.max by forgetting to '
                     'override it.')

gflags.DEFINE_boolean('delete_workers_on_error', True,
                      'Indicates whether workers should be deleted in the '
                      'event of an error.  If set to False, workers will still '
                      'auto-delete themselves as configured in '
                      'worker_startup.sh.')

gflags.DEFINE_boolean('use_local_worker', False,
                      'Run locally with a single worker.  (--num_workers will '
                      'be ignored.)')

gflags.DEFINE_boolean('keep_h5_logs', False,
                      'Indicates whether the raw h5 log files should be '
                      'archived.')

gflags.DEFINE_boolean('keep_sparse_h5_logs', False,
                      'Indicates whether the sparse h5 log files should be '
                      'archived.')

gflags.DEFINE_boolean('reduce_only', False,
                      'Skip the worker creation and computation step and only '
                      'reduce the data that has already been calculated.  A '
                      'local output directory must be specified.')

gflags.DEFINE_boolean('quiet', False,
                      'Limit console output to warnings and errors.')

gflags.DEFINE_boolean('show_events', True,
                      'Show scoring function events in the result page of '
                      'each run.')

FLAGS = gflags.FLAGS

DEFAULT_BUCKET = 'makani'


class BatchSimClientError(Exception):
  pass


# TODO: Make a more generic "batch job client" class. This class has a
# few elements specific to batch sims -- the check on sim_time, and logic for
# HDF5 logs.
class BatchSimClient(object):
  """Client for a batch simulation.

  A proper subclass must provide the following:
    - Define the _GenerateConfigs() method to generate configuration
        dictionaries.
    - Define the _ReduceWorkerOutput() method, which reduces workers' outputs
        and reports them as desired.
    - Use the constructor arg `worker_main_script` to specify the filepath
        (relative to makani.HOME) for the worker's main script.
    - Use the constructor arg `extra_apt_dependencies` to specify any
        prerequisite packages that are not accounted for in
        _BASE_APT_DEPENDENCIES below.

  There are two additional constructor args:
     - `sim_name` is used to identify the simulation in both Cloud Storage and
         Compute Engine.  This must not match the name of any other currently
         running batch simulation.
     - `num_workers` indicates the number of instances that should be used.
  By default, these will be set using --sim_name and --num_workers,
  respectively.

  The main function need only initialize flags (a helper InitMain() is provided
  below), instantiate the client, and call its Run() method.  E.g.
      def main(argv):
        client_module.InitMain(argv)  # From this module.
        client = MyClientSubclass()
        client.Run()
  """

  # The base path for this batch sim.
  #
  # The names of the worker script and worker package are inferred from this.
  _BASE_PATH = None

  # Path to the bash script that workers will run on startup.
  _WORKER_STARTUP_SCRIPT = os.path.join(
      makani.HOME, 'lib/python/batch_sim/worker_startup.sh')

  # Baseline dependencies.
  # Before adding dependencies here, consider if building a new worker image is
  # more appropriate.  These packages will be installed on each worker startup.
  _BASE_APT_DEPENDENCIES = []

  # Names of any flags that should be forwarded from the client.
  _FORWARDED_FLAGS = []

  def __init__(self,
               extra_apt_dependencies=None,
               num_workers=None,
               sim_name=None):
    """Initialize the client.

    Args:
      extra_apt_dependencies: List of any packages that should be installied via
          apt-get, in addition to those specified by _BASE_APT_DEPENDENCIES.
      num_workers: Number of worker instances to run.  Defaults to
          --num_workers.
      sim_name: Name of the batch sim.  This is used to identify packages
          on GCS and workers on GCE.  Defaults to --sim_name.

    Raises:
      BatchSimClientError: Running with keep_h5_logs and use_local_worker but
      local_h5_logs_dir is unset.
    """

    if FLAGS.use_local_worker:
      try:
        subprocess.check_call([
            'bash', '-c',
            'source lib/scripts/mbash.sh && mbash::check_multicast_route',
            'lo'])
      except subprocess.CalledProcessError:
        print ('Multicast has not been configured for interface "lo".\n'
               'If trying to run a desktop simulation, please read '
               'lib/scripts/install/README_local_multicast.')
        sys.exit(1)
      else:
        num_workers = 1
    self._num_workers = (num_workers if num_workers is not None
                         else FLAGS.num_workers)
    if not FLAGS.delete_old_workers and FLAGS.max_jobs_per_worker is None:
      assert self._num_workers > 0, 'Number of workers must be positive.'

    self._zone = FLAGS.worker_zone

    self._sim_name = (sim_name if sim_name is not None else FLAGS.sim_name)
    assert self._sim_name, 'Sim name must be nonempty.'

    self._worker_name_prefix = ('batch-sim-%s-worker-'
                                % self._sim_name.replace('_', '-'))

    self._apt_dependencies = self._BASE_APT_DEPENDENCIES[:]
    if extra_apt_dependencies is not None:
      self._apt_dependencies += extra_apt_dependencies

    # GCS paths are relative to gs://makani/.
    self._gcs_base_dir = 'batch_sim/%s' % self._sim_name
    self._gcs_error_dir = self._gcs_base_dir + '/error'
    self._gcs_output_dir = self._gcs_base_dir + '/output'
    self._gcs_package_dir = self._gcs_base_dir + '/packages'
    self._gcs_h5_log_dir = self._gcs_base_dir + '/h5_logs'

    self._packager = shell_interfaces.Packager()
    self._gstorage = gcloud_util.CloudStorageApi(DEFAULT_BUCKET)
    self._gcompute = gcloud_util.ComputeEngineApi()

    self._local_output_dir = FLAGS.local_output_dir

    # Records whether an error occurred in the remote workers, so
    # cleanup can be controlled accordingly.
    self._worker_error = False

    if (FLAGS.keep_h5_logs
        or FLAGS.keep_sparse_h5_logs) and FLAGS.use_local_worker:
      if not FLAGS.local_h5_logs_dir:
        raise BatchSimClientError('local_h5_logs_dir is unset.')
      if not os.path.exists(FLAGS.local_h5_logs_dir):
        os.makedirs(FLAGS.local_h5_logs_dir)

  def _GetWorkerArgs(self, config_range):
    """Returns arguments for a worker's shell command.

    Args:
      config_range: Pair of integers indicating the inclusive range of configs
          to be processed.

    Returns:
      List of arguments for worker command.
    """
    range_string = '%d,%d' % config_range

    forwarded_flags = [FLAGS.FlagDict()[f].Serialize()
                       for f in self._FORWARDED_FLAGS]

    args = ['./worker.par', '--config_dir=gce_config',
            '--config_range=' + range_string,
            '--%sscoring_events' % ('' if FLAGS.show_events else 'no')
           ] + forwarded_flags
    if not FLAGS.use_local_worker:
      args.append('--gcloud_authentication_method=service_account')

    return args

  def _GenerateConfigs(self):
    """Generator that yields config dictionaries.

    Config dictionaries get fairly large, so if this method keeps them all in
    memory (e.g. by returning a list rather than yielding them as they are
    created), the client may OOM if producing thousands fo them.
    """
    raise NotImplementedError

  def _ReduceWorkerOutput(self, output_paths):
    """Reduces output from workers.

    This method produces the final output of the batch simulation.

    Args:
      output_paths: Iterable of output file paths, one for each simulation
          completed by a worker.  These are in the same order as their
          corresponding configuration files.  Their contents are determined by
          the workers' _ProcessSimOutput() method.
    """
    raise NotImplementedError

  def _MakeConfigPackage(self, package_path):
    """Produces the config package.

    Args:
      package_path: Path for the output file.

    Returns:
      Number of configs generated.

    Raises:
      BatchSimClientError: The final simulation time is unset or is too large,
      or running with keep_h5_logs and use_local_worker but local_h5_logs_dir
      is unset.
    """
    self._packager.OpenPackage(package_path)

    num_configs = 0
    for i, config in enumerate(self._GenerateConfigs()):
      if 'sim_time' not in config['sim']:
        raise BatchSimClientError('Sim time is unset.')
      elif config['sim']['sim_time'] > FLAGS.max_sim_time:
        raise BatchSimClientError(
            'Sim time (%s) exceeds the value of --max_sim_time (%s).' %
            (config['sim']['sim_time'], FLAGS.max_sim_time))

      if FLAGS.use_local_worker:
        config['output_file_path'] = os.path.abspath(
            os.path.join(self._local_output_dir, '%d.json' % i))
      else:
        # TODO: It's weird having this one case in which we use the
        # gs://makani/ prefix for a Cloud Storage path.  It's probably better to
        # include it universally - knowing whether a path is meant to be remote
        # or local is useful debugging information.
        config['output_file_path'] = ('gs://makani/%s/%d.json' %
                                      (self._gcs_output_dir, i))

      config['h5_keep_sparse_log_only'] = (
          FLAGS.keep_sparse_h5_logs and not FLAGS.keep_h5_logs)
      config['h5_log_file_path'] = ''
      if FLAGS.keep_h5_logs or FLAGS.keep_sparse_h5_logs:
        if FLAGS.use_local_worker:
          if FLAGS.local_h5_logs_dir:
            config['h5_log_file_path'] = os.path.join(
                FLAGS.local_h5_logs_dir, '%d.h5' % i)
          else:
            raise BatchSimClientError('local_h5_logs_dir is unset.')
        else:
          config['h5_log_file_path'] = gcloud_util.GcsPath(
              'gs://makani/', self._gcs_h5_log_dir, '%d.h5' % i)

      self._packager.AddString(
          json.dumps(config, indent=2, separators=(',', ': ')),
          'gce_config/%d.json' % i)
      num_configs += 1

    self._packager.ClosePackage()
    return num_configs

  def _WaitForWorkerOutput(self, total_expected):
    """Waits for all worker outputs to be complete.

    Also watches self._gcs_error_dir for log files that indicate worker
    failures.  If an error log is found, the client terminates early.

    Args:
      total_expected: Number of expected outputs.
    """
    expected_files = {'%d.json' % i for i in xrange(total_expected)}

    while expected_files:
      error_log_files = self._gstorage.List(self._gcs_error_dir)
      if error_log_files:
        self._worker_error = True
        logging.error(
            'At least one worker failed.  See logs at:\n  %s',
            '  \n'.join(['gs://makani/' + f for f in error_log_files]))
        return

      remote_files = self._gstorage.List(self._gcs_output_dir)
      for remote_path in remote_files:
        basename = re.sub('^' + self._gcs_output_dir + '/', '', remote_path)
        if basename in expected_files:
          local_path = os.path.join(self._local_output_dir, basename)
          with io.FileIO(local_path, 'wb') as f:
            # GCS has an annoying habit of (we think) telling us a file exists
            # during the List command, then saying it doesn't exist when we go
            # to download it.
            #
            # TODO: Get hard evidence that GCS is mistaken and file
            # a bug.
            try:
              self._gstorage.DownloadFile(remote_path, f)
            except gapi_errors.HttpError as e:
              if e.resp['status'] == '404':
                logging.error('Received 404 while attempting to download %s '
                              'from GCS. Assuming that GCS lied about its '
                              'presence.', remote_path)
                continue
              raise
          expected_files.remove(basename)

      if expected_files:
        logging.info('%d of %d output files complete.',
                     total_expected - len(expected_files), total_expected)
        time.sleep(15.0)
      else:
        logging.info('All output files complete.')

  def _UploadFile(self, local_path, remote_path):
    self._gstorage.UploadFile(local_path,
                              self._gcs_package_dir + '/' + remote_path)

  def Run(self):
    """Runs the client from start to finish."""
    def OutputCmpKey(filename):
      """For filename X.json, uses int(X) as a sorting key."""
      basename = os.path.basename(filename)
      parts = basename.split('.')
      assert len(parts) == 2 and parts[1] == 'json'
      return int(basename.split('.')[0])

    if FLAGS.delete_old_workers:
      self.DeleteOldWorkers()
      return

    if FLAGS.reduce_only:
      assert self._local_output_dir, 'No local_output_dir was specified.'
      assert os.path.exists(self._local_output_dir), (
          'local_output_dir (%s) does not exist.' % self._local_output_dir)

      outputs = set([f for f in os.listdir(self._local_output_dir)
                     if re.match(r'\d+\.json', f)])
      if outputs:
        assert '%d.json' % (len(outputs) - 1) in outputs, (
            'Some output files are missing.')

      output_paths = sorted(
          [os.path.join(self._local_output_dir, f) for f in outputs],
          key=OutputCmpKey)

      self._ReduceWorkerOutput(output_paths)
      return

    with os_util.TempDir(self._sim_name) as working_dir:
      if not self._local_output_dir:
        self._local_output_dir = os.path.join(working_dir, 'worker_output')

      if not os.path.exists(self._local_output_dir):
        os.makedirs(self._local_output_dir)

      worker_package_path = os.path.join(
          makani.HOME, self._BASE_PATH + '_worker_package.par')
      if FLAGS.use_local_worker:
        shutil.copy(worker_package_path,
                    os.path.join(working_dir, 'worker.par'))
      elif FLAGS.upload_worker_package:
        self._UploadFile(worker_package_path, 'worker.par')

      logging.info('Making config package.')
      config_package_path = os.path.join(working_dir, 'configs.tar.gz')
      num_configs = self._MakeConfigPackage(config_package_path)
      if not FLAGS.use_local_worker:
        self._UploadFile(config_package_path, 'configs.tar.gz')

      if FLAGS.use_local_worker:
        self._RunLocalWorker(num_configs, working_dir)
        worker_names = []
      else:
        worker_names = self._RunRemoteWorkers(num_configs)

      if not self._worker_error:
        expected_outputs = ['%d.json' % i for i in xrange(num_configs)]
        actual_outputs = set([f for f in os.listdir(self._local_output_dir)
                              if re.match(r'\d+\.json', f)])
        for e in expected_outputs:
          assert e in actual_outputs, (
              'Expected file %s is not in the output directory: %s.'
              % (e, self._local_output_dir))

        output_paths = [os.path.join(self._local_output_dir, f)
                        for f in expected_outputs]
        self._ReduceWorkerOutput(output_paths)

      self._DeleteWorkers(worker_names)

  def _RunRemoteWorkers(self, num_configs):
    """Runs remote workers.

    Args:
      num_configs: Number of config files.

    Returns:
      worker_names: List of names of still-running worker instances.
    """

    # Clear the output, error, and h5 log archive directories.
    self._gstorage.DeletePrefix(self._gcs_output_dir)
    self._gstorage.DeletePrefix(self._gcs_error_dir)
    self._gstorage.DeletePrefix(self._gcs_h5_log_dir)

    active_workers, failed_workers = self._StartWorkers(num_configs)
    if failed_workers:
      logging.error('Worker creation unsuccessful.  Failed to create the '
                    'following instances:\n  %s',
                    '\n  '.join(failed_workers))
      self._worker_error = True
      return [], active_workers

    logging.info('All workers started.')
    self._WaitForWorkerOutput(num_configs)

    return active_workers

  def _DeleteWorkers(self, worker_names):
    """Deletes worker instances, waiting for completion.

    Args:
      worker_names: Names of the worker instances on Compute Engine.
    """
    if self._worker_error and not FLAGS.delete_workers_on_error:
      return

    logging.info('Deleting worker instances.')
    responses = []
    for worker_name in worker_names:
      try:
        response = self._gcompute.DeleteInstance(worker_name, zone=self._zone)
        responses.append(response)
      except gapi_errors.HttpError as e:
        if e.resp.status == 404:
          logging.info('Worker "%s" does not exist. Assuming it completed long '
                       'enough ago that it self-deleted.', worker_name)

    for response in responses:
      self._gcompute.WaitForCompletion(response)
    logging.info('All workers deleted.')

  def DeleteOldWorkers(self):
    instances = [i for i in self._gcompute.ListInstances(zone=self._zone)
                 if re.match(self._worker_name_prefix + r'\d+', i)]
    self._DeleteWorkers(instances)

  def _GetWorkerMetadata(self, config_range):
    """Generates the 'metadata' field for a worker's GCE instance definition.

    Args:
      config_range: Pair of integers indicating the inclusive range of configs
          to be processed.

    Returns:
      List of {'key': ..., 'value': ...} pairs with which to specify metadata
          to GCE.
    """

    config_package_path = 'gs://makani/%s/configs.tar.gz' % (
        self._gcs_package_dir)
    worker_package_path = 'gs://makani/%s/worker.par' % (self._gcs_package_dir)

    items = [{
        'key': 'startup-script',
        'value': open(self._WORKER_STARTUP_SCRIPT, 'r').read()
    }, {
        'key': 'apt_dependencies',
        'value': ' '.join(self._apt_dependencies)
    }, {
        'key': 'config_package_path',
        'value': config_package_path
    }, {
        'key': 'worker_package_path',
        'value': worker_package_path
    }, {
        'key': 'error_log_dir',
        'value': 'gs://makani/' + self._gcs_error_dir
    }, {
        'key': 'exec_cmd',
        'value': ' '.join(self._GetWorkerArgs(config_range)),
    }]
    return {'items': items}

  def _StartWorkers(self, num_configs):
    """Starts worker instances on GCE.

    Args:
      num_configs: Number of configuration files, used to set worker args that
        control sharding.

    Returns:
      A list of the name of all started workers.
    """

    if FLAGS.max_jobs_per_worker is not None:
      quotient, remainder = divmod(num_configs, FLAGS.max_jobs_per_worker)
      self._num_workers = quotient + (1 if remainder > 0 else 0)

    # Check whether there are more workers than configurations.
    if self._num_workers > num_configs:
      logging.info('Only using %d of %d requested workers.',
                   num_configs, self._num_workers)
      self._num_workers = num_configs

    # Partition the configs across workers.
    base_num_configs = num_configs / self._num_workers
    remainder = num_configs % self._num_workers
    config_ranges = []

    for worker_index in xrange(self._num_workers):
      num_configs = base_num_configs
      if remainder:
        num_configs += 1
        remainder -= 1
      first = 0 if not config_ranges else config_ranges[-1][1] + 1
      config_ranges.append((first, first + num_configs - 1))

    # Create workers and accumulate initial responses, which will be used to
    # wait for completion.
    creation_responses_by_name = {}

    if not FLAGS.worker_image:
      worker_image = 'batch-sim-worker-stretch-20190905'
    else:
      worker_image = FLAGS.worker_image

    for worker_index in range(self._num_workers):
      config_range = config_ranges[worker_index]
      worker_name = self._worker_name_prefix + str(worker_index)
      logging.info('Worker %d will process config range %s.',
                   worker_index, config_range)

      # TODO: Check whether any worker names are in use
      # before issuing creation requests.
      creation_responses_by_name[worker_name] = self._gcompute.CreateInstance(
          worker_name,
          worker_image,
          image_project=FLAGS.worker_image_project,
          machine_type=FLAGS.worker_machine_type,
          metadata=self._GetWorkerMetadata(config_range),
          zone=self._zone)

    logging.info('Waiting for worker creation requests to complete.')
    active_workers = []
    failed_workers = []
    for name, creation_response in creation_responses_by_name.iteritems():
      done_response = self._gcompute.WaitForCompletion(creation_response)
      if 'error' in done_response:
        logging.error('Creation of worker %s failed with error:\n%s',
                      name, json.dumps(done_response['error'], indent=2))
        failed_workers.append(name)
      else:
        active_workers.append(name)

    return active_workers, failed_workers

  def _RunLocalWorker(self, num_configs, worker_dir):
    """Runs a local worker.

    Args:
      num_configs: Number of config files.
      worker_dir: Directory in which the worker should be run.  Must already
          contain the repo, binary, and config packages.
    """

    with os_util.ChangeDir(worker_dir):
      with tarfile.open('configs.tar.gz', 'r:gz') as f:
        f.extractall()

      worker_env = os.environ.copy()
      worker_env['MAKANI_HOME'] = worker_dir
      args = self._GetWorkerArgs((0, num_configs - 1))

      logging.info('Running worker with command:\n  %s', ' '.join(args))

      # The worker will automagically save the output to the right location.
      shell_interfaces.Executor().CheckRun(args, env=worker_env)


def InitMain(argv):
  """Helper for initializing a client's main()."""
  gcloud_util.InitializeFlagsWithOAuth2(argv)
  batch_sim_util.SetUpLogging(FLAGS.quiet)


def JsonReducer(reducer):
  """Decorator for a reducer that operates on JSON objects.

  Apply this to the _ReduceWorkerOutput method of a BatchSimClient
  child class to pre-parse worker ouputs as JSON strings.  The
  elements of the `outputs` arg will then be dictionaries.

  Args:
    reducer: Method to wrap.

  Returns:
    Wrapped reducer function.
  """

  @functools.wraps(reducer)
  def _Wrapper(client, output_paths):
    json_outputs = []
    for path in output_paths:
      with open(path, 'r') as f:
        json_outputs.append(json.load(f))
    return reducer(client, json_outputs)

  return _Wrapper
