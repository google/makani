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

"""Test utilities for the batch simulator.

This module sets up an elaborate series of fakes to allow end-to-end testing of
batch sim clients and workers.  Only Patcher and FakeBinaries should be needed
for general use.

Usage is as follows:
  1. Define a FakeBinaries class, whose methods define runnable 'binaries', and
     a dictionary that maps binary paths to the FakeBinaries methods.
  2. Create a PatchAllFakes object by supplying the objects from step 1 along
     with the worker class.
  3. Under the context of the PatchAllFakes, instantiate and run the sim client.

For sake of interest, or if you happen to be performing upkeep on this module,
here is roughly the sim client/worker flow using this module's internal fakes:
  1. Fake binaries are "saved" to the local gcloud_fakes.FakeFilesystem.
  2. _FakePackager packages these binaries on the local
     gcloud_fakes.FakeFilesystem.
  3. gcloud_fakes.FakeCloudStorageApi copies packages to the cloud
     gcloud_fakes.FakeFilesystem.
  4. _FakeComputeEngineApi does the following for each intended worker:
     a. Use gcloud_fakes.FakeCloudStorageApi to copy packages to the worker's
        gcloud_fakes.FakeFilesystem.
     b. Unpacks packages.
     c. Instantiates a worker and runs it.  The worker copies its outputs back
        to the cloud's gcloud_fakes.FakeFilesystem.
  5. Control returns to the sim client, which copies outputs from the cloud back
     to its own gcloud_fakes.FakeFilesystem.
  6. Sim client reduces the outputs.

In general, the workers themselves use real files in temporary directories.  We
don't want to go anywhere near a full-featured fake filesystem, and users should
be free to do whatever they want to in their worker subclasses.
"""

import argparse
import io
import json
import os

import gflags
import makani
from makani.lib.python import os_util
from makani.lib.python import test_util
from makani.lib.python.batch_sim import gcloud_fakes
from makani.lib.python.batch_sim import gcloud_util
import mock

FLAGS = gflags.FLAGS


class PatchAllFakes(object):
  """Patches all the fakes needed for a batch simulation unit test.

  Exclusively for use as a context manager.
  """

  def __init__(self, binaries=None, binary_map=None, worker_factory=None,
               client_class=None):
    """Creates the Patcher.

    Args:
      binaries: Instance of a User-defined child of FakeBinaries class, with
          methods added to define fake binary behavior.
      binary_map: Dictionary mapping target names to method names on `binaries`.
      worker_factory: Zero-arg factory function that returns a worker instance.
          If the worker's constructor satisfies this condition, then the class
          itself suffices.  The flags --config_range and --config_dir will be
          set appropriately for each worker.
      client_class: The class of the batch sim client.
    """

    # Create factory functions to patch in place of the real classes.
    executor_factory = lambda: _FakeExecutor(binaries)
    compute_api_factory = lambda: _FakeComputeEngineApi(worker_factory)

    self._binary_map = binary_map
    self._worker_package = client_class._BASE_PATH + '_worker_package.par'  # pylint: disable=protected-access

    self._patchers = [
        mock.patch.multiple('makani.lib.python.shell_interfaces',
                            Packager=_FakePackager,
                            Executor=executor_factory),
        mock.patch.multiple(gcloud_util.__name__,
                            CloudStorageApi=gcloud_fakes.FakeCloudStorageApi,
                            ComputeEngineApi=compute_api_factory),
        mock.patch.multiple(gcloud_fakes.__name__ + '.FakeFilesystem',
                            LOCAL=gcloud_fakes.FakeFilesystem(),
                            CLOUD=gcloud_fakes.FakeFilesystem()),
        mock.patch('time.sleep'),
        mock.patch('os.chmod'),
    ]

  def __enter__(self):
    for p in self._patchers:
      p.start()

    if self._binary_map is not None:
      filesystem = gcloud_fakes.FakeFilesystem.LOCAL

      # Create the worker package on the fake local filesystem.
      packager = _FakePackager()
      packager.OpenPackage(os.path.join(makani.HOME, self._worker_package))
      for path, method in self._binary_map.iteritems():
        full_path = os.path.join(makani.HOME, path)
        filesystem.Save(full_path, _MakeFakeExecutable(method))
        packager.AddFile(full_path, path)
      packager.ClosePackage()

  def __exit__(self, exc_type, exc_value, traceback):
    for p in self._patchers:
      p.stop()


class FakeBinaries(object):
  """Object whose methods represent binaries available to workers."""

  def _ParseArg(self, args, arg_name):
    """Parses a single flag value from `args`.

    This is a helper method for user-supplied "binary" methods.

    Args:
      args: Argument list similar to sys.argv[1:].
      arg_name: Name of the arg to parse.

    Returns:
      Value corresponding to `arg_name`.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(arg_name)
    parsed = parser.parse_known_args(args)[0]
    return getattr(parsed, arg_name.lstrip('-'))

  def _ParsePositionalArg(self, args, arg_number):
    """Parses a positional arg from `args`.

    Args:
      args: Argument list similar to sys.argv[1:].
      arg_number: 0-indexed number of the positional arg.

    Returns:
      Value of the positional arg.
    """
    parser = argparse.ArgumentParser()
    for i in xrange(arg_number + 1):
      parser.add_argument('arg%d' % i)
    parsed = parser.parse_known_args(args)[0]
    return getattr(parsed, 'arg%d' % arg_number)

  def TcpDump(self, args):
    filename = self._ParsePositionalArg(args, 0)
    with open(filename, 'w') as f:
      f.write('Fake tcpdump output file.')
    # Despite being killed with SIGINT, the sim_tcpdump.sh script returns 0.
    return 0


class _FakeError(Exception):
  """Fake error to use for testing purposes."""
  pass


class _FakeExecutor(object):
  """Fake of shell_interfaces.Executor.

  The user should derive from this class and supply additional methods that
  define 'binaries' that may be run.  A 'binary' is simply a string identifying
  such a method.  For example,
     class My_FakeExecutor(testing_fakes._FakeExecutor):

       def RunFoo(self, args):
         print 'foo'
         return 0
  defines a _FakeExecutor that would understand a fake file consisting of the
  string 'RunFoo'.

  The return value of a 'binary' specifies the return code of the corresponding
  fake process.
  """

  class _FakePopen(object):
    """A very incomplete fake of subprocess.Popen.

    It's just here to make the returncode attribute available.
    """

    def __init__(self, method, method_args):
      self.returncode = method(method_args)

    def poll(self):  # pylint: disable=invalid-name
      return self.returncode

    def send_signal(self, signal):  # pylint: disable=invalid-name
      # This is a quick hack.  Signal termination probably needs more thought.
      pass

    def wait(self):  # pylint: disable=invalid-name
      return self.returncode

  def __init__(self, fake_binaries):
    self._filesystem = gcloud_fakes.FakeFilesystem.LOCAL
    self._processes = set()
    self._binaries = fake_binaries

  def Run(self, popen_args, **unused_kwargs):
    executable_path = popen_args[0]
    executable = json.loads(self._filesystem.Load(executable_path))
    method = getattr(self._binaries, executable['method_name'])
    return self._FakePopen(method, popen_args[1:])

  def RunInBackground(self, popen_args):
    fake_popen = self.Run(popen_args)
    self._processes.add(fake_popen)
    return fake_popen

  def CheckRun(self, popen_args):
    return self.Run(popen_args)

  def KillIfRunning(self, fake_popen):
    self._processes.remove(fake_popen)


def _MakeFakeExecutable(method_name):
  return json.dumps({
      'method_name': method_name,
  })


class _FakePackager(object):
  """A fake of shell_interfaces.Packager.

  A 'package' generated by a _FakePackager is a JSON string of the form
      {
          'files': {
              <file-name-1>: <contents-1>
              <file-name-2>: <contents-2>
              ...
           }
      }
  """

  def __init__(self):
    self._name = None
    self._descriptor = None
    self._filesystem = gcloud_fakes.FakeFilesystem.LOCAL

  def CreateRepoPackage(self, package_name):
    # Create a file name, so we can ensure that it appears on the worker.
    # Currently, we don't do anything useful with this file.  It could perhaps
    # store a pickled function that _FakeComputeEngineApi could run, removing
    # the need to specify the worker type.
    self._filesystem.Save(package_name, json.dumps({
        'files': {'worker_script.py': 'UNUSED'}
    }))

  def OpenPackage(self, package_name):
    assert self._name is None and self._descriptor is None
    self._name = package_name
    self._descriptor = {'files': {}}
    self._files = self._descriptor['files']

  def AddFile(self, filename, name_in_package):
    self._files[name_in_package] = self._filesystem.Load(filename)

  def AddString(self, contents, name_in_package):
    self._files[name_in_package] = contents

  def ClosePackage(self):
    assert self._name is not None and self._descriptor is not None
    self._filesystem.Save(self._name, json.dumps(self._descriptor))
    self._name = None
    self._descriptor = None


class _FakeComputeEngineApi(object):
  """A fake of gcloud_util.ComputeEngineApi.

  CreateInstance() will store the metadata for a worker instance and return an
  identifier for the creation request.  When WaitForCompletion() is called with
  the creation request ID, the worker's environment is created by downloading
  files from the fake cloud and 'unzipping' packages.  Then the worker is
  created, and its Run method is called.

  With mock.patch, use _FakeComputeEngineApiBuilder below to establish the
  worker class.
  """
  # TODO: This should perhaps talk to some sort of FakeComputeEngine
  # object that controls the running of instances and can be queried to assert
  # proper cleanup.

  def __init__(self, worker_factory):
    self._worker_factory = worker_factory
    self.metadata_by_instance_name = {}

  def CreateInstance(self,
                     instance_name,
                     image_name,
                     image_project=None,
                     machine_type=None,
                     metadata=None,
                     zone=None):
    # Arg names are determined by the real ComputeEngineApi.
    # pylint: disable=unused-argument
    self.metadata_by_instance_name[instance_name] = metadata
    return {'create': instance_name}

  def DeleteInstance(self, instance_name, zone=None):
    return {'delete': instance_name, 'zone': zone}

  def WaitForCompletion(self, initial_response):
    if 'create' in initial_response:
      self._SetupAndRunWorker(initial_response['create'])
    elif 'delete' in initial_response:
      self.metadata_by_instance_name.pop(initial_response['delete'])
    return {}

  def _SetupAndRunWorker(self, instance_name):
    """Sets up and runs a worker.

    This method fakes a lot of the behavior that is ordinarily performed by the
    worker's startup script.  It would be nice to find a good way to keep the
    two in sync.

    Args:
      instance_name: Name of the worker instance.
    """

    # Convert metadata from {'key': ..., 'value': ...} pairs to a dict.
    metadata = {}
    for item in self.metadata_by_instance_name[instance_name]['items']:
      metadata[item['key']] = item['value']

    with mock.patch(gcloud_fakes.__name__ + '.FakeFilesystem.LOCAL',
                    gcloud_fakes.FakeFilesystem()) as worker_fs:
      worker_fs.Save(os.path.join(makani.HOME, 'lib/scripts/sim_tcpdump.sh'),
                     _MakeFakeExecutable('TcpDump'))
      gstorage = gcloud_fakes.FakeCloudStorageApi()

      # Download and unpack packages.
      for package_path in (metadata['worker_package_path'],
                           metadata['config_package_path']):
        relative_path = package_path.replace('gs://makani/', '')
        stream = io.BytesIO()
        gstorage.DownloadFile(relative_path, stream)

        package = json.loads(stream.getvalue())
        assert 'files' in package
        for filename, contents in package['files'].items():
          worker_fs.Save(filename, contents)

      with test_util.FlagValueSaver(), os_util.TempDir() as config_dir:
        FLAGS.Reset()
        FLAGS(metadata['exec_cmd'].split())

        # We cheat a little bit with the config directory.  FLAGS.config_dir
        # refers to worker_fs, whereas we want the worker itself to load files
        # from a real directory.
        first, last = [int(i) for i in FLAGS.config_range]
        for i in range(first, last + 1):
          content = worker_fs.Load('%s/%d.json' % (FLAGS.config_dir, i))
          open(os.path.join(config_dir, '%d.json' % i), 'w').write(content)
        FLAGS.config_dir = config_dir

        worker = self._worker_factory()
        try:
          worker.Run()
        except _FakeError as e:
          error_dir = metadata['error_log_dir'].replace('gs://makani/', '')
          stream = io.BytesIO(str(e))
          gstorage.UploadStream(stream,
                                '%s/%s.LOG' % (error_dir, instance_name))
