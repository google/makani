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

import copy
import json
import os
import socket
import string
import tempfile
import unittest

from makani.lib.log_synchronizer import auto_upload
from makani.lib.log_synchronizer import synchronizer as logsync
from makani.lib.python import gsutil
from makani.lib.python import gsutil_fakes
from makani.lib.python.batch_sim import gcloud_fakes
from makani.lib.python.batch_sim import gcloud_util
import mock

# The cloud folder that serves all log collections.
CLOUD_BUCKET = 'gs://makani'
CLOUD_ROOT_PATH = 'sandbox/logs'
CLOUD_BASE_DIR = gcloud_util.GcsPath(CLOUD_BUCKET, CLOUD_ROOT_PATH)
# The cloud folder to upload the collection of logs during test.
CLOUD_LOG_PATH = 'cloud/subfolder/'


class PatchCloudFakes(object):
  """Patches to fake cloud storage needed for a log synchronizer unit test.

  Exclusively for use as a context manager.
  """

  def __init__(self, test_dir, right_checksum=True):
    """Creates the Patcher.

    Args:
      test_dir: The local directory containing the testing files.
      right_checksum: If False, the faked GsutilApi returns the
          wrong checksum.
    """

    def _FakeDeleteLocalFile(filename):
      if filename in self._local_fs.files:
        self._local_fs.files.pop(filename)

    self._test_dir = test_dir
    self._local_fs = gcloud_fakes.FakeFilesystem()
    self._cloud_fs = gcloud_fakes.FakeFilesystem()

    local_checksum = 'abc'
    cloud_checksum = local_checksum if right_checksum else 'xyz'

    self._patchers = [
        mock.patch('makani.lib.python.gsutil.GsutilApi',
                   lambda: gsutil_fakes.FakeGsutilApi(  # pylint:disable=g-long-lambda
                       local_checksum=local_checksum,
                       cloud_checksum=cloud_checksum)),
        mock.patch(
            'makani.lib.python.batch_sim.gcloud_fakes.FakeFilesystem.LOCAL',
            self._local_fs),
        mock.patch(
            'makani.lib.python.batch_sim.gcloud_fakes.FakeFilesystem.CLOUD',
            self._cloud_fs),
        mock.patch(
            'makani.lib.log_synchronizer.auto_upload._DeleteLocalFile',
            _FakeDeleteLocalFile),
    ]

  def GetLocalFiles(self):
    return self._local_fs.files

  def SetLocalFiles(self, local_files):
    self._local_fs.files = local_files

  def GetCloudFiles(self):
    return self._cloud_fs.files

  def SetCloudFiles(self, cloud_files):
    self._cloud_fs.files = cloud_files

  def __enter__(self):
    for p in self._patchers:
      p.start()

    test_dir = os.path.join(self._test_dir, 'logs')
    for root, _, files in os.walk(test_dir):
      for f in files:
        filename = os.path.join(root, f)
        gcloud_fakes.FakeFilesystem.LOCAL.Save(filename, '')
    return self

  def __exit__(self, exc_type, exc_value, traceback):
    for p in self._patchers:
      p.stop()


class PatchEnvFakes(object):
  """Patches to fake local environment needed for a log synchronizer unit test.

  Exclusively for use as a context manager.
  """

  def __init__(self, has_internet, is_idle):
    """Creates the Patcher.

    Args:
      has_internet: True if the faked environment has internet.
      is_idle: True if the faked environment is idle.
    """
    self._patchers = [
        mock.patch('makani.lib.log_synchronizer.auto_upload.HasInternet',
                   lambda: has_internet),
        mock.patch('makani.lib.log_synchronizer.auto_upload._IsIdle',
                   lambda cpu_percent, mem_percent: is_idle),
        mock.patch(
            'makani.lib.log_synchronizer.auto_upload._ArePriorityTasksRunning',
            lambda exclusive_binaries: not is_idle),
    ]

  def __enter__(self):
    for p in self._patchers:
      p.start()

  def __exit__(self, exc_type, exc_value, traceback):
    for p in self._patchers:
      p.stop()


class TestSynchronizer(unittest.TestCase):

  def setUp(self):
    # Obtain the absolute path for testdata/.
    cur_path = os.path.dirname(os.path.realpath(__file__))
    self._test_dir = os.path.join(cur_path, 'testdata')

  def _LoadTestConfig(self):
    """Loads the generic cloud configuration file and initializes it."""
    test_config = os.path.join(self._test_dir, 'configs.json')
    with open(test_config) as fp:
      content = fp.read()
    macros = {
        'test_cloud_base_dir': CLOUD_BASE_DIR,
        'test_cloud_log_path': CLOUD_LOG_PATH,
        'test_machine_hostname': socket.gethostname(),
        'test_machine_log_path': os.path.join(self._test_dir, 'logs'),
    }
    content_template = string.Template(content)
    config_string = content_template.safe_substitute(macros)
    config = json.loads(config_string)
    return config

  def _CreateLogSynchronizerFromJson(self, config):
    """Saves a JSON config to a file and creates a log synchronizer with it."""
    with tempfile.NamedTemporaryFile() as temp_fp:
      json.dump(config, temp_fp)
      temp_fp.flush()
      return logsync.Synchronizer(temp_fp.name)

  def testConfigMissingBaseDir(self):
    test_config = self._LoadTestConfig()
    del test_config['local_basedir']
    with self.assertRaises(logsync.SynchronizerError):
      self._CreateLogSynchronizerFromJson(test_config)

    test_config = self._LoadTestConfig()
    del test_config['remote_basedir']
    with self.assertRaises(logsync.SynchronizerError):
      self._CreateLogSynchronizerFromJson(test_config)

  def testConfigMissingCollections(self):
    test_config = self._LoadTestConfig()
    del test_config['collections']
    with self.assertRaises(logsync.SynchronizerError):
      self._CreateLogSynchronizerFromJson(test_config)

  def testInvalidLocalPath(self):
    test_config = self._LoadTestConfig()
    for collection in test_config['collections']:
      collection['local_path'] = '__non_existing_path__'
    synchronizer = self._CreateLogSynchronizerFromJson(test_config)
    with mock.patch(logsync.__name__ + '.logging.warning') as mock_log:
      uploaded = synchronizer.Upload(preserve_local=True, clean_uploaded=False)
      self.assertEqual(uploaded, [])
      self.assertTrue(mock_log.assert_any_call)

  @unittest.skip('Test failing, seems to be accessing real GS bucket.')
  def testUploadGoodTiming(self):
    def AssertRemoteFiles():
      gs_api = gsutil.GsutilApi()
      filenames = gs_api.List(
          gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH))
      prefix = os.path.join(CLOUD_BASE_DIR, CLOUD_LOG_PATH)
      self.assertEqual(set(filenames), {
          os.path.join(prefix, 'w7/logs/logs1-folder/subfolder/dummy.json'),
          os.path.join(prefix,
                       'w7/logs/logs1-folder/another_subfolder/dummy.json'),
          os.path.join(prefix,
                       'w7/logs/logs1-folder/one_more_subfolder/dummy.json'),
          os.path.join(prefix, 'w7/logs/w7-2013.h5'),
          os.path.join(prefix, 'M600A/logs/m600.h5'),
          os.path.join(prefix, 'M600A/logs/dummy.json'),
          os.path.join(prefix, 'w7/logs/logs1-folder/dummy.json'),
          os.path.join(prefix, 'w7/logs/logs1-folder/dummy.txt'),
      })

    def RemoveRemoteFiles():
      gs_api = gsutil.GsutilApi()
      filenames = gs_api.List(
          gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH))
      for filename in filenames:
        gs_api.DeleteFile(filename)
      self.assertFalse(
          gs_api.List(gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH)))

    test_config = self._LoadTestConfig()
    synchronizer = self._CreateLogSynchronizerFromJson(test_config)
    upload_expected = [
        # 1. Uploading *.h5 files from the first source.
        (os.path.join(self._test_dir, 'logs/M600A/m600.h5'),
         gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH,
                             'M600A/logs/m600.h5')),
        (os.path.join(self._test_dir, 'logs/w7/w7-2013.h5'),
         gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH,
                             'w7/logs/w7-2013.h5')),
        # 2. Uploading the `folder` directory from the second source.
        # The files are uploaded before subdirectories.
        # Files are uploaded in reverse alphabetical order.
        (os.path.join(self._test_dir, 'logs/w7/logs1/folder/dummy.txt'),
         gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH,
                             'w7/logs/logs1-folder/dummy.txt')),
        (os.path.join(self._test_dir, 'logs/w7/logs1/folder/dummy.json'),
         gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH,
                             'w7/logs/logs1-folder/dummy.json')),
        # Subdirectories are uploaded in reverse alphabetical order.
        (os.path.join(self._test_dir,
                      'logs/w7/logs1/folder/subfolder/dummy.json'),
         gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH,
                             'w7/logs/logs1-folder/subfolder/dummy.json')),
        (os.path.join(self._test_dir, 'logs/w7/logs1/folder/one_more_subfolder/'
                      'dummy.json'),
         gcloud_util.GcsPath(
             CLOUD_BASE_DIR, CLOUD_LOG_PATH,
             'w7/logs/logs1-folder/one_more_subfolder/dummy.json')),
        (os.path.join(self._test_dir, 'logs/w7/logs1/folder/another_subfolder/'
                      'dummy.json'),
         gcloud_util.GcsPath(
             CLOUD_BASE_DIR, CLOUD_LOG_PATH,
             'w7/logs/logs1-folder/another_subfolder/dummy.json')),
        # 3. Uploading dummy.json from the third source.
        (os.path.join(self._test_dir, 'logs/M600A/dummy.json'),
         gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH,
                             'M600A/logs/dummy.json')),
    ]

    all_files_to_upload = set(pair[0] for pair in upload_expected)
    with PatchCloudFakes(self._test_dir) as cloud_fake:
      with PatchEnvFakes(has_internet=True, is_idle=True):
        uploaded = synchronizer.Upload(preserve_local=True,
                                       clean_uploaded=False)
        uploaded.sort(key=lambda x: x[0])
        upload_expected.sort(key=lambda x: x[0])
        self.assertEqual(uploaded, upload_expected)
        AssertRemoteFiles()
        # Upload again should result in no updates
        uploaded = synchronizer.Upload(preserve_local=True,
                                       clean_uploaded=False)
        self.assertEqual(uploaded, [])
        AssertRemoteFiles()

        # Make a snapshot of files in the faked file system.
        local_files_copy = copy.copy(cloud_fake.GetLocalFiles())
        # If we remove remote files and retry, they should be uploaded again.
        RemoveRemoteFiles()
        uploaded = synchronizer.Upload(preserve_local=False,
                                       clean_uploaded=False)
        uploaded.sort(key=lambda x: x[0])
        upload_expected.sort(key=lambda x: x[0])
        self.assertEqual(uploaded, upload_expected)
        # However, local files should be deleted if preserve_local is False.
        self.assertFalse(all_files_to_upload & set(cloud_fake.GetLocalFiles()))

        # Now restore the faked file system as if no local files are deleted.
        cloud_fake.SetLocalFiles(local_files_copy)
        # Upload again. No files should be uploaded because they are already
        # in the cloud, but local files should be deleted.
        uploaded = synchronizer.Upload(preserve_local=True,
                                       clean_uploaded=True)
        self.assertEqual(uploaded, [])
        self.assertFalse(all_files_to_upload & set(cloud_fake.GetLocalFiles()))
        # Make a snapshot of files in the faked cloud system.
        cloud_files_copy = copy.copy(cloud_fake.GetCloudFiles())

    with PatchCloudFakes(self._test_dir, right_checksum=False) as cloud_fake:
      with PatchEnvFakes(has_internet=True, is_idle=True):
        # Restore the cloud file system as if files are uploaded already.
        cloud_fake.SetCloudFiles(cloud_files_copy)
        AssertRemoteFiles()
        # No upload should be successful, due to the wrong checksum.
        uploaded = synchronizer.Upload(preserve_local=False,
                                       clean_uploaded=True)
        self.assertEqual(uploaded, [])
        AssertRemoteFiles()
        # All files should be preserved locally due to mismatched checksum.
        self.assertEqual(all_files_to_upload & set(cloud_fake.GetLocalFiles()),
                         all_files_to_upload)

  def testUploadBadTiming(self):
    def AssertNoRemoteFiles():
      gs_api = gsutil.GsutilApi()
      filenames = gs_api.List(
          gcloud_util.GcsPath(CLOUD_BASE_DIR, CLOUD_LOG_PATH))
      self.assertEqual(filenames, [])

    test_config = self._LoadTestConfig()
    synchronizer = self._CreateLogSynchronizerFromJson(test_config)
    with PatchCloudFakes(self._test_dir):
      with PatchEnvFakes(has_internet=False, is_idle=False):
        with self.assertRaises(auto_upload.BadTimeToUploadError):
          synchronizer.Upload(preserve_local=True, clean_uploaded=False)
        AssertNoRemoteFiles()

      with PatchEnvFakes(has_internet=True, is_idle=False):
        with self.assertRaises(auto_upload.BadTimeToUploadError):
          synchronizer.Upload(preserve_local=True, clean_uploaded=False)
        AssertNoRemoteFiles()

      with PatchEnvFakes(has_internet=False, is_idle=True):
        with self.assertRaises(auto_upload.BadTimeToUploadError):
          synchronizer.Upload(preserve_local=True, clean_uploaded=False)
        AssertNoRemoteFiles()


if __name__ == '__main__':
  unittest.main()
