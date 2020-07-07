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

"""Fake gcloud utils for testing without cloud access."""

from makani.lib.python.batch_sim import gcloud_util


class FakeFilesystem(object):
  """A fake filesystem.

  A FakeFilesystem instance is simply a dictionary of file names to file
  contents, with Save() and Load() methods to make access look a bit more
  file-like.

  The class itself also contains LOCAL and CLOUD variables intended to store
  references to particular FakeFilesystem instances.  These are initialized to
  None and intended to be defined as needed via mock.patch.  For example:
      with mock.patch('makani.batch_sim.gcloud_fakes.FakeFilesystem.LOCAL',
                      FakeFilesystem()) as local_fs:
        <Do something with local files>
        with mock.patch('makani.batch_sim.gcloud_fakes.FakeFilesystem.CLOUD',
                        FakeFilesystem()) as remote_fs:
          <Do something with remote files>

  In particular, many of the fakes in this module use FakeFilesystem.LOCAL and
  FakeFilesystem.CLOUD to simulate actual storage patterns.
  """

  LOCAL = None
  CLOUD = None

  def __init__(self):
    self.files = {}

  def Save(self, filename, descriptor):
    self.files[filename] = descriptor

  def Load(self, filename):
    return self.files[filename]


class FakeCloudStorageApi(object):
  """A fake of gcloud_util.CloudStorageApi.

  This performs simple transfers between FakeFilesystem.LOCAL and
  FakeFilesystem.CLOUD.

  To simulate working with different local filesystems, FakeFilesystem.LOCAL
  may be patched before instantiating the FakeCloudStorageApi.
  """

  def __init__(self, bucket=None):
    self._local_fs = FakeFilesystem.LOCAL
    self._cloud_fs = FakeFilesystem.CLOUD
    self._bucket = bucket

  def _RemoveBucketFromCloudName(self, cloud_name):
    cloud_name = cloud_name.strip()
    if cloud_name.startswith('gs://'):
      _, cloud_name = gcloud_util.ParseBucketAndPath(cloud_name, None)
    return cloud_name

  def DownloadFile(self, cloud_name, stream):
    cloud_name = self._RemoveBucketFromCloudName(cloud_name)
    stream.write(self._cloud_fs.Load(cloud_name))

  def UploadFile(self, local_name, cloud_name):
    cloud_name = self._RemoveBucketFromCloudName(cloud_name)
    self._cloud_fs.Save(cloud_name, self._local_fs.Load(local_name))

  def UploadStream(self, stream, cloud_name):
    cloud_name = self._RemoveBucketFromCloudName(cloud_name)
    self._cloud_fs.Save(cloud_name, stream.getvalue())

  def DeletePrefix(self, prefix):
    for filename in self.List(prefix):
      if filename.startswith(prefix):
        self._cloud_fs.files.pop(filename)

  def DeleteFile(self, cloud_name):
    cloud_name = self._RemoveBucketFromCloudName(cloud_name)
    self._cloud_fs.files.pop(cloud_name)

  def List(self, prefix):
    prefix = self._RemoveBucketFromCloudName(prefix)
    return [name for name in self._cloud_fs.files if name.startswith(prefix)]

