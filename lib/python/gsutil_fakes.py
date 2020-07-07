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

"""Fake class for gsutil.GsutilApi."""

from makani.lib.python.batch_sim import gcloud_fakes
from makani.lib.python.batch_sim import gcloud_util


class FakeGsutilApi(object):
  """Fake class for gsutil.GsutilApi."""

  def __init__(self, local_checksum='', cloud_checksum='', bucket=None):
    self._local_fs = gcloud_fakes.FakeFilesystem.LOCAL
    self._cloud_fs = gcloud_fakes.FakeFilesystem.CLOUD
    self._local_checksum = local_checksum
    self._cloud_checksum = cloud_checksum
    self._bucket = bucket

  def _IsCloudPath(self, path):
    return path.startswith('gs://')

  def _RemoveBucketFromCloudName(self, cloud_name):
    cloud_name = cloud_name.strip()
    if cloud_name.startswith('gs://'):
      _, cloud_name = gcloud_util.ParseBucketAndPath(cloud_name, None)
    return cloud_name

  def HashFile(self, path):
    return (self._cloud_checksum if path.startswith('gs://')
            else self._local_checksum)

  def Copy(self, source_name, dest_name):
    if self._IsCloudPath(source_name) and not self._IsCloudPath(dest_name):
      self._DownloadFile(source_name, dest_name)
    elif not self._IsCloudPath(source_name) and self._IsCloudPath(dest_name):
      self._UploadFile(source_name, dest_name)

  def _DownloadFile(self, cloud_name, local_name):
    cloud_name = self._RemoveBucketFromCloudName(cloud_name)
    self._local_fs.Save(local_name, self._cloud_fs.Load(cloud_name))

  def _UploadFile(self, local_name, cloud_name):
    cloud_name = self._RemoveBucketFromCloudName(cloud_name)
    self._cloud_fs.Save(cloud_name, self._local_fs.Load(local_name))

  def DeleteFile(self, cloud_name):
    cloud_name = self._RemoveBucketFromCloudName(cloud_name)
    self._cloud_fs.files.pop(cloud_name)

  def List(self, path):
    bucket_name, prefix = gcloud_util.ParseBucketAndPath(path, None)
    return ['gs://%s/%s' % (bucket_name, name)
            for name in self._cloud_fs.files if name.startswith(prefix)]
