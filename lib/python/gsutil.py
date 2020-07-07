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

"""Utilities for using Gsutil."""
import os
import subprocess


_GSUTIL_PATH = 'gsutil'


class GsutilError(Exception):
  pass


class GsutilApi(object):
  """The API for managing the cloud storage through gsutil."""

  def _IsCloudPath(self, path):
    return path.startswith('gs://')

  def _GetHashOption(self, checksum_type):
    if checksum_type == 'crc32c':
      return '-c'
    elif checksum_type == 'md5':
      return '-m'
    else:
      assert False

  def _CloudPathExists(self, path):
    # gsutil -q stat <path> returns 0 for existing paths.
    return subprocess.call([_GSUTIL_PATH, '-q', 'stat', path]) == 0

  def _HashLocal(self, local_path, checksum_type):
    try:
      stdout = subprocess.check_output(
          [_GSUTIL_PATH, 'hash', self._GetHashOption(checksum_type),
           local_path])
    except subprocess.CalledProcessError:
      raise GsutilError('Cannot get hash for %s' % local_path)
    return stdout.split()[-1]

  def _HashCloud(self, cloud_path, checksum_type):
    """Get the checksum from the cloud."""
    try:
      stdout = subprocess.check_output([_GSUTIL_PATH, 'stat', cloud_path])
    except subprocess.CalledProcessError:
      raise GsutilError('Cannot get hash for %s' % cloud_path)
    else:
      assert checksum_type in ['md5', 'crc32c']
      checksum = None
      lines = stdout.split('\n')
      for line in lines:
        if line.strip().startswith('Hash (%s):' % checksum_type):
          checksum = line.split()[-1]
          break
      if checksum is None:
        raise GsutilError('Cannot get hash for %s' % cloud_path)
      else:
        return checksum

  def HashFile(self, path, checksum_type='crc32c'):
    """Get checksum of a cloud storage file.

    Note that all files in the cloud storage have crc32c checksums,
    but not all have md5 checksums.

    Args:
      path: Path to the local or cloud file.
      checksum_type: 'crc32c' or 'md5'.

    Returns:
      The checksum string.
    """

    if path.startswith('gs://'):
      return self._HashCloud(path, checksum_type)
    else:
      return self._HashLocal(path, checksum_type)

  def _UploadFile(self, local_path, cloud_path, overwrite=False):
    assert os.path.exists(local_path)
    if self._CloudPathExists(cloud_path) and not overwrite:
      return False

    local_checksum = self._HashLocal(local_path, 'md5')
    try:
      # This makes sure the file does not appear in cloud storage until
      # the uploading process succeeds with the correct checksum.
      subprocess.check_call(
          [_GSUTIL_PATH, '-m', '-h', 'Content-MD5:' + local_checksum,
           'cp', local_path, cloud_path])
    except subprocess.CalledProcessError:
      return False
    else:
      return True

  def _DownloadFile(self, cloud_path, local_path, overwrite=False):
    if os.path.exists(local_path):
      if not overwrite:
        return False

    # gsutil will overwrite the local file if it exists.
    try:
      subprocess.check_call(
          [_GSUTIL_PATH, '-m', 'cp', cloud_path, local_path])
    except subprocess.CalledProcessError:
      return False
    else:
      return True

  def _CopyLocalFiles(self, source_path, dest_path, overwrite=False):
    assert os.path.exists(source_path)
    if os.path.exists(dest_path):
      if not overwrite:
        return False

    # gsutil will overwrite the dest file if it exists.
    try:
      subprocess.check_call(
          [_GSUTIL_PATH, 'cp', source_path, dest_path])
    except subprocess.CalledProcessError:
      return False
    else:
      return True

  def _CopyCloudFiles(self, source_path, dest_path, overwrite=False):
    if self._CloudPathExists(dest_path) and not overwrite:
      return False

    assert self._CloudPathExists(source_path)

    try:
      subprocess.check_call([_GSUTIL_PATH, 'cp', source_path, dest_path])
    except subprocess.CalledProcessError:
      return False
    else:
      return True

  def Copy(self, source_path, dest_path, overwrite=False):
    if self._IsCloudPath(dest_path) and not self._IsCloudPath(source_path):
      self._UploadFile(source_path, dest_path, overwrite=overwrite)
    elif self._IsCloudPath(source_path) and not self._IsCloudPath(dest_path):
      self._DownloadFile(source_path, dest_path, overwrite=overwrite)
    elif self._IsCloudPath(source_path) and self._IsCloudPath(dest_path):
      self._CopyCloudFiles(source_path, dest_path, overwrite=overwrite)
    else:
      self._CopyLocalFiles(source_path, dest_path, overwrite=overwrite)

  def List(self, cloud_dir_or_file):
    """List the full cloud path of files referred to by a cloud directory.

    Invalid paths will return an empty list.

    Args:
      cloud_dir_or_file: The cloud path (gs://...) to a cloud directory or file.

    Returns:
      The list of paths found in the cloud dir or file.
    """

    try:
      stdout = subprocess.check_output([_GSUTIL_PATH, 'ls', cloud_dir_or_file])
    except subprocess.CalledProcessError:
      return []
    else:
      return [f.strip() for f in stdout.split()]

  def Remove(self, cloud_dir_or_file):
    try:
      stdout = subprocess.check_output([_GSUTIL_PATH, 'rm', cloud_dir_or_file])
    except subprocess.CalledProcessError:
      raise GsutilError('Cannot gsutil remove "%s".' % cloud_dir_or_file)
    else:
      return [f.strip() for f in stdout.split()]
