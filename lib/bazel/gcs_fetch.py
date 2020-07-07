#!/usr/bin/python
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

"""Fetches a file from Google Cloud Storage with local caching."""
# TODO: Figure out a good way to test this.

import gzip
import os
import shutil
import subprocess
import sys

import gflags
from makani.lib.python.batch_sim import gcloud_util

from google.cloud import storage

gflags.DEFINE_string('gcs_path', None,
                     'Path of the file on Google Cloud Storage.')
gflags.DEFINE_string('package', None,
                     'Name of the package containing the build target.')
gflags.DEFINE_string('target', None,
                     'Name of the build target.')
gflags.DEFINE_string('sha256', None,
                     'SHA-256 checksum of the downloaded file.')
gflags.DEFINE_string('output_path', None,
                     'Path to the output.')
gflags.DEFINE_bool('gunzip', False,
                   'Whether to gunzip the remote file.')

FLAGS = gflags.FLAGS


def GetSha256(filename):
  stdout_parts = subprocess.check_output(['sha256sum', filename]).split()
  assert len(stdout_parts) == 2
  return stdout_parts[0]


def DownloadFileIfNecessary(cached_file, expected_sha256, gcs_path):
  """Downloads the file if it isn't cached with the proper checksum.

  Args:
    cached_file: Path of the locally-cached file.
    expected_sha256: Expected SHA-256 checksum of the file.
    gcs_path: Path of the remote file.

  Raises:
    ValueError: The downloaded file has the wrong checksum.
  """
  if os.path.exists(cached_file):
    if GetSha256(cached_file) == expected_sha256:
      return
    os.remove(cached_file)

  MakeDirsSafe(os.path.dirname(cached_file))

  storage_client = storage.Client.create_anonymous_client()
  bucket_name, source_blob_name = gcloud_util.ParseBucketAndPath(gcs_path)
  bucket = storage_client.bucket(bucket_name)
  blob = bucket.blob(source_blob_name)
  blob.download_to_filename(cached_file)

  actual_sha256 = GetSha256(cached_file)
  if actual_sha256 != expected_sha256:
    raise ValueError('Remote file %s has incorrect sha256:\n'
                     'Actual:   %s\nExpected: %s'
                     % (gcs_path, actual_sha256, expected_sha256))


def MakeDirsSafe(dirname):
  # The try/except accounts for both the possibility that the directory already
  # exists, and the possibility that we're racing with another process to create
  # the cache directory.
  try:
    os.makedirs(dirname)
  except OSError:
    assert os.path.isdir(dirname)


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  if FLAGS.gunzip:
    assert FLAGS.gcs_path.endswith('.gz'), (
        'Requested gunzip on a file without .gz extension.')

  cache_dir = '/opt/makani/bazel_cache/gcs_fetch'
  MakeDirsSafe(cache_dir)

  # Within the cache directory, the path is the GCS path with "gs://" stripped
  # off.
  cached_file = os.path.join(cache_dir, FLAGS.gcs_path[5:])

  for i in range(5):
    try:
      DownloadFileIfNecessary(cached_file, FLAGS.sha256, FLAGS.gcs_path)
    except ValueError:
      continue  # retry
    else:
      break

  if FLAGS.gunzip:
    with gzip.open(cached_file, 'rb') as source:
      with open(FLAGS.output_path, 'wb') as dest:
        dest.write(source.read())
  else:
    shutil.copyfile(cached_file, FLAGS.output_path)


if __name__ == '__main__':
  gflags.MarkFlagAsRequired('gcs_path')
  gflags.RegisterValidator('gcs_path', lambda p: p.startswith('gs://'))
  gflags.MarkFlagAsRequired('package')
  gflags.MarkFlagAsRequired('target')
  gflags.MarkFlagAsRequired('sha256')
  gflags.MarkFlagAsRequired('output_path')

  main(sys.argv)
