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

"""Utilities to iterate through files recursively."""

import os
import re
import tempfile

from makani.lib.python import gsutil
from makani.lib.python.batch_sim import gcloud_util


class _FileIterError(Exception):
  pass


def IterFilesFromCloud(path_prefix, regex_str):
  """Iterate through files in a cloud path recursively.

  The function downloads matching files in the cloud one by one.
  Downloaded files are removed after each iteration.

  Args:
    path_prefix: Path, or prefix, to the cloud storage. E.g., gs://bucket/dir/.
    regex_str: Regular expression to match the file from the beginning.

  Yields:
    The full cloud path to the file, and the path to the downloaded file.
  """

  gs_api = gsutil.GsutilApi()

  filenames = [f for f in gs_api.List(path_prefix)
               if f.endswith('.h5') and not f.endswith('-format.h5')]
  if regex_str:
    regex = re.compile(regex_str)
    filenames = [f for f in filenames
                 if regex.match(gcloud_util.GcsBasename(f))]
  if not filenames:
    print 'Found no files matching the criteria.'
    return

  for full_cloud_path in sorted(filenames):
    temp_fp = tempfile.NamedTemporaryFile(suffix='.h5', delete=False)
    temp_fp.close()
    print '------------------- %s ------------' % full_cloud_path
    print 'Downloading %s...' % full_cloud_path
    gs_api.Copy(full_cloud_path, temp_fp.name, overwrite=True)
    yield (full_cloud_path, temp_fp.name)
    os.remove(temp_fp.name)


def IterFromLocal(log_dir, regex_str):
  """Iterate through files in a local path recursively.

  Args:
    log_dir: The path to the parent directory of log files.
    regex_str: Regular expression to match the file from beginning.

  Yields:
    The full path to the file.
  """

  regex = re.compile(regex_str) if regex_str else None
  filenames = []
  for root, _, files in os.walk(log_dir):
    filenames += [os.path.join(root, f) for f in files if f.endswith('.h5') and
                  (not regex or regex.match(f))]

  if not filenames:
    print 'Found no files matching the criteria.'
    return

  for filename in filenames:
    yield filename
