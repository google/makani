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

"""Check the local folder and upload new files to a remote folder."""

import datetime
import logging
import os
import re
import string
import urllib2

import gflags
import httplib2
from makani.lib.python import gsutil
from makani.lib.python.batch_sim import gcloud_util
import psutil

gflags.DEFINE_integer('max_cpu_percent',
                      60,
                      'Do not upload if the cpu utilization goes above it.')

gflags.DEFINE_integer('max_mem_percent',
                      90,
                      'Do not upload if the memory utilization goes above it.')

gflags.DEFINE_list('exclusive_binaries',
                   ['sim', 'recorder', 'vis'],
                   'Do not upload if any of the listed binaries is running.')

gflags.DEFINE_boolean('preserve_local',
                      True,
                      'True if the local logs should remain after they '
                      'are uploaded.')

gflags.DEFINE_boolean('clean_uploaded',
                      False,
                      'True if a local log should be removed if the scan '
                      'finds it is already uploaded.')

FLAGS = gflags.FLAGS


class BadTimeToUploadError(Exception):
  """Error raised to signal that this is not a good time to upload logs."""


class _UploadFailureError(Exception):
  """Error raised to signal that log upload has failed."""


def IterFiles(local_root, regex_pattern):
  """Check the local folder and iterate through files to be uploaded.

  Local files can exist in sub-directories of local_root. All uploaded files
  reside directly under dest_dir. If a file already exists in dest_dir, it
  will not be uploaded.

  Args:
    local_root: The local directory in which to scan for things to upload.
    regex_pattern: Regular expression used to identify what files to upload.
        It is used to match the relative file path within `local_root`.

  Yields:
    A tuple of (file name, directory path) to the file to be uploaded.
  """

  regex = re.compile(regex_pattern)
  for dirpath, directories, files in os.walk(local_root):
    # Files in upper level directories are uploaded first; we assume files in
    # subdirectories take lower priority / are less interesting.
    # TODO: If it is preferred to treat files/subdirectories in the same
    # ordered stream, then it is better to collect all paths first, then sort
    # and upload them in order. This applies to CheckAndUploadDirectories too.

    # In one directory, files tagged with larger timestamps are uploaded first.
    files.sort(reverse=True)
    for filename in files:
      base_relpath = os.path.join(dirpath, filename)
      rel_path = os.path.relpath(base_relpath, local_root)
      if regex and not regex.match(rel_path):
        continue

      yield filename, dirpath

    # Traverse directories with larger timestamps first.
    directories.sort(reverse=True)


def IterDirectories(local_root, regex_dir_pattern):
  """Iterate matching directories.

  Local directories must have a relative path matching a particular pattern.
  All uploaded directories reside directly under dest_dir. If a directory
  already exists in dest_dir, it will not be uploaded.

  Args:
    local_root: The local directory containing directories to upload.
    regex_dir_pattern: Regular expression to identify what directories to
                       upload.

  Yields:
    A tuple of (directory name, parent directory) of the files to upload.
  """

  regex_dir = re.compile(regex_dir_pattern) if regex_dir_pattern else None
  for dirpath, directories, _ in os.walk(local_root):
    # Upload directories tagged with larger timestamps first.
    for directory in sorted(directories, reverse=True):
      base_relpath = os.path.join(dirpath, directory)
      rel_path = os.path.relpath(base_relpath, local_root)
      if regex_dir and not regex_dir.match(rel_path):
        continue
      directories.remove(directory)
      yield directory, dirpath


def PrepareToUpload(local_root, dest_dir):
  """Check necessary conditions required for uploading.

  Args:
    local_root: The local directory containing directories to upload.
    dest_dir: The remote directory to upload to.

  Returns:
    existing_dest_paths: A set of filenames for existing files in the
        destination directory.
    gs_api: The gsutil.GsutilApi object.

  Raises:
    BadTimeToUploadError: Internet is not available.
    ValueError: local_root is not a valid path.
  """
  if not os.path.isdir(local_root):
    raise ValueError('Cannot find local directory %s.' % local_root)

  if not HasInternet():
    raise BadTimeToUploadError('No internet connection detected.')

  gs_api = gsutil.GsutilApi()
  try:
    existing_dest_paths = set(gs_api.List(dest_dir))
  except httplib2.ServerNotFoundError:
    raise BadTimeToUploadError('Internet has become unavailable.')

  return existing_dest_paths, gs_api


def TryUploadDirectory(directory, parent_relpath, dest_dir, source_file_regex,
                       rename_template, gs_api, preserve_local, check_timing,
                       clean_uploaded, uploaded_files):
  """Attempt to upload a directory.

  Args:
    directory: The name of the directory.
    parent_relpath: The local directory where the directory resides.
    dest_dir: The remote directory to upload to.
    source_file_regex: The precompiled regular expression to test whether a file
        should be uploaded. If None, all files are uploaded.
        The regex is used to match the subpath within `directory`.
    rename_template: The template used to rename the file at the destination.
        If None, the original file name is preserved.
    gs_api: The gsutil.GsutilApi object.
    preserve_local: If True, the source files will remain after uploading.
    check_timing: If True, the upload will begin only if preconditions are met.
    clean_uploaded: True if a local log should be removed if the scan
        finds it is already uploaded.
    uploaded_files: A list of tuples, each has the form of (local_filename,
        uploaded_filename).

  Raises:
    BadTimeToUploadError: Raised if it is not the right time to upload.
  """
  base_relpath = os.path.join(parent_relpath, directory)

  renamed_directory = _RenameFile(directory, rename_template, parent_relpath)
  full_cloud_path = gcloud_util.GcsPath(dest_dir, renamed_directory)
  # Test if there exists any file with such prefix.
  # TODO: Could be made more efficient if there is an "Exist" call.
  is_new_path = not bool(gs_api.List(full_cloud_path))

  # Upload all files (except symbolic links) within the directory.
  # Do not rename any files within the directory.
  rename_template = None
  for sub_directory, sub_directories, files in os.walk(base_relpath):
    rel_path = os.path.relpath(sub_directory, base_relpath)
    if rel_path == '.':
      sub_cloud_directory = full_cloud_path
    else:
      sub_cloud_directory = gcloud_util.GcsPath(full_cloud_path, rel_path)
    if is_new_path:
      existing_dest_paths = set()
    else:
      try:
        existing_dest_paths = set(gs_api.List(sub_cloud_directory))
      except httplib2.ServerNotFoundError:
        # Internet becomes unavailable.
        return
    # Files in upper level directories are uploaded first; we assume files in
    # subdirectories take lower priority / are less interesting.
    # In one directory, files tagged with larger timestamps are uploaded first.
    files.sort(reverse=True)
    for filename in files:
      file_path = os.path.join(sub_directory, filename)
      rel_path = os.path.relpath(file_path, base_relpath)
      if source_file_regex and not source_file_regex.match(rel_path):
        continue

      try:
        result = TryUploadFile(filename, sub_directory, sub_cloud_directory,
                               existing_dest_paths, rename_template, gs_api,
                               preserve_local, check_timing, clean_uploaded)
      except BadTimeToUploadError:
        return
      else:
        if result:
          uploaded_files.append(result)
    # Traverse directories with larger timestamps first.
    sub_directories.sort(reverse=True)


def TryUploadFile(filename, root, dest_dir, existing_dest_paths,
                  rename_template, gs_api, preserve_local, check_timing,
                  clean_uploaded):
  """Attempt to upload a file.

  Args:
    filename: The name of the file.
    root: The local directory where the file resides.
    dest_dir: The remote directory to upload to.
    existing_dest_paths: A set of filenames for existing files in the
        destination directory.
    rename_template: The template used to rename the file at the destination.
        If None, the original file name is preserved.
    gs_api: The gsutil.GsutilApi object.
    preserve_local: If True, the source files will remain in local_root after
        uploading.
    check_timing: If True, the upload will begin only if preconditions are met.
    clean_uploaded: True if a local log should be removed if the scan
        finds it is already uploaded.

  Returns:
    None if upload failed.
    (full_local_path, full_cloud_path) if upload succeeded.
        full_local_path is the full local path of the file to be uploaded.
        full_cloud_path is the destination path to upload the file.

  Raises:
    BadTimeToUploadError: Raised if it is not the right time to upload.
    _UploadFailureError: Raised if the upload is unsuccessful.
  """
  # Check whether the uploading conditions are met.
  is_valid, full_local_path = _CheckIsUploadable(filename, root)
  if not is_valid:
    return None
  full_cloud_path = _GetRemoteFilename(root, filename, dest_dir,
                                       rename_template=rename_template)
  is_already_uploaded = _CheckAndCleanUploaded(
      full_local_path, full_cloud_path, existing_dest_paths,
      gs_api, clean_uploaded=clean_uploaded)

  if is_already_uploaded:
    return None

  # Terminate early if the system is no longer idle.
  if check_timing and not _TimeForUpload():
    logging.error('Bad time to upload files.')
    raise BadTimeToUploadError()

  # Upload the file and remove local file if desired.
  is_uploaded = _UploadFile(full_local_path, full_cloud_path, gs_api)

  if is_uploaded:
    if not preserve_local:
      _DeleteLocalFile(full_local_path)
    existing_dest_paths.add(full_cloud_path)
    return (full_local_path, full_cloud_path)
  elif check_timing and not HasInternet():
    logging.error('Internet is not available.')
    raise BadTimeToUploadError()
  else:
    return None


def HasInternet():
  """Test whether internet is available or not.

  The function is based on the assumption that Google NEVER goes down!

  Returns:
    True if internet is available.
  """
  try:
    urllib2.urlopen('http://gstatic.com/generate_204', timeout=1)
    return True
  except urllib2.URLError:
    return False


def _UploadFile(full_local_path, full_cloud_path, gs_api):
  """Upload the file to the cloud.

  Args:
    full_local_path: The full local path of the file to be uploaded.
    full_cloud_path: The destination path of the uploaded file.
    gs_api: The gsutil.GsutilApi object.

  Returns:
    True if the file has been uploaded successfully.
  """
  logging.info('Uploading %s to %s ...', full_local_path, full_cloud_path)
  try:
    gs_api.Copy(full_local_path, full_cloud_path)
  except httplib2.ServerNotFoundError:
    logging.error('Cannot upload file %s to %s: server not found.',
                  full_local_path, full_cloud_path)
    return False
  else:
    return True


def _GetRemoteFilename(local_dir, filename, dest_dir, rename_template=None):
  """Get name of the remote file to upload to.

  Args:
    local_dir: The local directory where the file resides.
    filename: The name of the file.
    dest_dir: The remote directory to upload to.
    rename_template: The template used to rename the file at the destination
        (default: None (preserve the original filename)).

  Returns:
    full_cloud_path: Full path to the cloud file to upload to.
  """
  renamed_filename = _RenameFile(filename, rename_template, local_dir)
  return gcloud_util.GcsPath(dest_dir, renamed_filename)


def _DeleteLocalFile(full_local_path):
  if os.path.isfile(full_local_path):
    os.remove(full_local_path)


def _CheckAndCleanUploaded(full_local_path, full_cloud_path,
                           existing_dest_paths, gs_api, clean_uploaded=False):
  """Check if file is already uploaded, and remove it if needed.

  Args:
    full_local_path: Full path to the local file to be uploaded.
    full_cloud_path: Full path to the cloud file to upload to.
    existing_dest_paths: A set of filenames for existing files in the
        destination directory.
    gs_api: gsutil.GsutilApi object.
    clean_uploaded: True if a local log should be removed if the scan
        finds it is already uploaded.

  Returns:
    True if the file is already uploaded.
  """

  # Check if the file has already been uploaded.
  if full_cloud_path not in existing_dest_paths:
    return False

  local_file_checksum = gs_api.HashFile(full_local_path)

  try:
    cloud_file_checksum = gs_api.HashFile(full_cloud_path)
  except gsutil.GsutilError:
    logging.warning('Cannot retrieve the checksum of %s from cloud storage. '
                    'Skip uploading %s.',
                    full_cloud_path, full_local_path)
    return True

  if local_file_checksum == cloud_file_checksum:
    # The target file already exists.
    logging.info('%s already exists.', full_cloud_path)
    if clean_uploaded:
      logging.info('Deleting local copy at %s.', full_cloud_path)
      _DeleteLocalFile(full_local_path)
  else:
    # To be conservative, no remote files are deleted due to the checksum
    # error, in case they are meant for different files with the same name.
    logging.warning('%s exists but has an checksum mismatching that of %s.',
                    full_cloud_path, full_local_path)
  return True


def _CheckIsUploadable(filename, local_dir):
  """Check whether the file is uploadable.

  Args:
    filename: The name of the file.
    local_dir: The local directory where the file resides.

  Returns:
    is_valid: True if the file should be uploaded.
    full_local_path: The full local path of the file to be uploaded.
  """
  full_local_path = os.path.join(local_dir, filename)
  if not os.path.exists(full_local_path):
    logging.error('Local file at %s does not exist!', full_local_path)
    return False, full_local_path
  elif os.path.islink(full_local_path):
    logging.info('Skip symlink %s.', full_local_path)
    return False, full_local_path
  elif os.path.isdir(full_local_path):
    logging.info('Skip directory %s.', full_local_path)
    return False, full_local_path
  else:
    return True, full_local_path


def _RenameFile(filename, rename_template, directory_path):
  """Rename a file according to a template.

  Args:
    filename: The source filename.
    rename_template: The template used to rename uploaded files. It replaces
        several variables with components of the source file name. If None,
        the original filename is used. Using my_file.txt as an example,
            '$source_dir' is the name of the source file's folder;
            '$source_base' is the base filename (my_file);
            '$source_ext' is the extension of the source file (txt);
            '$upload_time' is a string representing the uploaded time.
        Note that if the log files are already timestamped, it's preferred to
        preserve the original timestamp without adding $upload_time.
    directory_path: Path to the local directory where the file resides.

  Returns:
    The renamed file.
  """
  if rename_template is None:
    return filename
  source_base, extension = os.path.splitext(filename)
  if extension:
    extension = extension[1:]
  source_dir = os.path.basename(os.path.abspath(directory_path))
  now_time = datetime.datetime.utcnow()
  replace_map = {
      'source_dir': source_dir,
      'source_base': source_base,
      'source_ext': extension,
      'upload_time': unicode(now_time).replace(' ', '_'),
  }
  output_file = string.Template(rename_template).substitute(replace_map)
  return output_file


def _TimeForUpload():
  """Return True if this is a good time to upload files."""
  if _ArePriorityTasksRunning(FLAGS.exclusive_binaries):
    return False

  if not _IsIdle(FLAGS.max_cpu_percent, FLAGS.max_mem_percent):
    return False

  return True


def _IsIdle(max_cpu_percent, max_mem_percent):
  """Return True if the system is considered idle.

  Args:
    max_cpu_percent: The max CPU utilization for the system to be considered
        as idle.
    max_mem_percent: The max memory utilization for the system to be considered
        as idle.

  Returns:
    True if the system can be considered as idle.
  """
  if (psutil.cpu_percent() > max_cpu_percent or
      psutil.virtual_memory().percent > max_mem_percent):
    return False
  return True


def _ArePriorityTasksRunning(exclusive_binaries):
  """Return True if any high priority task is running.

  Args:
    exclusive_binaries: The list of binaries that mark the system as busy
        if running.

  Returns:
    True if the system can be considered idle.
  """
  if exclusive_binaries:
    proc_cmds = {proc.name() for proc in psutil.process_iter()}
    if set(exclusive_binaries) & proc_cmds:
      return True
  return False

