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

"""The log synchronizer that automatically checks for and uploads new logs."""

import json
import logging
import os
import re
import string
import sys

import gflags
import makani
from makani.lib.log_synchronizer import auto_upload
from makani.lib.python.batch_sim import gcloud_util

gflags.DEFINE_string('config_path',
                     os.path.join(makani.HOME,
                                  'lib/log_synchronizer/logsync_config.json'),
                     'The configuration file on the cloud or local.')
gflags.DEFINE_list('collections', None,
                   'Collections to synchronize (default is all).')
gflags.DEFINE_list('systems', None,
                   'Systems to synchronize (default is all).')

gflags.DEFINE_boolean('debug', False, 'Produces debugging output.')

FLAGS = gflags.FLAGS


class SynchronizerError(RuntimeError):
  """Error raised to signal failures during synchronizing the logs."""


class Synchronizer(object):
  """The log synchronizer that automatically checks and uploads new logs."""

  def __init__(self, config_file):
    """Initialize the synchronizer.

    Args:
      config_file: The path of the configuration file. This can be a local file
          or a remote file on BigStore.
    """
    self._config = {}
    self._LoadConfig(config_file)

  def _LoadConfig(self, config_file):
    """Parse and load the configuration.

    Args:
      config_file: The path of the configuration file. This can be a local file
          or a remote file on BigStore.

    Raises:
      SynchronizerError: Raised if the configuration failed.
    """
    config_file = config_file.strip()

    with open(config_file, 'r') as fp:
      content = fp.read()
    try:
      self._config = json.loads(content)
    except ValueError, e:
      raise SynchronizerError(
          'Cannot parse JSON content from the configuration file: %s' %
          e.message)
    self._ValidateConfig()

  def _ValidateConfig(self):
    """Validate the configuration.

    Raises:
      SynchronizerError: Raised if the configuration file is invalid.
    """
    config = self._config
    required_fields = ['remote_basedir', 'local_basedir', 'collections']
    for field_name in required_fields:
      if field_name not in config:
        raise SynchronizerError('Config is missing field "%s".', field_name)
    if not config['collections']:
      raise SynchronizerError('Config field "collection" is empty!')

    for collection in config['collections']:
      if 'name' not in collection:
        raise SynchronizerError('Missing "name" attribute in collection.')
      if 'local_path' not in collection:
        raise SynchronizerError('Missing "local_path" attribute in collection.')
      if 'remote_path' not in collection:
        raise SynchronizerError(
            'Missing "remote_path" attribute in collection.')

      # Check sources.
      if 'sources' not in collection:
        raise SynchronizerError('Missing "sources" attribute in collection.')

      for source in collection['sources']:
        # Check host name and local directories.
        if 'src_pattern' not in source and 'src_dir_pattern' not in source:
          raise SynchronizerError('Missing "src_pattern" or "src_dir_pattern" '
                                  'attribute in source.')

  def Upload(self, preserve_local, clean_uploaded):
    """Attempt to upload files according to the configuration.

    Args:
      preserve_local: True if local files should not be removed after
          uploading.
      clean_uploaded: True if local files that are previously uploaded
          should be removed.

    Raises:
      SynchronizerError: If an issue was found with the configuration.

    Returns:
      A list of tuples, each tuple has the form of (local_filename,
          uploaded_filename).
    """
    uploaded_files = []
    for system in self._config['systems']:
      if FLAGS.systems and system not in FLAGS.systems:
        continue
      for collection in self._config['collections']:
        if FLAGS.collections and collection['name'] not in FLAGS.collections:
          continue
        path_string = os.path.join(
            self._config['local_basedir'], system, collection['local_path'])
        local_path_template = string.Template(path_string)
        try:
          local_path = local_path_template.substitute(os.environ)
        except KeyError as e:
          logging.error('Local path %s expects a missing environment '
                        'variable: %s', path_string, e.message)
          continue
        if not os.path.isdir(local_path):
          logging.debug('Skipped nonexistent local directory "%s".', local_path)
          continue
        else:
          logging.info('Uploading local directory "%s" for collection "%s".',
                       local_path, collection['name'])

        for source in collection['sources']:
          # Upload logs from one local directory to the cloud.
          dest_name = source.get('dest_name', None)
          dest_path = gcloud_util.GcsPath(self._config['remote_basedir'],
                                          system,
                                          collection['remote_path'])
          src_dir_pattern = source.get('src_dir_pattern', None)
          src_pattern = source.get('src_pattern', None)

          existing_dest_paths, gs_api = auto_upload.PrepareToUpload(
              local_path, dest_path)

          if src_dir_pattern:
            regex_file = re.compile(src_pattern) if src_pattern else None
            for directory, dirpath in auto_upload.IterDirectories(
                local_path, src_dir_pattern):
              auto_upload.TryUploadDirectory(
                  directory, dirpath, dest_path, regex_file,
                  dest_name, gs_api, preserve_local,
                  True, clean_uploaded, uploaded_files)
          elif src_pattern:
            for filename, dirpath in auto_upload.IterFiles(
                local_path, src_pattern):
              result = auto_upload.TryUploadFile(
                  filename, dirpath, dest_path, existing_dest_paths,
                  dest_name, gs_api, preserve_local, True, clean_uploaded)
              if result:
                uploaded_files.append(result)
          else:
            raise SynchronizerError('A source requires at least a src_pattern '
                                    'or src_dir_pattern.')
    return uploaded_files


def _RunSynchronizer(argv):
  """Run the log synchronizer using the default configuration."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError as e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  logging.basicConfig(level=logging.INFO)

  # Start the synchronizer.
  try:
    synchronizer = Synchronizer(FLAGS.config_path)
  except SynchronizerError as e:
    logging.error(e.message)
    return

  uploaded_files = synchronizer.Upload(FLAGS.preserve_local,
                                       FLAGS.clean_uploaded)
  # Report results.
  for local_path, dest_path in uploaded_files:
    print 'Uploaded %s to %s' % (local_path, dest_path)


if __name__ == '__main__':
  _RunSynchronizer(sys.argv)

