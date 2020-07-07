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

import logging
import os
import re
import sys

import gflags
from makani.lib.log_synchronizer import auto_upload

gflags.DEFINE_string('destination_path', '',
                     'The destination folder where uploaded files will reside.')

gflags.DEFINE_string('source_path', '',
                     'The source path from where to upload data.')

gflags.DEFINE_string('source_pattern', '',
                     'The regular expression to filter files to be uploaded.')

FLAGS = gflags.FLAGS


def _Upload(argv):
  """Run the log synchronizer using the default configuration."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  logging.basicConfig(level=logging.INFO)

  if not (FLAGS.source_path and FLAGS.destination_path
          and os.path.exists(FLAGS.source_path)):
    logging.error('Missing valid source_path or destination_path.')
    return

  source_pattern = FLAGS.source_pattern
  file_regex = re.compile(source_pattern) if source_pattern else None
  dest_pattern = None  # Keep the original names.

  uploaded_files = []
  if os.path.isdir(FLAGS.source_path):
    existing_dest_paths, gcs = auto_upload.PrepareToUpload(
        FLAGS.source_path, FLAGS.destination_path)
    if gcs is None:
      return
    parent_path = os.path.abspath(os.path.join(FLAGS.source_path, '..'))
    directory = os.path.relpath(FLAGS.source_path, parent_path)
    auto_upload.TryUploadDirectory(directory, parent_path,
                                   FLAGS.destination_path, file_regex,
                                   dest_pattern, gcs, FLAGS.preserve_local,
                                   False, uploaded_files)
    if (not FLAGS.preserve_local) and (not file_regex):
      # All files/directories should be uploaded if file_regex is None.
      logging.info('Everything in %s is uploaded. You can remove folder.',
                   FLAGS.source_path)
  elif os.path.isfile(FLAGS.source_path):
    filename = os.path.basename(FLAGS.source_path)
    dirpath = os.path.dirname(FLAGS.source_path)
    existing_dest_paths, gcs = auto_upload.PrepareToUpload(
        dirpath, FLAGS.destination_path)
    if gcs is None:
      return
    rename_template = None
    result = auto_upload.TryUploadFile(filename, dirpath,
                                       FLAGS.destination_path,
                                       existing_dest_paths, file_regex,
                                       rename_template, gcs,
                                       FLAGS.preserve_local, False)
    if result:
      uploaded_files.append(result)
  else:
    logging.error('Cannot upload %s. Must be a file or a directory.',
                  FLAGS.source_path)
    return
  # Report results.
  for local_path, dest_path in uploaded_files:
    logging.info('Uploaded %s to %s', local_path, dest_path)


if __name__ == '__main__':
  gflags.MarkFlagAsRequired('destination_path')
  gflags.MarkFlagAsRequired('source_path')
  gflags.RegisterValidator('source_path',
                           lambda p: os.path.isfile(p) or os.path.isdir(p),
                           '--source_path must point to a file or directory.')

  _Upload(sys.argv)
