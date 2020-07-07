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

"""Utilities for interacting with the operating system."""

import logging
import os
import shutil
import tempfile

import gflags

gflags.DEFINE_boolean('delete_temp_dirs', True,
                      'Indicates whether directories created by TempDir should '
                      'be automatically deleted.')

gflags.DEFINE_boolean('delete_temp_dirs_on_error', True,
                      'Indicates whether directories created by TempDir should '
                      'be deleted if an exception is encountered.')

FLAGS = gflags.FLAGS


class OsUtilError(Exception):
  pass


class ChangeDir(object):
  """Context manager with which to temporarily enter another directory."""

  def __init__(self, path):
    self._target_dir = path

  def __enter__(self):
    self._original_dir = os.getcwd()
    os.chdir(self._target_dir)

  def __exit__(self, exc_type, exc_value, traceback):
    os.chdir(self._original_dir)


class TempDir(object):
  """Context manager that creates a temporary directory that is removed on exit.

  Directories are created within the base directory <makani.HOME>/tmp.  The base
  directory is created if it does not exist, and it is never removed.

  All intermediate directories between the base directory and that specified by
  the constructor's `path` arg must already exist when the TempDir is created.

  A TempDir may not use a directory containing or equal to one in use by an
  pre-existing TempDir.

  The flags --delete_temp_dirs and --delete_temp_dirs_on_error may be used to
  adjust TempDir's behavior for debugging purposes.  A likely scenario would be
  to run a script with --nodelete_temp_dirs_on_error so that state at the point
  of error may be inspected.  Once the problem has been fixed, running the
  script again will re-use and then remove any fixed-name directories that were
  left by the previous run.
  """

  _paths_in_use = set()

  def __init__(self, path=None):
    """Create a TempDir.

    Args:
      path: Path of the temporary directory relative to <makani.HOME>/tmp.  If
          None, then a path is chosen via tempfile.mkdir.

    Raises:
      OsUtilError: If `path` is specified as an absolute path.
    """
    self._base_dir = os.getcwd()
    self._MakeDir(self._base_dir)

    if path is not None and os.path.abspath(path) == path:
      raise OsUtilError('TempDir does not accept absolute paths.  (%s was '
                        'provided.)' % path)
    self._path_arg = path

  def __enter__(self):
    """Creates the temporary directory.

    Returns:
      Absolute path to the directory.

    Raises:
      OsUtilError: The requested directory is in use by another TempDir, or
          intermediate directories do not exist.
    """
    if self._path_arg is None:
      self._path = tempfile.mkdtemp(dir=self._base_dir)
    else:
      self._path = os.path.join(self._base_dir, self._path_arg)

    for p in self._paths_in_use:
      if CommonPrefix((self._path, p)) == self._path:
        raise OsUtilError(
            'Requested temp directory (%s) will delete the directory belonging '
            'to another TempDir (%s) upon removal.' % (self._path, p))

    parent = os.path.dirname(self._path.rstrip(os.path.sep))
    if parent and not os.path.isdir(parent):
      raise OsUtilError('Intermediate directories between %s and %s must '
                        'already exist.' % (self._base_dir, self._path))

    self._MakeDir(self._path)

    return self._path

  def __exit__(self, exception_type, unused_exception_value, unused_traceback):
    """Removes the temporary directory unless flags specify otherwise."""
    self._paths_in_use.remove(self._path)

    if not FLAGS.delete_temp_dirs:
      logging.info('Not deleting temp dir %s because --nodelete_temp_dirs '
                   'was specified.', self._path)
    elif exception_type is not None and not FLAGS.delete_temp_dirs_on_error:
      logging.warning('An exception of type %s was encountered.  Temporary '
                      'directory %s is left for user inspection.',
                      exception_type, self._path)
    else:
      shutil.rmtree(self._path)

  def _MakeDir(self, path):
    """Makes a directory with the specified path if it doesn't exist.

    Intermediate directories are also created.

    Args:
      path: Absolute path of the directory.

    Raises:
      OsUtilError: `path` points to a non-directory object.
    """

    self._paths_in_use.add(path)

    if not os.path.exists(path):
      os.makedirs(path)
    elif not os.path.isdir(path):
      raise OsUtilError('%s exists and is not a directory.' % path)


def CommonPrefix(paths):
  """Returns the common prefix, by directory, of a collection of paths.

  This is a substitute for os.path.commonprefix, which bizarrely operates by
  character. See http://rosettacode.org/wiki/Find_common_directory_path#Python.

  Args:
    paths: An iterable of paths.

  Returns:
    The longest common prefix, by directory, of the paths.
  """
  paths = [os.path.normpath(p) for p in paths]
  components = paths[0].split(os.sep)

  for p in paths[1:]:
    new_components = []
    for c1, c2 in zip(components, p.split(os.sep)):
      if c1 == c2:
        new_components.append(c1)
      else:
        break
    components = new_components

  if components == ['']:
    return '/'
  else:
    return os.sep.join(components)
