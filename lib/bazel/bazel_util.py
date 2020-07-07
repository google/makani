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

"""Utility functions to access Makani workspace paths."""

import os
import subprocess


def GetWorkspaceRoot():
  """Get the root of the Makani workspace.

  The workspace directory is nominally the location of MAKANI_HOME in the base
  environment.  If we use 'bazel run', the BUILD_WORKSPACE_DIRECTORY should
  point to the root of the workspace.  In a build, the workspace directory is
  not visible.  This should only be used in very limited circumstances:
    - Accessing cross-platform bazel outputs (ex. TMS570 binaries from x86)
    - Creating files in the Makani workspace (ex. log files).
  If you need to access other files in the workspace, declare them as deps or
  data dependencies, as appropriate.  If you need to create temporary files,
  use tempfile instead.

  Returns:
      Location of the Makani workspace, either BUILD_WORKSPACE_DIRECTORY or
      MAKANI_HOME.

  Raises:
      RuntimeError: MAKANI_HOME and BUILD_WORKSPACE_DIRECTORY are not defined.
  """
  if os.getenv('BUILD_WORKSPACE_DIRECTORY'):
    return os.getenv('BUILD_WORKSPACE_DIRECTORY')
  elif os.getenv('MAKANI_HOME'):
    return os.getenv('MAKANI_HOME')

  raise RuntimeError('Unable to determine workspace root: '
                     'No environment variable MAKANI_HOME.')


def _GetDirectory(subdir):
  """Return the subdir within MAKANI_HOME.

  Args:
    subdir: The sub-directory to search for within the path defined by
        MAKANI_HOME.
  Returns:
    The complete base path of the sub-directory subdir if it exists and is a
    valid directory or symlink to a directory.
  Raises:
    RuntimeError: The environment variable MAKANI_HOME is not set.
    IOError: subdir is not a directory within the MAKANI_HOME path.
  """
  environ_var = GetWorkspaceRoot()
  subdir_path = os.path.join(environ_var, subdir)
  if os.path.isdir(subdir_path):
    return subdir_path
  else:
    raise IOError('No such directory: \'{}\''.format(subdir_path))


def GetTms570BinDirectory():
  """Return tms570-bin directory in MAKANI_HOME."""
  return _GetDirectory('tms570-bin')


# TODO(b/140448660): Deprecate this and use genquery instead.
def Query(query_expr):
  return subprocess.check_output(
      ['bazel', 'query', '--color', 'no', '--noshow_progress', query_expr],
      cwd=GetWorkspaceRoot()).strip().split()
