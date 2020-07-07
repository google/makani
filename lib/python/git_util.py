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

"""Utilities for Git manipulations."""

import logging
import os
import subprocess

from makani.lib.python import shell_interfaces


class GitUtil(object):
  """Class to check and retrieve status of git branches and commits."""

  def __init__(self, repo_url):
    self._repo_url = repo_url
    self._executor = shell_interfaces.Executor()

  def _GetShellResults(self, cmd):
    """Get the output from running a command on a shell.

    Args:
      cmd: Command to run.

    Returns:
      A tuple with the stdout and the stderr from the command run.
    """
    cwd = None
    if 'BUILD_WORKSPACE_DIRECTORY' in os.environ:
      # This variable is set when the program is executed by running
      # `bazel run`. Because `bazel run` does not execute the program under
      # $MAKANI_HOME, running `git` commands will fail. Therefore, we set the
      # `cwd` parameter to the root of the bazel workspace.
      cwd = os.environ['BUILD_WORKSPACE_DIRECTORY']
    _, stdout, stderr = self._executor.CheckRun(
        cmd.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        cwd=cwd)
    return stdout, stderr

  def IsMerged(self, branch_name):
    lines = self._GetShellResults('git branch --merged')[0].split('\n')
    branches = [line.split()[-1] for line in lines if line.strip()]
    return branch_name in branches

  def IsReadyToSync(self, allow_untracked=False):
    if (not allow_untracked and
        self._GetShellResults('git status --porcelain')[0]):
      logging.error('Uncommitted files exist: Please commit and push '
                    'your changes to gerrit.')
      return False

    if self.GetRemoteChange(self.GetCurrentCommit()):
      return True
    else:
      logging.error('Please push your commit to gerrit and try again.')
      return False

  def GetCurrentBranch(self):
    """Get name of the current branch."""
    return self._GetShellResults('git rev-parse --abbrev-ref HEAD')[0].strip()

  def GetCurrentCommit(self):
    """Get the most recent commit Id."""
    return self._GetShellResults('git log --pretty=oneline -n 1')[0].split()[0]

  def GetRemoteChange(self, current_commit):
    """Get the remote change reference code. E.g. "36/1236/3"."""
    remote_heads = self._GetShellResults('git ls-remote')[0]
    remote_change = None
    # Scan the lines to find the remote change according to the commit.
    # There may be multiple change Is associated with the same commit.
    # They have equivalent code, and we return whichever comes first.
    for line in remote_heads.split('\n'):
      if current_commit in line:
        change_id = line.split()[1]
        remote_change = change_id
        break
    if remote_change:
      return remote_change
    else:
      return None

  def GetUserEmail(self):
    """Get the user email from the git configuration."""
    username = self._GetShellResults('git config user.email')[0].strip()
    if '@' not in username:  # Handle git error such as git not installed.
      return None
    return username
