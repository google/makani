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

"""Makani namespace package providing access to HOME."""

from __future__ import absolute_import
import os

# If in a bazel build, RUNFILES_DIR will be set by bazel.  If we are being
# called by a parent process in the bazel environment, RUNFILES_DIR should first
# be set by calling SetRunfilesDirFromBinaryPath.  Otherwise, we use the
# location of this module.
if os.getenv('RUNFILES_DIR'):
  HOME = os.path.join(os.getenv('RUNFILES_DIR'), 'makani')
else:
  HOME = os.path.dirname(__file__)


def SetRunfilesDirFromBinaryPath():
  """Sets the RUNFILES_DIR environment variable based on this module path.

  This can aid in calling subprocesses which retain the same RUNFILES_DIR
  as this process.
  """

  os.environ['RUNFILES_DIR'] = os.path.normpath(
      os.path.join(os.path.dirname(__file__), '..'))
