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

"""Utilities to identify environment information."""

import os
import subprocess
import sys

import git_util

_MAKANI_REPOSITORY = 'https://makani-private.googlesource.com/makani'


def ResolveUsername(skip_prompts, exception_class):
  """Grab username from system/git or ask user to input it."""
  g_util = git_util.GitUtil(_MAKANI_REPOSITORY)

  username = g_util.GetUserEmail()
  if username:
    return username.split('@')[0]

  print 'Your Google LDAP could not be found from your git configuration.'

  if skip_prompts:
    print 'Use the flag --username or -u to pass your username.'
    sys.exit(1)

  print ('You can input it now, or you can press Ctrl+C to exit and pass the '
         'username using the flag --username or -u.')
  username = raw_input('Username (or Ctrl+C to exit): ')
  if not username:
    raise exception_class('Error: username cannot be empty.')

  return username.split('@')[0]


def _SystemCode():
  p = subprocess.Popen(['lsb_release', '-c'],
                       stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE)
  output, _ = p.communicate()
  return output.split()[1]


# TODO: delete
def IsStretch():
  return _SystemCode() == 'stretch'


# TODO: delete
def IsGcertValid():
  """Checks if there is a gcert valid for at least 30 minutes."""
  return False


# TODO: delete
def IsInGoogleComputeEngine():
  """Checks if the system is running in Google Compute Engine."""
  return bool(subprocess.check_output(
      ['dig', '+short', 'metadata.google.internal']).strip())
