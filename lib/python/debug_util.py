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

"""Utilities to handle errors."""

import traceback

import makani


def FormatTraceback():
  """Get a traceback string."""
  # Print a traceback with only the last call.
  lines = []
  for line in reversed(traceback.format_exc().splitlines()):
    lines.insert(0, line)
    if line.strip().startswith('File "'):
      break
  if lines:
    lines[0] = lines[0].replace(makani.HOME + '/', '')
  return '\n'.join(lines)
