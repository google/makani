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

"""Generates a bash script to garbage-collect old batch sims.

Usage:
  gc_batch_sims.py <output_script>

The generated script runs a sequence of gsutil commands that deletes all batch
sim directories more than DAYS_TO_KEEP days old. The script includes comments
that indicate the last update time (in UTC) to all such directories. It should
be manually sanity-checked before execution.

To preserve a batch sim directory, move it to gs://makani/archived_batch_sims/.
"""

import datetime
import os
import re
import subprocess
import sys

# The intent is to GC after 90 days. Add an extra day of fudge room rather than
# being careful about time zones.
DAYS_TO_KEEP = 91

DATE_REGEX = re.compile(r'20\d\d-\d\d-\d\dT\d\d:\d\d:\d\dZ')


def ProcessDirectory(dirname, output_lines):
  """Processes a directory of batch sim files.

  If directory contents are sufficiently old, appends to outfile comments
  that describe the directory and a command to remove it.

  Args:
    dirname: Full Cloud Storage path of the directory, with trailing slash.
    output_lines: Cumulative list of lines for the GC script.
  """

  contents = subprocess.check_output(['gsutil', 'ls', '-lh', dirname + '*'])

  last_update = None

  for line in contents.splitlines():
    dates = DATE_REGEX.findall(line)
    if dates:
      assert len(dates) == 1
      t_updated = datetime.datetime.strptime(dates[0],
                                             '%Y-%m-%dT%H:%M:%SZ')
      if last_update is None or t_updated > last_update:
        last_update = t_updated

  assert last_update is not None

  if (datetime.datetime.today() - t_updated).days <= DAYS_TO_KEEP:
    return

  output_lines += ['# Directory %s' % dirname,
                   '# Last update: %s' % last_update,
                   'gsutil -m rm -rf %s' % dirname,
                   '']


def main(argv):
  assert len(argv) == 2, '\nUsage:\n    %s <output_script_name>' % argv[0]

  output_script = argv[1]

  directories = subprocess.check_output(
      'gsutil ls gs://makani/batch_sim/'.split()).strip().split()

  output_lines = []
  for dirname in directories:
    ProcessDirectory(dirname, output_lines)

  if not output_lines:
    print ('Scanned %d directories; none to GC. Not creating output script.'
           % len(directories))
    sys.exit(0)

  with open(output_script, 'w') as f:
    f.write('#!/bin/bash -ex\n\n')
    f.write('\n'.join(output_lines))
  os.chmod(output_script, 0770)


if __name__ == '__main__':
  main(sys.argv)
