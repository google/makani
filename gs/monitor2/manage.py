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

"""The Django entry point to start and manage the monitor server."""

import importlib
import os
import sys


def _Run(argv):
  """Main function to start the server operations."""

  os.environ.setdefault('DJANGO_SETTINGS_MODULE',
                        'makani.gs.monitor2.project.settings')

  management = importlib.import_module('django.core.management')

  management.execute_from_command_line(argv)


if __name__ == '__main__':
  _Run(sys.argv)
