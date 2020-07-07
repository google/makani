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

"""Classes to check for humidity and temperature."""
import json
import os

import jsmin
import makani
from makani.analysis.checks import gradebook
from makani.analysis.checks import gradebook_base_check


class PreflightChecks(gradebook_base_check.GradebookChecks):

  # Needed to identify sources of message types when initializing the Gradebook.
  _NETWORK_YAML_FILE = os.path.join(makani.HOME,
                                    'avionics/network/network.yaml')
  _GRADEBOOK_FILE = os.path.join(
      makani.HOME, 'gs/monitor2/apps/plugins/layouts/gradebook.json')

  def __init__(self, for_log):
    super(PreflightChecks, self).__init__()

    with open(self._GRADEBOOK_FILE, 'r') as fp:
      json_data = fp.read()

    try:
      # Use jsmin to allow comments in the gradebook JSON file.
      json_data = json.loads(jsmin.jsmin(json_data))
    except ValueError:
      raise ValueError('Invalid JSON format in "%s".' % self._GRADEBOOK_FILE)

    self.Initialize(
        gradebook.Gradebook(json_data, self._NETWORK_YAML_FILE),
        for_log, use_full_name=True)
