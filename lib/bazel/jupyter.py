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

"""Launch jupyter notebook."""

import os
import re
import sys

from makani.lib.bazel import bazel_util
from notebook.notebookapp import main
from notebook.serverextensions import toggle_serverextension_python

if __name__ == '__main__':
  os.chdir(bazel_util.GetWorkspaceRoot())
  sys.argv[0] = re.sub(r'(-script\.pyw?|\.exe)?$', '', sys.argv[0])
  toggle_serverextension_python('jupyter_http_over_ws', enabled=True)
  sys.exit(main())
