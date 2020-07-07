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

"""Utility to import classes."""

import importlib


def ImportClass(path_to_module):
  """Import the module give the path to its file and the class.

  Args:
    path_to_module: A string specifying the location of the module.
        E.g. makani.analysis.my_checks.MyModule.

  Returns:
    The module object.
  """
  class_path = path_to_module.split('.')
  class_name = class_path[-1]
  module_path = '.'.join(class_path[:-1])
  module = importlib.import_module(module_path)
  try:
    cls = getattr(module, class_name)
    return cls
  except AttributeError, e:
    raise AttributeError(('Cannot import "%s" from "%s" because of '
                          'AttributeError: %s.') %
                         (class_name, path_to_module, e.message))
