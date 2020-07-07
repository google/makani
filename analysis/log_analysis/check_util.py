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

"""Utilities for checks."""

from makani.analysis.checks import base_check
from makani.lib.python import import_util


# TODO: Move this to //analysis/checks/base_check.py
def LoadListOfChecks(path_to_checks):
  """Load the ListOfChecks object given the path to its file and class.

  Args:
    path_to_checks: A string specifying the location of the checks.
        E.g. makani.analysis.my_checks.MyCheck.

  Returns:
    The ListOfChecks object.
  """
  cls = import_util.ImportClass(path_to_checks)
  return cls(for_log=True)


def LoadJsonCheck(path_to_check, parameters_json):
  r"""Load the Check object given the path to its classpath and parameters.

  Args:
    path_to_check: A string specifying the location of the checks.
        E.g. makani.analysis.my_checks.MyCheck
    parameters_json: A JSON serialized string of the parameters needed to
        instantiate the class.
        E.g. "{\"for_log\": true, \"warning_ranges\": {\"ranges\": [0, 180]},
             \"normal_ranges\": {\"ranges\": [80, 150]}}"

  Returns:
    The Check object.
  """
  cls = import_util.ImportClass(path_to_check)
  parameters = base_check.ParseCheckSpecs(parameters_json)
  return cls(**parameters)


def LoadCheck(path_to_check, params):
  """Load the ListOfChecks object given the path to its file and class.

  Args:
    path_to_check: A string specifying the location of the checks.
        E.g. makani.analysis.my_checks.MyCheck.
    params: A string specifying parameters to be passed into the check.

  Returns:
    The CheckItem object.
  """
  cls = import_util.ImportClass(path_to_check)
  return cls(**params)
