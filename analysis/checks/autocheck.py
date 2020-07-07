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

"""Functions to perform log autochecking."""

from makani.analysis.checks import base_check
from makani.analysis.checks import gradebook
from makani.analysis.checks import gradebook_base_check
from makani.analysis.checks import iter_files
from makani.lib.python import lru_cache
from makani.lib.python import struct_tree


def AutoCheck(log_file, check_list):
  """Run checks for a log file.

  Args:
    log_file: The log file to check.
    check_list: A CheckList object with criteria to check.

  Returns:
    A dictionary of checked results. Example:
    results[<message_type>][<aio_node>][<top_attribute>][<sub_field>] = {
      'total': 1000,
      'warnings': {
          'count': 123,
          'range': [48.8, 55.6],
      },
      'errors': {
          'count': 123,
          'range': [78.8, 91.6],
      },
    }
  """

  data = struct_tree.StructTree(log_file, True)
  cache = lru_cache.LruCache(50)
  for check_item in check_list.List():
    # Check every element.
    args = check_item.Populate(data, cache)
    check_item.Check(*args)


def _GetFullListOfChecks(check_list, gradebook_file):
  """Merge the regular checklist with the one defined as a gradebook."""

  full_check_list = base_check.ListOfChecks()
  if gradebook_file:
    book = gradebook.Gradebook(gradebook_file)
    gradebook_list = gradebook_base_check.GradebookChecks()
    gradebook_list.Initialize(book, for_log=True, use_full_name=True)
    full_check_list.Concatenate(gradebook_list)

  if check_list:
    full_check_list.Concatenate(check_list)

  return full_check_list


def RunFromLocal(log_dir, prefix, check_list, gradebook_file, verbose):
  """Autocheck logs in a local directory."""

  full_check_list = _GetFullListOfChecks(check_list, gradebook_file)
  results_by_file = {}
  for filename in iter_files.IterFromLocal(log_dir, prefix):
    if verbose:
      print 'Processing %s .......................' % filename
    AutoCheck(filename, full_check_list)
    if verbose:
      print full_check_list.TextSummary()
    results_by_file[filename] = full_check_list.MergeResults()
  return results_by_file


def RunFromCloud(cloud_path, prefix, check_list, gradebook_file, verbose):
  """Autocheck logs in a cloud directory."""

  results_by_file = {}
  full_check_list = _GetFullListOfChecks(check_list, gradebook_file)
  for filename, temp_name in iter_files.IterFilesFromCloud(cloud_path, prefix):
    if verbose:
      print 'Processing %s ..............................' % filename
    AutoCheck(temp_name, full_check_list)
    if verbose:
      print full_check_list.TextSummary()
    results_by_file[filename] = full_check_list.MergeResults()
  return results_by_file


def MergeResultsFromMultipleFiles(results_by_file, info_levels):
  """Merge multiple results to show occurrences in files and range of values.

  Args:
    results_by_file: A dict of check results indexed by file name.
    info_levels: A list of report levels to merge. E.g., ['warning', 'error']

  Returns:
    A dict of check results in the form of:
    {`check_name`: {`file_name`: [lower_bound, upper_bound]}}
  """

  merged = {}
  for filename, results in results_by_file.iteritems():
    if results:
      for check_name, values in results.iteritems():
        bounds = [float('inf'), -float('inf')]
        tracebacks = set()
        # Example `values`:
        #   {'warning', {'count': 100, 'range': [0, 1]}}
        is_set = False
        for field, details in values.iteritems():
          if field in info_levels:
            if 'range' in details:
              bounds[0] = min(bounds[0], details['range'][0])
              bounds[1] = max(bounds[1], details['range'][1])
              is_set = True
            if 'traceback' in details:
              tracebacks.add(details['traceback'])

        report = {}
        if is_set:
          report['range'] = bounds
        if tracebacks:
          report['traceback'] = list(tracebacks)
        if report:
          if check_name not in merged:
            merged[check_name] = {}
          merged[check_name][filename] = report
  return merged


def GatherResultsFromMultipleFiles(results_by_file):
  """Gather multiple results to organize them by check name and file name.

  Args:
    results_by_file: A dict of check results indexed by file name.

  Returns:
    A dict of check results in the form of:
    {`check_name`: {`file_name`: {
        'warning': {
            'range': [lower_bound, upper_bound]
            'count': number of occurrences that fall into the range,
            'total': total number of data points,
        },
        'error': ...
    }}}
  """

  merged = {}
  for filename, results in results_by_file.iteritems():
    if results:
      for check_name, values in results.iteritems():
        if check_name not in merged:
          merged[check_name] = {filename: values}
        else:
          merged[check_name][filename] = values
  return merged
