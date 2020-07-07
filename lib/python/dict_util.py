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

"""Utility functions for dictionary manipulation."""

import collections
import copy


def IsNestedField(d, field_list):
  """Examine a dict of dicts, to see if the nested field name exists."""

  for f in field_list:
    try:
      d = d[f]
    except (KeyError, ValueError):
      return False
  return True


def GetByPath(d, field_list):
  """Returns the value of a nested field of a dict of dicts."""
  for i, f in enumerate(field_list):
    try:
      d = d[f]
    except (KeyError, ValueError):
      # Re-raise with a more sensible error message.
      raise KeyError('Can\'t find field %s.' % '.'.join(field_list[:i+1]))
  return d


def SetByPath(d, field_list, value):
  """Sets the value of a nested field of a dict of dicts."""
  GetByPath(d, field_list[:-1])[field_list[-1]] = value


def OrderDict(d):
  """Recursively construct an OrderedDict with alphabetically sorted keys.

  Recursion will occur on dictionary values that are themselves dict or
  list objects.

  Args:
    d: A dictionary.

  Returns:
    A new data structure where the encountered dict objects are replaced
    by collections.OrderedDict structures with keys sorted by sorted().
  """
  if isinstance(d, dict):
    return collections.OrderedDict([
        (k, OrderDict(v)) for k, v in sorted(d.items())
    ])
  elif isinstance(d, list):
    return [OrderDict(v) for v in d]
  else:
    return d


class DictMergeError(Exception):
  pass


def MergeNestedDicts(dict_a, dict_b):
  """Merge two nested dictionaries.

  No actual leaves can be common to the two dictionaries.

  Args:
    dict_a: First dictionary to merge.
    dict_b: Second dictionary to merge.

  Returns:
    A new nested dict combining the inputs.

  Raises:
    DictMergeError: If there is a conflict in combining the dictionaries.
  """
  if not isinstance(dict_a, dict) or not isinstance(dict_b, dict):
    raise DictMergeError('Both arguments must be dictionaries.')
  merged = {}
  all_keys = set(dict_a.keys()) | set(dict_b.keys())
  for key in all_keys:
    if key in dict_a and key in dict_b:
      if (isinstance(dict_a[key], dict)
          and isinstance(dict_b[key], dict)):
        merged[key] = MergeNestedDicts(dict_a[key], dict_b[key])
      else:
        raise DictMergeError('Inputs cannot both set the same value %s.' % key)
    elif key in dict_a:
      merged[key] = copy.deepcopy(dict_a[key])
    else:
      merged[key] = copy.deepcopy(dict_b[key])
  return merged


def UpdateNestedDict(d, u):
  """Returns a new updated nested dictionary."""

  d = copy.deepcopy(d)

  for k, v in u.iteritems():
    if isinstance(v, dict):
      d[k] = UpdateNestedDict(d.get(k, {}), v)
    else:
      d[k] = v
  return d


class UnreadKeysError(Exception):
  pass


class MustConsumeAllDictEntries(dict):
  """Context manager requiring that all keys in a dict be accessed.

  Args:
    d: Dictionary to guard.

  Returns:
    A new dictionary that records which fields are accessed.

  Raises:
    UnreadKeysError: if, upon leaving the protected context, there
    exist keys in the underlying dictionary, that have not been read.
  """

  def __init__(self, d):
    super(MustConsumeAllDictEntries, self).__init__(d)
    self._access_log = set()

  def __getitem__(self, key):
    # Try __getitem__ before setting access log, in case key is not found.
    result = dict.__getitem__(self, key)
    self._access_log.add(key)
    return result

  def GetUnreadKeys(self):
    return set(self.keys()) - self._access_log

  def __enter__(self):
    return self

  def __exit__(self, *args):
    if args[0]:
      # An exception occurred.
      return False
    if self.GetUnreadKeys():
      raise UnreadKeysError(self.GetUnreadKeys())


def GetAllDictPaths(tree_dict):
  """Obtain list of paths to all leaves in dictionary.

  The last item in each list entry is the value at the leaf. For items in
  dictionary that are a list of dictionaries, each list entry is indexed by a
  string repesenting its position in the list.
  Implementation inspired by https://stackoverflow.com/a/40555856.

  Args:
    tree_dict: Input dictionary.

  Returns:
    List of lists with all paths to leaf items in dictionary.

  Example:

  >> a = {'a':[1 , 2, 3],
          'b':[{'c': 10},
               {'d': 20}]
          }

  >> print get_all_dict_paths(a)

    [['a', [1, 2, 3]],
     ['b', '0', 'c', 10],
     ['b', '1', 'd', 20]]
  """
  if isinstance(tree_dict, list):
    if isinstance(tree_dict[0], dict):
      return [[str(i)] + path
              for i, value in enumerate(tree_dict)
              for path in GetAllDictPaths(value)]
    else:
      return [[tree_dict]]
  elif not isinstance(tree_dict, dict):
    return [[tree_dict]]
  return [[key] + path
          for key, value in tree_dict.items()
          for path in GetAllDictPaths(value)]
