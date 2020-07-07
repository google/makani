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

"""Utilities for the layout packages."""


def ExpectDict(obj):
  """Return a dictionary or raises an exception."""
  if isinstance(obj, dict):
    return obj
  elif obj is None:
    return {}
  else:
    raise ValueError('Expecting a dictionary.')


def _CommonPrefix(strings):
  """Return the longest prefix of all list elements."""
  if not strings: return ''
  s1 = min(strings)
  s2 = max(strings)
  for i, c in enumerate(s1):
    if c != s2[i]:
      return s1[:i]
  return s1


def ShortKeyDict(resp, index, prefix=None):
  """Return a dictionary where keys have no common prefix."""
  return _StripDictPrefix(ExpectDict(resp[index]), prefix)


def StripPrefix(data, prefix=None):
  """Removes the prefixes in dictionary keys or a list of strings.

  Args:
    data: A dict or list of keys.
    prefix: The prefix to strip. If None, it will be auto-detected.

  Returns:
    The dict or list where prefix is removed from keys.

  Raises:
    ValueError: If `data` is neither a dict nor list.
  """

  if isinstance(data, dict):
    return _StripDictPrefix(data, prefix)
  elif isinstance(data, list):
    return _StripListPrefix(data, prefix)
  else:
    raise ValueError('Expecting a dict or list.')


def _StripListPrefix(keys, prefix=None):
  """Return a list of strings where the strings have no common prefix.

  Args:
    keys: A list of keys.
    prefix: The prefix to strip. If None, it will be auto-detected.

  Returns:
    A list of new keys with prefix removed.
  """

  if prefix is None:
    prefix = _CommonPrefix(keys)
  prefix_len = len(prefix)
  return [key[prefix_len:] for key in keys]


def _StripDictPrefix(dict_data, prefix=None):
  """Return a dictionary where keys have no common prefix.

  Args:
    dict_data: A dictionary of data.
    prefix: The prefix to strip. If None, it will be auto-detected.

  Returns:
    A dictionary where the prefix is removed from keys.
  """

  keys = dict_data.keys()
  if prefix is None:
    prefix = _CommonPrefix(keys)
  prefix_len = len(prefix)
  return {key[prefix_len:]: value for key, value in dict_data.iteritems()
          if key.startswith(prefix)}


def GetDistinguishableNames(keys, delimiter, prefixes_to_remove):
  """Reduce keys to a concise and distinguishable form.

  Example:
    GetDistinguishableNames(['Day.NewYork.BigApple', 'Night.NewYork.BigMelon'],
                            '.', ['Big'])
    results in {'Day.NewYork.BigApple': 'Day.Apple',
                'Night.NewYork.BigMelon': 'Night.Melon'}.

    If a key has all parts commonly shared with others, then include
    the last shared part in the names. E.g.,

    GetDistinguishableNames(['Day.NewYork', 'Day.NewYork.BigMelon'],
                            '.', ['Big'])
    results in {'Day.NewYork.BigApple': 'NewYork',
                'Night.NewYork.BigMelon': 'NewYork.Melon'}.

  Args:
    keys: The list of strings, each is delimited by parts.
    delimiter: The delimiter to separate parts of each string.
    prefixes_to_remove: The list of prefix strings to be removed from the parts.

  Returns:
    short_names: A dictionary of shortened keys.
  """

  def RemovePrefix(part, prefixes_to_remove):
    for prefix in prefixes_to_remove:
      if part.startswith(prefix):
        return part[len(prefix):]
    return part

  key_part_lists = [key.split(delimiter) for key in keys]
  shortest_length = min(len(part_list) for part_list in key_part_lists)

  # common_part[i] = True if all parts at position i are the same across keys.
  common_part = [True] * shortest_length
  for part_list in key_part_lists[1:]:
    for i in range(shortest_length):
      if part_list[i] != key_part_lists[0][i]:
        common_part[i] = False

  # The prefix list to add if one of the key happens to be the concatenation of
  # all common parts.
  prefix_list = ([key_part_lists[0][shortest_length - 1]]
                 if all(common_part) else [])

  short_names = {}
  for key, part_list in zip(keys, key_part_lists):
    short_names[key] = delimiter.join(prefix_list + [
        RemovePrefix(part, prefixes_to_remove)
        for n, part in enumerate(part_list)
        if n >= shortest_length or not common_part[n]])

  return short_names


def GetShortNamesFromLabels(labels_helper):
  """Get sorted short names from ctype_helpers."""
  short_names = []
  for _, label in labels_helper:
    if label < 0:
      continue
    short_names.append(labels_helper.ShortName(label))
  short_names.sort()
  return short_names
