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

"""LRU (least recently used) cache."""

import collections


class LruCache(object):
  """LRU (least recently used) cache.

  This caches the last num_entries values that were added to it. When an item
  is added that exceeds the size of the cache, the least-recently-used item
  is evicted.
  """

  _DEFAULT_NUM_ENTRIES = 100

  def __init__(self, num_entries=None):
    self._num_entries = (self._DEFAULT_NUM_ENTRIES if num_entries is None
                         else num_entries)
    self._cache = collections.OrderedDict()

  def __getitem__(self, key):
    """Supports dict-style access: value = cache[key]."""
    if key not in self._cache:
      raise KeyError(key)

    # Move the value to the end of the cache.
    value = self._cache.pop(key)
    self._cache[key] = value

    return value

  def __setitem__(self, key, value):
    """Supports dict-style setting: cache[key] = value."""
    if len(self._cache) == self._num_entries:
      self._cache.popitem(last=False)
    self._cache[key] = value

  def __contains__(self, key):
    """Supports key testing: key in cache."""
    return key in self._cache
