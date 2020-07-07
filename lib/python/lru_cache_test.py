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

"""Tests for lru_cache."""

import unittest

from makani.lib.python import lru_cache


class LruCacheTest(unittest.TestCase):

  def testNumEntries(self):
    # Populate the cache.
    num_entries = 10
    cache = lru_cache.LruCache(num_entries=num_entries)
    for i in range(num_entries):
      cache[i] = i

    # Check that entries are bumped in the order in which they were added.
    for i in range(num_entries):
      cache[i + num_entries] = i + num_entries
      self.assertNotIn(i, cache)
      self.assertIn(i + 1, cache)

  def testLeastRecentlyUsed(self):
    # Populate the cache.
    num_entries = 10
    cache = lru_cache.LruCache(num_entries=num_entries)
    for i in range(num_entries):
      cache[i] = i

    # Accessing element 0 makes it the least-recently-used, so it should
    # persist after we add num_entries - 1 new entries.
    self.assertEqual(0, cache[0])
    for i in range(num_entries - 1):
      cache[i + num_entries] = i
    self.assertEqual(0, cache[0])

  def testKeyError(self):
    cache = lru_cache.LruCache()
    with self.assertRaises(KeyError):
      cache['foo']  # pylint: disable=pointless-statement


if __name__ == '__main__':
  unittest.main()
