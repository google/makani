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

"""Function to traverse a nested dictionary."""


def TraverseNestedDict(root):
  """Generator that traverses a nested dictionary.

  Args:
    root: The dict to iterate over.

  Returns:
    A generator that yields data for each non-dict entry (as determined by
    absence of the 'iterkeys' method). The data yielded is:
        (<tuple of keys from top level>, value).
  """

  parts = []

  def Visit(node):
    if not hasattr(node, 'iterkeys'):
      yield tuple(parts), node
      return

    for key in node.iterkeys():
      parts.append(key)
      for element in Visit(node[key]):
        yield element
      parts.pop()

  return Visit(root)
