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

import unittest

from makani.lib.python import build_info


class TestBuildinfo(unittest.TestCase):

  # Since the git SHA is volatile, we can only test that it is in the expected
  # format.
  def testGetGitSha(self):
    sha = build_info.GetGitSha()
    self.assertRegexpMatches(sha, r'[0-9a-f]{40}')


if __name__ == '__main__':
  unittest.main()
