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

"""Tests for makani.analysis.aero.avl."""

import os
import unittest

from makani.analysis.aero.avl import avl_reader


class AvlReaderTest(unittest.TestCase):

  # TODO: Add more substantial tests.
  def testParseM600LowTail(self):
    m600_low_tail = avl_reader.AvlReader(
        os.path.join(os.path.dirname(__file__),
                     'm600_low_tail_no_winglets.avl'))
    self.assertAlmostEqual(m600_low_tail.avl['Sref'], 32.9285)
    self.assertAlmostEqual(m600_low_tail.avl['Cref'], 1.2831)
    self.assertAlmostEqual(m600_low_tail.avl['Bref'], 25.6626)


if __name__ == '__main__':
  unittest.main()
