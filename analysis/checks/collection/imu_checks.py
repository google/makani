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

"""Check the IMU's DIAG_STS field for errors."""

from makani.analysis.checks import base_check


class ImuCheck(base_check.BaseCheckItem):
  """Check the IMU's DIAG_STS field for errors."""

  @base_check.RegisterSpecs
  def __init__(self, for_log, node):
    self._node = node
    super(ImuCheck, self).__init__(for_log, name='Imu Flags (%s)' % node)

  def _RegisterInputs(self):
    return [self._Arg('FlightComputerImu', self._node, 'error')]

  @base_check.SkipIfAnyInputIsNone
  def _Check(self, diag_sts):
    # A set bit in DIAG_STS indicates an error.
    self._CheckForFailure('DIAG_STS', diag_sts, [0])


class ImuChecks(base_check.ListOfChecks):
  """Check all IMUs for DIAG_STS field for errors."""

  def __init__(self, for_log):
    self._items_to_check = [
        ImuCheck(for_log, 'FcA'),
        ImuCheck(for_log, 'FcB'),
        ImuCheck(for_log, 'FcC')]
