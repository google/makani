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

"""Utilities to access and check avionics hardware configurations."""
from makani.analysis.checks import check_range


class VoltageRange(check_range.RangeChecker):
  """A range to check for equality."""

  # Default acceptable voltage ranges.
  _RANGE_INDEX = {
      '1v2': [1.140, 1.260],
      '2v5': [2.375, 2.625],
      '3v3': [3.135, 3.465],
      '5v': [4.750, 5.250],
      '6v': [5.700, 6.300],
      '12v': [11.400, 12.600],
      'IServo': [0.000, 10.000],
      'LvA': [55.000, 75.000],
      'LvB': [55.000, 75.000],
      'VServo': [50.000, 85.000],
      'VAux': [10.000, 14.000],
      'VIn': [10.000, 14.000],
  }

  def __init__(self, voltage_name):
    ranges = [
        b for n, b in self._RANGE_INDEX.iteritems()
        if voltage_name.startswith(n)]
    if not ranges:
      raise KeyError('Voltage name "%s" is not recognized.' % voltage_name)
    super(VoltageRange, self).__init__(ranges)


def CheckWarning(status, mask):
  if isinstance(status, dict):
    warning = status['warning']
  else:
    warning = status.warning
  return warning & mask == mask


def CheckError(status, mask):
  if isinstance(status, dict):
    error = status['error']
  else:
    error = status.error
  return error & mask == mask


def TestMask(populated, index):
  return bool(populated & (1 << index))


def IsDevicePopulated(populated, device_id):
  """Determine whether a device exists according to the `populated` bit mask."""
  return TestMask(populated, device_id)
