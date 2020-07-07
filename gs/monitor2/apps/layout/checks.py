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

"""Utilities to perform common checks."""


def CheckForSize(collection, expected_size, equal_flag, unequal_flag,
                 unexpectedly_empty_flag=None):
  """Check conditions for collection size.

  Args:
    collection: A collection can be a list, set or dictionary.
    expected_size: The expected size.
    equal_flag: The value to return if the collection size equals the expected.
    unequal_flag: The value to return if the collection size differs from
        the expected.
    unexpectedly_empty_flag: The value to return if the collection size is
        unexpectedly empty.

  Returns:
    The flag chosen according to whether the collection size equals the
    expected. It can be one of
    [equal_flag, unequal_flag, unexpectedly_empty_flag].
  """

  if len(collection) == expected_size:
    return equal_flag
  elif collection or unexpectedly_empty_flag is None:
    return unequal_flag
  else:
    return unexpectedly_empty_flag


def CheckForExistence(requested_data, available_data):
  """Determine whether the requested data exists.

  Args:
    requested_data: The specific data being requested.
    available_data: The full set of available data to tell whether it is just
        the requested data that does not exist, or no data exists at all.

  Returns:
    1 if the requested data exists.
    -1 if the requested data does not exit.
    0 if no data exits at all.
  """
  if requested_data is not None:
    return 1  # The requested data exists.
  elif available_data:
    return -1  # The requested data does not exist.
  else:
    return 0  # No data exists at all.


def CheckForCompleteness(expected_collection, available_collection):
  """Check the completeness of a collection.

  Args:
    expected_collection: A dictionary/set/list of expected items.
    available_collection: A dictionary/set/list of items to check.

  Returns:
    availability: A dictionary telling whether an item is available.
  """

  availability = {}
  for item in expected_collection:
    availability[item] = item in available_collection
  return availability


def GetActuatorsWithStatus(status_flags, status_helper, status_to_check):
  """Get a list of actuators with the specified status.

  Args:
    status_flags: A dictionary of status keyed by actuator names.
    status_helper: A c_helpers.EnumHelper class hold enums for statuses.
    status_to_check: A list of status to select, e.g. ['Armed'].

  Returns:
    A list of actuators with status in `status_to_check`.
  """

  flag = 0
  for status in status_to_check:
    flag |= status_helper.Value(status)
  return [key for key, value in status_flags.iteritems() if value & flag]
