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

"""Utilities to manipulate an object as JSON."""

import inspect
import json

import jsonpickle


class JsonObj(object):
  """A base class to ease the conversion to and from JSON."""

  def Dumps(self, unpicklable=True):
    """Create a JSON object as a dictionary from this object.

    Args:
      unpicklable: If False, no class information will be included.

    Returns:
      A JSON object.
    """

    return json.loads(jsonpickle.encode(self, unpicklable))

  def __eq__(self, other):
    return (isinstance(other, self.__class__)
            and self.__dict__ == other.__dict__)

  def __ne__(self, other):
    return not self.__eq__(other)


def Loads(json_dict):
  """Loads a JSON object into this object.

  Initialize the member variables with what's specified in json_data.

  Args:
    json_dict: A JSON object.

  Returns:
    A JsonObj object.
  """
  return jsonpickle.decode(json.dumps(json_dict))


def HasKey(record, key):
  """Return True if an object or JSON has a key/attribute."""
  if isinstance(record, dict):
    return key in record
  elif isinstance(record, JsonObj):
    return hasattr(record, key)
  else:
    raise ValueError('HasKey only applies to dict and JsonObj.')


def GetAttr(record, key):
  """Obtain the value of JSON field, or an object attribute."""
  if isinstance(record, dict):
    return record[key]
  elif isinstance(record, JsonObj):
    return getattr(record, key)
  else:
    raise ValueError('GetAttr only applies to dict and JsonObj.')


def GetAttrs(record, no_builtin=True):
  """Obtain the attributes and values of an object.

  The attributes include only member variables, not functions. If no_builtin
  is False, then builtin member variables is included in the returned data.

  Args:
    record: A dictionary or a JsonObj object.
    no_builtin: True if the returned data should not include builtin variables.

  Returns:
    A list of pairs, with the first pair being the name of the variable and the
    the second being its value.

  Raises:
    ValueError: Raised if record is neither a dict nor a JsonObj.
  """
  if isinstance(record, dict):
    return set(record.keys())
  elif isinstance(record, JsonObj):
    if no_builtin:
      # Not using inspect.isbuiltin because that does not include all builtin
      # members.
      return set(record.__dict__.keys())
    else:
      return {pair[0] for pair in
              inspect.getmembers(record, lambda a: not inspect.isroutine(a))}
  else:
    raise ValueError('GetAttrs only applies to dict and JsonObj.')

