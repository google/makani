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

"""Utility functions for string manipulation."""

import re

# Regex to detect upper cases followed by lower cases, e.g, Apple1D2.
# Numbers should not be used as postfixes, or "AB12" will become "a_b12" instead
# of "ab12".
_camel_by_postfix = re.compile(r'([^_])([A-Z][a-z]+)')
# Regex to detect upper cases followed by lower cases or numbers.
_camel_by_prefix = re.compile(r'([a-z0-9])([A-Z])')
# Regex to test for camel case.
_camel_pattern = re.compile(r'^[0-9]*(?:[A-Z][a-z0-9]*)*$')
# Regex to test for snake case.
_snake_pattern = re.compile(r'^_*(?:[a-z0-9]+_)*[a-z0-9]*$')


def IsCamelCase(s):
  return _camel_pattern.match(s) is not None


def IsSnakeCase(s):
  return _snake_pattern.match(s) is not None


def CamelToSnake(s, validate=True):
  """Converts a string with camel case into snake case.

  Example:
    StackedPowerSys returns stacked_power_sys

  Args:
    s: String or Unicode to convert.
    validate: True if `s` has to be in camel case to start with.

  Returns:
    The given string in snake case.
  """
  assert isinstance(s, (str, unicode))
  if validate:
    assert IsCamelCase(s)

  subbed = _camel_by_postfix.sub(r'\1_\2', s)
  return _camel_by_prefix.sub(r'\1_\2', subbed).lower()


def SnakeToCamel(s, validate=True):
  """Converts a string with snake case into camel case.

  Example:
    stacked_power_sys returns StackedPowerSys

  Args:
    s: String or Unicode to convert.
    validate: True if `s` has to be in snake case to start with.

  Returns:
    The given string in camel case.
  """
  assert isinstance(s, (str, unicode))
  if validate:
    assert IsSnakeCase(s)

  return ''.join(word[0].upper() + word[1:] for word in s.split('_') if word)


def GetPlural(name):
  """Do an English-sensitive pluralization.

  So far this supports only label names, which are all pretty simple.  If you
  need to support other endings, e.g. 'y', just add cases here.  See
  http://www.csse.monash.edu.au/~damian/papers/HTML/Plurals.html for tips.

  Args:
    name: A string containing a noun.
  Returns:
    The string, now plural.
  """
  if any(name.endswith(s) for s in ['ch', 'x', 's']):
    return '%ses' % name
  else:
    return '%ss' % name
