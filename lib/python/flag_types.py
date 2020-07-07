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

"""Custom flag type definitions."""

import gflags
import numpy


def DEFINE_linspace(name, default, help_string,
                    nonempty=False,
                    increasing=False,
                    flag_values=gflags.FLAGS,
                    **kwargs):  # pylint: disable=invalid-name
  """Defines a 'linspace' flag.

  The flag value should be specified as <lower>,<upper>,<count>.  The
  components are used as arguments to numpy.linspace, so they must be
  parsable as float, float, and int, respectively.  The parsed flag
  value will be a 1-dimensional numpy.ndarray.

  Args:
    name: Name of the flag.
    default: Default value (as unparsed string), or None if flag is unset by
        default.
    help_string: Helpful description of the flag.
    nonempty: Indicates whether the flag value is required to be nonempty.  If
        True, None is still an allowable default.  Use gflags.MarkFlagAsRequired
        to disallow None.
    increasing: Indicates whether the flag value should be an increasing array.
        This is only enforced if the parsed value has >=2 elements.
    flag_values: The gflags.FlagValues object in which to define the flag.
    **kwargs: See gflags.DEFINE.
  """
  gflags.DEFINE(_LinspaceParser(), name, default, help_string,
                flag_values=flag_values, **kwargs)

  if nonempty:
    # numpy.array can't be implicitly converted to a boolean.
    # pylint: disable=g-explicit-length-test
    gflags.RegisterValidator(name, lambda v: len(v) > 0,
                             '--%s must specify a nonempty range.' % name,
                             flag_values=flag_values)

  if increasing:
    gflags.RegisterValidator(name, lambda v: len(v) < 2 or v[-1] > v[0],
                             '--%s must specify an increasing range.',
                             flag_values=flag_values)


class _LinspaceParser(gflags.ArgumentParser):
  """Parser for 'linspace' flag type."""

  def Parse(self, argument):
    parts = argument.split(',')
    if len(parts) != 3:
      raise ValueError('Wrong number of components.  Must be of the form '
                       '<lower>,<upper>,<count>', argument)

    try:
      lower, upper, count = float(parts[0]), float(parts[1]), int(parts[2])
    except ValueError:
      raise ValueError('Bad value. Components must be parsable as float, '
                       'float, and int, respectively', argument)

    return numpy.linspace(lower, upper, count)

  def Type(self):
    return numpy.ndarray
