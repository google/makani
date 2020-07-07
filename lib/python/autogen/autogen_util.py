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

"""Utilities to handle Python files translated from C by clang2py."""

from __future__ import absolute_import
import re


# The list of system macros to exclude.
# If included, their right-hand-side definitions show up as undefined.
_SYMBOLS_TO_EXCLUDE = ['WCHAR_MIN', 'WCHAR_MAX', '__WCHAR_MAX', '__WCHAR_MIN',
                       '__restrict_arr', 'assert', 'bool', '__ASSERT_FUNCTION',
                       'SUBSYS_DRUM', 'static_assert', 'stdin', 'stdout',
                       'stderr', '__STD_TYPE', 'GSL_VAR', r'_IO_[\.\w]+',
                       r'_IOS_[\.\w]+']

_MACRO_DEFS_TO_SKIP = [
    re.compile(s + r'\s*\=\s*') for s in _SYMBOLS_TO_EXCLUDE]

# Regular expression to find top-level macro definitions.
_MACRO_REGEX = re.compile(
    r'^(?P<lhs>\w+)\s*\=\s*(?P<rhs>[\w\.\-+]+)\s*(?:\#.*)?$')


def FixClang2PyOutput(input_file, output_file):
  """Clean up a clang2py generated python file.

  Removes undefined and unnecessary macros, format numerics values to be
  python-compatible, and expand the macros to avoid out-of-order dependency
  chains of the definitions.

  Args:
    input_file: The python file to clean up.
    output_file: The python file to write to.
  """

  with open(input_file) as fp:
    lines = fp.readlines()

  cleaned_lines = []

  # A dictionary of macros definitions.
  macros = {}
  for ln in lines:
    regm = _MACRO_REGEX.match(ln)
    if regm:
      lhs = regm.group('lhs')
      rhs = regm.group('rhs')
      macros[lhs] = rhs

  for ln in lines:
    excluded = False
    for useless_macro_defs in _MACRO_DEFS_TO_SKIP:
      if useless_macro_defs.match(ln):
        excluded = True
        break
    if excluded:
      continue

    regm = _MACRO_REGEX.match(ln)
    if regm:
      rhs = regm.group('rhs')
      lhs = regm.group('lhs')
      while rhs in macros and rhs != macros[rhs]:
        rhs = macros[rhs]
      if rhs.startswith('__'):
        # Remove assignments whose values begin with '__', which
        # are likely system variables that are not defined in Python.
        continue

      # If the right hand side looks like a float ending with 'f', remove it.
      if re.match(r'[\-+\.]?\s*\d[\d\.e\-+]*f$', rhs):
        rhs = rhs[:-1]
      elif rhs.startswith('0x') and rhs.endswith('UL'):
        rhs = rhs[:-2]
      elif rhs.endswith('L'):
        rhs = rhs[:-1]
      ln = '%s = %s\n' % (lhs, rhs)

    cleaned_lines.append(ln)

  with open(output_file, 'w') as fp:
    fp.write(''.join(cleaned_lines))


def CStructName(t):
  """Obtain the name of the CType struct from Python."""
  # Clang2py translates C structs to python classes with
  # prefixes like 'struct_c__SA_Foo' and then assign
  # types like 'Foo = struct_c__SA_Foo'. This function
  # gets the real C struct name by removing those prefixes.
  struct_name = t.__name__
  if struct_name.startswith('struct_c__SA_'):
    struct_name = struct_name[len('struct_c__SA_'):]
  elif struct_name.startswith('struct_c_'):
    struct_name = struct_name[len('struct_c_'):]
  elif struct_name.startswith('struct_'):
    struct_name = struct_name[len('struct_'):]
  return struct_name


def GetCFields(struct):
  """Get all fields in struct, skipping padding."""
  # Clang2py artificially adds padding fields to align data to 4 byte
  # boundaries. These paddings start with 'PADDING_' and we need to
  # remove these fake fields.
  return [(n, t) for n, t in struct._fields_  # pylint: disable=protected-access
          if not n.startswith('PADDING_')]
