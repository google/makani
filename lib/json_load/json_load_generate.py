#!/usr/bin/python
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

"""Generates C code for loading structures from JSON files.

Usage: lib/json_load/json_load_generate.py <struct1> <struct2> ...

The struct arguments are names of structures from one of:
control/system_types.h, control/control_types.h, gs/monitor/monitor_types.h,
or sim/sim_types.h.

The prototypes for loading these functions are placed in
lib/json_load/json_load.h and the corresponding code is place in
lib/json_load/json_load.c.
"""

import ctypes
import datetime
import os
import sys

import gflags
import makani
from makani.control import control_types
from makani.gs.monitor import monitor_types
from makani.lib.json_load import json_load
from makani.sim import sim_types

gflags.DEFINE_string('autogen_root', makani.HOME,
                     'Root of the source tree for the output files.')

FLAGS = gflags.FLAGS


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  if len(argv) < 1:
    raise SystemExit('Usage: ' + __file__ + ' <header_files>')

  # This is the list of types which have implementations in
  # lib/json_load/json_load_basic.{c,h}.
  manual_impl = {ctypes.c_bool,
                 ctypes.c_double, ctypes.c_float,
                 ctypes.c_int8, ctypes.c_uint8,
                 ctypes.c_int16, ctypes.c_uint16,
                 ctypes.c_int,
                 ctypes.c_int32 * 0,
                 ctypes.c_uint32,
                 ctypes.c_char * 0,  # This represents a string.
                 # The next type is a 1D double array which is used in
                 # the manual implementations of Vec2, Vec3, and Quat.
                 ctypes.c_double * 0,
                 control_types.Vec2,
                 control_types.Vec3,
                 control_types.Quat}

  public_structs = _FindStructuresInModules(
      [control_types, monitor_types, sim_types], argv[1:])

  _WriteCode(public_structs, manual_impl, 'lib/json_load')


def _WriteCode(public_types, manual_impl, output_directory):
  """Write C source and header files for JSON loading.

  Args:
    public_types: A list of types to generate loading functions for.
    manual_impl: A set of types which have manual implementations in
        json_load_basic.{c,h}.
    output_directory: The sub-directory of MAKANI_HOME in which to place the
        the generated source.
  """
  manual_types_dict = _TypeSetToDict(manual_impl)
  public_types_dict = _TypeSetToDict(public_types)

  # Test that the types being generated don't belong to json_pack_basic.h.
  # Note that all type comparisons should be done using the result of
  # GetTypeName as arrays of different length are distinct ctypes.
  assert not [t for t in public_types
              if json_load.GetTypeName(t) in manual_types_dict]

  # Build the dict of all types for which we need loading functions.
  all_types = set(public_types).union(*[json_load.GetRequiredTypes(t)
                                        for t in public_types])

  # Remove manually implemented load functions.
  all_types_dict = {json_load.GetTypeName(t): t for t in all_types
                    if json_load.GetTypeName(t) not in manual_types_dict}

  public_fns = [json_load.GetLoadFunc(t)
                for (name, t) in all_types_dict.items()
                if name in public_types_dict]

  private_fns = [json_load.GetLoadFunc(t)
                 for (name, t) in all_types_dict.items()
                 if name not in public_types_dict]

  rel_path_sans_ext = output_directory + os.sep + 'json_load'
  abs_path_sans_ext = os.path.join(FLAGS.autogen_root, rel_path_sans_ext)

  with open(abs_path_sans_ext + '.c', 'w') as c_file:
    c_file.write(_CSourceString(public_fns + private_fns, rel_path_sans_ext))

  with open(abs_path_sans_ext + '_priv.h', 'w') as priv_h_file:
    priv_h_file.write(_HSourceString(private_fns, rel_path_sans_ext + '_priv'))

  with open(abs_path_sans_ext + '.h', 'w') as h_file:
    h_file.write(_HSourceString(public_fns, rel_path_sans_ext))


def _HSourceString(fns, rel_path_sans_ext):
  """Generate a header file for a set of functions."""
  prototypes = [prototype for (prototype, _) in fns]

  define_str = rel_path_sans_ext.upper().replace('/', '_') + '_H_'
  include_top, include_bot = _GetIncludeGuardStrTuple(define_str)
  cpp_top, cpp_bot = _GetCPlusPlusGuardStrTuple()

  lines = [_GetGoogleCopyrightStr(),
           '',
           include_top,
           '',
           cpp_top,
           '',
           '#include <jansson.h>',
           '#include <stdint.h>',
           '#include <stdio.h>',
           '',
           '#include "control/control_types.h"',
           '#include "control/system_types.h"',
           '#include "gs/monitor/monitor_types.h"',
           '#include "sim/sim_types.h"',
           ''
           '\n'.join([prototype + ';' for prototype in prototypes]),
           '',
           cpp_bot,
           '',
           include_bot,
           '']
  return '\n'.join(lines)


def _CSourceString(fns, rel_path_sans_ext):
  """Generate the C source file for a set of functions."""
  lines = [_GetGoogleCopyrightStr(),
           '',
           '#include "%s.h"' % rel_path_sans_ext,
           '#include "%s_priv.h"' % rel_path_sans_ext,
           '',
           '#include <jansson.h>',
           '#include <stdint.h>',
           '',
           '#include "%s_basic.h"' % rel_path_sans_ext,
           '',
           '#pragma GCC diagnostic push',
           '#pragma GCC diagnostic ignored "-Wsign-conversion"',
           '\n\n'.join(['%s {\n%s\n}' % (prototype, '\n'.join(body))
                        for (prototype, body) in fns]),
           '#pragma GCC diagnostic pop',
           '']

  return '\n'.join(lines)


def _FindStructuresInModules(modules, struct_names):
  """Find modules attributes by name from a list of modules.

  Args:
    modules: A list of python modules.
    struct_names: A list of strings giving the names of structs.

  Returns:
    A set of ctypes structures which are attributes of any module in the
    list with a name in struct_names.  If more than one module has
    an attribute with the same name, only one structure is returned.

  Raises:
    SystemExit: A name in struct_names is not an attribute of any member
        of modules.
  """
  structs = []
  for name in struct_names:
    t = None
    for m in modules:
      if hasattr(m, name):
        t = getattr(m, name)
        assert isinstance(t, type(ctypes.Structure))
        break
    if t is None:
      raise SystemExit('Could not find structure: %s.' % name)

    structs.append(t)
  return set(structs)


def _TypeSetToDict(type_set):
  return {json_load.GetTypeName(t): t for t in type_set}


def _GetGoogleCopyrightStr():
  return ('// Copyright %d, Google Inc. All Rights Reserved.'
          % datetime.date.today().year)


def _GetIncludeGuardStrTuple(define_str):
  return ('#ifndef %s\n'
          '#define %s' % (define_str, define_str),
          '#endif  // ' + define_str)


def _GetCPlusPlusGuardStrTuple():
  return ('\n'
          '#ifdef __cplusplus\n'
          'extern "C" {\n'
          '#endif',
          '\n'
          '#ifdef __cplusplus\n'
          '}  // extern "C"\n'
          '#endif')


if __name__ == '__main__':
  main(sys.argv)
