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

"""Generates C source for packing and unpacking functions."""

import collections
import ctypes
import imp
import os
import re
import subprocess
import sys
import tempfile
import textwrap
from xml.etree import ElementTree

import gflags
import makani
from makani.lib.python.autogen import autogen_util

# TODO(): Unit tests.

gflags.DEFINE_enum('endianness', 'host', ['big', 'host', 'little'],
                   'Endianness of the serialization.', short_name='e')

gflags.DEFINE_multistring('include_dir', [], 'Include paths.', short_name='I')

gflags.DEFINE_multistring('defines', [], 'Compilation definitions.',
                          short_name='D')

gflags.DEFINE_string('output_prefix', None, 'Output file prefix.')

gflags.DEFINE_string('autogen_root', os.getcwd(),
                     'Source tree root for the output files.')

FLAGS = gflags.FLAGS

# pylint: disable=W0212

# Determines whether underline or camel-case format is used for
# fundamental C types
use_underlines = False


def GatherStructs(header, xml_file, python_file):
  """Returns Python structures necessary to build structs in header file.

  Args:
    header: Path to header file.
    xml_file: Path to XML file.
    python_file: Path to Python file.

  Returns:
    3-tuple consisting of:
      parent_structs: Top-level structs needing pack functions.
      child_structs: Structs on which parent_structs depend.
      child_ctypes: ctype classes for child_structs.
  """
  def IsTypedefStruct(t):
    """Tells whether the struct is typedef'd.

    E.g. typedef struct {} a; versus struct a {};

    Args:
      t: The struct translated from C.

    Returns:
      True if the struct is typedef'ed.
    """
    return t.__name__.startswith('struct_')

  def RootTypeElmt(type_id, by_id):
    """Find the element that defines the base type.

    The root type is one that is not derived as a typedef or array.

    Args:
      type_id: ID of the XML Dom that defines a type.
      by_id: <key, value> dictionary of XML Dom ID and the Dom element.

    Returns:
      The XML Dom element that defines the root type.
    """
    while True:
      if type_id is None or type_id not in by_id:
        return None
      elmt = by_id[type_id]
      if elmt.tag == 'Typedef':
        type_id = elmt.attrib.get('type', None)
      elif elmt.tag == 'ArrayType':
        type_id = elmt.attrib.get('type', None)
      else:
        return elmt

  global use_underlines  # pylint: disable=W0603
  xml_elmts = ElementTree.parse(xml_file).getroot()
  file_ids = [elmt.attrib['id'] for elmt in xml_elmts
              if (elmt.tag == 'File'
                  and os.path.realpath(elmt.attrib['name']).endswith(header))]
  assert len(file_ids) == 1
  local_structs = {elmt.attrib['id']: elmt for elmt in xml_elmts
                   if (elmt.tag == 'Struct'
                       and elmt.attrib['file'] == file_ids[0])}
  by_id = {elmt.attrib['id']: elmt for elmt in xml_elmts
           if elmt.attrib.get('id', None)}
  all_structs = {elmt.attrib['id']: elmt for elmt in xml_elmts
                 if elmt.tag == 'Struct'}

  struct_names = collections.defaultdict(list)
  # The same struct can have multiple names.
  sid_by_name = {}

  for sid, elmt in all_structs.iteritems():
    if elmt.attrib.get('name', None):
      struct_names[sid].append(elmt.attrib['name'])
      sid_by_name[elmt.attrib['name']] = sid

  typedef_structs = []
  for root_id, elmt in by_id.iteritems():
    if elmt.tag == 'Typedef':
      name = elmt.attrib['name']
      child_id = root_id
      while child_id in by_id and by_id[child_id].tag == 'Typedef':
        child_id = by_id[child_id].attrib['type']
      if child_id in by_id and by_id[child_id].tag == 'Struct':
        typedef_structs.append(name)
        struct_names[child_id].append(name)
        sid_by_name[name] = child_id

  if all([not IsCamelCase(s) for s in typedef_structs] +
         [not IsCamelCase(s.attrib['name']) for s in local_structs.itervalues()
          if s.attrib.get('name', None)]):
    use_underlines = True

  mod = imp.load_source('tmp', python_file)
  child_strs = []
  ctype_strs = []
  for sid in local_structs.iterkeys():
    s = struct_names[sid][0]
    slst, clst = FindChildTypes(getattr(mod, s))
    child_strs += slst
    ctype_strs += clst
  parent_structs = set([
      getattr(mod, struct_names[sid][0]) for sid in local_structs.iterkeys()])
  child_structs = set([getattr(mod, s) for s in child_strs])
  child_structs.difference_update(parent_structs)
  child_ctypes = set([getattr(ctypes, s) for s in ctype_strs])

  # Sort lists to generate exactly the same code for each execution.
  child_ctypes = sorted(child_ctypes, key=str)

  # Adds a _typedef_ field that records whether the struct is a
  # typedef'd struct or not.  (e.g. typedef struct {} a; versus
  # struct a {};)
  for s in parent_structs | child_structs:
    s._typedef_ = IsTypedefStruct(s)

  # Add struct typedefs as an _aliases_ field.
  for s in parent_structs | child_structs:
    for typedef in typedef_structs:
      if not getattr(mod, typedef, None):
        continue
      if s == getattr(mod, typedef) and typedef != s.__name__:
        if not hasattr(s, '_aliases_'):
          s._aliases_ = []
        s._aliases_.append(typedef)

  for s in parent_structs | child_structs:
    if hasattr(s, '_aliases_'):
      s._aliases_.sort()

  # Add an _enums_ field to list fields in a struct which are enums.
  enum_types = set()
  for s in parent_structs | child_structs:
    s._enums_ = {}
    struct_name = autogen_util.CStructName(s)
    if struct_name not in sid_by_name:
      continue

    for field_id in all_structs[
        sid_by_name[struct_name]].attrib['members'].split():
      if field_id in by_id and by_id[field_id].tag == 'Field':
        name = by_id[field_id].attrib['name']
        base_type = RootTypeElmt(by_id[field_id].attrib['type'], by_id)
        if base_type.tag == 'Enumeration':
          enum_type = base_type.attrib['name']
          s._enums_[name] = enum_type
          enum_types.add(enum_type)

  # Sort the returned lists in a deterministic order for a definite CRC.
  enum_types = sorted(enum_types)
  return parent_structs, child_structs, child_ctypes, enum_types


def _IsCTypeStruct(s):
  """Returns True if `s` is converted from a C struct."""
  return not s.__name__.startswith('struct_')


def FindChildTypes(parent):
  """Return name of all the substructures on which a parent struct depends."""
  struct_lst = []
  ctype_lst = []
  for _, field_type in autogen_util.GetCFields(parent):
    if isinstance(field_type, type(ctypes.Array)):
      t = field_type()._type_
    else:
      t = field_type
    if isinstance(t, type(ctypes.Structure)):
      struct_lst.append(t.__name__)
      slst, clst = FindChildTypes(t)
      struct_lst += slst
      ctype_lst += clst
    # Matches any of the fundamental c types
    elif isinstance(t, type(ctypes.c_double)):
      ctype_lst.append(t.__name__)
  return struct_lst, ctype_lst


def ConvTypeToStrHelper(t):
  if t == ctypes.c_double:
    return 'Double', 'double'
  elif t == ctypes.c_float:
    return 'Float', 'float'
  elif t == ctypes.c_int8:
    return 'Int8', 'int8_t'
  elif t == ctypes.c_int16:
    return 'Int16', 'int16_t'
  elif t == ctypes.c_int32:
    return 'Int32', 'int32_t'
  elif t == ctypes.c_int64:
    return 'Int64', 'int64_t'
  elif t == ctypes.c_int:
    return 'Int', 'int'
  elif t == ctypes.c_uint8:
    return 'UInt8', 'uint8_t'
  elif t == ctypes.c_uint16:
    return 'UInt16', 'uint16_t'
  elif t == ctypes.c_uint32:
    return 'UInt32', 'uint32_t'
  elif t == ctypes.c_uint64:
    return 'UInt64', 'uint64_t'
  elif t == ctypes.c_uint:
    return 'UInt', 'unsigned int'
  elif t == ctypes.c_char:
    return 'Char', 'char'
  elif t == ctypes.c_bool:
    return 'Bool', 'bool'
  else:
    return None, None


def ConvTypeToStr(t):
  """Gets a string name and size information for a type.

  Args:
    t: The type object.

  Returns:
    (name, length, dimensions):
      name: String name of the type.
      length: Number of elements in the type (1 if not an array).
      dim: Number of dimensions of the type.

  Raises:
    ValueError: The provided type is not recognized.
  """
  cc_str, ul_str = ConvTypeToStrHelper(t)
  if use_underlines:
    cc_str = ul_str
  if cc_str is not None:
    return cc_str, 1, 0
  elif isinstance(t, type(ctypes.Structure)):
    if t._typedef_:
      return autogen_util.CStructName(t), 1, 0
    else:
      return 'struct ' + autogen_util.CStructName(t), 1, 0
  elif isinstance(t, type(ctypes.Array)):
    s, l, dim = ConvTypeToStr(t()._type_)
    return s, len(t()) * l, dim + 1

  raise ValueError('Unknown type: %s' % t)


# Boolean types are represented as a single byte on the wire with
# false being encoded as 0 and all other values being encoded as 1.
def WriteCTypeBoolFunc(f):
  """Writes PackBool and UnpackBool functions."""
  cc_str, ul_str = ConvTypeToStrHelper(ctypes.c_bool)
  if use_underlines:
    cc_str = ul_str
  f.write('\n\nstatic size_t '
          '%s(const bool *in, size_t num, uint8_t *out) {'
          '\n  size_t i;'
          '\n  for (i = 0U; i < num; ++i) out[i] = (in[i] != false);'
          '\n  return num * sizeof(*in);'
          '\n}'
          % (GenPackFuncName(cc_str)))
  f.write('\n\nstatic size_t '
          '%s(const uint8_t *in, size_t num, bool *out) {'
          '\n  size_t i;'
          '\n  for (i = 0U; i < num; ++i) out[i] = (in[i] != 0);'
          '\n  return num * sizeof(*out);'
          '\n}'
          % (GenUnpackFuncName(cc_str)))


# TODO: Profile and optimize. The generated functions lie on the
# critical path of all message sends and receives, but it's not clear that
# performance is an issue.
# - On big-endian architectures, memcpy is probably fastest.
# - On little-endian architectures, each element needs to be byte-reversed.
#   Because the packed buffer may be unaligned, the result generally cannot be
#   written in one memory write, which restricts memcpy optimizations. Writing
#   one byte at a time is probably not far from optimal.
# - On systems without an instruction cache, optimize for fewest instructions
#   fetched.
# - x86 and ARM both have instructions to optimize byte reversal.
def WriteCTypeFunc(f, child_ctype):
  cc_str, ul_str = ConvTypeToStrHelper(child_ctype)
  if use_underlines:
    cc_str = ul_str
  if child_ctype is ctypes.c_bool:
    WriteCTypeBoolFunc(f)
  elif FLAGS.endianness == 'host':
    f.write('\n\nstatic size_t '
            '%s(const %s *in, size_t num, uint8_t *out) {'
            '\n  memcpy(out, in, num * sizeof(%s));'
            '\n  return num * sizeof(%s);'
            '\n}'
            % (GenPackFuncName(cc_str), ul_str, ul_str, ul_str))
    f.write('\n\nstatic size_t '
            '%s(const uint8_t *in, size_t num, %s *out) {'
            '\n  memcpy(out, in, num * sizeof(%s));'
            '\n  return num * sizeof(%s);'
            '\n}'
            % (GenUnpackFuncName(cc_str), ul_str, ul_str, ul_str))
  else:
    num_bits = GetTypePackedSize(child_ctype) * 8
    if FLAGS.endianness == 'little':
      bit_shift_str = 'j'
    elif FLAGS.endianness == 'big':
      bit_shift_str = '(%d - j)' % (num_bits - 8)
    f.write('\n\nstatic size_t '
            '%s(const %s *in, size_t num, uint8_t *out) {'
            '\n  size_t i, j;'
            '\n  for (i = 0U; i < num; ++i) {'
            '\n    uint%d_t e;'
            '\n    memcpy(&e, &in[i], sizeof(*in));'
            '\n    for (j = 0U; j < 8U * sizeof(*in); j += 8U) {'
            '\n      *out = (uint8_t)(e >> %s);'
            '\n      out++;'
            '\n    }'
            '\n  }'
            '\n  return num * sizeof(*in);'
            '\n}'
            % (GenPackFuncName(cc_str), ul_str, num_bits,
               bit_shift_str))
    f.write('\n\nstatic size_t '
            '%s(const uint8_t *in, size_t num, %s *out) {'
            '\n  size_t i, j;'
            '\n  for (i = 0U; i < num; ++i) {'
            '\n    uint%d_t e = 0U;'
            '\n    for (j = 0U; j < 8U * sizeof(*out); j += 8U) {'
            '\n      e |= (uint%d_t)((uint%d_t)*in << %s);'
            '\n      in++;'
            '\n    }'
            '\n    memcpy(&out[i], &e, sizeof(*out));'
            '\n  }'
            '\n  return num * sizeof(*out);'
            '\n}'
            % (GenUnpackFuncName(cc_str), ul_str, num_bits, num_bits,
               num_bits, bit_shift_str))


def WriteEnumFunc(f, enum_type):
  """Generate pack/unpack functions for an enum type serialized as int32_t."""
  f.write(textwrap.dedent("""
      static size_t %s(const %s *in, size_t num, uint8_t *out) {
        size_t i;
        int32_t v;
        for (i = 0U; i < num; ++i) {
          v = (int32_t)in[i];
          out += PackInt32(&v, 1, out);
        }
        return num * sizeof(v);
      }
      """ % (GenPackFuncName(enum_type), enum_type)))
  f.write(textwrap.dedent("""
      static size_t %s(const uint8_t *in, size_t num, %s *out) {
        size_t i;
        int32_t v;
        for (i = 0U; i < num; ++i) {
          in += UnpackInt32(in, 1, &v);
          out[i] = (%s)v;
        }
        return num * sizeof(v);
      }
      """ % (GenUnpackFuncName(enum_type), enum_type, enum_type)))


def GetStaticStr(is_static):
  return 'static ' if is_static else ''


def IsCamelCase(s):
  return str.isupper(s[0])


def GenPackFuncName(s):
  if s.find('struct ') == 0:
    s = s[7:]
  if IsCamelCase(s):
    return 'Pack' + s
  else:
    return 'pack_' + s


def GenUnpackFuncName(s):
  if s.find('struct ') == 0:
    s = s[7:]
  if IsCamelCase(s):
    return 'Unpack' + s
  else:
    return 'unpack_' + s


def GenTypeCast(t, const_str=''):
  if const_str:
    const_str += ' '
  _, ul_type_str = ConvTypeToStrHelper(t)
  if ul_type_str is not None:
    return '(%s%s *)' % (const_str, ul_type_str)
  else:
    return ''


def WritePackFunc(f, s, is_static, name=None):
  """Writes a pack function."""
  if not name:
    name = ConvTypeToStr(s)[0]
  f.write('\n'
          '\n%ssize_t %s(const %s *in, size_t num, uint8_t *out) {'
          '\n  size_t byte_ind = 0U, elmt_ind;'
          '\n  for (elmt_ind = 0U; elmt_ind < num; ++elmt_ind) {'
          % (GetStaticStr(is_static), GenPackFuncName(name),
             name))
  for field_name, field_type in autogen_util.GetCFields(s):
    type_str, num, dim = ConvTypeToStr(field_type)
    if field_name in s._enums_:
      type_str = s._enums_[field_name]
    f.write('\n    byte_ind += %s(&in[elmt_ind].%s%s, %d, &out[byte_ind]);'
            % (GenPackFuncName(type_str), field_name, '[0]'*dim, num))
  f.write('\n  }'
          '\n  return byte_ind;'
          '\n}')


def WriteUnpackFunc(f, s, is_static, name=None):
  """Writes an unpack function."""
  if not name:
    name = ConvTypeToStr(s)[0]
  f.write('\n'
          '\n%ssize_t %s(const uint8_t *in, size_t num, %s *out) {'
          '\n  size_t byte_ind = 0U, elmt_ind;'
          '\n  for (elmt_ind = 0U; elmt_ind < num; ++elmt_ind) {'
          % (GetStaticStr(is_static), GenUnpackFuncName(name),
             name))
  for field_name, field_type in autogen_util.GetCFields(s):
    type_str, num, dim = ConvTypeToStr(field_type)
    if field_name in s._enums_:
      type_str = s._enums_[field_name]
    f.write('\n    byte_ind += %s(&in[byte_ind], %d, '
            '&out[elmt_ind].%s%s);'
            % (GenUnpackFuncName(type_str), num, field_name, '[0]'*dim))
  f.write('\n  }'
          '\n  return byte_ind;'
          '\n}')


def GetTypePackedSize(t):
  if isinstance(t, type(ctypes.c_double)):
    if t == ctypes.c_double:
      return 8
    elif t == ctypes.c_float:
      return 4
    elif t == ctypes.c_int8:
      return 1
    elif t == ctypes.c_int16:
      return 2
    elif t == ctypes.c_int32:
      return 4
    elif t == ctypes.c_int64:
      return 8
    elif t == ctypes.c_int:
      return 4
    elif t == ctypes.c_uint8:
      return 1
    elif t == ctypes.c_uint16:
      return 2
    elif t == ctypes.c_uint32:
      return 4
    elif t == ctypes.c_uint64:
      return 8
    elif t == ctypes.c_uint:
      return 4
    elif t == ctypes.c_char:
      return 1
    elif t == ctypes.c_bool:
      return 1
    else:
      raise ValueError('Unrecognized type: %s' % t)
  elif isinstance(t, type(ctypes.Structure)):
    num_bytes = 0
    for (_, field_type) in autogen_util.GetCFields(t):
      num_bytes += GetTypePackedSize(field_type)
    return num_bytes
  elif isinstance(t, type(ctypes.Array)):
    return GetTypePackedSize(t()._type_) * len(t())
  else:
    raise ValueError('Unrecognized type: %s' % t)


def WriteDefinePackedSize(f, s, name=None):
  if not name:
    name = autogen_util.CStructName(s)
  size = GetTypePackedSize(s)
  f.write('\n#define PACK_%s_SIZE %d'
          % (str.upper(name), size))


def WritePackFuncPrototype(f, s, is_static, name=None):
  if not name:
    name = ConvTypeToStr(s)[0]
  f.write('\n%ssize_t %s(const %s *in, size_t num, uint8_t *out);'
          % (GetStaticStr(is_static),
             GenPackFuncName(name),
             name))


def WriteUnpackFuncPrototype(f, s, is_static, name=None):
  if not name:
    name = ConvTypeToStr(s)[0]
  f.write('\n%ssize_t %s(const uint8_t *in, size_t num, %s *out);'
          % (GetStaticStr(is_static),
             GenUnpackFuncName(name),
             name))


def StructsToPackList(structs):
  """Order the structs in a deterministic order."""
  pack_list = []
  for s in structs:
    if _IsCTypeStruct(s):
      name = ConvTypeToStr(s)[0]
      pack_list.append((name, s))

    if hasattr(s, '_aliases_'):
      for alias in s._aliases_:
        pack_list.append((alias, s))

  pack_list.sort(key=lambda x: x[0])
  return pack_list


def WriteHeader(header_in, output_prefix, parent_structs):
  """Writes the C header file."""
  header_out = output_prefix + '.h'
  relpath = os.path.relpath(header_out, FLAGS.autogen_root)
  guard_str = re.sub(r'[-/\.]', '_', relpath).upper() + '_'
  f = open(header_out, 'w')
  f.write('#ifndef %s'
          '\n#define %s'
          '\n'
          '\n#ifdef __cplusplus'
          '\nextern "C" {'
          '\n#endif'
          '\n'
          '\n#include <stdbool.h>'
          '\n#include <stdint.h>'
          '\n#include <stddef.h>'
          '\n#include <assert.h>'
          '\n#include "%s"'
          '\n' % (guard_str, guard_str, header_in))

  pack_list = StructsToPackList(parent_structs)
  for n, s in pack_list:
    # Only write functions for the root class, not assigned ones.
    # This is to avoid duplicated functions.
    WriteDefinePackedSize(f, s, n)
    WritePackFuncPrototype(f, s, False, n)
    WriteUnpackFuncPrototype(f, s, False, n)

  f.write('\n'
          '\n#ifdef __cplusplus'
          '\n}  // Closing brace for extern "C"'
          '\n#endif'
          '\n'
          '\n#endif  // %s\n' % guard_str)
  f.close()


def WriteCode(output_prefix, parent_structs, child_structs, child_ctypes,
              enum_types):
  """Writes the C source file."""
  f = open(output_prefix + '.c', 'w')
  f.write('#include <stdint.h>'
          '\n#include <stddef.h>'
          '\n#include <string.h>'
          '\n#include "%s.h"\n' % os.path.relpath(output_prefix,
                                                  FLAGS.autogen_root))
  for c in child_ctypes:
    WriteCTypeFunc(f, c)
  for e in enum_types:
    WriteEnumFunc(f, e)

  pack_list = StructsToPackList(child_structs)
  for n, s in pack_list:
    WritePackFuncPrototype(f, s, True, n)
    WriteUnpackFuncPrototype(f, s, True, n)
  for n, s in pack_list:
    WritePackFunc(f, s, True, n)
    WriteUnpackFunc(f, s, True, n)

  pack_list = StructsToPackList(parent_structs)
  for n, s in pack_list:
    WritePackFunc(f, s, False, n)
    WriteUnpackFunc(f, s, False, n)
  f.write('\n')
  f.close()


def main(argv):
  def PrintUsage():
    print 'Usage: %s <header.h> \n%s' % (argv[0], FLAGS)

  # Parse flags.
  try:
    remaining_argv = FLAGS(argv)
  except gflags.FlagsError as e:
    PrintUsage()
    raise e

  if len(remaining_argv) != 2:
    PrintUsage()
    raise ValueError('%s takes exactly 1 argument' % argv[0])

  header = remaining_argv[1]

  dirname, filename = os.path.split(header)
  basename = os.path.splitext(filename)[0]

  include_dirs = FLAGS.include_dir + ['/usr/include/clang/7.0.1/include']
  for include_dir in include_dirs:
    if not os.path.isdir(include_dir):
      raise RuntimeError('Include directory %s not found.' % include_dir)
  clang_args = (['-I' + d for d in include_dirs] +
                ['-D' + d for d in FLAGS.defines])

  with tempfile.NamedTemporaryFile(suffix='.py') as temp_py:
    with tempfile.NamedTemporaryFile(suffix='.py') as py_file:
      with tempfile.NamedTemporaryFile(suffix='.xml') as xml_file:
        cmd = ([os.path.join(makani.HOME, 'lib/python/autogen/clang2py'), '-i',
                '--clang-args', '%s' % ' '.join(clang_args)]
               + ['-k', 'cdefstu', '-q']
               + ['-o', temp_py.name, header])
        subprocess.check_call(cmd)
        autogen_util.FixClang2PyOutput(temp_py.name, py_file.name)

        cmd = ['/usr/bin/castxml', '--castxml-gccxml', '-o', xml_file.name]
        cmd += (['-I' + d for d in FLAGS.include_dir] +
                ['-D' + d for d in FLAGS.defines])
        cmd.append(header)
        subprocess.check_call(cmd)

        parent_structs, child_structs, child_ctypes, enum_types = GatherStructs(
            header, xml_file.name, py_file.name)

  output_prefix = FLAGS.output_prefix
  if not output_prefix:
    output_prefix = os.path.join(dirname, 'pack_' + basename)

  WriteHeader(header, output_prefix, parent_structs)
  WriteCode(output_prefix, parent_structs, child_structs, child_ctypes,
            enum_types)


if __name__ == '__main__':
  main(sys.argv)
