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

"""This module generates type_info.cc for use with h5log_reader.cc."""

import ctypes
import os
import re
import sys
import textwrap

import gflags
import makani
from makani.avionics.common import build_info_types
from makani.avionics.common import pack_aio_header
from makani.avionics.network import aio_message
from makani.control import control_types
from makani.control import system_types
from makani.lib.pcap_to_hdf5 import pack_capture_info
from makani.lib.python import c_helpers
from makani.lib.python.autogen import autogen_util
from makani.sim import sim_types

gflags.DEFINE_string('source_file', None,
                     'Full path to output source file.',
                     short_name='s')
gflags.DEFINE_string('header_file', None,
                     'Full path to output header file.',
                     short_name='h')
gflags.DEFINE_string('autogen_root', makani.HOME,
                     'Root of the source tree for the output files.')
FLAGS = gflags.FLAGS

_TYPE_MAP = {
    ctypes.c_bool: 'bool',
    ctypes.c_char: 'char',
    ctypes.c_double: 'double',
    ctypes.c_float: 'float',
    ctypes.c_int16: 'int16_t',
    ctypes.c_int32: 'int32_t',
    ctypes.c_int64: 'int64_t',
    ctypes.c_int8: 'int8_t',
    ctypes.c_uint16: 'uint16_t',
    ctypes.c_uint32: 'uint32_t',
    ctypes.c_uint64: 'uint64_t',
    ctypes.c_uint8: 'uint8_t'}


def _GetType(type_id, path=''):
  """Recursively parse a ctypes structure to build a (path, type) list."""
  if issubclass(type_id, ctypes.Structure):
    out = _GetStruct(type_id, path)
  elif issubclass(type_id, ctypes.Array):
    out = _GetArray(type_id, path)
  else:
    out = [(path, _TYPE_MAP[type_id])]
  return out


def _GetStruct(type_id, path):
  """Recursively parse a ctypes structure to build a (path, type) list."""
  out = []
  for field_name, field_type in autogen_util.GetCFields(type_id):
    out.extend(_GetType(field_type, path + '.' + field_name))
  return out


def _GetArray(type_id, path):
  """Recursively parse a ctypes structure to build a (path, type) list."""
  out = []
  inst = type_id()
  for i in range(len(inst)):
    out.extend(_GetType(inst._type_, path + '[' + str(i) + ']'))  # pylint: disable=protected-access
  return out


def _GenerateConvert():
  """Generate function to convert between native variable types."""
  s = ''
  types = sorted(_TYPE_MAP.values())
  for src_type in ['src_type', 'float', 'double']:
    # Specialize Convert() function for conversion from float to bool types
    # to avoid compile warning regarding explicit comparison of floating
    # point types.
    if src_type == 'src_type':
      dst_type = 'dst_type'
      template = '<typename src_type, typename dst_type>'
      function = ''
      assign = 'static_cast<dst_type>(src_obj)'
    else:
      dst_type = 'bool'
      template = '<>'
      function = '<%s, bool>' % src_type
      assign = 'fabs(src_obj) > std::numeric_limits<%s>::epsilon()' % src_type

    s += textwrap.dedent("""\
        template{template}
        void Convert{function}(
            int64_t n, size_t src_offset, size_t src_stride,
            const void *src_data, size_t dst_offset, size_t dst_stride,
            void *dst_data) {{
          const uint8_t *s =
              reinterpret_cast<const uint8_t *>(src_data) + src_offset;
          uint8_t *d = reinterpret_cast<uint8_t *>(dst_data) + dst_offset;
          for (int64_t i = 0; i < n; ++i, s += src_stride, d += dst_stride) {{
            {src_type} src_obj;
            memcpy(&src_obj, s, sizeof(src_obj));
            {dst_type} dst_obj = {assign};
            memcpy(d, &dst_obj, sizeof(dst_obj));
          }}
        }}

        """).format(template=template, function=function, src_type=src_type,
                    dst_type=dst_type, assign=assign)

  s += textwrap.dedent("""\
      typedef void (*ConvertFunction)(
          int64_t n, size_t src_offset, size_t src_stride, const void *src_data,
          size_t dst_offset, size_t dst_stride, void *dst_data);

      const ConvertFunction kConvertMap[kNumDataTypes][kNumDataTypes] = {
      """)
  for src_type in types:
    s += '  {'
    s += ',\n   '.join(['Convert<{}, {}>'.format(src_type, d) for d in types])
    s += '},\n'
  s += '};\n\n'
  return s


def _GenerateTypeInfo(type_info, prefix):
  """Generate TypeInfo structures."""
  fields = ''
  for path, field_type in _GetType(type_info.ctype):
    fields += textwrap.dedent("""\
        {{".{prefix}.{path}", kDataType{type}, OFFSET_OF({name}, {path})}},
        """).format(name=type_info.struct_name, prefix=prefix, path=path[1:],
                    type=c_helpers.SnakeToCamel(field_type))

  s = textwrap.dedent("""\
      const TypeInfoField kTypeInfoField{struct_name}[] = {{
      {fields}
      }};

      const TypeInfo kTypeInfo{struct_name} = {{
        reinterpret_cast<PackTypeFunction>({pack_func}), {pack_size},
        reinterpret_cast<UnpackTypeFunction>({unpack_func}), {unpack_size},
        ARRAYSIZE(kTypeInfoField{struct_name}), kTypeInfoField{struct_name}
      }};

      """).format(fields=c_helpers.Indent(fields, 1)[:-1],
                  struct_name=type_info.struct_name,
                  pack_func=type_info.pack_func,
                  pack_size=type_info.pack_size,
                  unpack_func=type_info.unpack_func,
                  unpack_size=type_info.unpack_size)

  return s


def _GenerateParameterTypeInfo(module, name):
  """Generates TypeInfo for members of the 'parameters' group.

  These types are not required for hdf5_to_pcap, so no pack function, pack size,
  or unpack function are required.

  Args:
    module: Module that specifies the Python version of the type.
    name: Name of the type.

  Returns:
    String containing TypeInfo definitions.
  """
  type_info = aio_message.GetInfoByModule(module, name)

  fields = ''
  for path, field_type in _GetType(type_info.ctype):
    fields += textwrap.dedent("""\
        {{".{path}", kDataType{type}, OFFSET_OF({name}, {path})}},
        """).format(name=type_info.struct_name, path=path[1:],
                    type=c_helpers.SnakeToCamel(field_type))

  return textwrap.dedent("""\
      const TypeInfoField kTypeInfoField{struct_name}[] = {{
      {fields}
      }};

      const TypeInfo kTypeInfo{struct_name} = {{
        reinterpret_cast<PackTypeFunction>(NULL), 0,
        reinterpret_cast<UnpackTypeFunction>(NULL), {unpack_size},
        ARRAYSIZE(kTypeInfoField{struct_name}), kTypeInfoField{struct_name}
      }};

      """).format(fields=c_helpers.Indent(fields, 1)[:-1],
                  struct_name=type_info.struct_name,
                  unpack_size=ctypes.sizeof(type_info.ctype))


def _GenerateCaptureHeaderTypeInfo():
  """Generate CaptureHeader TypeInfo structure."""
  return _GenerateTypeInfo(
      aio_message.GetInfoByModule(pack_capture_info, 'CaptureHeader'),
      prefix='capture_header')


def _GenerateAioHeaderTypeInfo():
  """Generate AioHeader TypeInfo structure."""
  return _GenerateTypeInfo(
      aio_message.GetInfoByModule(pack_aio_header, 'AioHeader'),
      prefix='aio_header')


def _GenerateMessageTypeInfo():
  """Generate TypeInfo structures for each message type."""
  type_names = set()
  s = ''
  info_map = aio_message.GetMessageInfoMapByNetworkFile()
  for type_info in [v for v in info_map.values() if v]:
    if type_info.struct_name not in type_names:
      type_names.add(type_info.struct_name)
      s += _GenerateTypeInfo(type_info, prefix='message')
  return s


def _GenerateGetTypeInfo():
  """Generate template accessor function for TypeInfo structures."""
  s = textwrap.dedent("""\
      template <typename T>
      const TypeInfo *GetTypeInfo() {
        return nullptr;
      }

      """)
  return s


def _GenerateGetTypeInfoByName(name):
  """Generates accessor functions for a TypeInfo structure by name."""
  s = textwrap.dedent("""\
      const TypeInfo *Get{name}TypeInfo(void) {{
        return &kTypeInfo{name};
      }}

      template<> const TypeInfo *GetTypeInfo<{name}>() {{
        return &kTypeInfo{name};
      }}

      """).format(name=name)
  return s


def _GenerateGetMessageTypeInfo():
  """Generate accessor functions for the message type TypeInfo structures."""
  info_map = aio_message.GetMessageInfoMapByNetworkFile()
  supported_types = {k: v for k, v in info_map.iteritems() if v}
  unsupported_types = [k for k, v in info_map.iteritems() if not v]
  s = textwrap.dedent("""\
      const TypeInfo *GetMessageTypeInfo(MessageType message) {
        switch (message) {
      """)
  for message, info in supported_types.iteritems():
    s += c_helpers.Indent(textwrap.dedent("""\
        case {enum_name}:
          return &kTypeInfo{struct_name};
        """).format(enum_name=message.enum_name,
                    struct_name=info.struct_name), 2)
  for message in unsupported_types:
    s += c_helpers.Indent(textwrap.dedent("""\
        case {enum_name}:
          return nullptr;  // Not supported.
        """).format(enum_name=message.enum_name), 2)
  s += textwrap.dedent("""\
          case kNumMessageTypes:
          default:
            return nullptr;
        }
      }

      """)
  # GCC 6.3, differently than GCC 4.8, treats type-assignments as equal types,
  # so generating functions with equivalent types result in redefinition error.
  # Therefore we have to identify unique types not from names but from the
  # actual type definition.
  unique_types = set()
  for v in supported_types.values():
    struct_name = v.struct_name
    if v.ctype in unique_types:
      continue
    unique_types.add(v.ctype)
    s += textwrap.dedent("""\
        template<> const TypeInfo *GetTypeInfo<{struct_name}>() {{
          return &kTypeInfo{struct_name};
        }}

        """).format(struct_name=struct_name)
  return s


def _GenerateConvertDataType():
  """Generate function to convert between data types."""
  s = textwrap.dedent("""\
      void ConvertDataType(int64_t n, DataType src_type, size_t src_offset,
                           size_t src_step, const void *src, DataType dst_type,
                           size_t dst_offset, size_t dst_step, void *dst) {
        assert(0 <= src_type && src_type < kNumDataTypes);
        assert(0 <= dst_type && dst_type < kNumDataTypes);
        kConvertMap[src_type][dst_type](n, src_offset, src_step, src,
                                        dst_offset, dst_step, dst);
      }
      """)
  return s


def _GenerateHeader(header_file):
  """Generate output header file."""
  guard = re.sub('[/.]', '_', header_file).upper() + '_'
  data_types = sorted(['kDataType' + c_helpers.SnakeToCamel(s)
                       for s in _TYPE_MAP.values()])
  s = textwrap.dedent("""\
      #ifndef {guard}
      #define {guard}

      #include <stddef.h>
      #include <stdint.h>

      #include "avionics/network/message_type.h"

      typedef size_t (*PackTypeFunction)(const void *in,
                                         size_t num,
                                         uint8_t *out);

      typedef size_t (*UnpackTypeFunction)(const uint8_t *in,
                                           size_t num,
                                           void *out);

      typedef enum {{
      {data_types},
        kNumDataTypes
      }} DataType;

      typedef struct {{
        const char *path;
        DataType type;
        size_t offset;
      }} TypeInfoField;

      typedef struct {{
        PackTypeFunction pack;
        size_t pack_size;
        UnpackTypeFunction unpack;
        size_t unpack_size;
        int32_t num_fields;
        const TypeInfoField *field;
      }} TypeInfo;

      template <typename T> const TypeInfo *GetTypeInfo();

      const TypeInfo *GetCaptureHeaderTypeInfo(void);
      const TypeInfo *GetAioHeaderTypeInfo(void);
      const TypeInfo *GetMessageTypeInfo(MessageType message);

      void ConvertDataType(int64_t n, DataType src_type, size_t src_offset,
                           size_t src_step, const void *src, DataType dst_type,
                           size_t dst_offset, size_t dst_step, void *dst);

      #endif  // {guard}
      """).format(guard=guard,
                  data_types=c_helpers.Indent(',\n'.join(data_types)))
  return s


def _GenerateSource(header_file):
  """Generate output source file."""

  info_map = aio_message.GetMessageInfoMapByNetworkFile()
  includes = aio_message.GetHeaderFilesFromMessageInfoMap(info_map)
  includes += [
      'avionics/common/aio_header.h',
      'avionics/common/pack_aio_header.h',
      'avionics/network/message_type.h',
      'common/macros.h',
      'lib/pcap_to_hdf5/capture_info.h',
      'lib/pcap_to_hdf5/pack_capture_info.h',
  ]

  s = textwrap.dedent("""\
      #include "{header_file}"

      #include <stddef.h>
      #include <stdint.h>
      #include <string.h>

      #include <cmath>
      #include <limits>

      {includes}

      // The stock offsetof() implementation does not support nested structures.
      #define OFFSET_OF(type, mem) \\
          ((size_t)&reinterpret_cast<const type *>(0)->mem)

      """).format(
          header_file=header_file,
          includes='\n'.join(['#include "%s"' % f for f in sorted(includes)]))

  s += 'namespace {\n\n'
  s += _GenerateConvert()
  s += _GenerateParameterTypeInfo(build_info_types, 'BuildInfo')
  s += _GenerateParameterTypeInfo(control_types, 'ControlParams')
  s += _GenerateParameterTypeInfo(sim_types, 'SimParams')
  s += _GenerateParameterTypeInfo(system_types, 'SystemParams')
  s += _GenerateCaptureHeaderTypeInfo()
  s += _GenerateAioHeaderTypeInfo()
  s += _GenerateMessageTypeInfo()
  s += '}  // namespace\n\n'
  s += _GenerateGetTypeInfoByName('BuildInfo')
  s += _GenerateGetTypeInfoByName('ControlParams')
  s += _GenerateGetTypeInfoByName('SimParams')
  s += _GenerateGetTypeInfoByName('SystemParams')
  s += _GenerateGetTypeInfoByName('CaptureHeader')
  s += _GenerateGetTypeInfoByName('AioHeader')
  s += _GenerateGetMessageTypeInfo()
  s += _GenerateConvertDataType()
  return s[:-1]


def _GetHeaderRelPath():
  if FLAGS.header_file:
    return os.path.relpath(FLAGS.header_file, start=FLAGS.autogen_root)
  return ''


def main(argv):
  """Entry point."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  if FLAGS.header_file:
    with open(FLAGS.header_file, 'w') as f:
      f.write(_GenerateHeader(_GetHeaderRelPath()))

  if FLAGS.source_file:
    with open(FLAGS.source_file, 'w') as f:
      f.write(_GenerateSource(_GetHeaderRelPath()))


if __name__ == '__main__':
  main(sys.argv)
