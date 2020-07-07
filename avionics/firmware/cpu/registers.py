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


"""Writes the register header file based on a YAML configuration file."""

import os
import sys
import textwrap

import gflags
import makani
from makani.lib.python import string_util
import yaml

gflags.DEFINE_string('definition_file', None,
                     'Full path to YAML registers definition file.')
gflags.DEFINE_string('struct_file', None,
                     'Full path to structure-based output header file.')
gflags.DEFINE_string('define_file', None,
                     'Full path to define-based output header file.')
gflags.DEFINE_string('autogen_root', makani.HOME,
                     'Root of the source tree for the output files.')
FLAGS = gflags.FLAGS


def main(argv):
  """Entry point."""
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '{}\nUsage: {} ARGS\n{}'.format(e, sys.argv[0], FLAGS)
    sys.exit(1)

  with open(FLAGS.definition_file, 'r') as def_file:
    if FLAGS.struct_file:
      def_file.seek(0)
      output = StructBasedOutput(FLAGS.struct_file, FLAGS.define_file)
      ParseRegisterDefinitions(def_file, output)
    if FLAGS.define_file:
      def_file.seek(0)
      ParseRegisterDefinitions(def_file, DefineBasedOutput(FLAGS.define_file))


def ParseRange(s):
  tokens = map(int, str(s).split('-'))
  if len(tokens) == 1:
    return (tokens[0], tokens[0])
  else:
    return (tokens[0], tokens[1])


def ParseRegisterDefinitions(yaml_file, output):
  for module in yaml.full_load_all(yaml_file):
    ProcessModule(module, output)


def ProcessModule(module, output):
  """Process a register grouping."""
  name = module.pop('name')
  base = module.pop('base')
  stride = module.pop('stride')
  repeat = module.pop('repeat', 1)
  indexing = module.pop('indexing', 1)
  width = module.pop('align', 32)

  output.OpenRegister(name, base, repeat, stride, indexing)
  stride = abs(stride)
  current_offset = 0
  for r in sorted(module.iteritems(), key=lambda x: x[1]['offset']):
    bitfield_name = r[0]
    bitfield_offset = r[1].pop('offset')
    bitfield_repeat = r[1].pop('repeat', 1)
    bitfield_width = r[1].pop('width', width)
    if bitfield_offset != current_offset:
      output.PadRegister(current_offset, bitfield_offset)
    output.OpenBitfield(bitfield_name, bitfield_offset,
                        bitfield_width, bitfield_repeat)

    current_bit = bitfield_width - 1
    for f in sorted([(k, ParseRange(v)) for k, v in r[1].iteritems()],
                    key=lambda x: x[1], reverse=True):
      field_name = f[0]
      if field_name[0].isdigit():
        field_name = '_' + field_name
      bit_start = f[1][1]
      bit_end = f[1][0]
      if bit_end != current_bit:
        output.PadBitfield(bit_end, current_bit)
      output.AddBitfieldMember(field_name, bit_start, bit_end)
      current_bit = bit_start - 1
    if current_bit != -1:
      output.PadBitfield(-1, current_bit)
    output.CloseBitfield()
    current_offset = bitfield_offset + bitfield_width / 8 * bitfield_repeat

  if current_offset != stride:
    output.PadRegister(current_offset, stride)
  output.CloseRegister()


class RegisterOutput(object):
  """Base class for handling output header files."""

  def __init__(self, output_file):
    self.output = open(output_file, 'w')

    rel_path = os.path.relpath(output_file, start=FLAGS.autogen_root)
    self.guard = rel_path.replace(os.path.sep, '_').replace('.', '_').upper()
    self.guard += '_'
    self.output.write(textwrap.dedent('''
      #ifndef {0}
      #define {0}

      '''.format(self.guard)[1:]))

  def __del__(self):
    self.output.write('#endif  // {}\n'.format(self.guard))
    self.output.close()


class StructBasedOutput(RegisterOutput):
  """Output a structure-based register map."""

  def __init__(self, output_file, define_file):
    super(StructBasedOutput, self).__init__(output_file)
    self.register_name = None
    self.register_repeat = 1
    self.bitfield_name = None
    self.bitfield_type = None
    self.bitfield_repeat = 1
    self.output.write('#include <stdint.h>\n\n')
    if define_file:
      rel_path = os.path.relpath(define_file, start=FLAGS.autogen_root)
      self.output.write('#include "{}"\n\n'.format(rel_path))

  def OpenRegister(self, name, base, repeat, stride, indexing):
    """Start a new register entry."""
    self.register_name = name
    self.register_repeat = repeat
    self.typedef_name = string_util.SnakeToCamel(
        name.lower(), validate=False) + 'Registers'

    if repeat == 1:
      name_index = name
      index_str = '0'
    else:
      name_index = name + '(i)'
      if stride < 0:
        if indexing == 0:
          index_str = '-(i)'
        else:
          index_str = '{} - (i)'.format(indexing)
      else:
        if indexing == 0:
          index_str = '(i)'
        else:
          index_str = '(i) - {}'.format(indexing)
    self.output.write(
        '#define {} (((volatile {} *)0x{:08X})[{}])\n'.format(
            name_index, self.typedef_name, base, index_str))
    self.output.write('typedef struct {\n')

  def CloseRegister(self):
    self.output.write('}} {};\n\n'.format(self.typedef_name))

  def PadRegister(self, offset0, offset1):
    self.output.write('  uint8_t __unused_{:#x}_{:#x}[{}];\n'.format(
        offset0, offset1, offset1 - offset0))

  def OpenBitfield(self, name, unused_offset, width, repeat):  # pylint: disable=unused-argument
    """Start a new register bitfield entry."""
    self.bitfield_name = name
    self.bitfield_type = 'uint{}_t'.format(width)
    self.bitfield_repeat = repeat
    self.output.write('  union {}_{} {{\n'.format(self.register_name, name))
    self.output.write('    {} raw;\n'.format(self.bitfield_type))
    self.output.write('    struct {\n')

  def CloseBitfield(self):
    self.output.write('    };\n')
    if self.bitfield_repeat == 1:
      self.output.write('  }} {};\n'.format(self.bitfield_name))
    else:
      self.output.write('  }} {}[{}];\n'.format(
          self.bitfield_name, self.bitfield_repeat))

  def PadBitfield(self, bit0, bit1):
    self.output.write('      {} :{};\n'.format(
        self.bitfield_type, bit1 - bit0))

  def AddBitfieldMember(self, name, bit0, bit1):
    self.output.write('      {} {}:{};\n'.format(
        self.bitfield_type, name, bit1 - bit0 + 1))


class DefineBasedOutput(RegisterOutput):
  """Output a define-based register map."""

  def __init__(self, output_file):
    super(DefineBasedOutput, self).__init__(output_file)
    self.register_name = None
    self.bitfield_name = None

  def OpenRegister(self, name, base, repeat, stride, indexing):
    """Start a new register entry."""
    self.register_name = name
    self.register_base = base
    self.register_repeat = repeat
    self.register_stride = stride
    self.register_indexing = indexing
    self.bitfield_offset = 0

  def CloseRegister(self):
    pass

  def PadRegister(self, offset0, offset1):
    pass

  def OpenBitfield(self, name, offset, width, repeat):
    """Start a new register bitfield entry."""
    self.bitfield_name = name
    self.bitfield_repeat = repeat

    self.output.write('// {} {} register.\n'.format(self.register_name, name))
    for i in range(self.register_repeat):
      if self.register_repeat == 1:
        register_name = self.register_name
      else:
        register_name = self.register_name + str(i + self.register_indexing)
      for j in range(self.bitfield_repeat):
        if self.bitfield_repeat == 1:
          bitfield_name = self.bitfield_name
        else:
          bitfield_name = self.bitfield_name + str(j)

        register_base = self.register_base + self.register_stride * i
        bitfield_base = register_base + offset + width * j / 8
        self.output.write('#define {}_{}_ADDR 0x{:08X}\n'.format(
            register_name, bitfield_name, bitfield_base))

  def CloseBitfield(self):
    self.output.write('\n')

  def PadBitfield(self, bit0, bit1):
    pass

  def AddBitfieldMember(self, name, bit0, bit1):
    mask = (2 << bit1) - (1 << bit0)
    if bit0 == bit1:
      self.output.write('#define {}_{}_{} 0x{:08X}  // Bit {}.\n'.format(
          self.register_name, self.bitfield_name, name, mask, bit0))
    self.output.write('#define {}_{}_{}_MASK 0x{:08X}\n'.format(
        self.register_name, self.bitfield_name, name, mask))
    self.output.write('#define {}_{}_{}_SHIFT {}\n'.format(
        self.register_name, self.bitfield_name, name, bit0))
    self.output.write('#define {}_{}_{}_WIDTH {}\n'.format(
        self.register_name, self.bitfield_name, name, bit1 - bit0 + 1))


if __name__ == '__main__':
  main(sys.argv)
