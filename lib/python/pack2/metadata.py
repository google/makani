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

"""Pack2 metadata."""

import binascii
import copy
import re


def AlignTo(value, alignment):
  if value % alignment == 0:
    return value
  else:
    return value + alignment - (value % alignment)


def Indent(indent, string):
  s = ''
  for line in string.splitlines(True):
    s += indent + line
  return s


class Type(object):
  """Base class for pack2 types."""

  def __init__(self, name, width):
    self.name = name
    self.width = width
    self.alignment = width
    self.primitive = False
    self.included = False
    self.path = ''

  def __str__(self):
    return self.name

  def Source(self, emitted_types):
    emitted_types.add(self.name)
    return ''


class PrimitiveType(Type):
  """A primitive type.

  A primitive type is one that is natively supported by Pack2.  Supported
  primitive types are:
    - int8
    - uint8
    - int16
    - uint16
    - int32
    - uint32
    - float32
    - date
  """

  def __init__(self, name, width):
    super(self.__class__, self).__init__(name, width)
    self.primitive = True


class ScaledType(Type):
  """A scaled type.

  A scaled type consists of an unsigned integer, an float32 offset, and a
  float32 scale.  The data is stored in integer format (uint8, uint16, or
  uint32) and the scale and offset are only kept in the metadata.  This type is
  useful for storing and transmitting raw values that can be converted to/from
  engineering units.  Values should be transformed using the following formula:
     scaled_value = raw_value * scale + offset

  WARNING: This interface is not stable and may change!
  """

  def __init__(self, name, width, offset, scale):
    if width not in [1, 2, 4]:
      raise ValueError('Scaled width %d invalid.' % width)

    super(self.__class__, self).__init__(name, width)

    self.offset = offset
    self.scale = scale

  def _TypeName(self):
    if self.width == 1:
      return 'scaled8'
    elif self.width == 2:
      return 'scaled16'
    elif self.width == 4:
      return 'scaled32'
    else:
      raise ValueError('Scaled width %d invalid.' % self.width)

  def __str__(self):
    return ('%s %s {\n  offset = %g,\n  scale = %g,\n}'
            % (self._TypeName(), self.name, self.offset, self.scale))

  def Source(self, emitted_types):
    if self.name in emitted_types:
      return ''
    emitted_types.add(self.name)

    return ('%s %s{offset=%g,scale=%g,}'
            % (self._TypeName(), self.name, self.offset, self.scale))


class StructType(Type):
  """A struct type.

  A struct type, much like its C counterpart, has multiple fields,
  each with their own type.
  """

  def __init__(self, name, body):
    super(self.__class__, self).__init__(name, body.GetWidth())
    self.body = body
    self.alignment = 4

  def __str__(self):
    return 'struct %s {\n%s}' % (self.name, str(self.body))

  def Source(self, emitted_types):
    if self.name in emitted_types:
      return ''
    emitted_types.add(self.name)

    metadata = ''
    body = ''
    for f in self.body.fields:
      metadata += f.type_obj.Source(emitted_types)
      body += f.Source()

    metadata += 'struct %s{%s}' % (self.name, body)
    return metadata


class EnumType(Type):
  """An enum type.

  An Enum type, much like its C counterpart, is a mapping between semantic
  labels and integral values.  An enum can be 8, 16, or 32 bits wide.
  """

  def __init__(self, name, width, body):
    if width not in [1, 2, 4]:
      raise ValueError('Enum width %d invalid.' % width)

    super(self.__class__, self).__init__(name, width)
    self.body = body

    bits = width * 8 - 1
    min_limit = -(1 << bits)
    max_limit = (1 << bits) - 1

    for value in self.body.value_map.keys():
      if value < min_limit or value > max_limit:
        raise ValueError('Enum value %d outside allowed ranged (%d-%d.)' %
                         (value, min_limit, max_limit))

  def _TypeName(self):
    if self.width == 1:
      return 'enum8'
    elif self.width == 2:
      return 'enum16'
    elif self.width == 4:
      return 'enum32'
    else:
      raise ValueError('Enum width %d invalid.' % self.width)

  def __str__(self):
    return '%s %s {\n%s}' % (self._TypeName(), self.name, str(self.body))

  def Source(self, emitted_types):
    if self.name in emitted_types:
      return ''
    emitted_types.add(self.name)
    return '%s %s{%s}' % (self._TypeName(), self.name, self.body.Source())


class BitfieldType(Type):
  """A bitfield type.

  A bitfield type is a mapping between semantic labels and bit positions.  A
  bitfield can be 8, 16, or 32 bits wide.
  """

  def __init__(self, name, width, body):
    if width not in [1, 2, 4]:
      raise ValueError('Bitfield width %d invalid.' % width)

    bit_width = width * 8
    if body.max_bit >= bit_width:
      raise ValueError('Bitfield %s width %d not large enough to hold flags.'
                       % (name, width))

    super(self.__class__, self).__init__(name, width)
    self.body = body

  def _TypeName(self):
    if self.width == 1:
      return 'bitfield8'
    elif self.width == 2:
      return 'bitfield16'
    elif self.width == 4:
      return 'bitfield32'
    else:
      raise ValueError('Bitfield width %d invalid.' % self.width)

  def __str__(self):
    return '%s %s {\n%s}' % (self._TypeName(), self.name, str(self.body))

  def Source(self, emitted_types):
    if self.name in emitted_types:
      return ''
    emitted_types.add(self.name)
    return '%s %s{%s}' % (self._TypeName(), self.name, self.body.Source())


class StringType(Type):
  """A fixed size string type."""

  def __init__(self, length):
    super(self.__class__, self).__init__('string', length)
    self.alignment = 1

  def __str__(self):
    return 'string[%d]' % (self.width)

  def Source(self, emitted_types):
    return ''


class StructField(object):
  """A field in struct type.

  Args:
    type_obj:  A Type object representing the type of the field.
    name: The name of the field.
    extent: Number of elements in the field. 1 == Scalar. >1 == Array.
  """

  def __init__(self, type_obj, name, extent=1):
    if extent < 1:
      raise ValueError('Field extent %d is less than one.' % extent)

    self.type_obj = type_obj
    self.name = name
    self.offset = -1
    self.extent = extent

  def __str__(self):
    if self.extent == 1:
      return '%s %s;  // offset: %d' % (self.type_obj, self.name, self.offset)
    else:
      return '%s %s[%d];  // offset: %d' % (self.type_obj, self.name,
                                            self.extent, self.offset)

  def Source(self):
    if self.type_obj.name == 'string':
      return 'string[%d] %s;' % (self.type_obj.width, self.name)
    elif self.extent == 1:
      return '%s %s;' % (self.type_obj.name, self.name)
    else:
      return '%s %s[%d];' % (self.type_obj.name, self.name, self.extent)


class StructBody(object):
  """The body of a struct-like object.

  This objects wraps the list of fields in a struct-like object.  It is used by
  struct, header, and param statements.
  """

  def __init__(self):
    self.fields = []
    self.width = 0
    self.name_map = {}

  def __getitem__(self, n):
    return self.fields[n]

  def __len__(self):
    return len(self.fields)

  def __str__(self):
    s = ''
    for f in self.fields:
      s += Indent('  ', str(f) + '\n')
    return s

  def AddField(self, field):
    """Adds a field to the struct body."""
    if field.name in self.name_map:
      raise SyntaxError('Field name \'%s\' already declared.' % field.name)

    field_alignment = field.type_obj.alignment
    field.offset = AlignTo(self.width, field_alignment)
    field_width = AlignTo(field.type_obj.width, field_alignment)

    self.width = field.offset + field_width * field.extent
    self.fields.append(field)

    self.name_map[field.name] = field

  def GetWidth(self):
    return self.width


class EnumBody(object):
  """The body of an enum statement."""

  def __init__(self):
    self.name_map = {}
    self.value_map = {}

  def AddValue(self, name, value):
    if name in self.name_map:
      raise SyntaxError('Enum value name %s already declared.' % name)

    if value in self.value_map:
      raise SyntaxError('Enum value %d already defined.' % value)

    self.name_map[name] = value
    self.value_map[value] = name

  def __str__(self):
    s = ''
    for value in sorted(self.value_map):
      s += '  %s = %d,\n' % (self.value_map[value], value)
    return s

  def Source(self):
    s = ''
    for value in sorted(self.value_map):
      s += '%s=%d,' % (self.value_map[value], value)
    return s


class BitfieldBody(object):
  """The body of an bitfield statement."""

  def __init__(self):
    self.max_bit = -1
    self.label_map = {}
    self.bit_map = {}

  def AddFlag(self, label, bit):
    if label in self.label_map:
      raise SyntaxError('Bitfield flag %s already declared.' % label)

    if bit in self.bit_map:
      raise SyntaxError('Bitfield bit %d already defined.' % bit)

    self.label_map[label] = bit
    self.bit_map[bit] = label
    if bit > self.max_bit:
      self.max_bit = bit

  def __str__(self):
    s = ''
    for bit in sorted(self.bit_map):
      s += '  %d: %s,\n' % (bit, self.bit_map[bit])
    return s

  def Source(self):
    s = ''
    for bit in sorted(self.bit_map):
      s += '%d:%s,' % (bit, self.bit_map[bit])
    return s


class Pack2Object(Type):
  """Base class for Pack2 objects."""

  def __init__(self, type_name, name, body):
    super(Pack2Object, self).__init__(name, body.GetWidth())
    self.type_name = type_name
    self.body = body
    self.alignment = 4
    self.forced_crc = None
    self.fields = {}

  def __str__(self):
    return '%s %s {\n%s}\n' % (self.type_name, self.name, self.body)

  def Source(self):
    emitted_types = set()

    metadata = ''
    body = ''
    for f in self.body.fields:
      metadata += f.type_obj.Source(emitted_types)
      body += f.Source()

    metadata += '%s %s{%s}' % (self.type_name, self.name, body)
    return metadata

  def SpecializeField(self, field):
    """Apply specialized filed to self."""

    if field.name not in self.body.name_map:
      raise ValueError('Parent type has not field named %s.' % field.name)

    parent_field = self.body.name_map[field.name]

    if parent_field.type_obj.name not in ['int8', 'int16', 'int32']:
      raise ValueError('Field %s type %s can not be specialized.'
                       % (field.name, parent_field.type_obj.name))

    if parent_field.type_obj.width != field.type_obj.width:
      raise ValueError("Field %s width (%d) does not match parent's width (%d.)"
                       % (field.name, field.type_obj.width,
                          parent_field.type_obj.width))

    if parent_field.extent != 1:
      raise ValueError('Parent field %s has an extent greater than 1 (%d.)'
                       % (field.name, parent_field.extent))

    if field.extent != 1:
      raise ValueError('Field %s has an extent greater than 1 (%d.)'
                       % (field.name, field.extent))

    parent_field.type_obj = field.type_obj

  def Specialize(self, name, body):
    """Specialized a pack2 object.

    Args:
      name: Name of the new object.
      body: StructBody containing the new fields to specialize.

    Returns:
      A new object with the specialization applied.

    Specialize creates a copy of this pack2 object.  It then replaces fields in
    that object with fields in body of the same name.  Only int fields may be
    replaced and only enum fields may be listed in body.
    """

    new_obj = copy.deepcopy(self)
    new_obj.name = name
    new_obj.included = False
    new_obj.path = ''
    new_obj.forced_crc = self.Crc32()

    for field in body.fields:
      new_obj.SpecializeField(field)

    return new_obj

  def Crc32(self):
    if self.forced_crc:
      return self.forced_crc
    else:
      return binascii.crc32(str(self)) & 0xffffffff


class Param(Pack2Object):
  """A param object."""

  def __init__(self, name, body):
    super(self.__class__, self).__init__('param', name, body)


class Header(Pack2Object):
  """A header object."""

  def __init__(self, name, body):
    super(self.__class__, self).__init__('header', name, body)


class Metadata(object):
  """A database of Pack2 Metadata."""

  def __init__(self):
    # Types is a bit of a misnomer here as it contains both Types and Objects.
    self.includes = []
    self.types = []
    self.type_map = {}

    self.backref_re = re.compile(r'^\.\./|/\.\./')
    self.current_dir_re = re.compile(r'^\./|/\./')

    # Seed metadata with primitive types.
    self.AddType(PrimitiveType('int8', 1))
    self.AddType(PrimitiveType('uint8', 1))
    self.AddType(PrimitiveType('int16', 2))
    self.AddType(PrimitiveType('uint16', 2))
    self.AddType(PrimitiveType('int32', 4))
    self.AddType(PrimitiveType('uint32', 4))
    self.AddType(PrimitiveType('float32', 4))
    self.AddType(PrimitiveType('date', 4))

  def __str__(self):
    string = ''

    for type_obj in self.types:
      if issubclass(type_obj.__class__, Pack2Object):
        string += str(type_obj)

    return string

  def AddType(self, type_obj):
    if type_obj.name in self.type_map:
      raise SyntaxError("Type '%s' redefined." % type_obj.name)

    self.type_map[type_obj.name] = type_obj
    self.types.append(type_obj)

  def AddInclude(self, path, metadata):
    """Add metadata from an included file."""

    if self.backref_re.search(path):
      raise ValueError("Include path %s.p2 contains a '..' directory reference."
                       % path)
    if self.current_dir_re.search(path):
      raise ValueError("Include path %s.p2 contains a '.' directory reference."
                       % path)
    if path[0] == '/':
      raise ValueError('Include path %s.p2 is an absolute path.'
                       % path)

    self.includes.append(path)
    for type_obj in metadata.types:
      # Don't import primitive types.
      if type_obj.primitive:
        continue

      type_obj.path = path
      type_obj.included = True
      self.AddType(type_obj)
