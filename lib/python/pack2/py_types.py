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

"""A set of helper classes to for use by generated python code."""

import ctypes
import re
import struct

from makani.lib.python.pack2 import metadata

# In order to maintain compatibility with ctypes we need to use some protected
# member of that package.  We disable those lint errors here.
# pylint: disable=protected-access


def _IsCtypeArray(obj):
  # Ctype arrays have a _length_ attr, scalars do not.
  return hasattr(obj, '_length_')


class PackableCType(object):

  def PackInto(self, buff, offset):
    struct.pack_into('>' + self._type_, buff, offset, self.value)

  def UnpackFrom(self, buff, offset):
    self.value = struct.unpack_from('>' + self._type_, buff, offset)[0]


class UInt8(ctypes._SimpleCData, PackableCType):
  _type_ = 'B'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '0x%02x' % self.value

  def __setstate__(self, state):
    if isinstance(state, basestring):
      self.value = int(state, 0)
    else:
      self.value = int(state)


class Int8(ctypes._SimpleCData, PackableCType):
  _type_ = 'b'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '%d' % self.value

  def __setstate__(self, state):
    if isinstance(state, basestring):
      self.value = int(state, 0)
    else:
      self.value = int(state)


class UInt16(ctypes._SimpleCData, PackableCType):
  _type_ = 'H'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '0x%04x' % self.value

  def __setstate__(self, state):
    if isinstance(state, basestring):
      self.value = int(state, 0)
    else:
      self.value = int(state)


class Int16(ctypes._SimpleCData, PackableCType):
  _type_ = 'h'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '%d' % self.value

  def __setstate__(self, state):
    if isinstance(state, basestring):
      self.value = int(state, 0)
    else:
      self.value = int(state)


class UInt32(ctypes._SimpleCData, PackableCType):
  _type_ = 'L'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '0x%08x' % self.value

  def __setstate__(self, state):
    if isinstance(state, basestring):
      self.value = int(state, 0)
    else:
      self.value = int(state)


class Int32(ctypes._SimpleCData, PackableCType):
  _type_ = 'l'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '%d' % self.value

  def __setstate__(self, state):
    if isinstance(state, basestring):
      self.value = int(state, 0)
    else:
      self.value = int(state)


class Float32(ctypes._SimpleCData, PackableCType):
  _type_ = 'f'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '%g' % self.value

  def __setstate__(self, state):
    self.value = float(state)


class Date(ctypes._SimpleCData, PackableCType):
  _type_ = 'L'
  size = struct.calcsize(_type_)

  def __repr__(self):
    return '0x%08x' % self.value

  def __setstate__(self, state):
    if isinstance(state, basestring):
      self.value = int(state, 0)
    else:
      self.value = int(state)


class String(object):
  """Pack2 string field."""

  def __init__(self, size, value):
    self.size = size
    self.value = value

  def __repr__(self):
    return self.value

  def PackInto(self, buff, offset):
    struct.pack_into('%ds' % self.size, buff, offset, self.value)

  def UnpackFrom(self, buff, offset):
    self.value = struct.unpack_from('%ds' % self.size, buff, offset)[0]

  def __setstate__(self, state):
    self.value = state


# Things get pretty gross below.  Most of this comes from maintaining
# compatibility with ctypes.  For more info see _NormalizeType() and
# _ForEachElement().
class Structure(ctypes.Structure):
  """Pack2 structure Python base class."""

  def __init__(self, state=None):
    super(Structure, self).__init__()
    if state:
      self.__setstate__(state)

  def _NormalizeType(self, field):
    # When a field's value is assigned to a ctypes Structure, the field
    # in the python object is replaced by whatever type was in the rvalue
    # or the assignment.  Then, some deep magic in the ctypes library
    # ensures that the correct representation ends up in the in memory
    # representation of the object.  This is great for ctypes, not so great
    # for us.  Here we go through the fields and fix up the types.
    field_name, field_class = field
    field_obj = getattr(self, field_name)

    if field_obj.__class__ == field_class:
      return field_obj

    # Arrays require a bit more massaging when constructing them from native
    # types.
    if _IsCtypeArray(field_class):
      # Arrays of c_char are special in ctypes and are treated as strings.
      if field_class._type_ == ctypes.c_char:
        return String(field_class._length_, field_obj)
      field_obj = field_class(*list(field_obj))
    else:
      field_obj = field_class(field_obj)
    return field_obj

  def _ForEachField(self, func):
    for field in self._fields_:
      field_name, _ = field
      field_obj = self._NormalizeType(field)
      field_obj = func(field_name, field_obj)
      if hasattr(field_obj, 'value'):
        setattr(self, field_name, field_obj.value)
      else:
        setattr(self, field_name, field_obj)

  # More ctypes fun here!  Ctype arrays of the primitive types above
  # (i.e. Uint8) have a bit of a split personality.  While array_obj._type_
  # is the correct type (Uint8,) array_obj[i].__class__ is a native Python
  # type (int.)  This helper lets us work solely in our types.
  def _ForEachElement(self, field_obj, func):
    for i in range(len(field_obj)):
      obj = field_obj[i]
      if obj.__class__ != field_obj._type_:
        obj = field_obj._type_(field_obj[i])
      field_obj[i] = func(i, obj)

  def _AlignElemSize(self, elem):
    if hasattr(elem, 'alignment'):
      return metadata.AlignTo(elem.size, elem.alignment)
    else:
      return elem.size

  def PackInto(self, buff, offset):
    def Work(field_name, field_obj):
      """PackInto Work."""
      field_offset = self._offsets_[field_name]
      if _IsCtypeArray(field_obj):
        def ElementWork(i, elem):
          size = self._AlignElemSize(elem)
          elem.PackInto(buff, offset + field_offset + size * i)
          return elem
        self._ForEachElement(field_obj, ElementWork)
      else:
        field_obj.PackInto(buff, offset + field_offset)
      return field_obj
    self._ForEachField(Work)

  def Pack(self):
    buff = ctypes.create_string_buffer(self.size)
    self.PackInto(buff, 0)
    return buff

  def UnpackFrom(self, buff, offset):
    def Work(field_name, field_obj):
      """UnpackFrom Work."""
      field_offset = self._offsets_[field_name]
      if _IsCtypeArray(field_obj):
        def ElementWork(i, elem):
          size = self._AlignElemSize(elem)
          elem.UnpackFrom(buff, offset + field_offset + size * i)
          return elem
        self._ForEachElement(field_obj, ElementWork)
      else:
        field_obj.UnpackFrom(buff, offset + field_offset)
      return field_obj
    self._ForEachField(Work)

  def Unpack(self, buff):
    self.UnpackFrom(buff, 0)

  def SetField(self, name, value):
    def Work(field_name, field_obj):
      if field_name == name:
        if _IsCtypeArray(field_obj):
          def ElementWork(i, elem):
            elem.__setstate__(value[i])
            return elem
          self._ForEachElement(field_obj, ElementWork)
        else:
          field_obj.__setstate__(value)
      return field_obj
    self._ForEachField(Work)

  def _ObjToDictValue(self, obj):
    if hasattr(obj, 'ToDict'):
      return obj.ToDict()
    elif hasattr(obj, 'value'):
      return obj.value
    elif hasattr(obj, '__repr__'):
      return obj.__repr__()
    else:
      return obj

  def ToDict(self):
    state = {}
    def Work(field_name, field_obj):
      """Convert field to dict value."""
      if _IsCtypeArray(field_obj):
        value = [None] * len(field_obj)
        def ElementWork(i, elem):
          value[i] = self._ObjToDictValue(elem)
          return elem
        self._ForEachElement(field_obj, ElementWork)
        state[field_name] = value
      else:
        state[field_name] = self._ObjToDictValue(field_obj)
      return field_obj
    self._ForEachField(Work)
    return state

  def _FieldYamlValue(self, field_obj, indent):
    if hasattr(field_obj, 'ToYaml'):
      value = field_obj.ToYaml(False, indent + '  ')
    else:
      value = repr(field_obj)

    if isinstance(field_obj, String):
      value = '"' + value + '"'

    return value

  # We do our own YAML output for a couple reasons:
  #   - yaml.YAMLObject is has a metaclass.  This appears incompatible with
  #   ctypes.
  #   - We only want to omit !tags on the toplevel object.
  def ToYaml(self, output_tag=True, indent='  '):
    if output_tag:
      out = ['!' + self.__class__.__name__ + '\n']
    else:
      out = ['\n']

    def Work(field_name, field_obj):
      """Convert a field to YAML."""
      if _IsCtypeArray(field_obj):
        out[0] += '{indent}{field_name}:\n'.format(indent=indent,
                                                   field_name=field_name)
        def ElementWork(i, elem):  # pylint: disable=unused-argument
          value = self._FieldYamlValue(elem, indent + '  ')
          # Structures come with leading and trailing whitespace.  They are
          # not needed in this context
          value = re.sub(r'^[\s\n]*(.*?)[\s\n]*$', r'\1', value,
                         flags=re.DOTALL)
          out[0] += '{indent}  - {value}\n'.format(indent=indent, value=value)
          return elem
        self._ForEachElement(field_obj, ElementWork)
      else:
        value = self._FieldYamlValue(field_obj, indent)
        out[0] += '%s%s:' % (indent, field_name)
        if value.startswith('\n'):
          # Substructures start with a newline and come pre-formatted.
          out[0] += value
        else:
          if value:
            out[0] += ' {}'.format(value)
          out[0] += '\n'
      return field_obj
    self._ForEachField(Work)
    return out[0]

  def CopyFrom(self, orig):
    def Work(field_name, field_obj):
      orig_obj = getattr(orig, field_name)

      if hasattr(field_obj, 'CopyFrom'):
        field_obj.CopyFrom(orig_obj)
      else:
        field_obj = orig_obj

      return field_obj
    self._ForEachField(Work)

  def __reduce__(self):
    return (self.__class__, (self.ToDict(),))

  def __setstate__(self, state):
    field_names = set(state.keys())

    has_template = False
    if 'TEMPLATE' in state:
      self.CopyFrom(state['TEMPLATE'])
      has_template = True
      field_names.remove('TEMPLATE')

    def Work(field_name, field_obj):
      """Set the state of a field."""

      if field_name not in state:
        if has_template:
          return field_obj
        raise KeyError("'%s' not found when setting state for class '%s'"
                       % (field_name, self.__class__.__name__))

      field_names.remove(field_name)

      if _IsCtypeArray(field_obj):
        field_state = state[field_name]
        def ElementWork(i, elem):
          elem.__setstate__(field_state[i])
          return elem

        self._ForEachElement(field_obj, ElementWork)
      else:
        field_obj.__setstate__(state[field_name])
      return field_obj

    self._ForEachField(Work)

    if field_names:
      raise KeyError('Unknown fields %s in state for class %s.'
                     % (list(field_names), self.__class__.__name__))
