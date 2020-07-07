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

"""Helper functions for wrapped C code.

Pack() and Unpack() automate boilerplate interactions with the pack module. If
using a message type defined in module path.to.foo that is not itself a pack
module, these functions will attempt to look up path.to.pack_foo.  Practically
speaking, it may be best to avoid this usage altogether. But this use case
involves a subtle complication that's worth documenting/solving explicitly.

EnumHelper centralizes common operations involving wrapped enums.
"""

from __future__ import absolute_import

import ctypes
import importlib
import re

from makani.lib.python.autogen import autogen_util
import six


def _GetPackModule(message_module):
  """Returns the pack module corresponding to a message's module."""

  path_components = message_module.split('.')
  if (len(path_components) <= 1
      or path_components[0] not in ('h2py', 'makani')):
    return None

  if not path_components[-1].startswith('pack_'):
    path_components[-1] = 'pack_' + path_components[-1]

  try:
    return importlib.import_module('makani.' + '.'.join(path_components[1:]))
  except ImportError:
    return None


def Pack(message):
  """Return a packed representation of the message.

  Args:
    message: Message to unpack.

  Returns:
    Byte array (as c_ubyte_Array_N, where N is the packed size) containing the
    packed message. Returns None if packing was unsuccessful.
  """

  # TODO: Make AioClient selective about sending raw bytes, and then
  # make Pack() raise an exception if the message doesn't live in a pack module.
  pack_module = _GetPackModule(type(message).__module__)
  if not pack_module:
    return

  class_name = autogen_util.CStructName(type(message))
  try:
    pack_class = getattr(pack_module, class_name)
    pack_function = getattr(pack_module, 'Pack' + class_name)
    pack_size = getattr(pack_module, 'PACK_' + class_name.upper() + '_SIZE')
  except AttributeError:
    return

  # The pack function expects a reference to the message type defined in its own
  # module. This constructs an appropriate object using the same memory
  # underlying the original message.
  if not isinstance(message, pack_class):
    message = pack_class.from_buffer(message)

  packed = (ctypes.c_uint8 * pack_size)()
  assert pack_function(ctypes.byref(message), 1, packed)
  return packed


class UnpackError(Exception):
  pass


def PackSize(message_type):
  pack_module = _GetPackModule(message_type.__module__)
  if not pack_module:
    return

  class_name = autogen_util.CStructName(message_type)
  try:
    return getattr(pack_module, 'PACK_' + class_name.upper() + '_SIZE')
  except AttributeError:
    return


def Unpack(packed, message_type):
  """Unpack a packed message.

  Args:
    packed: Packed message data in a (ctypes.c_uint8 * N), where N is the pack
       size, or a string of the same length.
    message_type: Class of the unpacked message.

  Returns:
    An instance of type `message_type` populated with the packed data, or
    None if the unpacking was unsuccessful.

  Raises:
    UnpackError: `packed` is of the wrong type.
  """

  pack_module = _GetPackModule(message_type.__module__)
  if not pack_module:
    return

  class_name = autogen_util.CStructName(message_type)
  try:
    pack_class = getattr(pack_module, class_name)
    unpack_function = getattr(pack_module, 'Unpack' + class_name)
    pack_size = getattr(pack_module, 'PACK_' + class_name.upper() + '_SIZE')
  except AttributeError:
    return

  buffer_type = ctypes.c_uint8 * pack_size
  if isinstance(packed, str) and len(packed) == pack_size:
    packed = buffer_type.from_buffer_copy(packed)
  if not isinstance(packed, buffer_type):
    raise UnpackError('Packed message must be of type %s or a string of the '
                      'same length. Actual type is %s.' %
                      (buffer_type.__name__, type(packed)))

  # The pack function expects a reference to the message type defined in its own
  # module.
  message = pack_class()
  assert unpack_function(packed, 1, ctypes.byref(message)) == pack_size
  if pack_class() != message_type:
    message = message_type.from_buffer(message)
  return message


class EnumError(Exception):
  pass


class EnumHelper(object):
  """Helper for imported C enums.

  EnumHelper provides functions to simplify interface to a C enum of the form
      typedef enum {
        kFooTypeValue1,
        kFooTypeValue2,
        ...,
        kFooTypeValueN;
      } FooType;

  Attributes of the enum are referenced as follows:
    value: A numerical value taken by a member of the enum.
    name: The full name of one of the members, e.g. "kFooTypeValue2".
    short name: A member name with the "k<type name>" prefix removed, e.g.
        "Value2".

  The conversion methods of EnumHelper map to one attribute from either of the
  other two attributes.
  """

  def __init__(self, type_name, module, prefix=None, exclude=None):
    if not re.match('([A-Z][a-z0-9]+)+', type_name):
      raise EnumError('Only camel case enums are supported.')

    self._type_name = type_name
    self._prefix = prefix if prefix else 'k' + type_name

    # Collect enum values, dropping the "ForceSigned" element if it exists. We
    # also perform a casual check for a "number of" element, which, if found,
    # is used for a sanity check on the number of enum values.
    #
    # TODO: Consider making the match for num_name more robust.
    self._values_by_name = {}
    num_name_regex = re.compile('kNum' + self._type_name + 'e?s')
    name_force_size_regex = re.compile('k' + self._type_name + 'ForceSize')
    num_name = None
    for name, value in six.iteritems(module.__dict__):
      if name_force_size_regex.match(name):
        continue
      if exclude and name.startswith('k' + exclude):
        continue
      elif name.startswith(self._prefix) and not name.endswith('ForceSigned'):
        self._values_by_name[name] = value
      elif num_name_regex.match(name):
        num_name = name
        count = value

    if num_name is not None:
      assert count == len(self._values_by_name), (
          'Number of values found (%d) does not agree with %s (%d).' %
          (len(self._values_by_name), num_name, count))

    # Populate a value-to-name mapping, and check that each value is unique.
    self._names_by_value = {}
    for name, value in six.iteritems(self._values_by_name):
      assert value not in self._names_by_value, (
          'EnumHelper does not support enums with non-unique values.\n'
          'Value {value:d} was matched twice by prefix {prefix} under names '
          '{name1} and {name2}.').format(value=value, prefix=self._prefix,
                                         name1=self._names_by_value[value],
                                         name2=name)
      self._names_by_value[value] = name

    self._short_names = [self.ShortName(n)
                         for n in sorted(self._names_by_value)]

    # This information helps code generators.
    if hasattr(module, 'H2PY_HEADER_FILE'):
      self._header_file = module.H2PY_HEADER_FILE
    else:
      self._header_file = None

  def __iter__(self):
    return six.iteritems(self._values_by_name)

  def __len__(self):
    return len(self._values_by_name)

  def Value(self, arg):
    """Returns the numerical enum value for a name or short name."""

    value = self._values_by_name.get(arg, None)
    if value is None:
      value = self._values_by_name.get(self._prefix + arg, None)

    if value is None:
      raise EnumError('%s is not a valid name or short name for %s.' %
                      (arg, self._type_name))

    return value

  def __contains__(self, name_or_value):
    if name_or_value in self._names_by_value:
      return True
    elif name_or_value in self._values_by_name:
      return True
    elif name_or_value in self.ShortNames():
      return True
    else:
      return False

  def Name(self, arg):
    """Returns the name corresponding to an enum value or short name."""

    if isinstance(arg, int):
      try:
        return self._names_by_value[arg]
      except KeyError:
        raise EnumError('%d is not a valid value for %s.' %
                        (arg, self._type_name))

    name = self._prefix + arg
    if name in self._values_by_name:
      return name
    else:
      raise EnumError('%s is not a valid short name for %s.' %
                      (arg, self._type_name))

  def ShortName(self, arg):
    """Returns the short name corresponding to a name or enum value."""
    if isinstance(arg, int):
      return self.Name(arg)[len(self._prefix):]

    if arg in self._values_by_name:
      return arg[len(self._prefix):]
    else:
      raise EnumError('%s is not a valid name for %s.' %
                      (arg, self._type_name))

  def ShortNames(self):
    """Returns a list of all ShortNames, sorted by value."""
    return self._short_names

  def TypeName(self):
    return self._type_name

  def HeaderFile(self):
    return self._header_file

  def Names(self):
    """Returns a list of all names, sorted by value."""
    return sorted(list(self._values_by_name.keys()),
                  key=self._values_by_name.get)

  def Values(self):
    """Returns a sorted list of the enum values."""
    return sorted(self._names_by_value.keys())


def CamelToSnake(s):
  """Convert a CamelCaseString to a snake_case_string.

  This keeps all-upper-case acronyms together and groups numerals with their
  leading letters.

  Args:
    s: input string
  Returns:
    the converted string
  """

  assert isinstance(s, (str, six.text_type))
  s = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s)
  return re.sub('([A-Z])([A-Z][a-z])', r'\1_\2', s).lower()


def SnakeToCamel(s):
  """Converts a snake_case_string to CamelCaseString.

  Example:
    stacked_power_sys returns StackedPowerSys
    TimeStamp returns TimeStamp

  Args:
    s: String or Unicode to convert.

  Returns:
    The given string in camel case.
  """

  assert isinstance(s, (str, six.text_type))
  return ''.join(word[0].upper() + word[1:] for word in s.split('_') if word)


def Indent(text, depth=1):
  """Indents a block of text."""
  return ''.join('  ' * depth + l for l in text.splitlines(True))


def GetFieldType(struct, field_name):
  """Return ctype for a field within a ctype structure."""
  for field in struct._fields_:  # pylint: disable=protected-access
    if field[0] == field_name:
      return field[1]
  raise ValueError('Field %s not found in structure' % field_name)


def CopyToEquivalentType(src, new_type):
  """Copies src to an equivalent type.

  This is intented to copy between equivalent Python representations of the same
  underlying C type. Doing so is a workaround for argument type-checks performed
  by h2py-generated function wrappers.

  Args:
    src: The instance of a ctypes structure, or array of structures, to copy.
    new_type: The type to copy to. Must have the same fields.

  Returns:
    An instance of new_type with the contents of src.
  """
  # pylint: disable=protected-access

  # Handle the array case.
  if hasattr(new_type, '_length_'):
    if not hasattr(src, '_length_') or src._length_ != new_type._length_:
      raise TypeError(
          'Source type (%s) must be an array of length %d to match %s.'
          % (type(src), new_type._length_, new_type))
    dest = new_type()
    for i, s in enumerate(src):
      dest[i] = CopyToEquivalentType(s, new_type._type_)
    return dest
  else:
    if hasattr(src, '_length_'):
      raise TypeError('Source type (%s) should not be an array to match %s.'
                      % (type(src), new_type))

  # Handle the single-element case.
  assert src._fields_ == new_type._fields_
  dest = new_type()
  for field, _ in new_type._fields_:
    setattr(dest, field, getattr(src, field))
  return dest
