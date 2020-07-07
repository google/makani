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


"""Generates formatting information to be compiled into pcap_to_hdf5.

Generates an HDF5 file with the structure:

  /aio_header                  -- Dataset whose type describes the AioHeader
                                  structure.

  /bad_packet_info             -- Dataset whose type describes the BadPacketInfo
                                  structure (see capture_info.h).

  /capture_header              -- Dataset whose type describes the CaptureHeader
                                  structure (see capture_info.h).

  /info                        -- Dataset whose type describes the LogInfo
                                  structure (see capture_info.h).

  /messages/kMessageType<name> -- Dataset whose type describes the corresponding
                                  message structure.

  /parameters/control_params   -- Dataset whose type describes the ControlParams
                                  structure.

  /parameters/sim_params       -- Dataset whose type describes the SimParams
                                  structure.

  /parameters/sys_params       -- Dataset whose type describes the SystemParams
                                  structure.

Then, generates a C source and header combination so that this files data
can be compiled into the pcap_to_hdf5 binary.
"""

import ctypes
import os
import struct
import sys
import textwrap

import gflags
import h5py
import makani
from makani.avionics.common import build_info_types
from makani.avionics.common import pack_aio_header as aio_header
from makani.avionics.network import aio_message
from makani.control import control_types
from makani.lib.pcap_to_hdf5 import pack_capture_info
from makani.lib.python.autogen import autogen_util
from makani.sim import sim_types
import numpy

gflags.DEFINE_string('autogen_root', makani.HOME,
                     'Root of the source tree for the output files.')

FLAGS = gflags.FLAGS

_ALLOWED_BASIC_CTYPES = [
    ctypes.c_bool,
    ctypes.c_double,
    ctypes.c_float,
    ctypes.c_int16,
    ctypes.c_int32,
    ctypes.c_int64,
    ctypes.c_int8,
    ctypes.c_uint16,
    ctypes.c_uint32,
    ctypes.c_uint64,
    ctypes.c_uint8,
]


class InvalidCtypeException(Exception):
  pass


def _MessageStructToH5T(c_struct):
  """Creates an HDF5 Type object from a C structure."""

  def _MessageStructToDtype(ctype):
    """Construct a numpy dtype from a ctypes type.

    This function exists because numpy's dtype constructor squeezes
    multidimensional arrays and converts ctypes.c_char to length one
    strings (see see b/26933384).

    This function does not respect the underlying ctype's alignment.
    In the context of _MessageStructToH5T this is fine, as we later
    pack the structure.

    Args:
      ctype: Type to convert.

    Returns:
      A numpy.dtype.  This type may not have the correct offsets.

    Raises:
      InvalidCtypeException: if a type is encountered that isn't a
          Structure, Array, c_char, or a member of the _ALLOWED_BASIC_CTYPES.
    """
    if isinstance(ctype, type(ctypes.Structure)):
      names, fields = zip(*autogen_util.GetCFields(ctype))
      field_dtypes = [_MessageStructToDtype(f) for f in fields]
      return numpy.dtype(zip(names, field_dtypes))
    elif type(ctype) is type(ctypes.Array):
      return numpy.dtype((_MessageStructToDtype(getattr(ctype, '_type_')),
                          (getattr(ctype, '_length_'),)))
    elif ctype == ctypes.c_char:
      # Numpy converts ctypes.c_char to 'S1', i.e. a string of length
      # one.  This is generally not desired, and in particular, causes
      # MATLAB to expect scalar characters to be null-terminated strings
      # (see b/26933384).  This branch is added to force character
      # arrays to be regarded as byte arrays.
      return 'u1'
    elif ctype in _ALLOWED_BASIC_CTYPES:
      return numpy.dtype(ctype)
    else:
      raise InvalidCtypeException('Invalid ctypes type.', ctype)

  # Create a compound type with the byte order forced to be big-endian.
  dtype = _MessageStructToDtype(c_struct).newbyteorder('>')
  h5t = h5py.h5t.py_create(dtype)
  h5t.pack()  # Pack the HDF5 type.
  return h5t


def _NativeStructToH5T(ctype):
  """Creates an HDF5 Type object from a ctypes type.

  This function exists as h5t.py_create does not respect field
  alignment, and the conversion of ctypes.c_char to length one strings
  is undesired.

  Args:
    ctype: Type to convert.

  Returns:
    HDF5 type object with correct size and offsets.

  Raises:
    InvalidCtypeException: if a type is encountered that isn't a
        Structure, Array, c_char, or a member of the _ALLOWED_BASIC_CTYPES.
  """
  if isinstance(ctype, type(ctypes.Structure)):
    t = h5py.h5t.create(h5py.h5t.COMPOUND, ctypes.sizeof(ctype))
    for (field_name, field_type) in autogen_util.GetCFields(ctype):
      t.insert(field_name, getattr(ctype, field_name).offset,
               _NativeStructToH5T(field_type))
    return t
  elif type(ctype) is type(ctypes.Array):
    return h5py.h5t.array_create(
        _NativeStructToH5T(getattr(ctype, '_type_')),
        (getattr(ctype, '_length_'),))
  elif ctype == ctypes.c_char:
    # The py_create function, like numpy's dtype constructor, converts
    # characters into strings. This branch is added to force
    # character arrays to be regarded as byte arrays.
    return h5py.h5t.py_create(ctypes.c_uint8)
  elif ctype in _ALLOWED_BASIC_CTYPES:
    return h5py.h5t.py_create(ctype)
  else:
    raise InvalidCtypeException('Invalid ctypes type.', ctype)


def _AddNativeTypeSingleton(loc, name, ctype):
  """Add a dataset to a group encoding a C type including alignment."""
  # The high-level h5py API call create_dataset is very aggressive
  # about ignoring the alignment of the the dtype argument (even when
  # passed an HDF5 type directly), so we use the lower level HDF5 API
  # here.
  h5py.h5d.create(loc.id, name, _NativeStructToH5T(ctype),
                  h5py.h5s.create_simple((1,), (1,)))


def _WriteCSource(dest_directory, file_path):
  """Write the HDF5 format file contents into a C++ header and source file."""
  h_file_path = os.path.join(dest_directory, 'hdf5_format.h')
  with open(os.path.join(FLAGS.autogen_root, h_file_path), 'w') as h_file:
    def_str = h_file_path.replace('/', '_').replace('.', '_').upper() + '_'
    h_file.write(textwrap.dedent(
        """
        #ifndef {0}
        #define {0}

        #include <stdint.h>

        int32_t H5FormatFileImageSize(void);
        uint8_t *H5FormatFileImage(void);

        #endif  // {0}""".format(def_str))[1:])

  with open(os.path.join(FLAGS.autogen_root, dest_directory,
                         'hdf5_format.cc'), 'w') as cc_file:
    with open(file_path, mode='rb') as f:
      file_content = f.read()
      def_data = struct.unpack('B'*len(file_content), file_content)
      data_str = ', '.join(['0x%02X' % v for v in def_data])

      cc_file.write(textwrap.dedent(
          """
          #include "%s"

          #include <stdint.h>

          static uint8_t kH5FormatFileImage[%d] = {
            %s
          };

          int32_t H5FormatFileImageSize(void) {
            return sizeof(kH5FormatFileImage);
          }

          uint8_t *H5FormatFileImage(void) {
            return kH5FormatFileImage;
          }""")[1:] % (h_file_path, len(file_content),
                       '\n  '.join(textwrap.wrap(data_str, 76))))


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  # Parameter structures list.
  parameter_structures = {
      'build_info': build_info_types.BuildInfo,
      'control_params': control_types.ControlParams,
      'sim_params': sim_types.SimParams,
      'system_params': control_types.SystemParams
  }

  # Dictionary mapping message types to (message name, ctypes structure).
  info_map = aio_message.GetMessageInfoMapByNetworkFile()
  message_types = {m.enum_value: (m.enum_name, i.ctype)
                   for m, i in info_map.iteritems() if i}

  dest_directory = 'lib/pcap_to_hdf5'
  file_path = os.path.join(FLAGS.autogen_root, dest_directory, 'format.h5')
  log_file = h5py.File(file_path, 'w')

  # Add parameters group and types.
  parameters = log_file.create_group('parameters')

  for (name, ctype) in parameter_structures.iteritems():
    _AddNativeTypeSingleton(parameters, name, ctype)

  # Add AioHeader type.
  log_file.create_dataset('aio_header', shape=(1,),
                          dtype=_MessageStructToH5T(aio_header.AioHeader))

  _AddNativeTypeSingleton(log_file, 'bad_packet_info',
                          pack_capture_info.BadPacketInfo)

  _AddNativeTypeSingleton(log_file, 'capture_header',
                          pack_capture_info.CaptureHeader)

  _AddNativeTypeSingleton(log_file, 'info',
                          pack_capture_info.LogInfo)

  # Add single element structures to the messages.
  messages = log_file.create_group('messages')
  for msg_type in message_types:
    (message_name, message_struct) = message_types[msg_type]
    messages.create_dataset(message_name, shape=(1,),
                            dtype=_MessageStructToH5T(message_struct))

  log_file.close()

  # Convert the HDF5 file into an array that can be compiled in.
  _WriteCSource(dest_directory, file_path)


if __name__ == '__main__':
  main(sys.argv)
