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

"""Pack2 binary data encoding library.

Metadata file format:
  Metadata files are used to describe the format of a pack2 object.

  Includes
    One pack2 file may include another.  The parser gracefully handled duplicate
    and recursive includes.  This syntax is:
      include "path/to/file.p2";

  Pack2 Objects
    Pack2 objects are much like C structs and can have multiple fields of
    native types, complex types, and arrays of the two.  An object, however can
    not contain another object.  There are currently two object types: header
    and param.

    Examples:
      param SerialParams {
        int32 serial_number;
        date date_of_manufacture;
        string[32] part_number;
      }

      header ParamHeader {
        ParamHeaderVersion param_format_version;
        int32 unused;
        int32 data_length;  // Length of parameter data following header.
        uint32 data_crc;  // Crc32 of the parameter data.
        uint32 version_number;  // Crc32 of the parameter metadata.
      }

  Pack2 Native Types
    Pack2 supports the following native types:
      - int8: Signed 8 bit integer.
      - uint8: Unsigned 8 bit integer.
      - int16: Signed 16 bit integer.
      - uint16: Unsigned 16 bit integer.
      - int32: Signed 32 bit integer.
      - uint32: Unsigned 32 bit integer.
      - float32: 32 bit floating point number.
      - date: Binary Coded Decimal date stored as a uint32.
      - string[size]: A null terminated string of fixed storage size.

  Complex Types
    Struct
      A struct behaves much like a pack2 object except it can be included in
      other structs and objects.  Because it is not a pack2 object it can not be
      used directly for encoding/decoding.

      Example:
        struct pixel {
          uint8 red;
          uint8 green;
          uint8 blue;
        }

    Enum
      An Enum type, much like its C counterpart, is a mapping between semantic
      labels and integral values.  An enum can be 8, 16, or 32 bits wide.

      Example:
        enum8 fruits {
          APPLE = 0,
          ORANGE = 1,
          GRAPE = 3,
        }

    Bitfield
      A bitfield type is a mapping between semantic labels and bit positions.  A
      bitfield can be 8, 16, or 32 bits wide.  Bit ranges are not supported.

      Example:
        bitfield16 errors {
          0: NO_POWER,
          1: ON_FIRE,
          2: APOCALYPSE,
        }

    Scaled Value

      WARNING: This interface is not stable and may change!

      A scaled type consists of an unsigned integer, an float32 offset, and a
      float32 scale.  The data is stored in integer format (uint8, uint16, or
      uint32) and the scale and offset are only kept in the metadata.  This
      type is useful for storing and transmitting raw values that can be
      converted to/from engineering units.  Values should be transformed using
      the following formula:

        scaled_value = raw_value * scale + offset

      TBD: Support signed types.

      Example:
        // Imaginary sensor that reports degrees Fahrenheit.
        scaled16 fahrenheit_reading {
          scale: 0.55556,
          offset: -17.7778,
        }

  Specialized Objects
    A reoccurring pattern is seen where an object is used in multiple places
    with only a single enum field being different.  It is desirable to be able
    to treat these objects as the same while also benefiting from the type
    safety of an enum.  The specialized statement allows for this.  Consider
    the following example:

    Our serial parameter data is described by:
      param SerialParams {
        string[32] serial_number;
        string[32] part_number;
        int32 hardware_revision;
        date date_of_manufacture;
      }

    The hardware revision field's meaning varies depending on which part is
    being described (i.e. AIO board or core switch.)  We'd like
    to both be able to operate on all these generically as well as take
    advantage of the type saftey having the field declared as the appropriate
    enum type.

    The specialized statement supports this by allowing us to declare a sub-type
    while redeclaring an int type fields as an enum type of the same width.

    Example AIO serial params:
      enum32 AioHardwareRevision {
        kAioHardwareRevisionInvalid = -1,  // Use for pre-AIO module boards.
        kAioHardwareRevision011Aa = 0,
        kAioHardwareRevision011Ab = 1,
      }

      specialized(SerialParams) AioSerialParams {
        AioHardwareRevision hardware_revision;
      }

      enum32 CsHardwareRevision {,
        kCsHardwareRevision001Aa = 0,
        kCsHardwareRevision001Ab = 1,
        kCsHardwareRevision001Ac = 2,
      }

      specialized(SerialParams) CsSerialParams {
        CsHardwareRevision hardware_revision;
      }

"""
import gflags

from makani.lib.python.pack2 import backend_py
from makani.lib.python.pack2 import generator
from makani.lib.python.pack2 import parser


_params = {}


def RegisterParam(param):
  _params[param.crc] = param


def LookupParamByCrc(crc):
  return _params.get(crc)


def ImportPack2(source):
  """Dynamically add classes from source.

  ImportPack2() parses source, generates python code for the types in source,
  and inserts them into the parameter database.

  Args:
    source: Pack2 source code to import.
  """
  p = parser.Parser()
  metadata = p.Parse(source)

  backend = backend_py.BackendPy(c_header_path='')
  gen = generator.Generator(backend)
  gen.Generate(metadata)

  py_source = gen.GetSourceString('source')

  # Using exec here is safe as we're only executing code generated from
  # our python backend not arbitrary code.
  exec(py_source)  # pylint: disable=exec-used
