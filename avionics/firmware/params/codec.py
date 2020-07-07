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

"""Functions to encode and decode parameters in binary and yaml formats."""
import binascii
import bz2

from makani.avionics.firmware.params import param_header
from makani.avionics.firmware.params import param_source_header
from makani.lib.python import pack2
import makani.lib.python.pack2.paramdb  # pylint: disable=unused-import
import yaml

_COMPAT_CRC_MAP = {
    0x02142937: 0x2745885f,  # Legacy SerialParamsV1.
    0x7c8945bf: 0x987a0540,  # Legacy SerialParamsV2.
}


def _DecodeSource(data, offset):
  """Load source binary from data at offset and insert it into the param_db."""
  source_header = param_source_header.ParamSourceHeader()
  source_header.UnpackFrom(data, offset)

  if source_header.magic != param_source_header.ParamSourceHeaderMagic.MAGIC:
    raise ValueError('Source header has bad magic (0x%08x).'
                     % source_header.magic)
  if source_header.encoding != param_source_header.ParamSourceEncoding.BZIP2:
    raise ValueError('Unknown source encoding (0x%08x).'
                     % source_header.encoding)

  source_data_start = offset + source_header.size
  source_data_end = offset + source_header.size + source_header.length

  if source_data_end >= len(data):
    raise ValueError('Data to short (%d) to read %d bytes at 0x%x.'
                     % (len(data), source_data_end - offset, offset))

  source_data = data[source_data_start:source_data_end]
  source_crc = binascii.crc32(source_data) & 0xffffffff

  if source_crc != source_header.crc:
    raise ValueError('Calculated CRC (%08x) does match header value (%08x).'
                     % (source_crc, source_header.crc))
  source = bz2.decompress(source_data)

  pack2.ImportPack2(source)


def DecodeBin(data, read_source=True):
  """Decode binary parameter data."""
  header = param_header.ParamHeader()
  header.Unpack(data)

  if read_source:
    try:
      _DecodeSource(data, header.size + header.data_length)
    except ValueError:
      pass  # If we can't load source, fall back on paramdb.

  # Support select pre-pack2 CRCs.
  if header.version_number in _COMPAT_CRC_MAP:
    header.version_number = _COMPAT_CRC_MAP[header.version_number]

  param_class = pack2.LookupParamByCrc(header.version_number)
  if not param_class:
    raise TypeError("Can't find pack2 type for CRC 0x%08x."
                    % header.version_number)
  param = param_class()
  param.UnpackFrom(data, header.size)
  return param


def DecodeBinFile(bin_file, read_source=None):
  with open(bin_file, 'r') as f:
    return DecodeBin(f.read(), read_source)


def DecodeYaml(yaml_string, yaml_key=None):
  try:
    data = yaml.safe_load(yaml_string)
  except yaml.constructor.ConstructorError as e:
    raise TypeError('%s. Is the pack2 target listed in '
                    '//lib/python/pack2:paramdb?' % e.problem)

  # TODO: Check if data is a dict and if yaml_key is defined.  Error
  # out on mismatch.
  if yaml_key:
    return data[yaml_key]
  else:
    return data


def DecodeYamlFile(yaml_file, yaml_key=None):
  with open(yaml_file, 'r') as f:
    return DecodeYaml(f.read(), yaml_key)


def EncodeBin(param, write_source=True):
  """Encode parameter data as binary data."""
  packed = param.Pack()
  checksum = binascii.crc32(packed) & 0xffffffff

  header = param_header.ParamHeader()
  header.param_format_version = param_header.ParamHeaderVersion.CURRENT
  header.data_length = len(packed)
  header.data_crc = checksum
  header.version_number = param.crc

  param_data = bytearray(header.Pack()) + bytearray(packed)

  if not write_source:
    return param_data

  bzip_source = bz2.compress(param.source)

  source_header = param_source_header.ParamSourceHeader()
  source_header.magic = param_source_header.ParamSourceHeaderMagic.MAGIC
  source_header.encoding = param_source_header.ParamSourceEncoding.BZIP2
  source_header.length = len(bzip_source)
  source_header.crc = binascii.crc32(bzip_source) & 0xffffffff

  return bytearray(param_data + source_header.Pack()) + bytearray(bzip_source)


def EncodeYaml(param, yaml_key=None):
  """Encode parameter data in yaml format."""
  if yaml_key:
    yaml_string = '%s: ' % yaml_key
  else:
    yaml_string = ''
  yaml_string += param.ToYaml()

  return yaml_string
