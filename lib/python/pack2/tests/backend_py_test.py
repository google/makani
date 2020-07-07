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

import binascii
import pickle
import textwrap
import unittest

from makani.lib.python.pack2.tests import test_param
import yaml


class TestBackendPy(unittest.TestCase):
  yaml_data = textwrap.dedent("""\
      !TestConfigParams
        enum_field: kValue3
        uint8_field: 0xa5
        int8_field: -101
        uint16_field: 0xa5a5
        int16_field: -12345
        struct32_field:
          uint32_field: 0xa5a5a5a5
          int32_field: -123456789
        float32_field: 3.14159
        string_field: "test"
        date_field: 0x20151105
      """)
  yaml_data_dict = textwrap.dedent("""\
      a: &A {data}
      b: !TestConfigParams
        TEMPLATE: *A
      """).format(data=yaml_data)

  yaml_array_data = textwrap.dedent("""\
      !ArrayConfigParams
        data:
          - name: "first"
            val: 0x01
          - name: "second"
            val: 0x02
          - name: "third"
            val: 0x03
        udata:
          - 0xde
          - 0xca
          - 0xfb
          - 0xad
      """)
  yaml_array_data_dict = {
      'data': [
          {
              'name': 'first',
              'val': 0x1,
          }, {
              'name': 'second',
              'val': 0x2,
          }, {
              'name': 'third',
              'val': 0x3,
          }],
      'udata': [0xde, 0xca, 0xfb, 0xad],
      }
  yaml_array_data_hex = ('66697273740000000000000000000000'
                         '00000000000000000000000000000001'
                         '7365636f6e6400000000000000000000'
                         '00000000000000000000000000000002'
                         '74686972640000000000000000000000'
                         '00000000000000000000000000000003'
                         'decafbad')

  yaml_deep_array_data = textwrap.dedent("""\
      null_entry: &NULL_ENTRY
        p: -1
        m: [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]
      empty_table: &EMPTY_TABLE
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
        - *NULL_ENTRY
      params: !DeepArrayConfigParams
        e: 1
        x: 0x1000
        s: *EMPTY_TABLE
      """)
  yaml_deep_array_data_dict = {
      'e': 1,
      'x': 0x1000L,
      's': [
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
          {'p': -1, 'm': [0xff, 0xff, 0xff, 0xff, 0xff, 0xff]},
      ]
      }
  yaml_deep_array_data_hex = ('0000000100001000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000'
                              'ffffffffffffffffffff0000')

  yaml_data_unknown_field = textwrap.dedent("""\
      !TestConfigParams
        enum_field: kValue3
        uint8_field: 0xa5
        int8_field: -101
        uint16_field: 0xa5a5
        int16_field: -12345
        struct32_field:
          uint32_field: 0xa5a5a5a5
          int32_field: -123456789
        float32_field: 3.14159
        string_field: "test"
        date_field: 0x20151105
        unknown_field: "hello"
      """)

  def testYamlLoadStore(self):
    param = yaml.safe_load(self.yaml_data)
    out_str = param.ToYaml()
    self.assertEqual(out_str, self.yaml_data)

  def testYamlLoadDict(self):
    params = yaml.safe_load(self.yaml_data_dict)
    out_str = params['a'].ToYaml()
    self.assertEqual(out_str, self.yaml_data)

  def testYamlRef(self):
    params = yaml.safe_load(self.yaml_data_dict)
    out_str = params['b'].ToYaml()
    self.assertEqual(out_str, self.yaml_data)

  def testPackUnpack(self):
    param = yaml.safe_load(self.yaml_data)
    data = param.Pack()
    param2 = test_param.TestConfigParams()
    param2.Unpack(data)
    self.assertEqual(param.ToDict(), param2.ToDict())

  def testPack(self):
    param = yaml.safe_load(self.yaml_data)
    data = param.Pack()
    hex_data = ('03a59b00a5a5cfc7a5a5a5a5f8a432eb40490fd0'
                '746573740000000000000000000000000000000020151105')
    self.assertEqual(binascii.hexlify(data), hex_data)

  def testYamlArrayLoadStore(self):
    param = yaml.safe_load(self.yaml_array_data)
    out_str = param.ToYaml()
    self.assertEqual(out_str, self.yaml_array_data)

  def testArrayPackUnpack(self):
    param = yaml.safe_load(self.yaml_array_data)
    data = param.Pack()
    param2 = test_param.ArrayConfigParams()
    param2.Unpack(data)
    self.assertEqual(param.ToDict(), param2.ToDict())

  def testArrayPack(self):
    param = yaml.safe_load(self.yaml_array_data)
    data = param.Pack()
    self.assertEqual(binascii.hexlify(data), self.yaml_array_data_hex)

  def testArraySetField(self):
    param = test_param.ArrayConfigParams()
    param.SetField('data', self.yaml_array_data_dict['data'])
    param.SetField('udata', self.yaml_array_data_dict['udata'])
    data = param.Pack()
    self.assertEqual(binascii.hexlify(data), self.yaml_array_data_hex)

  def testArrayFromDict(self):
    param = test_param.ArrayConfigParams(self.yaml_array_data_dict)
    data = param.Pack()
    self.assertEqual(binascii.hexlify(data), self.yaml_array_data_hex)

  def testArrayToDict(self):
    param = test_param.ArrayConfigParams(self.yaml_array_data_dict)
    new_dict = param.ToDict()
    pickle1 = pickle.dumps(self.yaml_array_data_dict)
    pickle2 = pickle.dumps(new_dict)
    self.assertEqual(pickle1, pickle2)

  def testArrayCopyFrom(self):
    param1 = test_param.ArrayConfigParams(self.yaml_array_data_dict)
    param2 = test_param.ArrayConfigParams()
    param2.CopyFrom(param1)

    param2.udata[0] = 0x00
    param2.udata[1] = 0x00
    param2.udata[2] = 0x00
    param2.udata[3] = 0x00

    data1 = binascii.hexlify(param1.Pack())
    data2 = binascii.hexlify(param2.Pack())

    self.assertNotEqual(data1, data2)

  def testDeepArrayToDict(self):
    param = yaml.safe_load(self.yaml_deep_array_data)['params']
    new_dict = param.ToDict()
    pickle1 = pickle.dumps(self.yaml_deep_array_data_dict)
    pickle2 = pickle.dumps(new_dict)
    self.assertEqual(pickle1, pickle2)

  def testDeepArrayPackUnpack(self):
    param = yaml.safe_load(self.yaml_deep_array_data)['params']
    data = param.Pack()
    param2 = test_param.DeepArrayConfigParams()
    param2.Unpack(data)
    self.assertEqual(param.ToDict(), param2.ToDict())

  def testDeepArrayPack(self):
    param = yaml.safe_load(self.yaml_deep_array_data)['params']
    data = param.Pack()
    self.assertEqual(binascii.hexlify(data), self.yaml_deep_array_data_hex)

  def testDeepArraySize(self):
    # Make sure that we properly align/pad struct elements of an array.
    param = test_param.DeepArrayConfigParams()
    self.assertEqual(param.size, 128)

  def testYamlUnknownField(self):
    with self.assertRaises(KeyError):
      yaml.safe_load(self.yaml_data_unknown_field)

if __name__ == '__main__':
  unittest.main()
