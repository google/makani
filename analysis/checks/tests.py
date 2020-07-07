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

import itertools
import json
import os
import string
import tempfile
import unittest

import jsmin
import makani
from makani.analysis.checks import base_check
from makani.analysis.checks import check_range
from makani.analysis.checks import gradebook
from makani.analysis.checks import gradebook_base_check
from makani.analysis.checks import log_util
from makani.gs.monitor2.apps.layout import stoplights
from makani.gs.monitor2.apps.receiver import aio_util
from makani.gs.monitor2.project import settings
from makani.lib.python import struct_tree
from makani.lib.python.h5_utils import h5_io
import mock
import numpy


_GRADEBOOK_FILE = os.path.join(
    makani.HOME, 'gs/monitor2/apps/plugins/layouts/gradebook.json')


def _Multiply(v, multiplier):
  return v * multiplier


def IsDevicePopulated(populated, device_id):  # pylint: disable=unused-argument
  return True


class PatchCheckers(object):

  def __init__(self):
    self._patchers = [
        mock.patch(
            ('makani.analysis.checks.avionics_util.'
             'IsDevicePopulated'),
            IsDevicePopulated),
    ]

  def __enter__(self):
    for p in self._patchers:
      p.start()

  def __exit__(self, exc_type, exc_value, traceback):
    for p in self._patchers:
      p.stop()


class TestAnalysis(unittest.TestCase):
  """Test utilities in the analysis module."""

  def _GradebookFromString(self, gradebook_string):
    return gradebook.Gradebook(json.loads(jsmin.jsmin(gradebook_string)),
                               settings.NETWORK_YAML)

  def testSeparateFieldIndex(self):
    segments, slices = base_check.SeparateFieldIndex('a[:].b[0].c')
    self.assertEqual(segments, ['a', 'b', 'c'])
    self.assertEqual(slices, [':', '0'])

    segments, slices = base_check.SeparateFieldIndex('ab[:][0].c')
    self.assertEqual(segments, ['ab', 'c'])
    self.assertEqual(slices, [':', '0'])

    segments, slices = base_check.SeparateFieldIndex('a.bc[:][0]')
    self.assertEqual(segments, ['a', 'bc'])
    self.assertEqual(slices, [':', '0'])

  def testIndicesToOrderAndDedup(self):
    sequence = numpy.array([1, 10, 10, 2, 2, 3, 4, 5, 4, 3, 7, 7, 7, 8, 8, 1])
    prefix_template = string.Template('$message_type.$aio_node')
    data = {'a': {'b': {'aio_header': {'sequence': sequence}}}}

    indices = log_util.MessageOrderedIndices(
        struct_tree.StructTree(data), 'a', 'b', prefix_template, wraparound=10)
    reference = numpy.array([1, 0, 3, 5, 6, 7, 10, 13, 15])
    output = numpy.array([10, 1, 2, 3, 4, 5, 7, 8, 1])
    self.assertTrue(numpy.array_equal(indices, reference))
    self.assertTrue(numpy.array_equal(sequence[indices], output))

    reference = numpy.array([1, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1])
    bitmask = log_util.MessageDedupSelection(
        struct_tree.StructTree(data), 'a', 'b', prefix_template, wraparound=10)
    self.assertTrue(numpy.array_equal(bitmask, reference))

  def testRangeClasses(self):
    array = numpy.array([0, 1, 2, 3, 4])
    crange = check_range.Singleton(1)
    self.assertIn(1, crange)
    self.assertNotIn(3, crange)
    self.assertIn(numpy.array([1, 1]), crange)
    self.assertNotIn(numpy.array([1, 2]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 1, 0, 0, 0])))
    self.assertEqual(str(crange), '1')

    crange = check_range.Container({1, 'a'})
    self.assertIn(1, crange)
    self.assertIn('a', crange)
    self.assertNotIn(3, crange)
    self.assertNotIn('b', crange)
    self.assertIn(numpy.array([1, 1]), crange)
    self.assertNotIn(numpy.array([1, 2]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 1, 0, 0, 0])))
    self.assertEqual(str(crange), "{1, 'a'}")

    crange = check_range.Container({1: 2, 'a': 'b'})
    self.assertIn(1, crange)
    self.assertIn('a', crange)
    self.assertNotIn(3, crange)
    self.assertNotIn('b', crange)
    self.assertIn(numpy.array([1, 1]), crange)
    self.assertNotIn(numpy.array([1, 2]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 1, 0, 0, 0])))
    self.assertEqual(str(crange), "{1, 'a'}")

    crange = check_range.Container([1, 2, 'a', 'b'])
    self.assertIn(1, crange)
    self.assertIn('a', crange)
    self.assertNotIn(3, crange)
    self.assertNotIn('c', crange)
    self.assertIn(numpy.array([1, 2]), crange)
    self.assertNotIn(numpy.array([1, 3]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 1, 1, 0, 0])))
    self.assertEqual(str(crange), "{1, 2, 'a', 'b'}")

    crange = check_range.Interval([1, 3])
    self.assertIn(1, crange)
    self.assertIn(3, crange)
    self.assertIn(2, crange)
    self.assertNotIn(5, crange)
    self.assertNotIn(0, crange)
    self.assertIn(numpy.array([1, 2, 3]), crange)
    self.assertNotIn(numpy.array([1, 2, 3, 4]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 1, 1, 1, 0])))
    self.assertEqual(str(crange), '[1, 3]')

    crange = check_range.Interval([1, 3], inclusiveness=[False, False])
    self.assertIn(1.1, crange)
    self.assertIn(2.9, crange)
    self.assertIn(2, crange)
    self.assertNotIn(1, crange)
    self.assertNotIn(3, crange)
    self.assertIn(numpy.array([2, 2]), crange)
    self.assertNotIn(numpy.array([1, 2, 3]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 0, 1, 0, 0])))
    self.assertEqual(str(crange), '(1, 3)')

    crange = check_range.Interval([3, None])
    self.assertIn(3, crange)
    self.assertNotIn(2, crange)
    self.assertIn(numpy.array([3, 4]), crange)
    self.assertNotIn(numpy.array([1, 2, 3]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 0, 0, 1, 1])))
    self.assertEqual(str(crange), '[3, inf]')

    crange = check_range.Interval([None, 3])
    self.assertIn(3, crange)
    self.assertNotIn(4, crange)
    self.assertIn(numpy.array([2, 3]), crange)
    self.assertNotIn(numpy.array([2, 3, 4]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([1, 1, 1, 1, 0])))
    self.assertEqual(str(crange), '[-inf, 3]')

    crange = check_range.Interval([None, None])
    self.assertIn(4, crange)
    self.assertIn(numpy.array([2, 2]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([1, 1, 1, 1, 1])))
    self.assertEqual(str(crange), '[-inf, inf]')

    crange = check_range.Container([])
    self.assertNotIn(4, crange)
    self.assertNotIn(numpy.array([2, 2]), crange)
    self.assertTrue(numpy.array_equal(
        crange.Select(array), numpy.array([0, 0, 0, 0, 0])))
    self.assertEqual(str(crange), '{}')

    with self.assertRaises(AssertionError):
      check_range.Singleton(None)

  def testRangeChecker(self):
    array = numpy.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])

    cranges = check_range.RangeChecker([{3, 4}, 5, [7], [8, None], [None, 1]])
    self.assertIn(0, cranges)
    self.assertIn(1, cranges)
    self.assertNotIn(2, cranges)
    self.assertIn(3, cranges)
    self.assertIn(4, cranges)
    self.assertIn(5, cranges)
    self.assertNotIn(6, cranges)
    self.assertIn(7, cranges)
    self.assertIn(8, cranges)
    self.assertIn(9, cranges)
    self.assertTrue(numpy.array_equal(
        cranges.Select(array), numpy.array([1, 1, 0, 1, 1, 1, 0, 1, 1, 1])))

    cranges = check_range.RangeChecker([])
    self.assertNotIn(4, cranges)
    self.assertTrue(numpy.array_equal(
        cranges.Select(array), numpy.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0])))

    with self.assertRaises(TypeError):
      check_range.RangeChecker(None)

    cranges = check_range.RangeChecker([[None, None]])
    self.assertIn(123, cranges)
    self.assertIn('a', cranges)

    cranges = check_range.RangeChecker([None])
    self.assertNotIn(123, cranges)
    self.assertNotIn('a', cranges)

  def testRolling(self):
    array = numpy.array([0, 1, 2, 3, 4, 5, 6])

    rolled = numpy.array([
        [0, 1, 2, 3],
        [2, 3, 4, 5],
    ])
    self.assertTrue(numpy.array_equal(log_util.Rolling(array, 4, 2), rolled))

    rolled = numpy.array([
        [1, 2, 3, 4],
        [3, 4, 5, 6],
    ])
    self.assertTrue(numpy.array_equal(log_util.Rolling(array, 4, 2, False),
                                      rolled))

  def testAioCheckByRange(self):
    normal_ranges = check_range.RangeChecker([[5, 8], [15, 18]])
    warning_ranges = check_range.RangeChecker([0, [3, 20]])

    self.assertEqual(
        aio_util.CheckByRange(
            5, normal_ranges, warning_ranges, 'test')[0]['stoplight'],
        stoplights.STOPLIGHT_NORMAL)

    self.assertEqual(
        aio_util.CheckByRange(
            0, normal_ranges, warning_ranges, 'test')[0]['stoplight'],
        stoplights.STOPLIGHT_WARNING)

    self.assertEqual(
        aio_util.CheckByRange(
            1, normal_ranges, warning_ranges, 'test')[0]['stoplight'],
        stoplights.STOPLIGHT_ERROR)

    array = numpy.array([5, 15, 18])
    self.assertEqual(
        aio_util.CheckByRange(
            array, normal_ranges, warning_ranges, '')[0]['stoplight'],
        stoplights.STOPLIGHT_NORMAL)

    array = numpy.array([0, 5, 18])
    self.assertEqual(
        aio_util.CheckByRange(
            array, normal_ranges, warning_ranges, '')[0]['stoplight'],
        stoplights.STOPLIGHT_WARNING)

    array = numpy.array([1, 5, 10, 18])
    self.assertEqual(
        aio_util.CheckByRange(
            array, normal_ranges, warning_ranges, '')[0]['stoplight'],
        stoplights.STOPLIGHT_ERROR)

  def testLogCheckByRange(self):
    normal_ranges = check_range.RangeChecker([[5, 8], [15, 18]])
    warning_ranges = check_range.RangeChecker([0, [3, 20]])

    array = numpy.array([0, 1, 5, 10, 18])
    results = log_util.CheckByRange(
        array, normal_ranges, warning_ranges, min_gap=1000)

    self.assertEqual(
        results['warning'],
        {'total': 5, 'count': 2, 'range': [0, 10], 'sections': [(0, 4)],
         'expecting': '[[5, 8], [15, 18]]'})

    self.assertEqual(
        results['error'],
        {'total': 5, 'count': 1, 'range': [1, 1], 'sections': [(1, 2)],
         'expecting': '[0, [3, 20]]'})

  def _GetValidSampleGradebook(self):
    gradebook_string = """
    {
        "imports": {
            "motor_thermal_types":
                "makani.avionics.common.motor_thermal_types",
            "analysis_tests":
                "makani.analysis.checks.tests"
        },
        "checks": {
            "ServoStatus": {
                "ServoA4": {
                    "angle_desired": {
                        "normal_ranges": [[0, 90]],
                        "warning_ranges": "any"
                    }
                }
            },
            "MotorStatus": {
                "(.*)": {
                    "temps[motor_thermal_types.kMotorThermalChannelBoard]": {
                        "normal_ranges": [[60, 70], 0],
                        "warning_ranges": "any",
                        // Callback to preprocess the data.
                        "callback": "analysis_tests._Multiply",
                        "callback_args": [2],
                        "name": "Board Temperature"
                    },
                    "temps[motor_thermal_types.kMotorThermalChannelControllerAir]": {
                        "normal_ranges": [[60, 70], 0],
                        "warning_ranges": [[60, 90]],
                        // Callback to preprocess the data.
                        "callback": "analysis_tests._Multiply",
                        "callback_args": [2]
                    }
                }
            }
        }
    }
    """
    return self._GradebookFromString(gradebook_string)

  def testGradebook(self):
    book = self._GetValidSampleGradebook()
    criteria = book.GetCriteria(['ServoStatus', 'ServoA4', 'angle_desired'])
    self.assertIn(80, criteria.normal_ranges)
    self.assertNotIn(100, criteria.normal_ranges)
    self.assertIn(100, criteria.warning_ranges)

    criteria = book.GetCriteria(['MotorStatus', 'MotorPti', 'temps[0]'])
    self.assertIn(0, criteria.normal_ranges)
    self.assertIn(60, criteria.normal_ranges)
    self.assertIn(70, criteria.normal_ranges)
    self.assertNotIn(20, criteria.normal_ranges)

    field_map = book.GetFieldMap('$message_type.')
    self.assertEqual(set(field_map.keys()), {'ServoStatus', 'MotorStatus'})
    self.assertEqual(set(field_map['MotorStatus'].keys()), {
        ''.join(x)
        for x in itertools.product(('DynoMotor', 'Motor'), 'PS', 'bt', 'io')})

    self.assertEqual(field_map['MotorStatus']['MotorPti'],
                     {
                         'temps[0]': ['MotorStatus.temps[0]'],
                         'temps[1]': ['MotorStatus.temps[1]'],
                     })

  def testGradebookOnAio(self):
    book = self._GetValidSampleGradebook()
    messages = struct_tree.StructTree({
        'MotorStatus': {
            'MotorPti': {
                'temps': [30, 40, 50, 60]
            }
        }
    }, True)

    checks = gradebook_base_check.GradebookChecks()
    checks.Initialize(book, for_log=False, use_full_name=False)

    for item in checks.List():
      item.Check(*item.Populate(messages))
      if item.FieldIndex() == 'MotorStatus.MotorPti.temps[0]':
        self.assertEqual(item.GetResults(), [{
            'name': 'Value',
            'value': 60,
            'stoplight': 3
        }])
      elif item.FieldIndex() == 'MotorStatus.MotorPti.temps[1]':
        self.assertEqual(item.GetResults(), [{
            'name': 'Value',
            'value': 80,
            'stoplight': 2
        }])
      else:
        self.assertFalse(item.GetResults())

  def testGradebookOnLog(self):
    book = self._GetValidSampleGradebook()
    message = numpy.array(
        [(((30., 31.),), (1,)),
         (((50., 51.),), (3,)),
         (((40., 41.),), (2,)),
         (((60., 61.),), (4,))],
        dtype=[
            ('message', [('temps', 'f', (2,))]),
            ('aio_header', [('sequence', '>u2'),])
        ]
    )
    dataset = {
        'messages': {
            'kAioNodeMotorPti': {
                'kMessageTypeMotorStatus': message
            }
        }
    }
    with tempfile.NamedTemporaryFile() as temp_hdf5:
      h5_io.H5Dump(temp_hdf5.name, dataset)

      checks = gradebook_base_check.GradebookChecks()
      checks.Initialize(book, for_log=True, use_full_name=True)

      for item in checks.List():
        item.Check(*item.Populate(struct_tree.StructTree(temp_hdf5.name)))
        # After ordering the sequence and applying the callback, temps[0]
        # becomes [60.0, 80.0, 100.0, 120.0] and temps[1] becomes
        # [62.0, 82.0, 102.0, 122.0].
        if item.FieldIndex() == 'MotorStatus.MotorPti.temps[0]':
          self.assertEqual(item.GetResults(), {
              'MotorPti.Board Temperature (Value)': {
                  'warning': {'count': 3, 'range': [80.0, 120.0], 'total': 4,
                              'sections': [(1, 4)],
                              'expecting': '[[60, 70], 0]'}
              }
          })
          self.assertTrue(item.HasWarnings())
          self.assertFalse(item.HasErrors())
        elif item.FieldIndex() == 'MotorStatus.MotorPti.temps[1]':
          self.assertEqual(item.GetResults(), {
              'MotorPti.temps[1] (Value)': {
                  'warning': {'count': 1, 'range': [82.0, 82.0], 'total': 4,
                              'sections': [(1, 2)],
                              'expecting': '[[60, 70], 0]'},
                  'error': {'count': 2, 'range': [102.0, 122.0], 'total': 4,
                            'sections': [(2, 4)],
                            'expecting': '[[60, 90]]'}
              }
          })
          self.assertTrue(item.HasWarnings())
          self.assertTrue(item.HasErrors())
        else:
          self.assertFalse(item.GetResults())
          self.assertFalse(item.HasWarnings())
          self.assertFalse(item.HasErrors())

  def testBadGradebook(self):
    gradebook_string = """
    {
        "imports": {
            "bad_module": "bad_package.bad_module"
        }
    }
    """

    with self.assertRaises(gradebook.GradebookParserError):
      self._GradebookFromString(gradebook_string)

    gradebook_string = """
    {
        "checks": {
            "BadMessage": {}
        }
    }
    """
    with self.assertRaises(gradebook.GradebookParserError):
      self._GradebookFromString(gradebook_string)

    gradebook_string = """
    {
        "imports": {
            "analysis_tests":
                "makani.analysis.checks.tests"
        },
        "checks": {
            "ServoStatus": {
                "ServoA4": {
                    "angle_bias": {
                        "normal_ranges": [[0, 90]],
                        "warning_ranges": "any",
                        // A missing function from a module.
                        "callback": "analysis_tests._Divide"
                    }
                }
            }
        }
    }
    """
    with self.assertRaises(gradebook.GradebookParserError):
      self._GradebookFromString(gradebook_string)

    gradebook_string = """
    {
        "checks": {
            "ServoStatus": {
                "ServoA4": {
                    "angle_measured": {
                        "normal_ranges": [[0, 90]],
                        "warning_ranges": "any",
                        // Bad callback format.
                        "callback": "analysis_tests___Divide"
                    }
                }
            }
        }
    }
    """
    with self.assertRaises(gradebook.GradebookParserError):
      self._GradebookFromString(gradebook_string)

    gradebook_string = """
    {
        "checks": {
            "ServoStatus": {
                "ServoA4": {
                    "angle_estimated": {
                        "normal_ranges": [[0, 90]],
                        "warning_ranges": "any",
                        // Missing module.
                        "callback": "missing_module._Add"
                    }
                }
            }
        }
    }
    """
    with self.assertRaises(gradebook.GradebookParserError):
      self._GradebookFromString(gradebook_string)

    gradebook_string = """
    {
        "checks": {
            "ServoStatus": {
                "ServoA4": {
                    "angle_estimated": {
                        // Missing normal_ranges.
                        "warning_ranges": [[0, 90]]
                    }
                }
            }
        }
    }
    """
    with self.assertRaises(gradebook.GradebookParserError):
      self._GradebookFromString(gradebook_string)

    gradebook_string = 'bad json'
    with self.assertRaises(ValueError):
      self._GradebookFromString(gradebook_string)

  def testRealGradebook(self):
    with open(_GRADEBOOK_FILE) as fp:
      definitions = json.loads(jsmin.jsmin(fp.read()))
      gradebook.Gradebook(definitions, settings.NETWORK_YAML)


class DerivedCheck(base_check.BaseCheckItem):

  def _RegisterInputs(self):
    return []


class DerivedCheckWithInit(base_check.FilteredCheckItem):

  @base_check.RegisterSpecs
  def __init__(self, find_history, history_len=12):
    super(DerivedCheckWithInit, self).__init__(
        find_history, history_len, 3, [16, 18])

  def _RegisterInputs(self):
    return []


class BadDerivedCheck(base_check.FilteredCheckItem):

  # A derived check with the decorator missing.
  def __init__(self, find_history, history_len=12):
    super(BadDerivedCheck, self).__init__(
        find_history, history_len, 3, [16, 18])

  def _RegisterInputs(self):
    return []


class TestCheckSpecs(unittest.TestCase):

  def testSpecs(self):
    normal_ranges = [[1, 3]]
    warning_ranges = [[0, 10]]
    derived_check = DerivedCheck(
        for_log=True, normal_ranges=check_range.RangeChecker(normal_ranges),
        warning_ranges=warning_ranges)
    derived_init_check = DerivedCheckWithInit(True, history_len=30)
    derived_init_check_default = DerivedCheckWithInit(True)
    self.assertEqual(
        derived_check.GetDeterministicSpecs(), (
            'makani.analysis.checks.tests.DerivedCheck', {
                'for_log': True,
                'name': None,
                'sort_by_sequence': True,
                'normal_ranges': normal_ranges,
                'warning_ranges': warning_ranges
            }
        )
    )
    self.assertEqual(
        derived_init_check.GetDeterministicSpecs(), (
            'makani.analysis.checks.tests.DerivedCheckWithInit', {
                'find_history': True,
                'history_len': 30,
            }
        )
    )
    self.assertEqual(
        derived_init_check_default.GetDeterministicSpecs(), (
            'makani.analysis.checks.tests.DerivedCheckWithInit', {
                'find_history': True,
                'history_len': 12,
            }
        )
    )

    with self.assertRaises(ValueError):
      BadDerivedCheck(True)

  def testIntValueError(self):
    normal_ranges_with_invalid_int = [[1, 3], 1]
    warning_ranges = [[0, 10]]
    derived_check_invalid_int = DerivedCheck(
        for_log=True, normal_ranges=normal_ranges_with_invalid_int,
        warning_ranges=warning_ranges)
    with self.assertRaises(ValueError):
      derived_check_invalid_int.GetDeterministicSpecs()

  def testStringValueError(self):
    normal_ranges_with_invalid_str = [[1, 3], 'string']
    warning_ranges = [[0, 10]]
    derived_check_invalid_str = DerivedCheck(
        for_log=True, normal_ranges=normal_ranges_with_invalid_str,
        warning_ranges=warning_ranges)
    with self.assertRaises(ValueError):
      derived_check_invalid_str.GetDeterministicSpecs()

  def testDictValueError(self):
    normal_ranges_with_invalid_dict = [[1, 3], {'dict': 4}]
    warning_ranges = [[0, 10]]
    derived_check_invalid_dict = DerivedCheck(
        for_log=True, normal_ranges=normal_ranges_with_invalid_dict,
        warning_ranges=warning_ranges)
    with self.assertRaises(ValueError):
      derived_check_invalid_dict.GetDeterministicSpecs()


class TestSerializeCheckSpecs(unittest.TestCase):

  def testTrivialSerialize(self):
    normal_ranges = [[1, 3]]
    warning_ranges = [[0, 10]]
    derived_check = DerivedCheck(
        for_log=True, normal_ranges=normal_ranges,
        warning_ranges=warning_ranges)
    specs = derived_check.GetDeterministicSpecs()
    expected_json = ('{"for_log": true, "name": null, "normal_ranges": '
                     '{"ranges": [[1, 3]]}, "sort_by_sequence": true, '
                     '"warning_ranges": {"ranges": [[0, 10]]}}')

    serialized_specs = base_check.SerializeCheckSpecs(specs[1])
    self.assertEqual(serialized_specs, expected_json)

  def testUnorderedSet(self):
    """Tests that an unordered set is ordered and turned into a list."""
    normal_ranges = [[1, 3], {20, 444, 3}]
    warning_ranges = [[0, 10]]
    derived_check = DerivedCheck(for_log=True, normal_ranges=normal_ranges,
                                 warning_ranges=warning_ranges)
    specs = derived_check.GetDeterministicSpecs()
    expected_json = ('{"for_log": true, "name": null, "normal_ranges": '
                     '{"ranges": [[1, 3]], "values": [3, 20, 444]}, '
                     '"sort_by_sequence": true, "warning_ranges": {"ranges": '
                     '[[0, 10]]}}')

    serialized_specs = base_check.SerializeCheckSpecs(specs[1])
    self.assertEqual(serialized_specs, expected_json)

  def testMultipleranges(self):
    """Tests that a  multiple ranges are correctly serialized."""
    normal_ranges = [[1, 3], [10, 19], set({20, 444, 3})]
    warning_ranges = [[0, 10]]
    derived_check = DerivedCheck(for_log=True, normal_ranges=normal_ranges,
                                 warning_ranges=warning_ranges)
    specs = derived_check.GetDeterministicSpecs()
    expected_json = ('{"for_log": true, "name": null, "normal_ranges": '
                     '{"ranges": [[1, 3], [10, 19]], "values": [3, 20, 444]}, '
                     '"sort_by_sequence": true, "warning_ranges": {"ranges": '
                     '[[0, 10]]}}')

    serialized_specs = base_check.SerializeCheckSpecs(specs[1])
    self.assertEqual(serialized_specs, expected_json)


class TestParseCheckSpecs(unittest.TestCase):

  def testTrivialParse(self):
    json_spec = ('{"sort_by_sequence": true, "for_log": true, '
                 '"warning_ranges": {"ranges": [[0, 10]]}, "name": null, '
                 '"normal_ranges": {"ranges": [[1, 3]]}}')

    expected_spec = {'sort_by_sequence': True, 'for_log': True,
                     'warning_ranges': [[0, 10]], 'name': None,
                     'normal_ranges': [[1, 3]]}

    parsed_spec = base_check.ParseCheckSpecs(json_spec)
    self.assertEqual(parsed_spec, expected_spec)

  def testSetWithParse(self):
    """Test with a set included in the ranges."""
    json_spec = ('{"sort_by_sequence": true, "for_log": true, '
                 '"warning_ranges": {"ranges": [[0, 10]]}, "name": null, '
                 '"normal_ranges": {"ranges": [[1, 3]], "values": '
                 '[3, 20, 444]}}')

    expected_warning_ranges = [[0, 10]]
    expected_normal_ranges = [[1, 3], set({3, 444, 20})]

    parsed_spec = base_check.ParseCheckSpecs(json_spec)

    self.assertEqual(parsed_spec['warning_ranges'], expected_warning_ranges)
    self.assertEqual(parsed_spec['normal_ranges'], expected_normal_ranges)

if __name__ == '__main__':
  unittest.main()
