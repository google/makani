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

import unittest

from makani.gs.monitor2.apps.layout import autogen
from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.layout import layout_util
from makani.gs.monitor2.apps.receiver import test_util
from makani.lib.python import struct_tree


class TestPlotData(unittest.TestCase):

  def testPlotData(self):
    template = base.PlotData(None)
    self.assertEqual(template.apple, 'apple')
    self.assertEqual(template.banana, 'banana')

    data = base.PlotData(template)
    data.apple = 'red'
    banana_status = {
        'yellow': 'good',
        'black': 'bad',
    }
    data.banana = banana_status
    self.assertEqual(data.Json(), {
        'apple': 'red',
        'banana': banana_status,
    })
    with self.assertRaises(base.MonitorLayoutException):
      data.orange = 1

  def testGetDistinguishableNames(self):
    self.assertEqual(
        layout_util.GetDistinguishableNames(['Day.NewYork.BigApple',
                                             'Night.NewYork.BigMelon'],
                                            '.', ['Big']),
        {'Day.NewYork.BigApple': 'Day.Apple',
         'Night.NewYork.BigMelon': 'Night.Melon'})

    self.assertEqual(
        layout_util.GetDistinguishableNames(['Day.NewYork',
                                             'Day.NewYork.BigMelon'],
                                            '.', ['Big']),
        {'Day.NewYork': 'NewYork',
         'Day.NewYork.BigMelon': 'NewYork.Melon'})


class TestAutogen(unittest.TestCase):

  def testBasics(self):
    messages = struct_tree.StructTree({
        'MotorStatus': {
            'MotorPbi': {
                'status': 0,
                'errors': [1, 2, 3],
                'details': {'temp': 60, 'voltage': 800},
            }
        }
    }, fail_silently=False, readonly=True)
    scenario = autogen.GenerateScenario(messages.Data(), 'Test')
    self.assertEqual(scenario, {
        'signals': {},
        'canvas': {
            'row_height_px': 40,
            'grid_width': 12
        },
        'views': [
            {
                'stripe': [
                    {
                        'indicators': [
                            {
                                'src': 'MotorStatus.MotorPbi.details.temp',
                                'name': 'details.temp',
                                'cols': 12,
                                'precision': None,
                                'indicator_src': None,
                                'template': 'ScalarIndicator',
                                'font_size': None,
                                'mode': 'horizontal'
                            },
                            {
                                'src': 'MotorStatus.MotorPbi.details.voltage',
                                'name': 'details.voltage',
                                'cols': 12,
                                'precision': None,
                                'indicator_src': None,
                                'template': 'ScalarIndicator',
                                'font_size': None,
                                'mode': 'horizontal'
                            },
                            {
                                'message_src': None,
                                'src': 'MotorStatus.MotorPbi.errors',
                                'mode': 'horizontal',
                                'template': 'ListIndicator',
                                'indicator_src': None,
                                'keys': [
                                    '[0]',
                                    '[1]',
                                    '[2]'
                                ],
                                'precision': None,
                                'cols': 12,
                                'font_size': None,
                                'name': 'errors'
                            },
                            {
                                'src': 'MotorStatus.MotorPbi.status',
                                'name': 'status',
                                'cols': 12,
                                'precision': None,
                                'indicator_src': None,
                                'template': 'ScalarIndicator',
                                'font_size': None,
                                'mode': 'horizontal'
                            }
                        ],
                        'rows': 3,
                        'name': 'MotorStatus.MotorPbi',
                        'grid_width': 12
                    }
                ],
                'grid_width': 12
            }
        ],
        'filters': [],
        'title': 'Test'
    })

  def testCtypes(self):
    messages = test_util.SynthesizeMessages(['ControlTelemetry'], 0)
    # Run and make sure there are no faults.
    autogen.GenerateScenario(messages.Data(convert_to_basic_types=True), 'Test')


if __name__ == '__main__':
  unittest.main()
