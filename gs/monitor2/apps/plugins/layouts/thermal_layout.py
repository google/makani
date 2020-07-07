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

"""Layout to monitor flight status."""

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.gs.monitor2.apps.plugins.indicators import motor
from makani.gs.monitor2.apps.plugins.indicators import servo


class ThermalLayout(base.BaseLayout):
  """The thermal layout."""

  _NAME = 'Thermal'
  _DESIRED_VIEW_COLS = 12
  _ORDER_HORIZONTALLY = False
  _TOP_MOTORS = ['Pto', 'Pti', 'Sti', 'Sto']
  _BOTTOM_MOTORS = ['Pbo', 'Pbi', 'Sbi', 'Sbo']
  _MODE = common.SPARSE_COMMS_MODE

  def Initialize(self):

    self._AddIndicators('Top Motors', [
        motor.BoardTemperatureIndicator(
            self._MODE, motor_labels=self._TOP_MOTORS, show_label=True),
        motor.CapacitorTemperatureIndicator(
            self._MODE, motor_labels=self._TOP_MOTORS, show_label=False),
        motor.HeatPlateTemperatureIndicator(
            self._MODE, motor_labels=self._TOP_MOTORS, show_label=False),
        motor.StatorCoreTemperatureIndicator(
            self._MODE, motor_labels=self._TOP_MOTORS, show_label=False),
        motor.WindingTemperatureIndicator(
            self._MODE, motor_labels=self._TOP_MOTORS, show_label=False),
    ], properties={'cols': 3})

    self._AddIndicators('Bottom Motors', [
        motor.BoardTemperatureIndicator(
            self._MODE, motor_labels=self._BOTTOM_MOTORS, show_label=True),
        motor.CapacitorTemperatureIndicator(
            self._MODE, motor_labels=self._BOTTOM_MOTORS, show_label=False),
        motor.HeatPlateTemperatureIndicator(
            self._MODE, motor_labels=self._BOTTOM_MOTORS, show_label=False),
        motor.StatorCoreTemperatureIndicator(
            self._MODE, motor_labels=self._BOTTOM_MOTORS, show_label=False),
        motor.WindingTemperatureIndicator(
            self._MODE, motor_labels=self._BOTTOM_MOTORS, show_label=False),
    ], properties={'cols': 3})

    self._AddIndicators('Servos', [
        avionics.AioMonTemperatureIndicator(
            'Port Servos', 'ServoStatus', ['A1', 'A2', 'A4'],
            source_prefix='Servo'),
        avionics.AioMonTemperatureIndicator(
            'Starboard Servos', 'ServoStatus', ['A5', 'A7', 'A8'],
            source_prefix='Servo'),
        avionics.AioMonTemperatureIndicator(
            'Tail Servos', 'ServoStatus', ['E1', 'E2', 'R1', 'R2'],
            source_prefix='Servo'),
        servo.R22TemperatureTailIndicator(self._MODE),
        servo.R22TemperaturePortIndicator(self._MODE),
        servo.R22TemperatureStarboardIndicator(self._MODE),
    ], properties={'cols': 3})

    self._AddBreak()

    self._AddIndicators('Avionics Temperatures', [
        avionics.AioMonTemperatureIndicator(
            'Core Switches', 'CoreSwitchStatus', ['CsA', 'CsB']),
        avionics.AioMonTemperatureIndicator(
            'Flight Computers', 'FlightComputerSensor'),
        avionics.AioMonTemperatureIndicator('Recorders', 'RecorderStatus',
                                            source_prefix='RecorderTms570'),
        avionics.AioMonTemperatureIndicator('GPS', 'GpsStatus'),
        avionics.AioMonTemperatureIndicator('Load cells', 'Loadcell',
                                            source_prefix='Loadcell'),
    ], properties={'cols': 3})

    self._AddBreak()

    self._AddIndicators('Ground Temperatures', [
        avionics.AioMonTemperatureIndicator(
            'Core Switches', 'CoreSwitchStatus', ['CsGsA', 'CsGsB']),
        avionics.AioMonTemperatureIndicator(
            'Drum Sensors', 'DrumSensorsMonitor'),
        avionics.AioMonTemperatureIndicator(
            'Ground Station', 'GroundStationPlcMonitorStatus'),
        avionics.AioMonTemperatureIndicator(
            'Joystick', 'JoystickMonitorStatus'),
        avionics.AioMonTemperatureIndicator(
            'Platform Sensors', 'PlatformSensorsMonitor',
            source_prefix='PlatformSensors'),
    ], properties={'cols': 3})
