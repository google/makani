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

from makani.control import system_params
from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import aio_comms
from makani.gs.monitor2.apps.plugins.indicators import avionics
from makani.gs.monitor2.apps.plugins.indicators import batt
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators import estimator
from makani.gs.monitor2.apps.plugins.indicators import gps
from makani.gs.monitor2.apps.plugins.indicators import ground_power
from makani.gs.monitor2.apps.plugins.indicators import ground_station
from makani.gs.monitor2.apps.plugins.indicators import motor
from makani.gs.monitor2.apps.plugins.indicators import mvlv
from makani.gs.monitor2.apps.plugins.indicators import network
from makani.gs.monitor2.apps.plugins.indicators import node_status
from makani.gs.monitor2.apps.plugins.indicators import servo
from makani.gs.monitor2.apps.plugins.indicators import short_stack

_SYSTEM_PARAMS = system_params.GetSystemParams().contents


class FlightLayout(base.BaseLayout):
  """The flight layout."""

  _NAME = 'Flight'
  _DESIRED_VIEW_COLS = 12
  _ORDER_HORIZONTALLY = False
  _TOP_MOTORS = ['Pto', 'Pti', 'Sti', 'Sto']
  _BOTTOM_MOTORS = ['Pbo', 'Pbi', 'Sbi', 'Sbo']
  # Derived class should set the _MODE.
  _MODE = '<unset>'

  def Initialize(self):
    self._AddIndicators('Comms Status', [
        network.TetherCommsStatusPoFIndicator('PoF', show_label=True),
        network.TetherCommsStatusEoPIndicator('EoP', show_label=False),
        network.TetherCommsStatusWifiIndicator('Wifi', show_label=False),
        network.JoystickRadioStatusIndicator('Joystick Radio'),
        network.TetherLongRangeRadioStatusIndicator('Long range'),
    ], properties={'cols': 3})

    self._AddIndicators('AIO Update', [
        aio_comms.CoreSwitchAioUpdateIndicator(),
        aio_comms.ControllerAioUpdateIndicator(),
        aio_comms.FlightComputerAioUpdateIndicator(),
        aio_comms.JoystickAioUpdateIndicator(),
    ], properties={'cols': 3})

    self._AddIndicators('Control', [
        control.ControllerInitStateIndicator(),
        control.VersionIndicator(),
        control.FlightPlanIndicator(),
        control.FlightModeIndicator(self._MODE),
        control.FlightModeGatesIndicator(self._MODE),
        control.ControlTimeIndicator(self._MODE),
        control.ControllerTimingIndicator(self._MODE),
        control.ExperimentIndicator(self._MODE),
        control.JoystickIndicator(),
        control.HoverGainRampScaleIndicator(self._MODE),
        control.TetherReleaseIndicator(),
        control.TetherReleaseReadinessIndicator(
            ['LoadcellPortA', 'LoadcellPortB',
             'LoadcellStarboardA', 'LoadcellStarboardB']),
    ], properties={'cols': 3})

    self._AddIndicators('Weather', [
        ground_station.WindIndicator(),
        ground_station.WeatherSensorIndicator(),
    ], properties={'cols': 3})

    self._AddIndicators('Ground Station', [
        ground_station.PerchAzimuthIndicator(),
        ground_station.DetwistErrorIndicator(),
        ground_station.EStopStatusIndicator(),
        ground_station.GsCommsIndicator(),
        ground_station.GsArmingStatusIndicator(),
        ground_power.SummaryIndicator('Ground Power'),
        ground_power.FaultIsolationIndicator('Power Fault Isolation [V]'),
    ], properties={'cols': 3})

    self._AddIndicators('Faults', [
        control.FdAllActiveIndicator(),
        short_stack.ShortStackFlightIndicator(),
    ], properties={'cols': 3})

    self._AddBreak()

    self._AddIndicators('Avionics', [
        servo.StatusIndicator(self._MODE),
        motor.StackBusPowerIndicator(self._MODE, 'Wing Power (Generated)'),
        avionics.FpvIndicator(self._MODE, ['A', 'B', 'C'], 'Fpv Enabled'),
        avionics.PitotCoverIndicator(common.FULL_COMMS_MODE, ['A', 'B', 'C'],
                                     'Pitot Cover'),
    ], properties={'cols': 3})

    self._AddIndicators('Power System',
                        self._GetPowerSystemIndicators(),
                        properties={'cols': 3})

    self._AddIndicators(
        'Top Motors',
        self._GetMotorIndicators(self._TOP_MOTORS), properties={'cols': 3})

    self._AddIndicators(
        'Bottom Motors',
        self._GetMotorIndicators(self._BOTTOM_MOTORS) +
        [control.FlutterWarningIndicator(self._MODE)], properties={'cols': 3})

    self._AddBreak()

    self._AddIndicators('Tail Servos', [
        servo.ArmedTailIndicator(self._MODE),
        servo.R22TemperatureTailIndicator(self._MODE),
        servo.ElePosChart(self._MODE, ylim=[-30, 30]),
        servo.RudPosChart(self._MODE, ylim=[-90, 90]),
    ], properties={'cols': 3})

    self._AddIndicators('Port Servos', [
        servo.ArmedPortIndicator(self._MODE),
        servo.R22TemperaturePortIndicator(self._MODE),
        servo.PortPosChart(self._MODE, ylim=[-40, 20]),
    ], properties={'cols': 3})

    self._AddIndicators('Starboard Servos', [
        servo.ArmedStarboardIndicator(self._MODE),
        servo.R22TemperatureStarboardIndicator(self._MODE),
        servo.StarboardPosChart(self._MODE, ylim=[-40, 20]),
    ], properties={'cols': 3})

    self._AddIndicators('Wing GPS', [
    ], properties={'cols': 3})

    self._AddIndicators('Estimator', [
        estimator.EstimatorGsgBias(common.FULL_COMMS_MODE),
        estimator.EstimatorGpsDiff(common.FULL_COMMS_MODE),
        estimator.EstimatorGyroDiffIndicator(common.FULL_COMMS_MODE),
        estimator.EstimatorMagnetometerDiffIndicator(common.FULL_COMMS_MODE),
        estimator.EstimatorAttitudeDiffIndicator(common.FULL_COMMS_MODE),
        estimator.EstimatorGyroBiasIndicator(common.FULL_COMMS_MODE),
        estimator.EstimatorGyroBiasDriftIndicator(common.FULL_COMMS_MODE),
        estimator.EstimatorGsgDiff(common.FULL_COMMS_MODE),
    ], properties={'cols': 3})

    self._AddIndicators('Vessel', [
        gps.TetherUpGpsIndicator(name='GPS BaseStation'),
        control.VesselPositionIndicator(common.FULL_COMMS_MODE),
        control.VesselAttitudeIndicator(common.FULL_COMMS_MODE),
    ], properties={'cols': 3})

    self._AddBreak()

    self._AddIndicators('Wing', [
        control.HoverAnglesChart(self._MODE, ylim=[-90, 90]),
        control.TetherAnglesChart(ylim=[-100, 100]),
        control.WingPosChart(self._MODE, ylim=[-600, 500]),
        control.AirSpeedChart(self._MODE, ylim=[0, 80]),
        control.TensionChart(self._MODE),
        control.TensionPilotOffsetIndicator(),
        avionics.BridleJunctionLoadcellIndicator('Bridle Junc'),
        control.ImpactZoneChart(self._MODE),
    ], properties={'cols': 3})

  def _GetPowerSystemIndicators(self):
    indicators = [
        motor.MotorLVInputIndicator(chars_per_line=15),
        node_status.TetherNodePowerSummary('Power'),
        batt.TetherDownLvBusVoltageIndicator(),
        batt.StateOfChargeIndicator(['A', 'B'], 'State of Charge'),
        # Show both MvLv and battery warnings and errors. There can be more than
        # 6 warnings/errors in the worst case, but set it to 6 to save space.
        # This is the only place showing Mvlv errors in SN01/Gin since Mvlv
        # errors are excluded from TetherNodeErrorSummary for noise issue in
        # SN1/Gin
        mvlv.LvSummaryIndicator(num_lines=6),
    ]
    if _SYSTEM_PARAMS.wing_serial != system_params.kWingSerial01:
      indicators.insert(0, motor.MotorVoltageIndicator(chars_per_line=15))
    return indicators

  def _GetMotorIndicators(self, motor_labels):
    indicators = [
        motor.ArmedIndicator(
            self._MODE, motor_labels=motor_labels, show_label=True),
        motor.ErrorIndicator(
            self._MODE, motor_labels=motor_labels, show_label=False),
        motor.WarningIndicator(
            self._MODE, motor_labels=motor_labels, show_label=False),
        motor.MotorBusVoltageIndicator(
            self._MODE, motor_labels=motor_labels, show_label=False),
        motor.BoardTemperatureIndicator(
            self._MODE, motor_labels=motor_labels, show_label=False),
        motor.CapacitorTemperatureIndicator(
            self._MODE, motor_labels=motor_labels, show_label=False),
    ]

    if _SYSTEM_PARAMS.wing_serial in (system_params.kWingSerial04Hover,
                                      system_params.kWingSerial04Crosswind):
      indicators += [
          motor.ModuleTemperatureIndicator(
              motor_labels=motor_labels, show_label=False),
      ]
    else:
      indicators += [
          motor.HeatPlateTemperatureIndicator(
              self._MODE, motor_labels=motor_labels, show_label=False),
      ]

    indicators += [
        motor.SpeedChart(
            self._MODE, 'Speed [rad/s]', motor_labels,
            ylim=[-250, 250], show_cmd=True),
    ]

    return indicators
