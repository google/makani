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

"""Pilot layout."""

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators import ground_station


class PilotLayout(base.BaseLayout):
  """The pilot layout."""

  _NAME = 'Pilot'
  _DESIRED_VIEW_COLS = 12
  _ORDER_HORIZONTALLY = False
  # Derived class should set the _MODE.
  _MODE = '<unset>'

  def Initialize(self):
    self._AddIndicators('Flight Controller', [
        control.FlightPlanIndicator(),
        control.FlightModeIndicator(self._MODE),
        control.FlightModeGatesIndicator(self._MODE),
        control.ControlTimeIndicator(self._MODE),
        control.JoystickIndicator(),
        control.HoverGainRampScaleIndicator(self._MODE),
        ground_station.WindIndicator(),
    ], properties={'cols': 3})

    self._AddIndicators('Kite Position State', [
        control.ApparentWindSpeedIndicator(self._MODE),
        control.AltitudeIndicator(self._MODE),
        control.PayoutIndicator(),
        control.LowBoundLoopAltitudeIndicator(self._MODE),
        control.CrosswindPlaybookIndicator(),
        control.AutoControllerIndicator(),
    ], properties={'cols': 3})

    self._AddIndicators('Throttle', [
        control.ThrottleIndicator(from_joystick=True),
    ], properties={'cols': 2.5})

    self._AddBreak()

    self._AddIndicators('Off-Tether Position', [
        control.GlideslopeChart(self._MODE),
        control.TopDownPositionChart(self._MODE),
    ], properties={'cols': 3})

    self._AddIndicators('Constraint Window', [
        control.ConstraintWindow(self._MODE),
    ], properties={'cols': 3})

    self._AddBreak()

    self._AddIndicators('Aero Angles', [
        control.AeroAnglesXYPlot(self._MODE),
    ], properties={'cols': 3})

    self._AddIndicators('Flight Circle', [
        control.CrosswindCircleWindow(self._MODE),
    ], properties={'cols': 3})

    self._AddIndicators('Rotor Moments', [
        control.RotorPitchYawWindow(),
    ], properties={'cols': 3})

    self._AddBreak()

    self._AddIndicators('Trans-in Trajectory', [
        control.TransInTrajectoryChart(self._MODE),
    ], properties={'cols': 3})

    self._AddIndicators('Impact Zone', [
        control.ImpactZoneChart(self._MODE),
    ], properties={'cols': 3})

    self._AddIndicators('Vessel Altitude', [
        ground_station.VesselPositionChart(),
    ], properties={'cols': 3})
