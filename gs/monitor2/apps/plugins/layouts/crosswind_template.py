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

"""Layout to monitor crosswind flight status."""

from makani.control import system_params
from makani.gs.monitor import monitor_params
from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators import ground_station
from makani.gs.monitor2.apps.plugins.indicators import motor

MONITOR_PARAMS = monitor_params.GetMonitorParams().contents
SYSTEM_PARAMS = system_params.GetSystemParams().contents


class CrosswindLayout(base.BaseLayout):
  """The crosswind layout."""

  _NAME = 'Crosswind'
  _DESIRED_VIEW_COLS = 12
  _ORDER_HORIZONTALLY = False
  # Derived class should set the _MODE.
  _MODE = '<unset>'

  def Initialize(self):

    self._AddIndicators('Indicators', [
        control.FlightPlanIndicator(),
        control.FlightModeIndicator(self._MODE),
        control.FlightModeGatesIndicator(self._MODE),
        control.ControlTimeIndicator(self._MODE),
        control.LoopCountIndicator(self._MODE),
        control.TetherSphereDeviationIndicator(self._MODE),
        control.CrosswindPlaybookIndicator(),
        control.AlphaErrorIndicator(),
        control.BetaErrorIndicator(),
        control.AirspeedErrorIndicator(),
    ], properties={'cols': 2})

    self._AddIndicators('Altitude', [
        control.AltitudeChart(
            self._MODE, panel_ratio=0.26, aspect_ratio=1.8, num_yticks=5,
            ylim=[-20, 500]),
        ground_station.DetwistErrorChart(
            num_yticks=5, aspect_ratio=1.5),
    ], properties={'cols': 2})

    self._AddIndicators('', [
        motor.StackBusPowerChart(
            self._MODE, 'Gen. Wing Power', num_yticks=5,
            aspect_ratio=2.5, ylim=[-1000, 1000]),
        ground_station.WindIndicator(),
    ], properties={'cols': 3})

    widget_kwargs = {
        'panel_ratio': 0.17,
        'aspect_ratio': 7.5,
        'num_yticks': 7,
    }
    max_tension_kn = round(MONITOR_PARAMS.tether.proof_load / 1e3)
    self._AddIndicators('Charts', [
        control.TensionChart(self._MODE, ylim=[0.0, max_tension_kn],
                             **widget_kwargs),
        control.AeroAnglesChart(self._MODE, ylim=[-10, 10], **widget_kwargs),
        control.AirSpeedChart(self._MODE, ylim=[0, 80], **widget_kwargs),
        control.BodyRatesChart(self._MODE, ylim=(-15.0, 15.0),
                               angles=['Pitch', 'Roll'], **widget_kwargs),
        control.CrosswindDeltasChart(
            self._MODE, ylim=[-15, 15], **widget_kwargs),
    ], properties={'cols': 8})

    self._AddBreak()

    self._AddIndicators('Flight Circle', [
        # TODO: Use full comms mode currently, because target
        # path radius is not in TetherDown yet.
        control.CrosswindCircleWindow(common.FULL_COMMS_MODE),
    ], properties={'cols': 2})

    self._AddIndicators('Aero angles', [
        control.AeroAnglesXYPlot(self._MODE),
    ], properties={'cols': 2})

    self._AddIndicators('Trans-in Trajectory', [
        control.TransInTrajectoryChart(self._MODE),
    ], properties={'cols': 2})

    self._AddIndicators('Crosswind Stats', [
        control.LowBoundLoopAltitudeIndicator(self._MODE),
    ], properties={'cols': 2})
