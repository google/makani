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

"""Layout to monitor hover status."""

from makani.gs.monitor2.apps.layout import base
from makani.gs.monitor2.apps.plugins import common
from makani.gs.monitor2.apps.plugins.indicators import control
from makani.gs.monitor2.apps.plugins.indicators import ground_station


class HoverLayout(base.BaseLayout):
  """The hover layout."""

  _NAME = 'Hover'
  _DESIRED_VIEW_COLS = 12
  _ORDER_HORIZONTALLY = True
  # Derived class should set the _MODE.
  _MODE = '<unset>'

  def Initialize(self):

    self._AddIndicators('Control', [
        control.FlightPlanIndicator(),
        control.FlightModeIndicator(self._MODE),
        control.FlightModeGatesIndicator(self._MODE),
        control.ControlTimeIndicator(self._MODE),
        control.ControllerTimingIndicator(self._MODE),
        control.JoystickIndicator(),
        ground_station.WindIndicator(),
        control.HoverGainRampScaleIndicator(self._MODE),
        control.HoverDistanceFromPerch(common.FULL_COMMS_MODE),
        control.WingVelocityIndicator(self._MODE),
        control.HoverAngleCommandIndicator(),
        control.HoverThrustMomentIndicator(self._MODE),
        ground_station.PerchAzimuthIndicator(),
        control.FdDisabledIndicator(),
        control.FdAllActiveIndicator(),
    ], properties={'cols': 4})

    widget_kwargs = {
        'panel_ratio': 0.22,
        'aspect_ratio': 2.8,
        'num_yticks': 5,
    }
    self._AddIndicators('Charts', [
        # TODO: Set value limits.
        control.HoverPathErrorsChart(**widget_kwargs),
        control.TensionChart(self._MODE, ylim=[0, 20], **widget_kwargs),
        control.HoverPositionErrorsChart(ylim=[-17.0, 17.0], **widget_kwargs),
        control.HoverVelocityErrorsChart(ylim=[-5.0, 5.0], **widget_kwargs),
        control.HoverAnglesChart(self._MODE, ylim=[-90, 90], **widget_kwargs),
    ], properties={'cols': 4})

    self._AddIndicators('', [
        control.RotorPitchYawWindow(),
        control.ConstraintWindow(self._MODE),
    ], properties={'cols': 3})
