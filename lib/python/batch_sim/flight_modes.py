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

"""This provides an h2py independent list of flight modes.

This is a temporary work-around until h2py libraries are
accessible from the batch sim workers.
"""


def GetFlightModes():
  return {
      name: index for index, name in enumerate((
          'kFlightModePilotHover',
          'kFlightModePerched',
          'kFlightModeHoverAscend',
          'kFlightModeHoverPayOut',
          'kFlightModeHoverFullLength',
          'kFlightModeHoverAccel',
          'kFlightModeTransIn',
          'kFlightModeCrosswindNormal',
          'kFlightModeCrosswindPrepTransOut',
          'kFlightModeHoverTransOut',
          'kFlightModeHoverReelIn',
          'kFlightModeHoverDescend',
          'kFlightModeOffTether',
          'kFlightModeHoverTransformGsUp',
          'kFlightModeHoverTransformGsDown',
          'kFlightModeHoverPrepTransformGsUp',
          'kFlightModeHoverPrepTransformGsDown'))
  }
