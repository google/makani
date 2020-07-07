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

"""Tether parameters."""
from makani.config import mconfig
from makani.control import system_types


@mconfig.Config(deps={
    'flight_plan': 'common.flight_plan',
    'gs_model': 'base_station.gs_model',
})
def MakeParams(params):
  """Return paramaters about the kite's tether."""
  if (params['gs_model'] == system_types.kGroundStationModelGSv2
      and params['flight_plan'] == system_types.kFlightPlanHoverInPlace):
    length = 80.0
  else:
    # Set the tether length.
    # From BigM600 config sheet for r07c_v01.
    # https://docs.google.com/spreadsheets/d/18eGSelEsldi6UuKKpqSIw5j9_UU21-UJzTv24IcN6oE
    length = 300.0

  # The following properties pertain to tether FC1-02 installed
  # for RPX-08. See b/70513834 for references.
  return {
      # Tether length [m] under zero load.
      'length': length,

      # Linear density [kg/m].
      'linear_density': 0.917,

      # Tether outer diameter [m].
      'outer_diameter': 0.0294,

      # Tensile stiffness, EA, [N] of the tether core.
      'tensile_stiffness': 18e6,

      # Bending stiffness, EI, [N*m**2] of the tether.
      'bending_stiffness': 35.0,

      # Cross-sectional drag coefficient [#].
      'section_drag_coeff': 0.7,

      # Distance [m] from the GSG elevation pin to the tether termination pin
      # on the top hat.
      #
      # Eli notes that the distance from the elevation pin to the nose of the
      # GSG is more like 1.2m, which is roughly the point at which the tether
      # can start bending.
      'gsg_ele_to_termination': 0.712,
  }
