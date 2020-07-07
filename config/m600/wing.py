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

"""Geometric and inertial properties of the wing."""

from makani.config import mconfig
from makani.control import system_types as m
import numpy as np


@mconfig.Config(deps={
    'wing_serial': 'common.wing_serial',
    'rotors': 'm600.rotors',
})
# Parameters must be updated separately in the matlab scripts in
# analysis/control/generate_hover_controllers.m and
# analysis/control/generate_crosswind_controllers.m.
def MakeParams(params):

  if params['wing_serial'] == m.kWingSerial01:
    # Reference mass [kg] of the wing, which is used for scaling the
    # moment-of-inertia tensor.  This is based on the projected SN02
    # wing and bridles mass taken from go/makanimass on 2016-03-04.
    reference_wing_mass = 1606.98

    # Kite mass [kg] including airframe, bridles (55.3 kg), wingside tether
    # termination (10.6 kg) and GoPros on termination (0.6 kg),
    # from go/makanimass SN01 2018-11 Release.
    wing_mass = 1648.2 + 55.3 + 10.6 + 0.6

    # Center-of-mass [m] for the High Hover configuration, based on an
    # estimate made by robbiesu on 2018-11-02 at go/makanimass.
    center_of_mass_pos = [-0.054, 0.033, 0.035]

    # Moment-of-inertia tensor [kg-m^2].  Because SN01 now has the low
    # tail, this uses a scaled version of the SN02 moment-of-inertia.
    I = np.array([[31401.2, 47.3, 22.3],
                  [47.3, 8228.9, 22.3],
                  [22.3, 22.3, 36864.5]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial04Hover:
    # The following mass, cg and inertia are for kite SN04 in constrained hover
    # configuration, per estimate from go/makanimass.
    # Mass calculated as (kite) + (bridles) + (wingside tether termination).
    wing_mass = 1746.4 + 52.1 + 10.6
    # CG is for the kite only (no bridles).
    center_of_mass_pos = [-0.090, 0.030, 0.097]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1731.0
    I = np.array([[30270.0, 22.08, 38.92],
                  [22.08, 9221.0, 15.23],
                  [38.92, 15.23, 36630.0]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial04Crosswind:
    # The following mass, cg and inertia are for kite SN04 in crosswind
    # configuration.
    # Mass calculated as (kite) + (bridles) + (wingside tether termination)
    # + (GoPros on termination).
    # NOTE: Parameters must be updated separately in the matlab scripts
    # in analysis/control.
    wing_mass = 1667.5 + 52.1 + 10.6 + 0.6
    # CG is for the rigid kite only (no bridles).
    center_of_mass_pos = [-0.085, 0.037, 0.108]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1600.0
    I = np.array([[30260.0, 21.35, 33.90],
                  [21.35, 9210.0, 16.51],
                  [33.90, 16.51, 36620.0]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial05Hover:
    # The following mass, cg and inertia are for kite SN05 in constrained hover
    # configuration.
    # Mass calculated as (kite) + (bridles) + (wingside tether termination).
    wing_mass = 1711.8 + 51.5 + 10.6
    center_of_mass_pos = [-0.088, -0.011, 0.114]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1731.0
    I = np.array([[30270.0, 22.08, 38.92],
                  [22.08, 9221.0, 15.23],
                  [38.92, 15.23, 36630.0]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial05Crosswind:
    # The following mass, cg and inertia are for kite SN05 in crosswind
    # configuration.
    # # Mass calculated as (kite) + (bridles) + (wingside tether termination)
    # + (GoPros on termination).
    wing_mass = 1629.6 + 51.5 + 10.6 + 0.6
    center_of_mass_pos = [-0.081, -0.006, 0.126]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1600.0
    I = np.array([[30260.0, 21.35, 33.90],
                  [21.35, 9210.0, 16.51],
                  [33.90, 16.51, 36620.0]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial06Hover:
    # The following mass, cg and inertia are for kite SN06 in CONSTRAINED HOVER
    # configuration.
    # Mass calculated as (kite) + (bridles) + (wingside tether termination).
    # CG is for the rigid kite only (no bridles).
    wing_mass = 1750.2 + 51.5 + 10.6
    center_of_mass_pos = [-0.093, 0.005, 0.110]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1731.0
    I = np.array([[30270.0, 22.08, 38.92],
                  [22.08, 9221.0, 15.23],
                  [38.92, 15.23, 36630.0]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial06Crosswind:
    # The following mass, cg and inertia are for kite SN06 in CROSSWIND
    # configuration.
    # Mass calculated as (kite) + (bridles) + (wingside tether termination)
    # + (GoPros on termination).
    # CG is for the rigid kite only (no bridles)."
    wing_mass = 1667.9 + 51.5 + 10.6 + 0.6
    center_of_mass_pos = [-0.086, 0.011, 0.121]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1600.0
    I = np.array([[30260.0, 21.35, 33.90],
                  [21.35, 9210.0, 16.51],
                  [33.90, 16.51, 36620.0]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial07Hover:
    # TODO(b/145244788): The following data pertain to SN04. Update.
    wing_mass = 1746.4 + 52.1 + 10.6
    center_of_mass_pos = [-0.090, 0.030, 0.097]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1731.0
    I = np.array([[30270.0, 22.08, 38.92],
                  [22.08, 9221.0, 15.23],
                  [38.92, 15.23, 36630.0]]) * wing_mass / reference_wing_mass

  elif params['wing_serial'] == m.kWingSerial07Crosswind:
    # TODO(b/145244788): The following data pertain to SN04. Update.
    wing_mass = 1667.5 + 52.1 + 10.6 + 0.6
    center_of_mass_pos = [-0.085, 0.037, 0.108]

    # The inertia matrix [kg-m^2] is obtained using the ASWING model
    # Refer b/112168012.
    reference_wing_mass = 1600.0
    I = np.array([[30260.0, 21.35, 33.90],
                  [21.35, 9210.0, 16.51],
                  [33.90, 16.51, 36620.0]]) * wing_mass / reference_wing_mass

  else:
    assert False, 'Unknown wing serial.'

  # Set bridle geometry.
  # More details for bridle_pos at go/makani-loadcell-coordinate-system.
  # Since CalcLoadcellForces requires equal x and z coordinates,
  # their mean is used for now. True bridle position given in comments.

  # Original bridle lengths.
  # Position [m] of port and starboard bridle attachments.
  bridle_pos = [[-0.1494, -5.8843, 0.13035],
                [-0.1494, 5.8661, 0.13035]]

  # Radial distance [m] between bridle pivot and bridle point.
  bridle_rad = 4.7860

  # Offset [m] of bridle point along y-axis.
  bridle_y_offset = -0.5

  # A mean rotor diameter [m] is calculated for use in wing aerodynamic methods.
  rotors_diameter = [params['rotors'][r]['D'] for r in range(m.kNumMotors)]
  mean_rotor_diameter = sum(rotors_diameter) / len(rotors_diameter)

  return {
      # Wing area [m^2].
      'A': 32.9,

      # Wing span [m].
      'b': 25.66,

      # Wing chord [m].
      'c': 1.28,

      # Wing incidence angle [deg].
      'wing_i': 12.0,

      # Estimated wing mass [kg]. This incorporates both the wing and bridle
      # mass, as given by go/makanimass.
      'm': wing_mass,

      # Estimated tail mass [kg] (tail = tub + fuselage + empennage)
      # Source: full kite finite element model m600assy_sn2_fem_r13_s.fem
      'm_tail': 189.2,

      # Moment of inertia tensor [kg-m^2] and inverse.
      'I': {'d': I.tolist()},
      'I_inv': {'d': np.linalg.inv(I).tolist()},

      # Tail matrix of inertia [kg-m^2] about tail CG
      # (tail = tub + fuselage + empennage).
      # Source: full kite finite element model m600assy_sn2_fem_r13_s.fem
      'i_tail': {'d': [[152.20, -0.5964, 61.80],
                       [-0.5964, 1356.5, 0.7337],
                       [61.80, 0.7337, 1280.2]]},

      # Center of mass [m].
      'center_of_mass_pos': center_of_mass_pos,

      # Tail CG [m] in body coordinates (tail = tub + fuselage + empennage).
      # Source: full kite finite element model m600assy_sn2_fem_r13_s.fem
      'tail_cg_pos': [-5.069, 0.002, 0.037],

      # Position [m] of port and starboard bridle attachments,
      'bridle_pos': bridle_pos,

      # Radial distance [m] between bridle pivot and bridle point.
      'bridle_rad': bridle_rad,

      # Offset [m] of bridle point along y-axis.
      'bridle_y_offset': bridle_y_offset,

      # Position [m] of the horizontal tail (taken as the
      # center-of-pressure) in body coordinates.  This is used to
      # calculate the proximity of the horizontal tail to the remote
      # perch.
      'horizontal_tail_pos': [-6.776, 0.045, 0.8165],

      # Position [m] of the constraint system attachment point on the
      # wing, in body coordinates.
      'proboscis_pos': [0.620, 0.0, -0.203],

      # Position [m] of the quarter-chord line center of each pylon. Y location
      # from ASWING input geometry file M600_sn3_r04_aero.asw. X and Z locations
      # approximated via integration of the quarter-chord location along the
      # pylon span, weighted by chord length.
      'pylon_pos': [[0.8075, -3.793, 0.1535],
                    [0.8075, -1.367, 0.1535],
                    [0.8075, 1.060, 0.1535],
                    [0.8075, 3.486, 0.1535]],

      # Pylon span [m]. From ASWING input file.
      'b_pylon': 3.26,

      # Mean rotor diameter [m].
      'mean_rotor_diameter': mean_rotor_diameter,
  }
