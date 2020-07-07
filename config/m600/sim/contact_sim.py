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

"""Contact simulation parameters."""
from makani.config import mconfig
from makani.config import physics_util


@mconfig.Config(deps={
    'wing': mconfig.WING_MODEL + '.wing'
})
def MakeParams(params):
  # Spring constant [N/m] of the main wheel of the landing gear.
  wheel_spring_const = 1.3e4

  # Spring constant [N/m] of the tusk of the landing gear.
  tusk_spring_const = 6.0e3

  # Spring constant [N/m] of the tail skid of the landing gear.
  tail_spring_const = 6.0e3

  # Damping ratio [#] of the landing gear.
  zeta = 0.05

  # Spring constant [N/m], damping coefficient [N/(m/s)], and friction
  # coefficient [#] of the talons.  The unrealistically high friction
  # coefficient is there to mimic the talons "grabbing" the perch
  # panels.
  talon_spring_const = 1e6
  talon_damping_coeff = 1e5
  talon_friction_coeff = 50.0

  # Spring constant [N/m] and damping coefficient [N/(m/s)] of the
  # perching peg.  The peg spring constant is based on an email from
  # cadman on 2014-08-25.
  peg_spring_const = 1e6
  peg_damping_coeff = 1e5

  # Typical generic friction coefficient [#],
  typical_friction_coeff = 0.8

  return {
      'ground_contactors': [{
          # Port main wheel position [m].
          'pos': [0.05, -1.55, 1.1],
          'spring_const': wheel_spring_const,
          'damping_coeff': physics_util.ConvertDampingRatioToCoeff(
              wheel_spring_const, params['wing']['m'], zeta),
          'friction_coeff': typical_friction_coeff
      }, {
          # Starboard main wheel position [m].
          'pos': [0.05, 1.55, 1.1],
          'spring_const': wheel_spring_const,
          'damping_coeff': physics_util.ConvertDampingRatioToCoeff(
              wheel_spring_const, params['wing']['m'], zeta),
          'friction_coeff': typical_friction_coeff
      }, {
          # Port landing tusk position [m].
          'pos': [1.9, -1.55, 1.1],
          'spring_const': tusk_spring_const,
          'damping_coeff': physics_util.ConvertDampingRatioToCoeff(
              tusk_spring_const, params['wing']['m'], zeta),
          'friction_coeff': typical_friction_coeff
      }, {
          # Starboard landing tusk position [m].
          'pos': [1.9, 1.55, 1.1],
          'spring_const': tusk_spring_const,
          'damping_coeff': physics_util.ConvertDampingRatioToCoeff(
              tusk_spring_const, params['wing']['m'], zeta),
          'friction_coeff': typical_friction_coeff
      }, {
          # Rear skid position [m].
          'pos': [-2.0, 0.0, 0.33],
          'spring_const': tail_spring_const,
          'damping_coeff': physics_util.ConvertDampingRatioToCoeff(
              tail_spring_const, params['wing']['m'], zeta),
          'friction_coeff': typical_friction_coeff
      }],

      'perch_contactors': [{
          # Port talon position [m]. See go/makani-perch-geometry.
          'pos': [-0.02, -1.31, 0.85],
          'spring_const': talon_spring_const,
          'damping_coeff': talon_damping_coeff,
          'friction_coeff': talon_friction_coeff
      }, {
          # Starboard talon position [m]. See go/makani-perch-geometry.
          'pos': [-0.02, 1.12, 0.85],
          'spring_const': talon_spring_const,
          'damping_coeff': talon_damping_coeff,
          'friction_coeff': talon_friction_coeff
      }, {
          # Peg position [m].
          'pos': [-3.0, -0.21, 0.5],
          'spring_const': peg_spring_const,
          'damping_coeff': peg_damping_coeff,
          'friction_coeff': typical_friction_coeff
      }],

  }
