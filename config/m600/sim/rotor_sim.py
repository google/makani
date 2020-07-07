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

"""Simulated rotor parameters."""

from makani.config import mconfig


@mconfig.Config(deps={
    'propellers': 'prop.propellers',
    'rotors': mconfig.WING_MODEL + '.rotors'
})
def MakeParams(params):
  return {
      # Array of rotor aerodynamic database file names.
      'database_names': [params['propellers'][r['version']]['database']
                         for r in params['rotors']],

      # Array of rotor aerodynamic 3D database file names.
      'database_3d_names': [params['propellers'][r['version']]['database3d']
                            for r in params['rotors']],

      # Boolean describing whether to apply the 3D rotor tables.
      'apply_3d_rotor_tables': True,

      # Cutoff frequency [Hz] for the rotor acceleration when using HITL rotors.
      'fc_hitl_rotor_acc': 1.0,

      # Boolean describing whether to apply the empirical blown wing effect
      # modification (also referred to as flow/thrust vectoring).
      'apply_blown_wing_effect': True,

      # Angle [rad] that the thrust vector from the rotors should be rotated
      # about the body y axis to account for the blown wing effect. This value
      # was determined empirically from the 2016-08-24 flight tests.
      'thrust_vectoring_angle': 0.3,

      # Position [m] in body coordinates where the additional forces created
      # by the blown wing effect (flow/thrust vectoring) should act.
      'thrust_vectoring_pos_b': [0.0, 0.0, 0.0],

      # Freestream velocity along rotor axis [m/s] at which the blown wing
      # effect is in full effect and at which it has zero effect.  CFD does not
      # show significant added lift at low axial airspeeds.
      # TODO: Check this assumption of where the blown wing effect
      # should fade out.
      'full_blown_wing_freestream_vel': 6.0,
      'zero_blown_wing_freestream_vel': 10.0,

      # Minimum magnitude of freestream velocity [m/s] to use in calculating a
      # thrust coefficient. The thrust coefficient is calculated using wind
      # turbine nondimensionalization and is used to model rotor wake
      # impingement on the tail in a manner that is only valid at high
      # airspeeds.
      #
      # See b/116346702#comment23 for the reasoning behind this choice.
      'min_freestream_vel_for_thrust_coeff': 30.0,
  }
