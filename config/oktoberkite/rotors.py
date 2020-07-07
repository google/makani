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

"""Rotor parameters."""

from makani.config import mconfig
from makani.control import system_types as m
import numpy as np


@mconfig.Config(deps={
    'flight_plan': 'common.flight_plan',
    'propellers': 'prop.propellers',
    'wing_serial': 'common.wing_serial',
})
def MakeParams(params):
  """Returns parameters about the kite rotors."""

  # Motor rotor moment-of-inertia [kg-m^2].
  yasa_rotor_moment_of_inertia = 0.33

  bottom_row = [m.kMotorSbo, m.kMotorSbi, m.kMotorPbi, m.kMotorPbo]

  # Assign propeller versions.
  propeller_versions = [None for _ in range(m.kNumMotors)]

  # This arrangment has been shown to minimize net torques.
  # Oktoberkite uses very similar rotors as Rev4.
  propeller_versions[m.kMotorSbo] = m.kPropVersionRev4NegativeX
  propeller_versions[m.kMotorSbi] = m.kPropVersionRev4PositiveX
  propeller_versions[m.kMotorPbi] = m.kPropVersionRev4PositiveX
  propeller_versions[m.kMotorPbo] = m.kPropVersionRev4NegativeX
  propeller_versions[m.kMotorPto] = m.kPropVersionRev4PositiveX
  propeller_versions[m.kMotorPti] = m.kPropVersionRev4NegativeX
  propeller_versions[m.kMotorSti] = m.kPropVersionRev4NegativeX
  propeller_versions[m.kMotorSto] = m.kPropVersionRev4PositiveX

  # Set rotor inclination axis.
  # Oktoberkite rotors are not inclined 3deg down as the M600's are.
  rotor_tilt_down_deg = 0.0

  rotors = [None for _ in range(m.kNumMotors)]
  for r in range(m.kNumMotors):
    rotors[r] = {
        # Normal vector to the propeller plane.
        'axis': [np.cos(rotor_tilt_down_deg * np.pi / 180.0), 0.0,
                 np.sin(rotor_tilt_down_deg * np.pi / 180.0)],

        # Direction cosine matrix from body to rotor frame.
        'dcm_b2r': {'d': [[np.cos(-rotor_tilt_down_deg * np.pi / 180.0), 0.0,
                           np.sin(-rotor_tilt_down_deg * np.pi / 180.0)],
                          [0.0, 1.0, 0.0],
                          [-np.sin(-rotor_tilt_down_deg * np.pi / 180.0), 0.0,
                           np.cos(-rotor_tilt_down_deg * np.pi / 180.0)]]},

        # Local pressure coefficient [#] at the rotor position.  The
        # pressure coefficient, C_P, is related to local airspeed
        # through the equation:
        #
        #    C_P = 1 - (v / v_freestream)^2
        #
        # There is a significant difference in airspeeds between the top
        # and bottom propellers caused by the lift of the wing.
        #
        # TODO: OktKite. Input proper pressure coefficient due to wing
        # circulation. Currently derived from M600 CFD with the slatted
        # kite at 4 deg alpha (https://goo.gl/yfkJJS):
        'local_pressure_coeff': 0.1448 if r in bottom_row else -0.1501,

        # The rotor direction, diameter [m] and moment of inertia [kg
        # m^2] are set from the corresponding propeller's information.
        'version': propeller_versions[r],
        'dir': params['propellers'][propeller_versions[r]]['dir'],
        'D': params['propellers'][propeller_versions[r]]['D'],
        'I': (yasa_rotor_moment_of_inertia +
              params['propellers'][propeller_versions[r]]['I']),
    }
    # We check that the rotor axis is normalized. because it is used
    # to determine the force-moment conversion matrix in
    # rotor_control.py.
    assert abs(np.linalg.norm(rotors[r]['axis']) - 1.0) < 1e-9

  # Rotor positions [m].
  # TODO: Confirm values, provide source.
  # Looks like M600 x and z values with new y values.
  rotors[m.kMotorSbo]['pos'] = [1.613, 4.5, 1.597]
  rotors[m.kMotorSbi]['pos'] = [1.613, 1.5, 1.597]
  rotors[m.kMotorPbi]['pos'] = [1.613, -1.5, 1.597]
  rotors[m.kMotorPbo]['pos'] = [1.613, -4.5, 1.597]
  rotors[m.kMotorPto]['pos'] = [1.960, -4.5, -1.216]
  rotors[m.kMotorPti]['pos'] = [1.960, -1.5, -1.216]
  rotors[m.kMotorSti]['pos'] = [1.960, 1.5, -1.216]
  rotors[m.kMotorSto]['pos'] = [1.960, 4.5, -1.216]

  return rotors
