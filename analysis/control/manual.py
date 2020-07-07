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

"""Off-tether, manual flight analysis module."""

import sys

from makani.analysis.control import dynamics
from makani.analysis.control import geometry
from makani.config import mconfig
from makani.control import control_types
import numpy as np
from scipy import optimize


def _Vec3Zero():
  return np.matrix([0.0, 0.0, 0.0]).T


class TrimSelector(object):
  """Class for trimming and calculating gains for manual flight."""

  def __init__(self, system_params, control_params, sim_params):
    """Constructs a TrimSelector object from parameters."""
    motor_model = dynamics.PureForceMomentMotorModel(
        system_params['rotors'], system_params['wing']['center_of_mass_pos'])

    self._wing = dynamics.Wing(
        system_params, sim_params, dynamics.SwigAeroModel(),
        motor_model, dynamics.ConstantTetherForceModel(_Vec3Zero()))

    self._initial_flap_offsets = np.matrix(
        control_params['manual']['output']['flap_offsets']).T

    self._angle_of_attack = (
        control_params['manual']['auto_glide']['angle_of_attack'])

  def CalcTrim(self):
    """Find trim conditions for the wing.

    Determines attitude trim for stabilized manual flight, assuming zero ambient
    wind speed. The table below provides a rough idea of the expected trim
    relationships.

        Trim Input            | Trim Output
        ----------------------+----------------------
        roll                  | lateral acceleration
        (glide angle,         | (vertical acceleration,
         freestream velocity) |  horizontal acceleration)
        aileron               | roll moment
        elevator              | pitch moment
        rudder                | yaw moment

    Returns:
      A tuple (state, inputs) where state is a WingState and inputs is a
      dynamics.WingInputs.
    """
    state_0 = dynamics.WingState(
        omega_b=_Vec3Zero(),
        dcm_g2b=geometry.AngleToDcm(0.0, 0.0, 0.0),
        wing_vel_g=np.matrix([20.0, 0.0, 0.0]).T,
        wing_pos_g=_Vec3Zero())

    inputs_0 = dynamics.WingInputs(
        thrust=np.matrix([[0.0]]),
        motor_moment=_Vec3Zero(),
        flaps=self._initial_flap_offsets.copy(),
        wind_g=_Vec3Zero())

    angle_of_attack = self._angle_of_attack

    def GetTrimStateAndInputs(x):
      """Returns a WingState from trim variables."""

      # Unpack trim variables.
      glide_angle, dv_app, roll, d_aileron, d_elevator, d_rudder = x

      # Calculate trim state.
      v_app = 20.0 + dv_app
      pitch = -glide_angle + angle_of_attack
      yaw = 0.0
      state = dynamics.WingState(
          omega_b=state_0.omega_b,
          dcm_g2b=geometry.AngleToDcm(yaw, pitch, roll),
          wing_vel_g=np.matrix([v_app * np.cos(glide_angle),
                                0.0,
                                v_app * np.sin(glide_angle)]).T,
          wing_pos_g=state_0.wing_pos_g)

      # Calculate trim inputs.
      flaps = inputs_0.flaps.copy()
      flaps[[control_types.kFlapA1, control_types.kFlapA2]] += -d_aileron
      flaps[[control_types.kFlapA7, control_types.kFlapA8]] += d_aileron
      flaps[control_types.kFlapEle] += d_elevator
      flaps[control_types.kFlapRud] += d_rudder
      inputs = dynamics.WingInputs(thrust=inputs_0.thrust,
                                   motor_moment=inputs_0.motor_moment,
                                   flaps=flaps, wind_g=inputs_0.wind_g)

      return state, inputs

    def TrimFunction(x):
      """Wrapper function for trimming the wing."""
      state, inputs = GetTrimStateAndInputs(x)
      state_dot = self._wing.CalcDeriv(state, inputs)

      return [state_dot.dwing_vel_g[0, 0],
              state_dot.dwing_vel_g[1, 0],
              state_dot.dwing_vel_g[2, 0],
              state_dot.domega_b[0, 0],
              state_dot.domega_b[1, 0],
              state_dot.domega_b[2, 0]]

    x = optimize.fsolve(TrimFunction, np.zeros((6, 1)))

    return GetTrimStateAndInputs(x)

  def PrintTrim(self, state, inputs):
    """Print information relevant to a trimmed state.

    Args:
      state: WingState structure.
      inputs: dynamics.WingInputs structure.
    """
    state_dot = self._wing.CalcDeriv(state, inputs)

    v_rel, alpha, beta = state.CalcAerodynamicAngles(inputs.wind_g)

    # Fixing total thrust coefficient to 0.0 for this application.
    thrust_coeff = 0.0
    _, cf, _ = self._wing.CalcAeroForceMomentPos(
        v_rel, alpha, beta, state.omega_b, inputs.flaps, thrust_coeff)

    yaw, pitch, roll = geometry.DcmToAngle(state.dcm_g2b)

    dcm_w2b = geometry.AngleToDcm(-beta, alpha, 0.0, order='ZYX')
    cf_w = dcm_w2b.T * cf

    values = [[
        ('Roll [deg]', np.rad2deg(roll)),
        ('Pitch [deg]', np.rad2deg(pitch)),
        ('Yaw [deg]', np.rad2deg(yaw)),
    ], [
        ('Port ail. [deg]', np.rad2deg(inputs.flaps[control_types.kFlapA1])),
        ('Starboard ail. [deg]',
         np.rad2deg(inputs.flaps[control_types.kFlapA8])),
        ('Ele. [deg]', np.rad2deg(inputs.flaps[control_types.kFlapEle])),
        ('Rud. [deg]', np.rad2deg(inputs.flaps[control_types.kFlapRud]))
    ], [
        ('Vrel [m/s]', v_rel),
        ('Alpha [deg]', np.rad2deg(alpha)),
        ('Beta [deg]', np.rad2deg(beta))
    ], [
        ('V X [m/s]', state.wing_vel_g[0]),
        ('V Y [m/s]', state.wing_vel_g[1]),
        ('V Z [m/s]', state.wing_vel_g[2])
    ], [
        ('A X [m/s^2]', state_dot.dwing_vel_g[0]),
        ('A Y [m/s^2]', state_dot.dwing_vel_g[1]),
        ('A Z [m/s^2]', state_dot.dwing_vel_g[2])
    ], [
        ('Pdot [rad/s^2]', state_dot.domega_b[0]),
        ('Qdot [rad/s^2]', state_dot.domega_b[1]),
        ('Rdot [rad/s^2]', state_dot.domega_b[2])
    ], [
        ('CL [#]', -cf_w[2]),
        ('CD [#]', -cf_w[0]),
        ('Glide ratio [#]', cf_w[2] / cf_w[0]),
    ]]

    for line_values in values:
      for name, value in line_values:
        print '%20s: %10.3f' % (name, value)


def main(unused_argv):
  all_params = mconfig.MakeParams('common.all_params')
  trimmer = TrimSelector(all_params['system'],
                         all_params['control'],
                         all_params['sim'])

  trimmer.PrintTrim(*trimmer.CalcTrim())


if __name__ == '__main__':
  main(sys.argv)
