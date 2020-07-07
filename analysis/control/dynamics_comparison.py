#!/usr/bin/python
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

"""Program for comparing a Wing model to the sim."""

import sys

import h5py
from makani.analysis.control import dynamics
from makani.analysis.control import geometry
from makani.config import mconfig
from makani.control import control_types
import numpy as np


def _CompareTimeStep(tether_force_model, wing, wing_telem):
  """Compares a Wing model to a sample point from telemetry.

  Args:
    tether_force_model: A dynamics.TetherForceModel.
    wing: A dynamics.Wing.
    wing_telem: A single WingTelemetry time sample.

  Returns:
    True if the dVb_center_of_mass and domega terms from telemetry
    match those computed from the wing model.
  """
  cartesian_dims = ['x', 'y', 'z']

  # Construct a WingState from this time sample.
  omega = np.matrix([[wing_telem['omega'][d]] for d in cartesian_dims])
  dcm_g2b = geometry.QuatToDcm(np.matrix([
      [wing_telem['q'][d]] for d in ['q0', 'q1', 'q2', 'q3']]))
  wing_pos_g = np.matrix([[wing_telem['Xg'][d]] for d in cartesian_dims])
  wing_vel_g = np.transpose(dcm_g2b) * np.matrix(
      [[wing_telem['Vb'][d]] for d in cartesian_dims])

  state = dynamics.WingState(omega_b=omega, dcm_g2b=dcm_g2b,
                             wing_vel_g=wing_vel_g, wing_pos_g=wing_pos_g)

  # Construct a WingInputs from this time sample.
  flaps = np.matrix([[wing_telem['flaps'][j]] for j in range(8)])
  thrust = np.matrix([[wing_telem['rotor_thrust_moment']['thrust']]])
  motor_moment = np.matrix([
      [wing_telem['rotor_thrust_moment']['moment'][d]]
      for d in cartesian_dims])
  inputs = dynamics.WingInputs(thrust=thrust, motor_moment=motor_moment,
                               flaps=flaps,
                               wind_g=np.matrix([[wing_telem['wind_g'][d]]
                                                 for d in cartesian_dims]))

  tether_pitch = wing_telem['tether_force_b']['pitch']
  tether_roll = wing_telem['tether_force_b']['roll']
  tether_force_b = wing_telem['tether_force_b']['tension'] * np.matrix([
      [np.cos(tether_roll) * np.sin(tether_pitch)],
      [-np.sin(tether_roll)],
      [np.cos(tether_roll) * np.cos(tether_pitch)]])
  tether_force_model.SetForce(np.transpose(dcm_g2b) * tether_force_b)

  state_dot = wing.CalcDeriv(state, inputs)

  # Check torques.
  domega = np.matrix([[wing_telem['domega'][d]] for d in cartesian_dims])
  assert np.linalg.norm(state_dot.domega_b - domega) < 1e-6

  # Check forces.
  vb_com_dot = wing.CalcDVbCom(state, state_dot)
  dvb_com = np.matrix([[wing_telem['dVb_center_of_mass'][d]]
                       for d in cartesian_dims])
  assert np.linalg.norm(vb_com_dot - dvb_com) < 1e-6


def main(argv):
  tether_force_model = dynamics.ConstantTetherForceModel(
      np.matrix(np.zeros((3, 1))))

  all_params = mconfig.MakeParams('common.all_params')
  wing = dynamics.Wing(
      all_params['system'], all_params['sim'],
      dynamics.SwigAeroModel(), tether_force_model)

  log_file = h5py.File(argv[1])

  sim = log_file['messages']['kAioNodeSimulator']
  sim_telem = sim['kMessageTypeSimTelemetry']['message']
  controller_a = log_file['messages']['kAioNodeControllerA']
  control_telem = controller_a['kMessageTypeControlDebug']['message']
  wing_telem = sim_telem['wing']

  control_idx = np.where(control_telem['flight_mode']
                         >= control_types.kFlightModeHoverFullLength)[0]
  assert control_idx.size > 0
  sim_idx = np.where(sim_telem['time']
                     > control_telem['time'][control_idx[0]])[0]
  assert sim_idx.size > 0

  for i in range(sim_idx[0], len(sim_telem)):
    _CompareTimeStep(tether_force_model, wing, wing_telem[i])


if __name__ == '__main__':
  main(sys.argv)
