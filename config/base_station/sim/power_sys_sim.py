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

"""Simulated power system parameters."""

from makani.config import mconfig
import numpy as np


@mconfig.Config(deps={
    'flight_plan': 'common.flight_plan',
    'motors': 'powertrain.motors',
    'power_sys': 'powertrain.power_sys'
})
def MakeParams(params):
  return {
      # Cutoff frequency [Hz] at which the ground inverter low pass
      # filters the measured tether current in the ground voltage
      # compensation controller.
      'current_filter_cutoff_freq': 20.0,

      # Ground voltage pole [rad/s], which models the dynamics of the
      # ground inverter.
      'ground_voltage_pole': -2.0 * np.pi * 30.0,

      # Minimum and maximum limits [V] for how much the ground voltage
      # compensation will change the nominal ground voltage.
      'min_ground_voltage_compensation': -700.0,
      'max_ground_voltage_compensation': 300.0,

      # From the model currently used in the simulator, the transfer
      # function between omega_cmd and omega is:
      #
      #     omega(s)                 H
      #   ------------ =  ------------------------------
      #   omega_cmd(s)    I_prop s + H + 2 k_P omega_nom
      #
      # where I_prop is the moment of inertia of the propeller, k_P is
      # the rotor's aerodynamic torque constant, and omega_nom is the
      # nominal angular rate.  H is the transfer function between
      # angular rate error and motor torque:
      #
      #   H(s) = (kp + ki/(s - p)) * 2 * pi * fc / (s + 2 * pi * fc)
      #
      # The motors were measured to have a bandwidth of approximately
      # 6 Hz. See go/makanimotorid.  We chose the following parameters
      # (kp_rotor_vel_err, ki_rotor_vel_err, fc_rotor_vel_err, and
      # rotor_vel_err_pole) to roughly match this bandwidth using the
      # current motor model.

      # Proportional gain [N-m-s/rad] and integral gain [N-m-s^2/rad]
      # between the rotor velocity error and the motor torque.  These
      # are based on the gains used in motor_foc.c.
      'kp_rotor_vel_err': 48.0,
      'ki_rotor_vel_err': 287.0,

      # Integrator pole [rad/s] for the rotor velocity error.  This
      # low frequency pole keeps the motor velocity loop from fighting
      # the stacking voltage loop.  It was chosen to be slow enough to
      # not affect the motor velocity loop significantly.
      'rotor_vel_err_pole': -2.0 * np.pi * 0.03,

      # Rate [rad/s] at which the speed integrator will approach the torque
      # command when in torque control mode.  In the firmware, which has a
      # discrete control law, this is instantaneous.  For now, just making it
      # fast compared to mechanical time constants.  16 Hz corresponds to about
      # 10 msec time constant.  Should be more than fast enough.
      # TODO: Figure out how to overwrite continuous variables and
      # make this occur instantaneously.
      'rotor_vel_err_torque_pole': -2.0 * np.pi * 16,

      # Voltage control parameters for maintaining stack balance.  This is
      # based off the parameters from voltage_control filter in
      # motor_stacking.c.  Gain in stacking.c * iq_to_torque conversion:
      # 23.8 [A/V] * 3.75 [Nm/A] = 89.25 [Nm/V].  Round up for simplicity.
      'kp_voltage_err': 90.0,
      'voltage_control_pole': -2.0 * np.pi * 1.0,
      'voltage_control_zero': -2.0 * np.pi * 20.0,

      # Filter cutoff frequency [Hz] for unwinding the battle
      # between speed loop and voltage control.
      # 2 * pi * 3 from stacking.c.
      'fc_stacking_speed_correction': 3.0,

      # Gain [(rad/s)/N-m] for speed correction.
      # From stacking.c: 1 (rad/sec)/A
      # From params.c and foc.c: kt is 3.75 N-m/A.
      'kp_stacking_speed_correction': 1.0 / 3.75,

      # The effective conductance [S] for draining a block capacitor
      # if a motor fault occurs.
      #
      # Note: 100.0 is probably a more realistic value, but that
      # causes the simulator's ODE system to be too stiff, and the
      # related dynamics aren't that important.  Should be increased
      # if trying to replicate shorting block.  For now, just trying
      # to slow down to improve simulation.
      'cap_drain_conductance': 0.5,

      'motor': params['motors'],

      # Rate limit [rad/s^2] on the angular rate of the motor command.
      # The motor controllers limit this (see
      # avionics/motor/firmware/motor_stacking.c) to prevent spikes in current.
      'omega_cmd_rate_limit': 500.0,

      # Speed command pole [rad/s] for filtering incoming speed commands.
      # Value is taken from speed_cmd_pole in stacking.c
      'speed_cmd_pole': -2.0 * np.pi * 16.0,

      # Minimum tether current [A] (positive is generating) and gain
      # from excess current to common mode speed reduction [rad/s-A].
      # If the tether current exceeds this limit the simulated stacked
      # power system reduces the common mode motor speeds based on
      # kp_excess_tether_current.  This limit does not exist in the
      # motor controller as of 2015-07-06.
      'min_tether_current': -1000.0,
      'kp_excess_tether_current': 6.0,

      # Voltage where stacking controller reverts to self-preservation.
      'voltage_average_upper_sat': 1550.0
  }
