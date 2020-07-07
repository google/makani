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

"""Worker for a power curve batch simulation."""

import json
import logging
import sys

import gflags
import h5py
from makani.lib.python import build_info
from makani.lib.python.batch_sim import batch_sim_util
from makani.lib.python.batch_sim import flight_modes
from makani.lib.python.batch_sim import worker as worker_base
import numpy

FLAGS = gflags.FLAGS


class PowerCurveSimWorker(worker_base.BatchSimWorker):
  """Worker for a power curve batch simulation."""

  def __init__(self, *args, **kwargs):
    super(PowerCurveSimWorker, self).__init__(*args, **kwargs)

  def _ProcessSimOutput(self, config_id, sim_success, log_file_name):
    """Calculates the mean power for each log file.

    Also calculates statistics (minimum, mean, maximum, and standard deviation)
    on many of the critical flight parameters such as the angle-of-attack, roll
    angle, tension, curvature, etc.

    Args:
      config_id: ID of the parameter file used.
      sim_success: Whether the sim was successful.
      log_file_name: Name of the log file to be parsed.

    Returns:
      See parent class.
    """

    # TODO: Improve the following attempt to capture simulator
    # output such that it only captures the simulator output (i.e. via
    # some use of "tee"), works when run locally, etc.
    sim_error_message = ''
    if not sim_success:
      sim_error_message = batch_sim_util.Tail('/var/log/syslog', 100)

    log_file = h5py.File(log_file_name, 'r')

    system_params = log_file['parameters']['system_params']
    wing_mass = system_params['wing']['m']

    simulator = log_file['messages']['kAioNodeSimulator']
    sim_telem = simulator['kMessageTypeSimTelemetry']['message']

    controller = log_file['messages']['kAioNodeControllerA']
    control_telem = controller['kMessageTypeControlDebug']['message']

    crosswind_control_inds = numpy.where(numpy.logical_or(
        control_telem['flight_mode'] ==
        flight_modes.GetFlightModes()['kFlightModeCrosswindNormal'],
        control_telem['flight_mode'] ==
        flight_modes.GetFlightModes()['kFlightModeCrosswindPrepTransOut']))[0]
    # The implicit length test fails with an error about using using
    # numpy.any or numpy.all to check the truth value of a numpy
    # array.
    #
    # pylint: disable=g-explicit-length-test
    if len(crosswind_control_inds) > 0:
      crosswind_start_time = control_telem['time'][crosswind_control_inds[0]]
      crosswind_end_time = control_telem['time'][crosswind_control_inds[-1]]
      crosswind_sim_inds = numpy.where(numpy.logical_and(
          crosswind_start_time <= sim_telem['time'],
          sim_telem['time'] <= crosswind_end_time))
    else:
      crosswind_sim_inds = []

    trans_in_control_inds = numpy.where(
        control_telem['flight_mode'] ==
        flight_modes.GetFlightModes()['kFlightModeTransIn'])[0]
    if len(trans_in_control_inds) > 0:
      trans_in_start_time = control_telem['time'][trans_in_control_inds[0]]
      trans_in_end_time = control_telem['time'][trans_in_control_inds[-1]]
      trans_in_sim_inds = numpy.where(numpy.logical_and(
          trans_in_start_time <= sim_telem['time'],
          sim_telem['time'] <= trans_in_end_time))
    else:
      trans_in_sim_inds = []

    crosswind_telem = control_telem['crosswind']
    flight_plane_x = (
        crosswind_telem['current_pos_cw']['x'][crosswind_control_inds])
    flight_plane_y = (
        crosswind_telem['current_pos_cw']['y'][crosswind_control_inds])
    radius_cmd = crosswind_telem['path_radius_target'][crosswind_control_inds]
    radii = numpy.sqrt(flight_plane_x**2 + flight_plane_y**2)
    tether_force_b = control_telem['state_est']['tether_force_b']
    tensions = tether_force_b['sph']['tension'][crosswind_control_inds]
    tether_pitches = tether_force_b['sph']['pitch'][crosswind_control_inds]
    airspeed_cmds = crosswind_telem['airspeed_cmd'][crosswind_control_inds]
    apparent_wind = control_telem['state_est']['apparent_wind']
    airspeeds = apparent_wind['sph']['v'][crosswind_control_inds]
    alpha_cmds = crosswind_telem['alpha_cmd'][crosswind_control_inds]
    alphas = apparent_wind['sph']['alpha'][crosswind_control_inds]
    beta_cmds = crosswind_telem['beta_cmd'][crosswind_control_inds]
    betas = apparent_wind['sph']['beta'][crosswind_control_inds]
    tether_roll_cmds = (
        crosswind_telem['tether_roll_cmd'][crosswind_control_inds])
    tether_rolls = tether_force_b['sph']['roll'][crosswind_control_inds]
    geom_curvature_cmds = crosswind_telem['k_geom_cmd'][crosswind_control_inds]
    geom_curvatures = crosswind_telem['k_geom_curr'][crosswind_control_inds]
    aero_curvature_cmds = crosswind_telem['k_aero_cmd'][crosswind_control_inds]
    aero_curvatures = crosswind_telem['k_aero_curr'][crosswind_control_inds]

    # Extract values that define the flight envelope on a
    # per-flight-mode basis.
    crosswind_airspeeds = (
        sim_telem['wing']['apparent_wind_b']['v'][crosswind_sim_inds])
    crosswind_alphas = (
        sim_telem['wing']['apparent_wind_b']['alpha'][crosswind_sim_inds])
    crosswind_betas = (
        sim_telem['wing']['apparent_wind_b']['beta'][crosswind_sim_inds])
    crosswind_tensions = (
        sim_telem['wing']['tether_force_b']['tension'][crosswind_sim_inds])
    crosswind_tether_pitches = (
        sim_telem['wing']['tether_force_b']['pitch'][crosswind_sim_inds])
    crosswind_tether_rolls = (
        sim_telem['wing']['tether_force_b']['roll'][crosswind_sim_inds])
    crosswind_acc_b_xs = (
        sim_telem['wing']['fm_total']['force']['x'][crosswind_sim_inds] /
        wing_mass)
    crosswind_acc_b_ys = (
        sim_telem['wing']['fm_total']['force']['y'][crosswind_sim_inds] /
        wing_mass)
    crosswind_acc_b_zs = (
        sim_telem['wing']['fm_total']['force']['z'][crosswind_sim_inds] /
        wing_mass)
    crosswind_omega_b_xs = (
        sim_telem['wing']['omega']['x'][crosswind_sim_inds])
    crosswind_omega_b_ys = (
        sim_telem['wing']['omega']['y'][crosswind_sim_inds])
    crosswind_omega_b_zs = (
        sim_telem['wing']['omega']['z'][crosswind_sim_inds])
    crosswind_domega_b_xs = (
        sim_telem['wing']['domega']['x'][crosswind_sim_inds])
    crosswind_domega_b_ys = (
        sim_telem['wing']['domega']['y'][crosswind_sim_inds])
    crosswind_domega_b_zs = (
        sim_telem['wing']['domega']['z'][crosswind_sim_inds])
    # TODO: We are using aerodynamic power at the rotors
    # as our power measurement for now because it is a more robust
    # estimate of power and gives the correct answer whether we use
    # the Stacked or SimplePowerSys.  We should switch back to
    # electrical power once the SimplePowerSys reports an accurate
    # power.
    crosswind_powers = (
        numpy.sum(sim_telem['rotors']['aero_power'][crosswind_sim_inds], 1))

    trans_in_airspeeds = (
        sim_telem['wing']['apparent_wind_b']['v'][trans_in_sim_inds])
    trans_in_alphas = (
        sim_telem['wing']['apparent_wind_b']['alpha'][trans_in_sim_inds])
    trans_in_betas = (
        sim_telem['wing']['apparent_wind_b']['beta'][trans_in_sim_inds])
    trans_in_tensions = (
        sim_telem['wing']['tether_force_b']['tension'][trans_in_sim_inds])
    trans_in_tether_pitches = (
        sim_telem['wing']['tether_force_b']['pitch'][trans_in_sim_inds])
    trans_in_tether_rolls = (
        sim_telem['wing']['tether_force_b']['roll'][trans_in_sim_inds])
    trans_in_acc_b_xs = (
        sim_telem['wing']['fm_total']['force']['x'][trans_in_sim_inds] /
        wing_mass)
    trans_in_acc_b_ys = (
        sim_telem['wing']['fm_total']['force']['y'][trans_in_sim_inds] /
        wing_mass)
    trans_in_acc_b_zs = (
        sim_telem['wing']['fm_total']['force']['z'][trans_in_sim_inds] /
        wing_mass)
    trans_in_omega_b_xs = (
        sim_telem['wing']['omega']['x'][trans_in_sim_inds])
    trans_in_omega_b_ys = (
        sim_telem['wing']['omega']['y'][trans_in_sim_inds])
    trans_in_omega_b_zs = (
        sim_telem['wing']['omega']['z'][trans_in_sim_inds])
    trans_in_domega_b_xs = (
        sim_telem['wing']['domega']['x'][trans_in_sim_inds])
    trans_in_domega_b_ys = (
        sim_telem['wing']['domega']['y'][trans_in_sim_inds])
    trans_in_domega_b_zs = (
        sim_telem['wing']['domega']['z'][trans_in_sim_inds])
    trans_in_powers = (
        numpy.sum(sim_telem['rotors']['aero_power'][trans_in_sim_inds], 1))

    flaps = control_telem['control_output']['flaps'][crosswind_control_inds]
    rotors = (
        control_telem['control_output']['motor_speed_upper_limit']
        [crosswind_control_inds])
    faults = control_telem['faults']['code'][crosswind_control_inds]

    time_series = {
        'radius': radii,
        'radius_error': radii - radius_cmd,
        'tension': tensions,
        'tether_pitch': tether_pitches,
        'airspeed_cmd': airspeed_cmds,
        'airspeed': airspeeds,
        'airspeed_error': airspeed_cmds - airspeeds,
        'alpha_cmd': alpha_cmds,
        'alpha': alphas,
        'alpha_error': alpha_cmds - alphas,
        'beta_cmd': beta_cmds,
        'beta': betas,
        'beta_error': beta_cmds - betas,
        'tether_roll_cmd': tether_roll_cmds,
        'tether_roll': tether_rolls,
        'tether_roll_error': tether_roll_cmds - tether_rolls,
        'geom_curvature_cmd': geom_curvature_cmds,
        'geom_curvature': geom_curvatures,
        'geom_curvature_error': geom_curvature_cmds - geom_curvatures,
        'aero_curvature_cmd': aero_curvature_cmds,
        'aero_curvature': aero_curvatures,
        'aero_curvature_error': aero_curvature_cmds - aero_curvatures,
        'flaps': flaps,
        'rotors': rotors,
        'crosswind_airspeed': crosswind_airspeeds,
        'crosswind_alpha': crosswind_alphas,
        'crosswind_beta': crosswind_betas,
        'crosswind_tension': crosswind_tensions,
        'crosswind_tether_pitch': crosswind_tether_pitches,
        'crosswind_tether_roll': crosswind_tether_rolls,
        'crosswind_acc_b_x': crosswind_acc_b_xs,
        'crosswind_acc_b_y': crosswind_acc_b_ys,
        'crosswind_acc_b_z': crosswind_acc_b_zs,
        'crosswind_omega_b_x': crosswind_omega_b_xs,
        'crosswind_omega_b_y': crosswind_omega_b_ys,
        'crosswind_omega_b_z': crosswind_omega_b_zs,
        'crosswind_domega_b_x': crosswind_domega_b_xs,
        'crosswind_domega_b_y': crosswind_domega_b_ys,
        'crosswind_domega_b_z': crosswind_domega_b_zs,
        'crosswind_power': crosswind_powers,
        'trans_in_airspeed': trans_in_airspeeds,
        'trans_in_alpha': trans_in_alphas,
        'trans_in_beta': trans_in_betas,
        'trans_in_tension': trans_in_tensions,
        'trans_in_tether_pitch': trans_in_tether_pitches,
        'trans_in_tether_roll': trans_in_tether_rolls,
        'trans_in_acc_b_x': trans_in_acc_b_xs,
        'trans_in_acc_b_y': trans_in_acc_b_ys,
        'trans_in_acc_b_z': trans_in_acc_b_zs,
        'trans_in_omega_b_x': trans_in_omega_b_xs,
        'trans_in_omega_b_y': trans_in_omega_b_ys,
        'trans_in_omega_b_z': trans_in_omega_b_zs,
        'trans_in_domega_b_x': trans_in_domega_b_xs,
        'trans_in_domega_b_y': trans_in_domega_b_ys,
        'trans_in_domega_b_z': trans_in_domega_b_zs,
        'trans_in_power': trans_in_powers,
    }

    sim_params = log_file['parameters']['sim_params']
    joystick_throttle = sim_params['joystick_sim']['updates'][0, 0]['value']
    wind_speed = sim_params['phys_sim']['wind_speed'][0]

    if not sim_success:
      logging.warning('Simulation failed for throttle = %f, wind_speed = %f',
                      joystick_throttle, wind_speed)

    full_output = {
        'git_commit': build_info.GetGitSha(),
        'config_id': config_id,
        'sim_success': sim_success,
        'sim_error_message': sim_error_message,
        'parameters': {
            'joystick_throttle': joystick_throttle,
            'wind_speed': wind_speed},
        'statistics': {
        }
    }

    def _calc_statistics(data):
      """Compute common statistics of the given data."""
      output = {
          'min': numpy.min(data) if len(data) > 0 else 0.0,
          'max': numpy.max(data) if len(data) > 0 else 0.0,
          'mean': numpy.mean(data) if len(data) > 0 else 0.0,
          'std': numpy.std(data) if len(data) > 0 else 0.0,
      }
      if not numpy.any(numpy.isnan(data)):
        output['histogram'] = [h.tolist() for h in numpy.histogram(data)]
      else:
        output['histogram'] = []
      return output

    for key, value in time_series.iteritems():
      if len(numpy.shape(value)) == 1:
        full_output['statistics'][key] = _calc_statistics(value)
      else:
        full_output['statistics'][key] = [_calc_statistics(value[:, i])
                                          for i in range(numpy.shape(value)[1])]

    full_output['faults'] = [int(f) for f in numpy.bitwise_or.reduce(faults)]

    log_file.close()

    return json.dumps(full_output, indent=2, separators=(',', ': '))


def main(argv):
  worker_base.InitMain(argv)
  worker = PowerCurveSimWorker()
  worker.Run()


if __name__ == '__main__':
  main(sys.argv)
