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

r"""Local client for running a power curve batch simulation.

Sample usage, running 10 configs processed by 2 workers:
    bazel run //analysis:power_curve_batch_sim -- \
      --sim_name=my_happy_power_curve \
      --wind_speeds='3.0, 15.0, 10' \
      --num_workers=2 \
      --output_dir=/tmp/power_curve
"""

import datetime
import importlib
import itertools
import logging
import os
import shutil
import sys
import textwrap

import gflags
import h5py
import makani
from makani.config import mconfig
from makani.control import control_types
from makani.lib.python import c_helpers
from makani.lib.python import flag_types
from makani.lib.python.batch_sim import batch_sim_util
from makani.lib.python.batch_sim import client as client_base
from makani.sim import sim_types
import matplotlib
import numpy

gflags.DEFINE_string('output_dir', makani.HOME + '/logs/power_curve',
                     'Directory in which to output .png file and .html page.  '
                     'This will be created if it does not exist.')

flag_types.DEFINE_linspace('wind_speeds', '3.0, 15.0, 20',
                           'Linspace range of wind speed values.',
                           nonempty=True)

# The lower bound should be set to the "high power" throttle position, which by
# convention is 0.5 in the controller.
flag_types.DEFINE_linspace('joystick_throttles', '0.5, 1.0, 1',
                           'Linspace range of joystick throttle values.',
                           nonempty=True, increasing=True)

gflags.DEFINE_float('sim_time', 600.0,
                    'Final time for each simulation.')

FLAGS = gflags.FLAGS

annotations = {
    # Maximum tether tension [N] (M600 spec.).
    'T_max': 250e3,

    # "Never exceed" tether tension [N] (M600 spec.).
    'T_NE': 280e3,

    # Maximum instantaneous aerodynamic power [W].
    'P_max': 800e3,

    # Reasonable power curve (2015-02-19).  This is a power curve from the
    # trajectory optimization tool for 'medium' shear coefficient (0.1).
    #
    # TODO: Compensate for propeller efficiency and wind altitude.
    'power_curve': {
        # Wind speed [m/s] at 80 m with shear coefficient of 0.1.
        'v_wind': numpy.linspace(4.0, 15.5, 24),

        # Shaft power [W].
        'P': 1e3 * numpy.array([-25.0, 2.0, 32.0, 69.0, 113.0, 165.0, 225.0,
                                295.0, 373.0, 453.0, 526.0, 591.0, 648.0, 686.0,
                                705.0, 706.0, 707.0, 707.0, 707.0, 707.0, 707.0,
                                707.0, 707.0, 707])
    }
}


def FindClusters(xs, flags, active_flag_val):
  """Find repeated ranges of a certain value in a list.

  Args:
    xs: x coordinate of each value.
    flags: values corresponding to each x-coordinate.
    active_flag_val: the flag value we're searching for.

  Example:
    FindClusters([0, 1, 2, 3, 4], [True, False, True, True, False], True)
    --> [(0, 0), (2, 3)]

  Returns:
    A list of tuples is returned, one tuple per identified cluster,
    where each tuple contains the first and last x-coordinate of the
    corresponding cluster.

  """
  # When working with a list of (x, flag) tuples, it's handy to have
  # some getter functions.
  get_xval = lambda tup: tup[0]
  get_flag = lambda tup: tup[1]

  # Combine the x-coordinates with their associated flags.
  tuple_seq = zip(xs, flags)

  # Find continuous subsequences with the appropriate flag value.
  runs = [list(subseq)
          for flag_val, subseq in itertools.groupby(tuple_seq, get_flag)
          if flag_val == active_flag_val]

  # Return the start and end x-coordinate for each run.
  return [(get_xval(run[0]), get_xval(run[-1])) for run in runs]


class PowerCurveSimClient(client_base.BatchSimClient):
  """Client for a power curve batch simulation."""

  _BASE_PATH = 'analysis/power_curve_batch_sim'

  def __init__(self, **kwargs):
    super(PowerCurveSimClient, self).__init__(**kwargs)

  def _GenerateConfigs(self):
    # Create a JSON file describing the simulation parameters for each
    # wind speed.  We override the flight plan because we are only
    # interested in power production during crosswind flight.  We
    # override the aerodynamic and other simulation parameters to make
    # the most realistic simulation of power production.
    for joystick_throttle in FLAGS.joystick_throttles:
      for v_wind in FLAGS.wind_speeds:
        overrides = {
            'system': {'flight_plan': sim_types.kFlightPlanStartDownwind},
            'sim': {
                'joystick_sim': {
                    'num_updates': 1,
                    'updates': [{
                        't_update': 0.0,
                        'type': sim_types.kSimJoystickUpdateThrottle,
                        'enum_value': sim_types.kSimJoystickThrottleManual,
                        'value': float(joystick_throttle),
                    }] * sim_types.MAX_JOYSTICK_UPDATES
                },
                # It is necessary to add the perch here so the winch
                # updates and payout is set correctly.  This helps the
                # wing hover properly by setting the proper
                # feed-forward thrust when the simulation begins.
                'sim_opt': (sim_types.kSimOptImperfectSensors
                            | sim_types.kSimOptPerch
                            | sim_types.kSimOptStackedPowerSystem),
                'phys_sim': {
                    'wind_model': sim_types.kWindModelDrydenTurbulence,
                    'wind_speed': float(v_wind),
                },
                'sim_time': FLAGS.sim_time
            }
        }

        yield mconfig.MakeParams('common.all_params', overrides,
                                 override_method='derived')

  def _ShadeFailureRegions(self, pylab, wind_speeds, success_flags):
    for r in FindClusters(wind_speeds, success_flags, False):
      pylab.axvspan(r[0] - 0.5, r[1] + 0.5, color='red', alpha=0.9)

  def _WriteReport(self, outputs):
    report_keys = [
        'crosswind_airspeed',
        'crosswind_alpha',
        'crosswind_beta',
        'crosswind_tension',
        'crosswind_tether_pitch',
        'crosswind_tether_roll',
        'crosswind_acc_b_x',
        'crosswind_acc_b_y',
        'crosswind_acc_b_z',
        'crosswind_omega_b_x',
        'crosswind_omega_b_y',
        'crosswind_omega_b_z',
        'crosswind_domega_b_x',
        'crosswind_domega_b_y',
        'crosswind_domega_b_z',
        'crosswind_power',
        'trans_in_airspeed',
        'trans_in_alpha',
        'trans_in_beta',
        'trans_in_tension',
        'trans_in_tether_pitch',
        'trans_in_tether_roll',
        'trans_in_acc_b_x',
        'trans_in_acc_b_y',
        'trans_in_acc_b_z',
        'trans_in_omega_b_x',
        'trans_in_omega_b_y',
        'trans_in_omega_b_z',
        'trans_in_domega_b_x',
        'trans_in_domega_b_y',
        'trans_in_domega_b_z',
        'trans_in_power',
    ]

    # Sort the outputs so the report is in the same order as the
    # config IDs.
    outputs.sort(key=lambda x: x['config_id'])

    lines = []
    lines.append(textwrap.dedent("""
        ****************************
        Power curve report
        Date: {0}
        Git commit: {1}
        ****************************

    """.format(str(datetime.date.today()),
               outputs[0]['git_commit'])))
    for o in outputs:
      lines.append(textwrap.dedent("""


          Run case: {0}
          Wind speed: {1} m/s
          Joystick throttle: {2}
      """.format(o['config_id'],
                 o['parameters']['wind_speed'],
                 o['parameters']['joystick_throttle'])))
      lines.append('{0} | {1} | {2} | {3} | {4}'.format('Parameter'.center(24),
                                                        'Minimum'.center(11),
                                                        'Maximum'.center(11),
                                                        'Mean'.center(11),
                                                        'Std.'.center(11)))
      lines.append('-' * 80)
      for key in report_keys:
        lines.append(
            '{0} | {1:11.2f} | {2:11.2f} | {3:11.2f} | {4:11.2f}'.format(
                key.ljust(24),
                o['statistics'][key]['min'],
                o['statistics'][key]['max'],
                o['statistics'][key]['mean'],
                o['statistics'][key]['std']))
      lines.append('-' * 80)

    with open(FLAGS.output_dir + '/power_curve_report.txt', 'w') as f:
      f.write('\n'.join(lines))

  @client_base.JsonReducer
  def _ReduceWorkerOutput(self, outputs):
    matplotlib.use('Agg')
    pylab = importlib.import_module('pylab')

    # Create directory to place output files.
    if not os.path.exists(FLAGS.output_dir):
      os.makedirs(FLAGS.output_dir)

    # Write a text report of extreme values experienced during flight.
    self._WriteReport(outputs)

    # pylint: disable=g-long-lambda
    outputs.sort(cmp=lambda x, y: cmp(x['parameters']['wind_speed'],
                                      y['parameters']['wind_speed']))
    data_all = batch_sim_util.CollateOutputs(outputs)
    data = batch_sim_util.CollateOutputs(
        [o for o in outputs if (o['parameters']['joystick_throttle']
                                == FLAGS.joystick_throttles[0])])
    stats_all = data_all['statistics']
    stats = data['statistics']
    wind_speeds_all = data_all['parameters']['wind_speed']
    wind_speeds = data['parameters']['wind_speed']

    # Save power curve to HDF5 file.
    #
    # TODO: Make a Python dictionary to HDF5 conversion
    # function.
    data_file = h5py.File(os.path.join(FLAGS.output_dir, 'data.h5'), 'w')
    data_file.create_dataset('wind_speed', data=wind_speeds)
    data_file.create_dataset('crosswind_power',
                             data=stats['crosswind_power']['mean'][:])
    data_file.create_dataset('sim_success', data=data['sim_success'])

    flap_labels = [
        'Port aileron (outer)',
        'Port aileron (inner)',
        'Center flap (port)',
        'Center flap (starboard)',
        'Starboard aileron (inner)',
        'Starboard aileron (outer)',
        'Elevator',
        'Rudder'
    ]

    subsystem_helper = c_helpers.EnumHelper(
        'SubsystemLabel', control_types, prefix='kSubsys')

    # Plot mean flap deflections.
    pylab.figure().patch.set_alpha(0.0)
    for flap_num in [0, 2, 4, 6, 7]:
      pylab.plot(wind_speeds, stats['flaps'][flap_num]['mean'],
                 label=flap_labels[flap_num])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.6, 0.1])
    pylab.title('Mean flap deflections')
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Deflection [rad]')
    pylab.legend(framealpha=0.5)
    pylab.grid()
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.savefig(FLAGS.output_dir + '/mean_flap_deflections.svg')
    pylab.close()

    # Plot standard deviation flap deflections.
    pylab.figure().patch.set_alpha(0.0)
    for flap_num in [0, 2, 4, 6, 7]:
      pylab.plot(wind_speeds, stats['flaps'][flap_num]['std'],
                 label=flap_labels[flap_num])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.0, 0.2])
    pylab.title('Standard deviation flap deflections')
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Deflection [rad]')
    pylab.legend(framealpha=0.5)
    pylab.grid()
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.savefig(FLAGS.output_dir + '/standard_deviation_flap_deflections.svg')
    pylab.close()

    # Plot controller faults.
    pylab.figure().patch.set_alpha(0.0)
    num_subsystems = numpy.shape(data['faults'])[0]
    # The values in data['faults'] are bitfields (see the FaultType
    # enum).  Here we want just a faults / no faults indication.
    pylab.imshow(numpy.array(data['faults']) != 0,
                 extent=[numpy.min(wind_speeds), numpy.max(wind_speeds),
                         0, num_subsystems],
                 origin='lower',
                 interpolation='none',
                 cmap=pylab.get_cmap('RdYlGn_r'),
                 aspect=0.3)
    pylab.title('Controller faults')
    pylab.xlabel('Wind speed [m/s]')
    pylab.gca().set_yticks(numpy.linspace(0.5, num_subsystems - 0.5,
                                          num_subsystems))
    pylab.gca().set_yticklabels([subsystem_helper.ShortName(i)
                                 for i in range(num_subsystems)],
                                fontsize=8)
    pylab.savefig(FLAGS.output_dir + '/faults_vs_wind_speed.png')
    pylab.close()

    # Plot power curves by throttle position.
    pylab.figure().patch.set_alpha(0.0)
    throttles_all = data_all['parameters']['joystick_throttle']
    throttle_list = sorted(set(throttles_all))
    for throttle in throttle_list:
      pylab.plot(wind_speeds_all[throttles_all == throttle],
                 stats_all['crosswind_power']['mean'][throttles_all ==
                                                      throttle] / 1e3,
                 label='%0.2f' % throttle)
    # Reset the color cycle so we get the same colors for tension.
    pylab.gca().set_color_cycle(None)
    for throttle in throttle_list:
      pylab.plot(wind_speeds_all[throttles_all == throttle],
                 stats_all['tension']['max'][throttles_all == throttle] / 1e3,
                 linestyle='--')
    self._ShadeFailureRegions(pylab, wind_speeds_all, data_all['sim_success'])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds_all), max(wind_speeds_all)])
    pylab.title('Power curve by throttle position')
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Aerodynamic power [kW] / Tension [kN]')
    pylab.legend(title='Throttle position', loc=2)
    pylab.grid()
    pylab.savefig(FLAGS.output_dir + '/power_curve_by_throttle.svg')
    pylab.close()

    # Plot power and tension curves.
    pylab.figure().patch.set_alpha(0.0)
    pylab.subplot(2, 1, 1)
    pylab.plot(annotations['power_curve']['v_wind'],
               annotations['power_curve']['P'] /1e3,
               color='gray', linestyle='-', linewidth=4, alpha=0.6)
    pylab.plot(wind_speeds, stats['crosswind_power']['mean'] / 1e3,
               label='Aerodynamic power')
    pylab.fill_between(wind_speeds,
                       stats['crosswind_power']['min'] / 1e3,
                       stats['crosswind_power']['max'] / 1e3,
                       color='blue', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       (stats['crosswind_power']['mean'] -
                        stats['crosswind_power']['std']) / 1e3,
                       (stats['crosswind_power']['mean'] +
                        stats['crosswind_power']['std']) / 1e3,
                       color='blue', alpha=0.3)
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.axhline(0, color='black')
    pylab.title('Power and tension curves')
    pylab.ylabel('Aerodynamic power [kW]')
    pylab.grid()
    # Annotate plot with the date and time of creation, and git commit hash.
    datestr = datetime.datetime.now().strftime('%Y-%m-%d %H:%M %Z')
    git_commit = outputs[0]['git_commit'] if 'git_commit' in outputs[0] else ''
    pylab.text(0.05, 0.8, '%s %s' % (datestr, git_commit[0:7]),
               transform=pylab.gca().transAxes,
               fontdict={'size': 16, 'color': 'grey'})
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-1.0 * annotations['P_max'] / 1e3,
                1.5 * annotations['P_max'] / 1e3])

    pylab.subplot(2, 1, 2)
    pylab.plot(wind_speeds, stats['tension']['max'] / 1e3,
               color='green', label='Tension')
    pylab.plot(wind_speeds, stats['tension']['mean'] / 1e3,
               color='green', linestyle=':', label='Tension')
    pylab.fill_between(wind_speeds,
                       stats['tension']['min'] / 1e3,
                       stats['tension']['max'] / 1e3,
                       color='green', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       (stats['tension']['mean']
                        - stats['tension']['std']) / 1e3,
                       (stats['tension']['mean']
                        + stats['tension']['std']) / 1e3,
                       color='green', alpha=0.3)
    pylab.axhline(y=annotations['T_max'] / 1e3,
                  color='gray', linestyle='--', label='Max Tension',
                  linewidth=2)
    pylab.axhline(y=annotations['T_NE'] / 1e3,
                  color='gray', linestyle='-', label='Tension NE',
                  linewidth=2)
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([0.0, 1.2 * annotations['T_max'] / 1e3])
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Tension [kN]')
    pylab.grid()
    pylab.savefig(FLAGS.output_dir + '/power_and_tension_curve.svg')
    pylab.close()

    # Plot angles.
    pylab.figure().patch.set_alpha(0.0)
    pylab.subplot(3, 1, 1)
    pylab.plot(wind_speeds, stats['tether_roll']['mean'], color='blue')
    pylab.fill_between(wind_speeds,
                       stats['tether_roll']['min'],
                       stats['tether_roll']['max'],
                       color='blue', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['tether_roll']['mean']
                       - stats['tether_roll']['std'],
                       stats['tether_roll']['mean']
                       + stats['tether_roll']['std'],
                       color='blue', alpha=0.3)
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.2, 0.4])
    pylab.title('Angles')
    pylab.ylabel('Tether roll [rad]')
    pylab.grid()
    pylab.subplot(3, 1, 2)
    pylab.plot(wind_speeds, stats['alpha']['mean'], color='green')
    pylab.fill_between(wind_speeds,
                       stats['alpha']['min'],
                       stats['alpha']['max'],
                       color='green', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['alpha']['mean'] - stats['alpha']['std'],
                       stats['alpha']['mean'] + stats['alpha']['std'],
                       color='green', alpha=0.3)
    pylab.axhline(0, color='black')
    pylab.ylabel('Angle-of-attack [rad]')
    pylab.grid()
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.15, 0.15])
    pylab.subplot(3, 1, 3)
    pylab.plot(wind_speeds, stats['beta']['mean'], color='red')
    pylab.fill_between(wind_speeds,
                       stats['beta']['min'],
                       stats['beta']['max'],
                       color='red', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['beta']['mean'] - stats['beta']['std'],
                       stats['beta']['mean'] + stats['beta']['std'],
                       color='red', alpha=0.3)
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.15, 0.15])
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Sideslip [rad]')
    pylab.grid()
    pylab.savefig(FLAGS.output_dir + '/angles_vs_wind_speed.svg')
    pylab.close()

    # Plot angle errors.
    pylab.figure().patch.set_alpha(0.0)
    pylab.subplot(3, 1, 1)
    pylab.plot(wind_speeds, stats['tether_roll_error']['mean'], color='blue')
    pylab.fill_between(wind_speeds,
                       stats['tether_roll_error']['min'],
                       stats['tether_roll_error']['max'],
                       color='blue', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['tether_roll_error']['mean']
                       - stats['tether_roll_error']['std'],
                       stats['tether_roll_error']['mean']
                       + stats['tether_roll_error']['std'],
                       color='blue', alpha=0.3)
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.3, 0.3])
    pylab.title('Angle errors')
    pylab.ylabel('Tether roll\nerror [rad]')
    pylab.grid()
    pylab.subplot(3, 1, 2)
    pylab.plot(wind_speeds, stats['alpha_error']['mean'], color='green')
    pylab.fill_between(wind_speeds,
                       stats['alpha_error']['min'],
                       stats['alpha_error']['max'],
                       color='green', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['alpha_error']['mean']
                       - stats['alpha_error']['std'],
                       stats['alpha_error']['mean']
                       + stats['alpha_error']['std'],
                       color='green', alpha=0.3)
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.axhline(0, color='black')
    pylab.ylabel('Angle-of-attack\nerror [rad]')
    pylab.grid()
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.15, 0.15])
    pylab.subplot(3, 1, 3)
    pylab.plot(wind_speeds, stats['beta_error']['mean'], color='red')
    pylab.fill_between(wind_speeds,
                       stats['beta_error']['min'],
                       stats['beta_error']['max'],
                       color='red', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['beta_error']['mean'] - stats['beta_error']['std'],
                       stats['beta_error']['mean'] + stats['beta_error']['std'],
                       color='red', alpha=0.3)
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.15, 0.15])
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Sideslip\nerror [rad]')
    pylab.grid()
    pylab.savefig(FLAGS.output_dir + '/angle_errors_vs_wind_speed.svg')
    pylab.close()

    # Plot radius versus wind speed.
    pylab.figure().patch.set_alpha(0.0)
    pylab.plot(wind_speeds, stats['radius']['mean'], color='blue')
    pylab.fill_between(wind_speeds,
                       stats['radius']['min'],
                       stats['radius']['max'],
                       color='blue', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['radius']['mean'] - stats['radius']['std'],
                       stats['radius']['mean'] + stats['radius']['std'],
                       color='blue', alpha=0.3)
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    radius_round_50 = 50.0 * round(numpy.median(stats['radius']['mean']) / 50.0)
    pylab.ylim([radius_round_50 - 25.0, radius_round_50 + 25.0])
    pylab.title('Radius')
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Radius [m]')
    pylab.grid()
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.savefig(FLAGS.output_dir + '/radius_vs_wind_speed.svg')
    pylab.close()

    # Plot curvature errors.
    pylab.figure().patch.set_alpha(0.0)
    pylab.subplot(2, 1, 1)
    pylab.plot(wind_speeds, stats['geom_curvature_error']['mean'], color='blue')
    pylab.fill_between(wind_speeds,
                       stats['geom_curvature_error']['min'],
                       stats['geom_curvature_error']['max'],
                       color='blue', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['geom_curvature_error']['mean']
                       - stats['geom_curvature_error']['std'],
                       stats['geom_curvature_error']['mean']
                       + stats['geom_curvature_error']['std'],
                       color='blue', alpha=0.3)
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.01, 0.01])
    pylab.title('Curvature errors')
    pylab.ylabel('Geometric curvature\nerror [1/m]')
    pylab.grid()
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.subplot(2, 1, 2)
    pylab.plot(wind_speeds, stats['aero_curvature_error']['mean'],
               color='green')
    pylab.fill_between(wind_speeds,
                       stats['aero_curvature_error']['min'],
                       stats['aero_curvature_error']['max'],
                       color='green', alpha=0.3)
    pylab.fill_between(wind_speeds,
                       stats['aero_curvature_error']['mean']
                       - stats['aero_curvature_error']['std'],
                       stats['aero_curvature_error']['mean']
                       + stats['aero_curvature_error']['std'],
                       color='green', alpha=0.3)
    pylab.axhline(0, color='black')
    pylab.xlim([min(wind_speeds), max(wind_speeds)])
    pylab.ylim([-0.01, 0.01])
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Aerodynamic curvature\nerror [1/m]')
    pylab.grid()
    self._ShadeFailureRegions(pylab, wind_speeds, data['sim_success'])
    pylab.savefig(FLAGS.output_dir + '/curvature_errors_vs_wind_speed.svg')
    pylab.close()

    pylab.figure().patch.set_alpha(0.0)
    for success_val, plot_style in [(True, 'g+'), (False, 'rs')]:
      pylab.plot(
          [output['parameters']['wind_speed'] for output in outputs
           if output['sim_success'] == success_val],
          [output['parameters']['joystick_throttle'] for output in outputs
           if output['sim_success'] == success_val],
          plot_style)
    pylab.xlabel('Wind speed [m/s]')
    pylab.ylabel('Joystick throttle')
    pylab.ylim([0.0, 1.1])
    pylab.legend(['successful simulations', 'crashed simulations'],
                 loc='lower right')
    pylab.savefig(FLAGS.output_dir + '/sim_success_param_space.svg')

    # Copy HTML file that displays the charts to the output directory.
    dashboard_file = os.path.join(
        makani.HOME, 'analysis/power_curve/power_curve_dashboard.html')
    final_output_file = os.path.join(FLAGS.output_dir, 'index.html')
    shutil.copyfile(dashboard_file, final_output_file)
    logging.info('Output may be viewed at file://%s.',
                 os.path.abspath(final_output_file))


def main(argv):
  client_base.InitMain(argv)
  client = PowerCurveSimClient()
  client.Run()


if __name__ == '__main__':
  gflags.RegisterValidator(
      'output_dir',
      lambda o: not os.path.exists(o) or os.path.isdir(o),
      '--output_dir cannot refer to an existing, non-directory object.')

  main(sys.argv)
