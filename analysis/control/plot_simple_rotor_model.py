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


"""Produces plots for examining the quality of SimpleRotorModels.

Iterates over all the propellers in the configuration system and produces
a plot comparing the corresponding SimpleRotorModel and rotor database files.
"""

import sys

import gflags
from makani.analysis.control import simple_aero
from makani.config import mconfig
from makani.control import system_types
from matplotlib import pyplot
import numpy as np


gflags.DEFINE_float('air_density', 1.2,
                    'Air density [kg/m^3].')
gflags.DEFINE_float('advance_ratio_stall_margin', 0.0,
                    'Margin [#] to keep advance ratio away from stall.')
gflags.DEFINE_float('power_limit', float('inf'),
                    'Maximum absolute power [W] to use in fit.')
gflags.DEFINE_float('torque_limit', float('inf'),
                    'Maximum absolute torque [N-m] to use in fit.')
gflags.DEFINE_float('tip_speed_limit', float('inf'),
                    'Maximum tip speed [m/s] to use in fit.')
gflags.DEFINE_integer('num_curves', 10,
                      'Number of freestream velocity curves to plot.')

FLAGS = gflags.FLAGS


def PlotSimpleRotorModel(air_density, rotor_database_path, power_limit,
                         torque_limit, tip_speed_limit,
                         advance_ratio_stall_margin, num_curves):
  """Plots the simple rotor model against the rotor database.

  Args:
    air_density: Air density [kg/m^3].
    rotor_database_path: Path to a rotor database file.
    power_limit: Maximum absolute power [W] used in fit.
    torque_limit: Maximum absolute torque [N-m] used in fit.
    tip_speed_limit: Maximum tip speed [m/s] used in fit.
    advance_ratio_stall_margin: Amount [#] to reduce the maximum
        allowed advance ratio from the stall advance ratio.
    num_curves: Number of freestream velocity curves to plot.
  """
  rotor_model = simple_aero.RotorModel(
      air_density, rotor_database_path, power_limit, torque_limit,
      tip_speed_limit, advance_ratio_stall_margin=advance_ratio_stall_margin)
  simple_model = rotor_model.CalcSimpleRotorModel()

  # Only plot up to num_curves of the freestream velocities.
  incr = np.ceil(rotor_model.num_v_freestreams / float(num_curves))

  # Calculate fit thrust curve, masking points that were not included
  # in fit.
  thrust_coeffs = simple_aero.CalcThrustCoeff(simple_model,
                                              rotor_model.advance_ratios)
  mask = np.ones(rotor_model.thrust.shape)
  mask[np.logical_not(rotor_model.keep)] = float('nan')
  thrust = mask * (thrust_coeffs * air_density * rotor_model.D**4.0
                   * (rotor_model.omegas / np.pi / 2.0)**2.0)

  fig = pyplot.figure()
  fig.canvas.set_window_title(rotor_database_path)

  # Plot thrust curves and fit.
  pyplot.subplot(2, 1, 1)
  pyplot.grid(True)
  pyplot.hold(True)
  pyplot.plot(rotor_model.omegas, rotor_model.thrust[:, 0:-1:incr], '-')
  pyplot.gca().set_color_cycle(None)
  pyplot.plot(rotor_model.omegas, thrust[:, 0:-1:incr], '.')
  pyplot.ylabel('Thrust [N]')
  pyplot.title('Thrust (solid) versus thrust fit (dashed)')
  pyplot.xlim(rotor_model.omegas[[0, -1]])

  # Plot thrust fit error.
  pyplot.subplot(2, 1, 2)
  pyplot.grid(True)
  pyplot.gca().set_color_cycle(None)
  pyplot.plot(rotor_model.omegas,
              rotor_model.thrust[:, 0:-1:incr] - thrust[:, 0:-1:incr], '.-')
  pyplot.xlabel('Rotor speed [rad/s]')
  pyplot.ylabel('Thrust error [N]')
  pyplot.title('Thrust fit error')
  pyplot.xlim(rotor_model.omegas[[0, -1]])

  print rotor_database_path
  print '  Max Thrust (v_freestream = 0.0): %g' % (
      rotor_model.CalcMaxThrusts([0.0])[0][0])
  print '  Max Thrust (v_freestream = 40.0): %g' % (
      rotor_model.CalcMaxThrusts([40.0])[0][0])


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  propellers = mconfig.MakeParams('m600.propellers')
  for i in range(system_types.kNumPropVersions):
    PlotSimpleRotorModel(
        FLAGS.air_density,
        propellers[i]['database']['name'],
        FLAGS.power_limit, FLAGS.torque_limit, FLAGS.tip_speed_limit,
        FLAGS.advance_ratio_stall_margin, FLAGS.num_curves)
  pyplot.show()

if __name__ == '__main__':
  main(sys.argv)
