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


"""Produce plots of rotor databases.

Iterates over all the propellers in the configuration system and produces
a plot comparing the corresponding SimpleRotorModel and rotor database files.
"""

import os
import sys

import gflags
import makani
from makani.analysis.aero import load_database
from makani.config import mconfig
import matplotlib
from matplotlib import pyplot
import numpy as np

FLAGS = gflags.FLAGS

gflags.DEFINE_float('max_advance_ratio', None,
                    'Display a mask over points where the advance ratio '
                    'exceeds this amount.')
gflags.DEFINE_float('max_mach_number', None,
                    'Display a mask over points where the tip Mach number '
                    'exceeds this amount.')
gflags.DEFINE_float('max_power', None,
                    'Display a mask over points where the absolute value of '
                    'power exceeds this amount.')
gflags.DEFINE_float('max_torque', None,
                    'Display a mask over points where the absolute value of '
                    'torque exceeds this amount.')
gflags.DEFINE_float('advance_ratio_stall_margin', None,
                    'Display a line where the advance ratio is equal to this '
                    'max_advance_ratio minus this margin.')


def _PlotPropValue(prop, diameter, plot_type='power',
                   torque_max=900.0, thrust_max_kn=5.0, power_max_kw=110.0,
                   speed_of_sound=340.0):
  """Plots values associate with a propeller.

  Args:
    prop: Rotor database (see analysis.aero.load_database).
    diameter: Diameter of the rotor [m].
    plot_type: One of 'power', 'torque', or 'thrust'.
    torque_max: Absolute maximum torque [N-m] for color scale.
    thrust_max_kn: Maximum thrust [kN] for color scale.
    power_max_kw: Maximum absolute power [kW] for color scale.
    speed_of_sound: Speed of sound [m/s].
  """
  omegas = np.reshape(prop['omegas'], (prop['num_omegas'], 1))
  v_freestreams = np.reshape(prop['v_freestreams'],
                             (1, prop['num_v_freestreams']))

  omegas = (np.ones(v_freestreams.shape) * omegas)
  v_freestreams = np.multiply(v_freestreams, np.ones(omegas.shape))

  power_kw = np.array(prop['power']) / 1e3
  torque = np.array(prop['power']) / omegas
  thrust_kn = np.array(prop['thrust']) / 1e3

  if plot_type == 'power':
    data = power_kw
    levels = np.linspace(-power_max_kw, power_max_kw, 10)
    title = 'Power [kW]'
  elif plot_type == 'torque':
    data = torque
    levels = np.linspace(-torque_max, torque_max, 20)
    title = 'Torque [N-m]'
  elif plot_type == 'thrust':
    data = thrust_kn
    levels = np.linspace(-thrust_max_kn, thrust_max_kn, 20)
    title = 'Thrust [kN]'

  cmap = matplotlib.cm.get_cmap('bone')
  cs = pyplot.contourf(omegas, v_freestreams, data, levels=levels, cmap=cmap)
  pyplot.colorbar(cs)

  if FLAGS.max_power is not None:
    pyplot.contourf(omegas, v_freestreams, power_kw,
                    levels=[FLAGS.max_power / 1e3, float('inf')], colors='b')
    pyplot.contourf(omegas, v_freestreams, power_kw,
                    levels=[-float('inf'), -FLAGS.max_power / 1e3], colors='b')

  if FLAGS.max_torque is not None:
    pyplot.contourf(omegas, v_freestreams, torque,
                    levels=[FLAGS.max_torque, float('inf')], colors='g')
    pyplot.contourf(omegas, v_freestreams, torque,
                    levels=[-float('inf'), -FLAGS.max_torque], colors='g')

  if FLAGS.max_mach_number is not None:
    tip_speed = diameter * omegas / 2.0
    mach = tip_speed / speed_of_sound
    pyplot.contourf(omegas, v_freestreams, mach,
                    levels=[FLAGS.max_mach_number, float('inf')], colors='r')

  if FLAGS.max_advance_ratio is not None:
    advance_ratio = 2.0 * np.pi * v_freestreams / (omegas * diameter)
    pyplot.contourf(omegas, v_freestreams, advance_ratio,
                    levels=[FLAGS.max_advance_ratio, float('inf')],
                    alpha=0.5, colors='y')
    if FLAGS.advance_ratio_stall_margin is not None:
      pyplot.contour(
          omegas, v_freestreams, advance_ratio,
          levels=[FLAGS.max_advance_ratio - FLAGS.advance_ratio_stall_margin],
          colors='y')
  pyplot.title(title)
  pyplot.xlabel('Rotor Speed [rad/s]')
  pyplot.ylabel('Airspeed [m/s]')


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  if len(argv) > 1:
    propeller_paths = argv[1:]
  else:
    propellers = mconfig.MakeParams('prop.propellers')
    propeller_paths = [
        os.path.join(makani.HOME, 'database', propeller['database']['name'])
        for propeller in propellers
    ]

  linked_axes = None
  for i, propeller_path in enumerate(propeller_paths):
    prop = load_database.LoadPropDatabase(propeller_path)

    fig = pyplot.figure(i + 1)
    fig.suptitle(os.path.basename(propeller_path))
    fig.canvas.set_window_title(os.path.basename(propeller_path))
    if linked_axes:
      linked_axes = pyplot.subplot(1, 3, 1, sharex=linked_axes,
                                   sharey=linked_axes)
    else:
      linked_axes = pyplot.subplot(1, 3, 1)
    _PlotPropValue(prop, prop['diameter'], plot_type='power')
    pyplot.subplot(1, 3, 2, sharex=linked_axes, sharey=linked_axes)
    _PlotPropValue(prop, prop['diameter'], plot_type='thrust')
    pyplot.subplot(1, 3, 3, sharex=linked_axes, sharey=linked_axes)
    _PlotPropValue(prop, prop['diameter'], plot_type='torque')
  pyplot.show()


if __name__ == '__main__':
  main(sys.argv)
