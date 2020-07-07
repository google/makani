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

"""Analysis to compute auto-glide command kite alpha."""
# This script computes the best commanded kite alpha for glide.
#
# To run it:
#
# ./autoglide_analysis.py -f 'm600_aswing_baseline_zero_angular_rate.json'
#

import json
import os
import sys
import gflags

from matplotlib import pyplot as plt
import numpy as np


gflags.DEFINE_string('filename', None,
                     'Aero database json in database folder.', short_name='f')

# The aero database is for a tether-less, bridle-less kite. This
# drag_coeff_offset is to address underestimating kite drag due to
# modeling simplifications.
# Default value of 0.075 is based on RPX-07 glide analysis and includes bridle
# drag as well. This likely overestimates CD slightly, but suffices given the
# scope of this approximate analysis.
gflags.DEFINE_float('drag_coeff_offset', 0.075,
                    'Parasitic drag offset.', short_name='d')

gflags.DEFINE_float('kite_mass', 1660.0,
                    'Glide (Tether-less) kite mass [kg].', short_name='m')

gflags.DEFINE_float('air_density', 1.026,
                    'Air density [kg/m^3].', short_name='r')

gflags.DEFINE_float('ref_area', 32.9,
                    'Reference area [m^2].', short_name='a')

gflags.DEFINE_boolean('show_plots', True,
                      'Toggle to show plots.', short_name='p')

FLAGS = gflags.FLAGS


def _autoglide_analysis(options):
  """Process aero database to find best kite glide alpha."""

  filename = options.filename
  drag_coeff_offset = options.drag_coeff_offset
  kite_mass = options.kite_mass
  air_density = options.air_density
  ref_area = options.ref_area
  show_plots = options.show_plots

  makani_home = os.environ['MAKANI_HOME']
  database_filename = makani_home + '/database/m600/' + filename

  with open(database_filename) as fp:
    aerodb = json.load(fp)

  alphads = np.array(aerodb['alphads'])
  zero_beta_index = aerodb['betads'].index(0)
  lift_coeff = np.squeeze(np.array(aerodb['CLtot']))[:, zero_beta_index]
  drag_coeff = np.squeeze(np.array(aerodb['CDtot']))[:, zero_beta_index]
  drag_coeff += drag_coeff_offset

  # Kite L/D and efficiency coefficient (CL^1.5 / CD).
  kite_lift_drag_ratio = np.divide(lift_coeff, drag_coeff)
  kite_eff_coef = np.divide(np.power(lift_coeff, 1.5), drag_coeff)

  # Airspeed to support kite weight at given CL. The calculation is based on the
  # assumption that L >> D, and is appropriate for scope of this analysis.
  airspeed = np.sqrt(np.divide(kite_mass * 9.81,
                               (0.5 * air_density * ref_area * lift_coeff)))
  vertical_airspeed = np.divide(airspeed, kite_lift_drag_ratio)
  horizontal_airspeed = np.sqrt(np.power(airspeed, 2.0) -
                                np.power(vertical_airspeed, 2.0))
  airspeed_ratio = np.divide(vertical_airspeed, horizontal_airspeed)
  glide_angle = np.arctan2(vertical_airspeed, horizontal_airspeed)
  glide_angle = np.rad2deg(glide_angle)

  # Kinetic energy [kJ] carried by the kite.
  vertical_ke = 0.5 * kite_mass * np.power(vertical_airspeed, 2.0) / 1000
  horizontal_ke = 0.5 * kite_mass * np.power(horizontal_airspeed, 2.0) /1000
  energy_ratio = np.divide(vertical_ke, horizontal_ke)

  # Kite alpha corresponding to best L/D.
  alpha_best_lift_over_drag = alphads[np.argmax(kite_lift_drag_ratio)]

  # Kite alpha corresponding to best efficiency (L^1.5/D).
  alpha_best_eff_coeff = alphads[np.argmax(kite_eff_coef)]

  print '\n'
  print 'Best L/D kite alpha is %.2f deg.' %(alpha_best_lift_over_drag)
  print '\n'
  print 'Best L^1.5/D kite alpha is %.2f deg.' %(alpha_best_eff_coeff)
  print '\n'

  if show_plots:
    plt.figure(1)
    ax1 = plt.subplot(3, 2, 1)
    plt.plot(alphads, lift_coeff, '-ro', label='CL', linewidth=2.0)
    plt.plot(alphads, drag_coeff, '-b^', label='CD', linewidth=2.0)
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(3, 2, 3, sharex=ax1)
    plt.plot(alphads, kite_lift_drag_ratio, '-ro', label='L / D',
             linewidth=2.0)
    plt.plot(alphads, kite_eff_coef, '-b^', label='L^1.5 / D',
             linewidth=2.0)
    plt.axvline(x=alpha_best_lift_over_drag, linestyle='--', color='k')
    plt.axvline(x=alpha_best_eff_coeff, linestyle='-.', color='g')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(3, 2, 5, sharex=ax1)
    plt.plot(alphads, glide_angle, '-ks', label='Glide angle',
             linewidth=2.0)
    plt.axvline(x=alpha_best_lift_over_drag, linestyle='--', color='k')
    plt.axvline(x=alpha_best_eff_coeff, linestyle='-.', color='g')
    plt.ylabel('[deg]')
    plt.xlabel('Kite Alpha [deg]')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(3, 2, 2, sharex=ax1)
    plt.plot(alphads, vertical_airspeed, '-ro', label='Vertical',
             linewidth=2.0)
    plt.plot(alphads, horizontal_airspeed, '--r^', label='Horizontal',
             linewidth=2.0)
    plt.axvline(x=alpha_best_lift_over_drag, linestyle='--', color='k')
    plt.axvline(x=alpha_best_eff_coeff, linestyle='-.', color='g')
    plt.legend(loc='best')
    plt.ylabel('Airspeed [m/s]')
    plt.grid()

    plt.subplot(3, 2, 4, sharex=ax1)
    plt.plot(alphads, vertical_ke, '-bo', label='Vertical KE',
             linewidth=2.0)
    plt.plot(alphads, horizontal_ke, '--b^', label='Horizontal KE',
             linewidth=2.0)
    plt.axvline(x=alpha_best_lift_over_drag, linestyle='--', color='k')
    plt.axvline(x=alpha_best_eff_coeff, linestyle='-.', color='g')
    plt.ylim(-10, 500)
    plt.legend(loc='best')
    plt.ylabel('Energy [kJ]')
    plt.grid()

    plt.subplot(3, 2, 6, sharex=ax1)
    plt.plot(alphads, airspeed_ratio, '-ro', label='Airspeed ratio',
             linewidth=2.0)
    plt.plot(alphads, energy_ratio, '-b^', label='Energy ratio',
             linewidth=2.0)
    plt.ylim(0, 0.15)
    plt.axvline(x=alpha_best_lift_over_drag, linestyle='--', color='k',
                label='best L/D alpha')
    plt.axvline(x=alpha_best_eff_coeff, linestyle='-.', color='g',
                label='best L^1.5/D alpha')
    plt.xlabel('Kite Alpha [deg]')
    plt.legend(loc='best')
    plt.grid()

    plt.show()


def main(argv):
  gflags.MarkFlagAsRequired('f')
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError as e:
    print ('%s\nUsage: %s -f filename.json\n%s'
           % (e, sys.argv[0], FLAGS))
    sys.exit(1)

  _autoglide_analysis(FLAGS)


if __name__ == '__main__':
  main(sys.argv)
