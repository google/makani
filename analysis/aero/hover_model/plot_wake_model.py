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


r"""Plots wake velocity overlaid on a diagram of the wing panels.

Example:

  ./plot_wake_model.py \
      --wing_model=m600 \
      --apparent_wind_speed=5.0 \
      --alpha_deg=90.0 \
      --x_table="-7.0, 0.0, 2"
"""

import sys

import gflags
from makani.analysis.aero import apparent_wind_util
from makani.analysis.aero.hover_model import hover_model
from makani.analysis.aero.hover_model import wake_model
from makani.lib.python import flag_types
from makani.lib.python import wing_flag
import matplotlib
from matplotlib import pyplot
from mpl_toolkits import mplot3d
import numpy as np

wing_flag.AppeaseLintWhenImportingFlagOnly()

gflags.DEFINE_string('wing_serial', '01', 'Wing serial number.')
gflags.DEFINE_float('apparent_wind_speed', '5.0', 'Apparent wind speed [m/s].')
gflags.DEFINE_float('alpha_deg', '90.0', 'Angle-of-attack [deg].')
gflags.DEFINE_float('beta_deg', '0.0', 'Sideslip angle [deg].')
flag_types.DEFINE_linspace('x_table', '-7.0, 0.0, 3',
                           'Linspace range of body x values [m] at which we '
                           'evaluate the wake increment.')
flag_types.DEFINE_linspace('y_table', '-10.0, 10.0, 50',
                           'Linspace range of body y values [m] at which we '
                           'evaluate the wake increment.')
flag_types.DEFINE_linspace('z_table', '-5.0, 5.0, 25',
                           'Linspace range of body z values [m] at which we '
                           'evaluate the wake increment.')

FLAGS = gflags.FLAGS


def PlotWakeModel(x_table, y_table, z_table, wake_speeds, params):
  """Plots wake velocity increment in body y-z plane.

  Args:
    x_table: List of body x values [m] at which to plot the wake speeds.
    y_table: List of body y values [m] at which to plot the wake speeds.
    z_table: List of body z values [m] at which to plot the wake speeds.
    wake_speeds: Wake increment speeds [m/s] represented as an ndarray
        with dimensions (len(x_table), len(y_table), len(z_table)).
    params: Wing geometry parameters used for overlaying the wing
        panels and rotors.
  """
  axes = pyplot.figure().add_subplot(1, 1, 1, projection='3d')

  # To display the wing in the hovering orientation, the axes are
  # rotated such that the x and z axes are flipped and the y axis is
  # inverted.

  # Plot slices of the wake speeds at each point in x_table.
  ys, zs = np.meshgrid(y_table, z_table, indexing='ij')
  for i in range(len(x_table)):
    axes.contourf(zs, -ys, np.squeeze(wake_speeds[i, :, :]), 20,
                  zdir='z', offset=x_table[i], alpha=0.2, cmap=pyplot.cm.jet)

  # Plot rotor disks.
  for rotor in params['rotors']:
    rotor_disk = matplotlib.patches.Circle(
        (rotor['pos'][2], -rotor['pos'][1]), rotor['radius'], alpha=0.3,
        color='k')
    axes.add_patch(rotor_disk)
    mplot3d.art3d.pathpatch_2d_to_3d(rotor_disk, z=rotor['pos'][0], zdir='z')

  # Plot panels.
  for panel in params['panels']:
    vertices_s = np.array([
        [panel['chord'] * 0.25, panel['span'] / 2.0, 0.0],
        [panel['chord'] * 0.25, -panel['span'] / 2.0, 0.0],
        [-panel['chord'] * 0.75, -panel['span'] / 2.0, 0.0],
        [-panel['chord'] * 0.75, panel['span'] / 2.0, 0.0]
    ])
    vertices_b = np.dot(panel['dcm_b2s'].T, vertices_s.T).T + panel['pos_b']
    vertices = np.dot([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]],
                      vertices_b.T).T
    rect = mplot3d.art3d.Poly3DCollection([vertices])
    rect.set_color('k')
    rect.set_alpha(0.3)
    axes.add_collection3d(rect)

  # Set axes properties.
  axes.set_xlabel('z')
  axes.set_ylabel('y')
  axes.set_zlabel('x')
  axes.set_xlim((-5.0, 5.0))
  axes.set_ylim((-15.0, 15.0))
  axes.set_zlim((-10.0, 2.0))
  axes.invert_yaxis()


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  apparent_wind_b = apparent_wind_util.ApparentWindSphToCart(
      FLAGS.apparent_wind_speed, np.deg2rad(FLAGS.alpha_deg),
      np.deg2rad(FLAGS.beta_deg))
  params = hover_model.GetParams(FLAGS.wing_model, FLAGS.wing_serial)
  xs, ys, zs = np.meshgrid(FLAGS.x_table, FLAGS.y_table, FLAGS.z_table,
                           indexing='ij')
  point_b = np.array([xs.T, ys.T, zs.T]).T
  wake_increments = wake_model.SumWakeVelocityIncrementsAtPoint(
      point_b, apparent_wind_b, params)

  PlotWakeModel(FLAGS.x_table, FLAGS.y_table, FLAGS.z_table,
                -wake_increments[..., 0], params)
  pyplot.show()


if __name__ == '__main__':
  main(sys.argv)
