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


r"""Plots critical derivatives from an aerodynamics database.

Examples:

  ./plot_aero_database \
      --alphas_deg=-45.0,45.0,10 \
      --betas_deg=-30,30,10 \
      --flaps_deg=0,0,0,0,0,0,0,0 \
      "${MAKANI_HOME}/database/m600/m600_avl.json" \
      "${MAKANI_HOME}/database/m600/hover_model_sn02_wind_10_no_wake_model.json"
"""

import itertools
import os
import sys

import gflags
from makani.analysis.aero import load_database
from makani.control import system_types
from makani.lib.python import flag_types
from makani.sim.physics import physics
from matplotlib import pyplot
import numpy as np


FLAGS = gflags.FLAGS

flag_types.DEFINE_linspace('alphas_deg', '-10.0, 10.0, 20',
                           'Linspace range of angle-of-attack values [deg].')
flag_types.DEFINE_linspace('betas_deg', '-15.0, 15.0, 20',
                           'Linspace range of sideslip angles [deg].')
gflags.DEFINE_boolean('plot_sim', False,
                      'Whether to display the sim aero model.')
gflags.DEFINE_list('flaps_deg', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   'Flap deflections [deg].')
gflags.RegisterValidator('flaps_deg',
                         lambda x: len(x) == system_types.kNumFlaps,
                         'Wrong number of flaps in input.')
gflags.DEFINE_float('reynolds_number', 1e6,
                    'Reynolds number for the simulator.')
gflags.DEFINE_float('p_hat', 0.0, 'Normalized roll rate.')
gflags.DEFINE_float('q_hat', 0.0, 'Normalized pitch rate.')
gflags.DEFINE_float('r_hat', 0.0, 'Normalized yaw rate.')
gflags.DEFINE_float('thrust_coeff', 0.0, 'Total thrust coefficient.')


def _SwigVec3ToArray(v):
  """Converts a Swig Vec3 data structure to an array."""
  return np.array([v.x, v.y, v.z])


def _ArrayToSwigVec3(a):
  """Converts a numpy array to a Swig Vec3."""
  v = physics.Vec3()
  a_linear = np.reshape(a, (3,))
  v.x = a_linear[0]
  v.y = a_linear[1]
  v.z = a_linear[2]
  return v


def _GetDatabaseLookupFunction(filename, flaps, omega_hat, thrust_coeff):
  """Produces a lookup function from an aero database file."""
  db = load_database.AeroDatabase(filename)
  def _Lookup(alpha, beta, dflaps=None, domega=None):
    if dflaps is None:
      dflaps = np.zeros((system_types.kNumFlaps,))
    if domega is None:
      domega = np.zeros((3,))

    return db.CalcFMCoeff(alpha, beta, flaps + dflaps, omega_hat + domega,
                          thrust_coeff)
  return _Lookup, db.format


def _GetSimAeroLookupFunction(reynolds_number, flaps, omega_hat, thrust_coeff):
  """Produces a lookup function from the simulator's Aero class."""
  db = physics.Aero(physics.GetAeroSimParams())
  def _Lookup(alpha, beta, dflaps=None, domega=None):
    """Lookup function returned by the enclosing method."""
    if dflaps is None:
      dflaps = np.zeros((system_types.kNumFlaps,))
    if domega is None:
      domega = np.zeros((3,))

    flaps_vec = physics.VecWrapper(system_types.kNumFlaps)
    for i in range(system_types.kNumFlaps):
      flaps_vec.SetValue(i, flaps[i] + dflaps[i])
    omega_hat_vec3 = _ArrayToSwigVec3(omega_hat + domega)
    force_moment = physics.ForceMoment()
    db.CalcForceMomentCoeff(alpha, beta, omega_hat_vec3.this,
                            flaps_vec.GetVec(), reynolds_number,
                            force_moment.this, thrust_coeff)
    return (_SwigVec3ToArray(force_moment.force),
            _SwigVec3ToArray(force_moment.moment))
  return _Lookup


def _GetStabilityCoefficients(lookup_function, alphas, betas, dflaps=None,
                              domega=None):
  """Generates array of force and moment coefficients in stability axes."""
  cf_s = np.zeros((3, len(alphas), len(betas)))
  cm_s = np.zeros((3, len(alphas), len(betas)))
  for i, alpha in enumerate(alphas):
    for j, beta in enumerate(betas):
      cf_b, cm_b = lookup_function(alpha, beta, dflaps=dflaps, domega=domega)
      cf_b_vec3 = _ArrayToSwigVec3(cf_b)
      cf_s_vec3 = physics.Vec3()
      physics.RotBToS(cf_b_vec3.this, alpha, cf_s_vec3.this)
      cm_b_vec3 = _ArrayToSwigVec3(cm_b)
      cm_s_vec3 = physics.Vec3()
      physics.RotBToS(cm_b_vec3.this, alpha, cm_s_vec3.this)
      cf_s[:, i, j] = _SwigVec3ToArray(cf_s_vec3)
      cm_s[:, i, j] = _SwigVec3ToArray(cm_s_vec3)
  return cf_s, cm_s


def _PlotCoefficients(x, ys, xlabel, ylabels, legend, color, database_label='',
                      context=None):
  """Plots aerodynamic coefficients in different line styles.

  Args:
    x: X-axis values.
    ys: Y-values for each subplot.
    xlabel: Label for the x-axis.
    ylabels: Labels for each subplot.
    legend: Entries to place in the legend.
    color: Color for all plots.
    database_label: Label of database that the coefficients come from.
    context: Context used to link various plots axes.
  """
  num_plots = len(ylabels)
  line_styles = ['--', '-', '-.']
  for i in range(num_plots):
    for j in range(len(legend)):
      if context and context['linked_axes']:
        pyplot.subplot(num_plots, 1, i + 1, sharex=context['linked_axes'])
      else:
        context['linked_axes'] = pyplot.subplot(num_plots, 1, i + 1)

      if color == 'black':
        modified_label = database_label + ' (' + legend[j] + ')'
      elif j == 1:
        modified_label = database_label
      else:
        modified_label = '_'

      pyplot.plot(x, ys[i][:, j],
                  linestyle=line_styles[j],
                  color=color, label=modified_label)
      pyplot.grid(True)

      pyplot.ylabel(ylabels[i], fontsize=24)
      if i == num_plots - 1:
        pyplot.xlabel(xlabel)

  pyplot.legend(loc='upper left', bbox_to_anchor=(0.75, 0.3), prop={'size': 8})


def _MakeCoefficientDict(lookup_function, alphas, betas, h=1e-3):
  """Makes coefficient, control, and stability derivative dictionary."""
  # Calculate coefficients.
  cf_s, cm_s = _GetStabilityCoefficients(lookup_function, alphas, betas)

  # Calculate lateral derivatives.
  dail = [0.0 for _ in range(system_types.kNumFlaps)]
  dail[system_types.kFlapA1] = -h
  dail[system_types.kFlapA2] = -h
  dail[system_types.kFlapA7] = h
  dail[system_types.kFlapA8] = h

  drud = [h if i == system_types.kFlapRud else 0.0
          for i in range(system_types.kNumFlaps)]

  cm_da_s = (_GetStabilityCoefficients(
      lookup_function, alphas, betas, dflaps=dail)[1] - cm_s) / h
  cm_dr_s = (_GetStabilityCoefficients(
      lookup_function, alphas, betas, dflaps=drud)[1] - cm_s) / h
  cm_dphat_s = (_GetStabilityCoefficients(
      lookup_function, alphas, betas, domega=[h, 0.0, 0.0])[1] - cm_s) / h
  cm_drhat_s = (_GetStabilityCoefficients(
      lookup_function, alphas, betas, domega=[0.0, 0.0, h])[1] - cm_s) / h

  # Calculate longitudinal derivatives.
  flap_delta = [h if i == system_types.kFlapEle else 0.0
                for i in range(system_types.kNumFlaps)]
  cf_de_s = (_GetStabilityCoefficients(lookup_function, alphas, betas,
                                       dflaps=flap_delta)[0] - cf_s) / h
  cm_de_s = (_GetStabilityCoefficients(lookup_function, alphas, betas,
                                       dflaps=flap_delta)[1] - cm_s) / h
  cm_dqhat_s = (_GetStabilityCoefficients(lookup_function, alphas, betas,
                                          domega=[0.0, h, 0.0])[1] - cm_s) / h

  return {
      'CD': {'label': r'$C_D$', 'value': -np.squeeze(cf_s[0, :, :])},
      'CY': {'label': r'$C_Y$', 'value': np.squeeze(cf_s[1, :, :])},
      'CL': {'label': r'$C_L$', 'value': -np.squeeze(cf_s[2, :, :])},
      'Cl': {'label': r'$C_l$', 'value': np.squeeze(cm_s[0, :, :])},
      'Cm': {'label': r'$C_m$', 'value': np.squeeze(cm_s[1, :, :])},
      'Cn': {'label': r'$C_n$', 'value': np.squeeze(cm_s[2, :, :])},
      'Clda': {'label': r'$C_{l_{\delta a}}$',
               'value': np.squeeze(cm_da_s[0, :, :])},
      'Cldr': {'label': r'$C_{l_{\delta r}}$',
               'value': np.squeeze(cm_dr_s[0, :, :])},
      'Cmde': {'label': r'$C_{m_{\delta e}}$',
               'value': np.squeeze(cm_de_s[1, :, :])},
      'Cnda': {'label': r'$C_{n_{\delta a}}$',
               'value': np.squeeze(cm_da_s[2, :, :])},
      'Cndr': {'label': r'$C_{n_{\delta r}}$',
               'value': np.squeeze(cm_dr_s[2, :, :])},
      'CLde': {'label': r'$C_{L_{\delta e}}$',
               'value': -np.squeeze(cf_de_s[2, :, :])},
      'Cldphat': {'label': r'$C_{l_{\hat{p}}}$',
                  'value': np.squeeze(cm_dphat_s[0, :, :])},
      'Cmdqhat': {'label': r'$C_{m_{\hat{q}}}$',
                  'value': np.squeeze(cm_dqhat_s[1, :, :])},
      'Cndrhat': {'label': r'$C_{n_{\hat{r}}}$',
                  'value': np.squeeze(cm_drhat_s[2, :, :])}
  }


def _MakePlots(database_label, color, lookup_function, alphas, betas, context):
  """Plots CY, Cl, and Cn as a function of angle-of-sideslip.

  The plots are given for three different angle-of-attack values in
  different line styles.

  Args:
    database_label: Label of database that the coefficients come from.
    color: Color of the plots.
    lookup_function: Function providing force and moment coefficients as a
        function of alpha and beta.
    alphas: Array of angle-of-attack values (only the first and last are used).
    betas: Array of angle-of-sideslip values.
    context: Context used to link various plots axes.
  """
  alpha_inds = [0, len(alphas) / 2, -1]
  beta_inds = [0, len(betas) / 2, -1]
  alpha_coeffs = _MakeCoefficientDict(lookup_function, alphas, betas[beta_inds])
  beta_coeffs = _MakeCoefficientDict(lookup_function, alphas[alpha_inds], betas)
  alpha_legend = ['beta = %0.1f deg' % np.rad2deg(beta)
                  for beta in betas[beta_inds]]
  beta_legend = ['alpha = %0.1f deg' % np.rad2deg(alpha)
                 for alpha in alphas[alpha_inds]]

  plot_tuples = [('CL', 'CD', 'Cm'),
                 ('CY', 'Cl', 'Cn'),
                 ('Clda', 'Cldr', 'Cldphat'),
                 ('CLde', 'Cmde', 'Cmdqhat'),
                 ('Cnda', 'Cndr', 'Cndrhat')]

  for i, plot_tuple in enumerate(plot_tuples):
    ys = [alpha_coeffs[label]['value'] for label in plot_tuple]
    ylabels = [alpha_coeffs[label]['label'] for label in plot_tuple]
    fig = pyplot.figure(i)
    fig.canvas.set_window_title(', '.join(plot_tuple))
    _PlotCoefficients(np.rad2deg(alphas), ys, 'Angle-of-attack [deg]',
                      ylabels, alpha_legend, color, database_label,
                      context['alpha'])

    ys = [beta_coeffs[label]['value'].T for label in plot_tuple]
    ylabels = [beta_coeffs[label]['label'] for label in plot_tuple]
    fig = pyplot.figure(i + len(plot_tuples))
    fig.canvas.set_window_title(', '.join(plot_tuple))
    _PlotCoefficients(np.rad2deg(betas), ys, 'Sideslip angle [deg]',
                      ylabels, beta_legend, color, database_label,
                      context['beta'])


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n' % e
    sys.exit(1)

  alphas = np.array(np.deg2rad(FLAGS.alphas_deg))
  betas = np.array(np.deg2rad(FLAGS.betas_deg))
  omega_hat = np.array([FLAGS.p_hat, FLAGS.q_hat, FLAGS.r_hat])
  flaps = np.array(np.deg2rad(map(float, FLAGS.flaps_deg)))
  thrust_coeff = FLAGS.thrust_coeff

  colors = itertools.cycle(['black', 'brown', 'red', 'orange', 'yellow',
                            'green', 'blue', 'violet', 'gray'])

  context = {'alpha': {'linked_axes': None}, 'beta': {'linked_axes': None}}

  # Optionally, plot the coefficients as used in the simulator.
  if FLAGS.plot_sim:
    aero_lookup = _GetSimAeroLookupFunction(FLAGS.reynolds_number, flaps,
                                            omega_hat, thrust_coeff)
    _MakePlots('sim', colors.next(), aero_lookup, alphas, betas, context)

  # Plot each database from the command line.
  for database in argv[1:]:
    aero_lookup, database_format = _GetDatabaseLookupFunction(database, flaps,
                                                              omega_hat,
                                                              thrust_coeff)
    database_label = (database_format + ': ' +
                      os.path.splitext(os.path.basename(database))[0])
    _MakePlots(database_label, colors.next(), aero_lookup, alphas, betas,
               context)

  pyplot.show()


if __name__ == '__main__':
  main(sys.argv)
