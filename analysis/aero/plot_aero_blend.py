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

r"""Plots the blend between low- and high-incidence aero databases.

Examples:
  - Variation of aerodynamic coefficients with alpha, at beta = 5 deg:
      bazel run //analysis/aero:plot_aero_blend -- \
        --alpha_degs '-15.0,15.0,31' \
        --beta_degs '5.0,5.0,1'
  - Variation of aerodynamic coefficients with beta, at alpha = 5 deg:
      bazel run //analysis/aero:plot_aero_blend -- \
        --beta_degs '-15.0,15.0,31'
        --alpha_degs '5.0,5.0,1'

The program will assert out if both alpha_degs and beta_degs contain more
than one element.
"""

import re
import sys

import gflags
import makani
from makani.lib.python import flag_types
from makani.sim.physics import physics
from makani.system import labels
import matplotlib
matplotlib.use('QT4Agg')
from matplotlib import pyplot  # pylint: disable=g-import-not-at-top

import numpy as np

makani.SetRunfilesDirFromBinaryPath()

_AERO_OUTPUTS = ['Cx', 'Cy', 'Cz', 'Cl', 'Cm', 'Cn', 'CL', 'CD']
_AERO_VARS = ['alpha', 'beta', 'p', 'q', 'r', 'ail', 'ele', 'rud']
_VALID_SPEC_DESCRIPTION = (
    'Valid specifiers are of the form "<output>" or "d<output>/d<var>", where '
    '<output> is one of %s, and <var> is one of %s.' % (_AERO_OUTPUTS,
                                                        _AERO_VARS))

gflags.DEFINE_float('re', 5e6, 'Reynolds number.')
gflags.DEFINE_integer('fig_rows', 4, 'Number of rows in figure grid.')
gflags.DEFINE_integer('fig_cols', 4, 'Number of columns in figure grid.')
flag_types.DEFINE_linspace('alpha_degs', '0.0, 12.0, 49',
                           'Linspace of alpha values in degrees.')
flag_types.DEFINE_linspace('beta_degs', '0.0, 0.0, 1',
                           'Linspace of beta values in degrees.')
gflags.DEFINE_list('specs',
                   ['CL', 'CD', 'Cy', 'Cl', 'Cm', 'Cn',
                    'dCL/dalpha', 'dCm/dalpha', 'dCm/dq',
                    'dCl/dail', 'dCm/dele', 'dCn/drud'],
                   'Comma-separated list of specifiers for values to plot. '
                   + _VALID_SPEC_DESCRIPTION)
gflags.DEFINE_float('thrust_coeff', 0.0, 'Total thrust coefficient.')

FLAGS = gflags.FLAGS


class ProgrammerError(Exception):
  """Indicates that whoever wrote the code screwed up.

  But they were nice enough to check whether they did.
  """
  pass


def IsDerivativeSpec(spec):
  match = re.match(r'^d(\w+)/d(\w+)$', spec)
  return (match and match.group(1) in _AERO_OUTPUTS
          and match.group(2) in _AERO_VARS)


def IsValidSpec(spec):
  return spec in _AERO_OUTPUTS or IsDerivativeSpec(spec)


class BlendDatum(object):
  """A piece of data recorded for low/high-incidence and blended aero models.

  This is essentially just a dictionary with keys KEYS. It exists primarily
  as documentation.
  """

  KEYS = ['low', 'high', 'blended']

  def __init__(self, fcn):
    self._datum = {key: fcn(key) for key in self.KEYS}

  def __getitem__(self, key):
    return self._datum[key]


def CalcBlendForceMomentCoeffs(aero_model, alpha, beta=0.0,
                               omega_hat=(0.0, 0.0, 0.0),
                               flaps=((0.0,) * labels.kNumFlaps),
                               reynolds_number=None):
  """Calculates a BlendDatum of force-moment coefficients.

  Args:
    aero_model: A physics.Aero instance.
    alpha: Angle-of-attack [rad].
    beta: Sideslip angle [rad].
    omega_hat: Length-3 object of body rates [rad/s].
    flaps: Length-8 object of flap deflections [rad].
    reynolds_number: Reynolds number [#].

  Returns:
    BlendDatum of force-moment coefficients.
  """

  omega_hat_vec3 = physics.Vec3()
  omega_hat_vec3.x, omega_hat_vec3.y, omega_hat_vec3.z = omega_hat

  flaps_vec = physics.VecWrapper(labels.kNumFlaps)
  for i, flap in enumerate(flaps):
    flaps_vec.SetValue(i, flap)

  if reynolds_number is None:
    reynolds_number = FLAGS.re

  cfms = BlendDatum(lambda key: physics.ForceMoment())
  thrust_coeff = FLAGS.thrust_coeff

  aero_model.CalcLowIncidenceCoeffs(alpha, beta, omega_hat_vec3,
                                    flaps_vec.GetVec(), reynolds_number,
                                    cfms['low'].this, thrust_coeff)
  aero_model.CalcHighIncidenceCoeffs(alpha, beta, omega_hat_vec3,
                                     flaps_vec.GetVec(), reynolds_number,
                                     cfms['high'].this, thrust_coeff)
  aero_model.CalcForceMomentCoeff(alpha, beta, omega_hat_vec3,
                                  flaps_vec.GetVec(), reynolds_number,
                                  cfms['blended'].this, thrust_coeff)
  return cfms


def RotBToW(cf_b, alpha, beta):
  cf_w = physics.Vec3()
  physics.RotBToW(cf_b.this, alpha, beta, cf_w.this)
  return cf_w


def AeroOutputGetter(name):
  """Returns a function mapping from (CFM, alpha, beta) to an aero output."""

  assert name in _AERO_OUTPUTS, ('Invalid value "%s". Must be one of %s.'
                                 % _AERO_OUTPUTS)
  if name == 'Cx':
    return lambda cfm, alpha, beta: cfm.force.x
  elif name == 'Cy':
    return lambda cfm, alpha, beta: cfm.force.y
  elif name == 'Cz':
    return lambda cfm, alpha, beta: cfm.force.z
  if name == 'Cl':
    return lambda cfm, alpha, beta: cfm.moment.x
  elif name == 'Cm':
    return lambda cfm, alpha, beta: cfm.moment.y
  elif name == 'Cn':
    return lambda cfm, alpha, beta: cfm.moment.z
  elif name == 'CL':
    return lambda cfm, alpha, beta: -RotBToW(cfm.force, alpha, beta).z
  elif name == 'CD':
    return lambda cfm, alpha, beta: -RotBToW(cfm.force, alpha, beta).x
  else:
    raise ProgrammerError('Case "%s" is not handled.' % name)


def CalcAeroOutput(aero_model, spec, alpha, beta,
                   omega_hat=(0.0, 0.0, 0.0),
                   flaps=((0.0,) * labels.kNumFlaps)):
  getter = AeroOutputGetter(spec)
  cfms = CalcBlendForceMomentCoeffs(aero_model, alpha, beta=beta,
                                    omega_hat=omega_hat, flaps=flaps)
  return BlendDatum(lambda key: getter(cfms[key], alpha, beta))


def CalcAeroDerivative(aero_model, spec, alpha, beta,
                       omega_hat=(0.0, 0.0, 0.0),
                       flaps=((0.0,) * labels.kNumFlaps),
                       reynolds_number=None):
  """Calculates an aerodynamic derivative.

  Args:
    aero_model: A physics.Aero instance.
    spec: A string specifier for the derivative, of the form
        d[output]/d[var]. E.g. 'dCL/dalpha' or 'dCm/dq'. See
        _AERO_OUTPUTS and _AERO_VARS for allowed values.
    alpha: Angle-of-attack [rad].
    beta: Sideslip angle [rad].
    omega_hat: Length-3 object of body rates [rad/s].
    flaps: Length-8 object of flap deflections [rad].
    reynolds_number: Reynolds number [#].

  Returns:
    BlendDatum of the aerodynamic derivative.

  Raises:
    ProgrammerError: In case of a coding mistake.
  """

  assert IsDerivativeSpec(spec), 'Invalid specifier: "%s"' % spec

  if reynolds_number is None:
    reynolds_number = FLAGS.re
  omega_hat = np.array(omega_hat)
  flaps = np.array(list(flaps))

  numerator, denominator = spec.split('/')
  getter = AeroOutputGetter(numerator[1:])
  var = denominator[1:]

  # Step size for finite differences. This is in either [rad] or [rad/s],
  # depending on what we're differentiating with respect to.
  h = 0.01

  dalpha, dbeta, domega_hat = 0.0, 0.0, np.zeros(3)
  dflaps = np.zeros(labels.kNumFlaps)

  if var == 'alpha':
    dalpha = h
  elif var == 'beta':
    dbeta = h
  elif var == 'p':
    domega_hat[0] = h
  elif var == 'q':
    domega_hat[1] = h
  elif var == 'r':
    domega_hat[2] = h
  elif var == 'ail':
    dflaps = h * np.array([1.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 0.0])
  elif var == 'ele':
    dflaps[labels.kFlapEle] = h
  elif var == 'rud':
    dflaps[labels.kFlapRud] = h
  else:
    raise ProgrammerError('Case "%s" is not handled.' % var)

  cfms_0 = CalcBlendForceMomentCoeffs(aero_model, alpha, beta=beta,
                                      omega_hat=omega_hat, flaps=flaps,
                                      reynolds_number=reynolds_number)
  cfms_1 = CalcBlendForceMomentCoeffs(aero_model, alpha + dalpha,
                                      beta=beta + dbeta,
                                      omega_hat=omega_hat + domega_hat,
                                      flaps=flaps + dflaps,
                                      reynolds_number=reynolds_number)

  def CalcDerivative(key):
    return np.deg2rad((getter(cfms_1[key], alpha + dalpha, beta + dbeta)
                       - getter(cfms_0[key], alpha, beta)) / h)

  return BlendDatum(CalcDerivative)


def Plot(aero_model, alpha_degs, beta_degs, spec):
  """Plots an aerodynamic quantity against angle-of-attack or sideslip.

  Args:
    aero_model: A physics.Aero instance.
    alpha_degs: An array of alpha values [deg].
    beta_degs: An array of beta values [deg].
    spec: Either an _AERO_OUTPUT or a derivative spec (see CalcAeroDerivative).

  This function will assert out if both alpha_degs and beta_degs contain more
  than one element.
  """

  assert (np.size(alpha_degs) == 1 or np.size(beta_degs) == 1), (
      'Invalid inputs. alpha_degs or beta_degs must contain a single element.')

  if IsDerivativeSpec(spec):
    calc_function = CalcAeroDerivative
  else:
    calc_function = CalcAeroOutput

  blend_data = BlendDatum(lambda key: list())
  for alpha_rad in np.deg2rad(alpha_degs):
    for beta_rad in np.deg2rad(beta_degs):
      datum = calc_function(aero_model, spec, alpha_rad, beta_rad)
      for key in BlendDatum.KEYS:
        blend_data[key].append(datum[key])

  if np.size(alpha_degs) > 1:
    abscissa = alpha_degs
    x_label = 'Angle-of-attack [deg]'
  else:
    abscissa = beta_degs
    x_label = 'Angle-of-sideslip [deg]'

  pyplot.plot(abscissa, blend_data['low'], 'b.:', label='Low incidence')
  pyplot.plot(abscissa, blend_data['high'], 'g.:', label='High incidence')
  pyplot.plot(abscissa, blend_data['blended'], 'r.-', label='Blended')

  pyplot.legend(loc='best').draggable()
  pyplot.title(spec, fontsize=20)
  pyplot.xlabel(x_label)
  pyplot.grid(linewidth=0.5)
  pyplot.gcf().canvas.set_window_title(spec)


def TileFigures(num_figures, num_rows=None, num_cols=None):
  if num_rows is None:
    num_rows = FLAGS.fig_rows
  if num_cols is None:
    num_cols = FLAGS.fig_cols

  for i in range(num_figures):
    offset_count, i_linear = divmod(i, num_rows * num_cols)
    i_row, i_col = divmod(i_linear, num_cols)
    manager = pyplot.figure(i).canvas.manager
    manager.toolbar.pan()
    manager.toolbar.hide()
    width, height = 500, 410
    offset = 30
    manager.window.setGeometry(width * i_col + offset * offset_count,
                               height * i_row + offset * offset_count,
                               width, height - 40)


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n' % e
    print 'Usage: %s\n%s' % (e, FLAGS)
    sys.exit(1)

  specs = FLAGS.specs
  for spec in specs:
    if not IsValidSpec(spec):
      raise RuntimeError('Invalid spec: %s. %s' % (spec,
                                                   _VALID_SPEC_DESCRIPTION))

  aero_model = physics.Aero(physics.GetAeroSimParams())

  for i, spec in enumerate(specs):
    pyplot.figure(i)
    Plot(aero_model, FLAGS.alpha_degs, FLAGS.beta_degs, spec)
  TileFigures(len(specs))
  pyplot.show()


if __name__ == '__main__':
  main(sys.argv)
