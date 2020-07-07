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


r"""Generates a hover database for a specific apparent wind speed.

Example:

  ./generate_hover_database.py \
      --filename="output.dat" \
      --apparent_wind_speed=5.0 \
      --alphas_deg="-40.0, 100.0, 15" \
      --betas_deg="-60.0, 60.0, 7"
"""

import collections
import json
import logging
import multiprocessing
import sys
import gflags
from makani.analysis.aero import airfoil
from makani.analysis.aero.hover_model import hover_model
from makani.lib.python import dict_util
from makani.lib.python import flag_types
import numpy

FLAGS = gflags.FLAGS

gflags.DEFINE_float('apparent_wind_speed', '5.0', 'Apparent wind speed [m/s].')
gflags.DEFINE_boolean('use_wake_model', True,
                      'Boolean that determines whether the advected rotor wake '
                      'is included with the local apparent wind.')
gflags.DEFINE_string('filename', 'output.json', 'Output filename of database.')
gflags.DEFINE_integer('num_workers', 5,
                      'Number of processes to run in parallel.')
gflags.DEFINE_string('wing_serial', '01',
                     'Wing serial to generate a database for.')
gflags.DEFINE_string('alpha_intervals_deg', '-180.0,-30.0,-9.0,9.0,30.0,180.0',
                     'Comma separated list of angle-of-attack values [deg], '
                     'indicating the start and end points of multiple '
                     'linspaces.')
gflags.DEFINE_string('num_alphas', '8,5,10,5,8',
                     'Number of angle-of-attack points to place in between '
                     'each of the start / end pairs in alpha_intervals_deg.')
gflags.DEFINE_string('beta_intervals_deg', '-60.0,-15.0,15.0,60.0',
                     'Comma separated list of side-slip angles [deg], '
                     'indicating the start and end points of multiple '
                     'linspaces.')
gflags.DEFINE_string('num_betas', '4,4,4',
                     'Number of side-slip angle points to place in between '
                     'each of the start / end pairs in beta_intervals_deg.')
flag_types.DEFINE_linspace('delta1s_deg', '-25.0, 25.0, 5',
                           'Linspace range of delta1 angles [deg].')
flag_types.DEFINE_linspace('delta2s_deg', '-25.0, 25.0, 5',
                           'Linspace range of delta2 angles [deg].')
flag_types.DEFINE_linspace('delta3s_deg', '-25.0, 25.0, 5',
                           'Linspace range of delta3 angles [deg].')
flag_types.DEFINE_linspace('delta4s_deg', '-25.0, 25.0, 5',
                           'Linspace range of delta4 angles [deg].')
flag_types.DEFINE_linspace('delta5s_deg', '-25.0, 25.0, 5',
                           'Linspace range of delta5 angles [deg].')
flag_types.DEFINE_linspace('delta6s_deg', '-25.0, 25.0, 5',
                           'Linspace range of delta6 angles [deg].')
flag_types.DEFINE_linspace('delta7s_deg', '-110.0, 10.0, 24',
                           'Linspace range of delta7 angles [deg].')
flag_types.DEFINE_linspace('delta8s_deg', '-25.0, 25.0, 5',
                           'Linspace range of delta8 angles [deg].')


# Number of flaps to use in the hover database.
_NUM_DELTAS = 8


def _VariableLinspaceFromStrings(breaks_string, counts_string):
  """Construct a variable density increasing array from command line flags.

  For example:

    _VariableLinspaceFromString('0,2,8', '1,3')

  will return the same value as:

    np.array([0.0, 1.0, 2.0, 3.5, 5.0, 6.5, 8.0])

  Args:
    breaks_string: A string giving a comma separated list of n + 1 float values
        in increasing order.
    counts_string: A string giving a list of n non-negative integers prescribing
        how many values to place in between the break points.

  Raises:
    ValueError: if the values encoded by the string are invalid.

  Returns:
    A numpy array of values as described above.
  """
  try:
    counts = [int(c) for c in counts_string.split(',')]
    breaks = [float(r) for r in breaks_string.split(',')]
  except ValueError:
    raise ValueError('Bad format for range argument.')

  if len(counts) != len(breaks) - 1:
    raise ValueError('Inconsistent lengths for range arguments.')

  total_range = []
  for i in range(len(counts)):
    first_range = numpy.linspace(breaks[i], breaks[i + 1], 2 + counts[i])
    if i > 0:
      first_range = first_range[1:]
    total_range += first_range.tolist()

  return numpy.array(total_range)


def CalcZeroDeflectionCoeffs(alpha_list, beta_list,
                             nominal_local_apparent_winds_sph,
                             params):
  """Calculates the coefficients at zero flap deflection."""
  alphas, betas = numpy.meshgrid(alpha_list, beta_list, indexing='ij')
  zeros = numpy.zeros((_NUM_DELTAS,) + numpy.shape(alphas))
  return hover_model.CalcForceMomentCoeffs(
      alphas, betas, zeros, [0.0, 0.0, 0.0], FLAGS.apparent_wind_speed, params,
      local_apparent_winds_sph=nominal_local_apparent_winds_sph)


def CalcDeltaIncrementCoeffs(alpha_list, beta_list, delta_list, delta_index,
                             nominal_coeffs, nominal_local_apparent_winds_sph,
                             params):
  """Calculates coefficient increments for each flap."""
  alphas, betas, deltas_i = numpy.meshgrid(alpha_list, beta_list, delta_list,
                                           indexing='ij')
  deltas = numpy.zeros((_NUM_DELTAS,) + numpy.shape(alphas))
  deltas[delta_index] = deltas_i
  coeffs = hover_model.CalcForceMomentCoeffs(
      alphas, betas, deltas, [0.0, 0.0, 0.0], FLAGS.apparent_wind_speed, params,
      local_apparent_winds_sph=nominal_local_apparent_winds_sph)
  coeffs_0 = numpy.tile(numpy.expand_dims(nominal_coeffs, axis=2),
                        (1, 1, len(delta_list), 1))
  return coeffs - coeffs_0


def CalcAngularRateDerivativeCoeffs(alpha_list, beta_list, omega_hat_step,
                                    positive_rate_local_apparent_winds_sph,
                                    negative_rate_local_apparent_winds_sph,
                                    params):
  """Calculates angular rate derivatives in the nominal flap position."""
  omega_hat_step = numpy.array(omega_hat_step)
  alphas, betas = numpy.meshgrid(alpha_list, beta_list, indexing='ij')
  zeros = numpy.zeros((_NUM_DELTAS,) + numpy.shape(alphas))
  coeffs_p = hover_model.CalcForceMomentCoeffs(
      alphas, betas, zeros, omega_hat_step / 2.0, FLAGS.apparent_wind_speed,
      params, local_apparent_winds_sph=positive_rate_local_apparent_winds_sph)
  coeffs_n = hover_model.CalcForceMomentCoeffs(
      alphas, betas, zeros, -omega_hat_step / 2.0, FLAGS.apparent_wind_speed,
      params, local_apparent_winds_sph=negative_rate_local_apparent_winds_sph)
  return (coeffs_p - coeffs_n) / numpy.linalg.norm(omega_hat_step)


def CalcDeltaIncrementAngularRateDerivativeCoeffs(
    alpha_list, beta_list, delta_list, delta_index, omega_hat_step,
    nominal_derivative_coeffs,
    positive_rate_local_apparent_winds_sph,
    negative_rate_local_apparent_winds_sph, params):
  """Calculates angular rate derivative increments for each flap."""
  omega_hat_step = numpy.array(omega_hat_step)
  alphas, betas, deltas_i = numpy.meshgrid(alpha_list, beta_list, delta_list,
                                           indexing='ij')
  deltas = numpy.zeros((_NUM_DELTAS,) + numpy.shape(alphas))
  deltas[delta_index] = deltas_i

  coeffs_p = hover_model.CalcForceMomentCoeffs(
      alphas, betas, deltas, omega_hat_step / 2.0, FLAGS.apparent_wind_speed,
      params,
      local_apparent_winds_sph=positive_rate_local_apparent_winds_sph)
  coeffs_n = hover_model.CalcForceMomentCoeffs(
      alphas, betas, deltas, -omega_hat_step / 2.0, FLAGS.apparent_wind_speed,
      params,
      local_apparent_winds_sph=negative_rate_local_apparent_winds_sph)

  coeffs_0 = numpy.tile(numpy.expand_dims(nominal_derivative_coeffs, axis=2),
                        (1, 1, len(delta_list), 1))
  return (coeffs_p - coeffs_n) / numpy.linalg.norm(omega_hat_step) - coeffs_0


def GenerateHoverDatabase():
  """Generates a hover aerodynamics database in the DVL format.

  A hover aerodynamic database models all the aerodynamic surfaces as
  independent airfoils and accounts for the effect of the propwash on
  these surfaces.  The database is output in the DVL format which
  includes the following coefficients and derivatives:

    cx, cy, cz, cl, cm, cn,
    dcx1, dcy1, dcz1, dcl1, dcm1, dcn1,
    ...
    dcx8, dcy8, dcz8, dcl8, dcm8, dcn8,
    dcx/dp, dcy/dp, dcz/dp, dcl/dp, dcm/dp, dcn/dp
    dcx/dq, dcy/dq, dcz/dq, dcl/dq, dcm/dq, dcn/dq
    dcx/dr, dcy/dr, dcz/dr, dcl/dr, dcm/dr, dcn/dr
    dcx1/dp, dcy1/dp, dcz1/dp, dcl1/dp, dcm1/dp, dcn1/dp
    dcx1/dq, dcy1/dq, dcz1/dq, dcl1/dq, dcm1/dq, dcn1/dq
    dcx1/dr, dcy1/dr, dcz1/dr, dcl1/dr, dcm1/dr, dcn1/dr
    ...

  """
  alpha_list = _VariableLinspaceFromStrings(
      FLAGS.alpha_intervals_deg, FLAGS.num_alphas) * numpy.pi / 180.0
  beta_list = _VariableLinspaceFromStrings(
      FLAGS.beta_intervals_deg, FLAGS.num_betas) * numpy.pi / 180.0
  delta1s = numpy.pi / 180.0 * FLAGS.delta1s_deg
  delta2s = numpy.pi / 180.0 * FLAGS.delta2s_deg
  delta3s = numpy.pi / 180.0 * FLAGS.delta3s_deg
  delta4s = numpy.pi / 180.0 * FLAGS.delta4s_deg
  delta5s = numpy.pi / 180.0 * FLAGS.delta5s_deg
  delta6s = numpy.pi / 180.0 * FLAGS.delta6s_deg
  delta7s = numpy.pi / 180.0 * FLAGS.delta7s_deg
  delta8s = numpy.pi / 180.0 * FLAGS.delta8s_deg

  logging.info('Building wing parameters and airfoils for %s:%s.',
               FLAGS.wing_model, FLAGS.wing_serial)
  params = hover_model.GetParams(FLAGS.wing_model, FLAGS.wing_serial,
                                 use_wake_model=FLAGS.use_wake_model)

  def _ProcessTasks(tasks, group_name=''):
    """Iterates through a dictionary of tasks executing them in parallel."""
    def _CreateTaskTarget(func):
      def _WrapFun(args, pipe):
        pipe.put(func(*args))
        pipe.close()
      return _WrapFun

    queues = [multiprocessing.Queue() for _ in tasks]
    task_names = []
    processes = []
    for i, (name, (func, args)) in enumerate(tasks.iteritems()):
      task_names += [name]
      processes.append(multiprocessing.Process(
          target=_CreateTaskTarget(func), args=(args, queues[i])))

    pending = [True for _ in range(len(task_names))]
    running = [False for _ in range(len(task_names))]
    results = {
        task_name: None for task_name in task_names
    }
    while numpy.sum(running) > 0 or numpy.sum(pending) > 0:
      # Fill the running queue.
      while numpy.sum(running) < FLAGS.num_workers and numpy.sum(pending) > 0:
        start_idx = numpy.argwhere(pending)[0, 0]
        processes[start_idx].start()
        logging.info('Calculating %s%s...',
                     group_name + ': ' if group_name else '',
                     task_names[start_idx])
        pending[start_idx] = False
        running[start_idx] = True

      done = numpy.logical_and(running, [not queue.empty() for queue in queues])
      for done_idx in numpy.argwhere(done):
        results[task_names[done_idx[0]]] = queues[done_idx[0]].get(
            block=True, timeout=None)
        running[done_idx[0]] = False

    # Wait for work to finish.
    for process in processes:
      process.join()

    return results

  database = {
      'reynolds_number': (params['phys']['rho'] * FLAGS.apparent_wind_speed
                          * params['wing']['c']
                          / params['phys']['dynamic_viscosity']),
      'alphas': alpha_list,
      'betas': beta_list,
      'delta1s': delta1s,
      'delta2s': delta2s,
      'delta3s': delta3s,
      'delta4s': delta4s,
      'delta5s': delta5s,
      'delta6s': delta6s,
      'delta7s': delta7s,
      'delta8s': delta8s,
  }

  # Pre-compute the local apparent wind speeds for each sampling point
  # on each surface for all the different alphas, betas, and angular
  # rates.  This is the most expensive part of the database
  # calculation, so there are huge gains from pre-computing these.
  alphas, betas = numpy.meshgrid(alpha_list, beta_list, indexing='ij')
  omega_hat_step = 1e-2
  tasks = {
      'nominal': (
          hover_model.PrecomputeLocalApparentWindSph,
          (alphas, betas, [0.0, 0.0, 0.0], FLAGS.apparent_wind_speed, params)),
      'positive_p': (
          hover_model.PrecomputeLocalApparentWindSph,
          (alphas, betas, [omega_hat_step / 2.0, 0.0, 0.0],
           FLAGS.apparent_wind_speed, params)),
      'negative_p': (
          hover_model.PrecomputeLocalApparentWindSph,
          (alphas, betas, [-omega_hat_step / 2.0, 0.0, 0.0],
           FLAGS.apparent_wind_speed, params)),
      'positive_q': (
          hover_model.PrecomputeLocalApparentWindSph,
          (alphas, betas, [0.0, omega_hat_step / 2.0, 0.0],
           FLAGS.apparent_wind_speed, params)),
      'negative_q': (
          hover_model.PrecomputeLocalApparentWindSph,
          (alphas, betas, [0.0, -omega_hat_step / 2.0, 0.0],
           FLAGS.apparent_wind_speed, params)),
      'positive_r': (
          hover_model.PrecomputeLocalApparentWindSph,
          (alphas, betas, [0.0, 0.0, omega_hat_step / 2.0],
           FLAGS.apparent_wind_speed, params)),
      'negative_r': (
          hover_model.PrecomputeLocalApparentWindSph,
          (alphas, betas, [0.0, 0.0, -omega_hat_step / 2.0],
           FLAGS.apparent_wind_speed, params))
  }
  local_apparent_wind_sph = _ProcessTasks(tasks, 'local apparent wind')

  # The delta increment tasks require the base CFM so we run it first.
  tasks = {
      'cfm': (CalcZeroDeflectionCoeffs, (alpha_list, beta_list,
                                         local_apparent_wind_sph['nominal'],
                                         params))
  }

  rate_steps = {
      'p': [omega_hat_step, 0.0, 0.0],
      'q': [0.0, omega_hat_step, 0.0],
      'r': [0.0, 0.0, omega_hat_step]
  }
  for name, step in rate_steps.iteritems():
    derivative_name = 'dcfm_d%s' % name
    tasks.update({
        derivative_name: (CalcAngularRateDerivativeCoeffs,
                          (alpha_list, beta_list, step,
                           local_apparent_wind_sph['positive_%s' % name],
                           local_apparent_wind_sph['negative_%s' % name],
                           params))
    })
  database.update(_ProcessTasks(tasks))

  # The delta increment angular rate tasks require the results of the
  # previous computations so we run them in a separate sweep.
  tasks = {}
  deltas_list = [delta1s, delta2s, delta3s, delta4s, delta5s, delta6s,
                 delta7s, delta8s]
  for delta_index, delta_list in enumerate(deltas_list):
    derivative_name = 'dcfm%d' % (delta_index + 1)
    tasks.update({
        derivative_name: (CalcDeltaIncrementCoeffs,
                          (alpha_list, beta_list, delta_list, delta_index,
                           database['cfm'], local_apparent_wind_sph['nominal'],
                           params))
    })

  for rate_name, step in rate_steps.iteritems():
    for delta_index, delta_list in enumerate(deltas_list):
      nominal_derivative_coeffs = database['dcfm_d%s' % rate_name]
      derivative_name = 'dcfm%d_d%s' % (delta_index + 1, rate_name)
      tasks.update({
          derivative_name: (CalcDeltaIncrementAngularRateDerivativeCoeffs,
                            (alpha_list, beta_list, delta_list, delta_index,
                             step, nominal_derivative_coeffs,
                             local_apparent_wind_sph['positive_%s' % rate_name],
                             local_apparent_wind_sph['negative_%s' % rate_name],
                             params))
      })
  database.update(_ProcessTasks(tasks))

  logging.info('Writing database to file %s.', FLAGS.filename)
  WriteJsonFile(FLAGS.filename, params, database)


def WriteJsonFile(filename, params, database):
  """Write database out as a .dat file.

  Args:
    filename: Name of output file to write database to.
    params: Parameter structure used to generate the database.
    database: Dictionary of ndarrays of aerodynamic coefficients and
        derivatives.
  """
  def _PrepareCoefficientArray(array):
    return numpy.reshape(numpy.rollaxis(array, -1), (array.size,)).tolist()

  keys_and_values = [
      ('num_alphas', len(database['alphas'])),
      ('num_betas', len(database['betas'])),
      ('num_deltas', [len(database['delta1s']), len(database['delta2s']),
                      len(database['delta3s']), len(database['delta4s']),
                      len(database['delta5s']), len(database['delta6s']),
                      len(database['delta7s']), len(database['delta8s'])]),
      ('reynolds_number', database['reynolds_number']),
      ('alphas', database['alphas']),
      ('betas', database['betas']),
      ('delta1s', database['delta1s']),
      ('delta2s', database['delta2s']),
      ('delta3s', database['delta3s']),
      ('delta4s', database['delta4s']),
      ('delta5s', database['delta5s']),
      ('delta6s', database['delta6s']),
      ('delta7s', database['delta7s']),
      ('delta8s', database['delta8s']),
      ('cfm', _PrepareCoefficientArray(database['cfm'])),
      ('dcfm_dp', _PrepareCoefficientArray(database['dcfm_dp'])),
      ('dcfm_dq', _PrepareCoefficientArray(database['dcfm_dq'])),
      ('dcfm_dr', _PrepareCoefficientArray(database['dcfm_dr'])),
      ('dcfm1', _PrepareCoefficientArray(database['dcfm1'])),
      ('dcfm1_dp', _PrepareCoefficientArray(database['dcfm1_dp'])),
      ('dcfm1_dq', _PrepareCoefficientArray(database['dcfm1_dq'])),
      ('dcfm1_dr', _PrepareCoefficientArray(database['dcfm1_dr'])),
      ('dcfm2', _PrepareCoefficientArray(database['dcfm2'])),
      ('dcfm2_dp', _PrepareCoefficientArray(database['dcfm2_dp'])),
      ('dcfm2_dq', _PrepareCoefficientArray(database['dcfm2_dq'])),
      ('dcfm2_dr', _PrepareCoefficientArray(database['dcfm2_dr'])),
      ('dcfm3', _PrepareCoefficientArray(database['dcfm3'])),
      ('dcfm3_dp', _PrepareCoefficientArray(database['dcfm3_dp'])),
      ('dcfm3_dq', _PrepareCoefficientArray(database['dcfm3_dq'])),
      ('dcfm3_dr', _PrepareCoefficientArray(database['dcfm3_dr'])),
      ('dcfm4', _PrepareCoefficientArray(database['dcfm4'])),
      ('dcfm4_dp', _PrepareCoefficientArray(database['dcfm4_dp'])),
      ('dcfm4_dq', _PrepareCoefficientArray(database['dcfm4_dq'])),
      ('dcfm4_dr', _PrepareCoefficientArray(database['dcfm4_dr'])),
      ('dcfm5', _PrepareCoefficientArray(database['dcfm5'])),
      ('dcfm5_dp', _PrepareCoefficientArray(database['dcfm5_dp'])),
      ('dcfm5_dq', _PrepareCoefficientArray(database['dcfm5_dq'])),
      ('dcfm5_dr', _PrepareCoefficientArray(database['dcfm5_dr'])),
      ('dcfm6', _PrepareCoefficientArray(database['dcfm6'])),
      ('dcfm6_dp', _PrepareCoefficientArray(database['dcfm6_dp'])),
      ('dcfm6_dq', _PrepareCoefficientArray(database['dcfm6_dq'])),
      ('dcfm6_dr', _PrepareCoefficientArray(database['dcfm6_dr'])),
      ('dcfm7', _PrepareCoefficientArray(database['dcfm7'])),
      ('dcfm7_dp', _PrepareCoefficientArray(database['dcfm7_dp'])),
      ('dcfm7_dq', _PrepareCoefficientArray(database['dcfm7_dq'])),
      ('dcfm7_dr', _PrepareCoefficientArray(database['dcfm7_dr'])),
      ('dcfm8', _PrepareCoefficientArray(database['dcfm8'])),
      ('dcfm8_dp', _PrepareCoefficientArray(database['dcfm8_dp'])),
      ('dcfm8_dq', _PrepareCoefficientArray(database['dcfm8_dq'])),
      ('dcfm8_dr', _PrepareCoefficientArray(database['dcfm8_dr']))
  ]

  output_dict = collections.OrderedDict(
      keys_and_values + [('params', dict_util.OrderDict(params))])

  class _ParamsEncoder(json.JSONEncoder):
    """JSON encoder which handles the Airfoil objects and numpy arrays."""

    def default(self, o):
      if isinstance(o, airfoil.Airfoil):
        return str(o)
      elif isinstance(o, numpy.ndarray):
        return o.tolist()
      return json.JSONEncoder.default(self, o)

  with open(filename, 'w') as f:
    output_string = json.dumps(output_dict, separators=(', ', ':\n  '),
                               cls=_ParamsEncoder)
    output_string = (output_string
                     .replace(', \"', ',\n\"')
                     .replace('], [', '],\n   [')
                     .replace(' [[', '[[')
                     .replace('{', '{\n')
                     .replace('}', '\n}')) + '\n'
    f.write(output_string)


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '%s\\nUsage: %s ARGS\\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)
  logging.basicConfig(stream=sys.stdout,
                      format='%(asctime)s %(levelname)-8s %(message)s',
                      level=logging.INFO)
  logging.info('Generating hover database.')
  GenerateHoverDatabase()


if __name__ == '__main__':
  main(sys.argv)
