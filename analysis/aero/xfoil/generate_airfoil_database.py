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

r"""Generates airfoil lift, drag, and moment coefficients.

Runs XFOIL (http://web.mit.edu/drela/Public/web/xfoil/) to generate
lift, drag, and moment coefficients over a tensor grid of flap
deflections and angles-of-attack.  The results are stored as a JSON
file.

Example Usage:

  ./generate_airfoil_database.py \
      --naca=0012 \
      --reynolds_number=1e6 \
      --alphas_deg='-10.0,10.0,20' \
      --json_output_file=naca0012.json

"""

import collections
import copy
import json
import logging
import os
import re
import subprocess
import sys

import gflags
import makani
from makani.lib.python import flag_types
from makani.lib.python import os_util
import numpy

# Angle [rad] over which to blend the XFOIL parameters with the high
# angle-of-attack parameters after stall.
_STALL_BLENDING_ANGLE = 5.0 * numpy.pi / 180.0

# Full path to the XFOIL binary.
_XFOIL_BIN = os.path.join(makani.HOME, 'third_party/xfoil/xfoil')

# Name of temporary file that XFOIL will use to store airfoils.
_TEMP_FILENAME = 'tmp_xfoil'

gflags.DEFINE_string('airfoil_file', None,
                     'Name of the airfoil definition file.')
gflags.RegisterValidator('airfoil_file', lambda x: not x or os.path.exists(x),
                         'airfoil_file does not exist.')

gflags.DEFINE_boolean('display_log', False, 'Displays output of XFOIL.')
gflags.DEFINE_boolean('display_input', False, 'Displays input to XFOIL.')

gflags.DEFINE_string('json_output_file', None, 'Name of output JSON file.')
gflags.RegisterValidator('json_output_file',
                         lambda f: f and os.path.basename(f),
                         'json_output_file mush have a valid file name.')

flag_types.DEFINE_linspace('flap_deflections_deg', '0.0, 0.0, 1',
                           'Linspace range of flap deflections [deg].')
flag_types.DEFINE_linspace('alphas_deg', '-10.0, 10.0, 20',
                           'Linspace range of angles-of-attack [deg].')

# TODO: Support 5 digit NACA airfoils.  This requires a
# minor change to the GetAirfoilName function.
gflags.DEFINE_integer('naca', None, '4 digit NACA airfoil.',
                      lower_bound=0, upper_bound=9999)
gflags.DEFINE_float('reynolds_number', 5e6, 'Reynolds number.',
                    lower_bound=0.0)
gflags.DEFINE_float('mach_number', 0.0, 'Mach number.',
                    lower_bound=0.0, upper_bound=1.0)
gflags.DEFINE_integer('num_iter', 130,
                      'Number of iterations allowed to converge.')
gflags.DEFINE_float('n_crit', 9.0, 'Amplification factor used in the e^n '
                    'method of determining boundary layer transition.')
gflags.DEFINE_list('trip_position', '1.0, 1.0',
                   'Top and bottom forced trip points (x/c).')
gflags.DEFINE_list('hinge_position', '1.0, 0.0', 'Location of the flap hinge.')

gflags.DEFINE_list('stall_angles_deg', '-inf, inf',
                   'Angles [deg] at which to start blending to typical '
                   'post-stall values')

FLAGS = gflags.FLAGS


def GetXfoilInput(flap_deflections, alphas, params):
  """Creates string of input commands to XFOIL."""

  # Set global options.
  input_str = (
      # Load NACA foil if one is assigned.
      '{naca_cmd}{naca_num}\n'

      # Save a copy of the airfoil that can be reloaded after
      # modifications.  (The second \n forces an overwrite.)
      'save {temp_filename}\n\n'

      # Disable plotting.
      'plop\n'
      'G\n\n'

      # Enter the operating point menu.
      'oper\n'

      # Increase default number of iterations.
      'iter {num_iter}\n'

      # Use viscous solution at a given Reynolds and Mach number.
      'visc\n'
      '{reynolds_number}\n'
      'mach {mach_number}\n'

      # Set boundary layer parameters including amplification factor
      # and forced trip points.
      'vpar\n'
      'n\n{n_crit}\n'
      'x\n{top_trip}\n{bottom_trip}\n\n'

      # Set flap hinge point for moment calculation.
      'fnew {hinge_x_c} {hinge_y_c}\n'

      # Return to main menu.
      '\n'
      ).format(
          naca_cmd='naca ' if params['naca'] else '',
          naca_num=params['naca'] if params['naca'] else '',
          temp_filename=_TEMP_FILENAME,
          num_iter=params['num_iter'],
          reynolds_number=params['reynolds_number'],
          mach_number=params['mach_number'],
          n_crit=params['n_crit'],
          top_trip=params['trip_position'][0],
          bottom_trip=params['trip_position'][1],
          hinge_x_c=params['hinge_position'][0],
          hinge_y_c=params['hinge_position'][1])

  for flap_deflection in flap_deflections:
    input_str += (
        # Reload original airfoil.
        'load {temp_filename}\n'

        # Enter the geometry design routine.
        'gdes\n'

        # Add points to reduce maximum panel angle.  This is somewhat
        # of a magic incantation that was necessary to be able to use
        # the flap routine on the vertical stabilizer.
        'cadd 5 2 -1 1\n'

        # Add a flap deflection to the airfoil.  This is broken into
        # multiple smaller flap deflections because XFOIL's flap
        # command introduces nasty effects for large deflections.
        'flap {hinge_x_c} {hinge_y_c} {fifth_flap_deflection_deg}\n'
        'flap {hinge_x_c} {hinge_y_c} {fifth_flap_deflection_deg}\n'
        'flap {hinge_x_c} {hinge_y_c} {fifth_flap_deflection_deg}\n'
        'flap {hinge_x_c} {hinge_y_c} {fifth_flap_deflection_deg}\n'
        'flap {hinge_x_c} {hinge_y_c} {fifth_flap_deflection_deg}\n\n'

        # Set panel nodes based on airfoil curvature.
        'pane\n'

        # Enter the operating point menu.
        'oper\n'

        # Initialize the boundary layer solution.  This is necessary
        # so the call to 'init' below resets the boundary layer
        # solution rather than toggling the state to assuming it is
        # initialized.
        'alfa 0\n'

        # Reinitialize the boundary layer solution.
        'init\n'
        ).format(
            temp_filename=_TEMP_FILENAME,
            hinge_x_c=params['hinge_position'][0],
            hinge_y_c=params['hinge_position'][1],
            fifth_flap_deflection_deg=180.0 / numpy.pi * flap_deflection / 5.0)

    # Help get converged solutions at the minimum alpha by stepping to
    # the minimum alpha in half degree increments.
    for alpha_deg in numpy.arange(0.0, min(alphas) * 180.0 / numpy.pi, -0.5):
      input_str += 'alfa %f\n' % alpha_deg

    # Calculate airfoil properties at each angle-of-attack.
    for alpha_deg in 180.0 / numpy.pi * numpy.array(alphas):
      input_str += (
          # Calculate values at given angle-of-attack.
          'alfa {alpha_deg}\n'

          # This is a marker that is used during the parsing of the
          # log file.  It has no effect on the program.
          'here\n'

          # Redisplay calculated values at given angle-of-attack.
          'alfa {alpha_deg}\n'

          # Display hinge moment information.
          'fmom\n').format(alpha_deg=alpha_deg)

    # Return to main menu.
    input_str += '\n'

  # Exit.
  input_str += 'quit'

  if FLAGS.display_input:
    print input_str

  return input_str


def ValidateXfoilLog(log):
  """Looks for common errors in the log file."""

  # Check whether XFOIL had trouble locating data files; it will
  # happily charge ahead with default data if that happens.
  file_not_found_regex = re.compile(r'.*File OPEN error.  Nonexistent file:')

  # Check whether the boundary layer initialization was ever toggled
  # to the wrong state.
  bl_initialized_regex = re.compile(r'BLs are assumed to be initialized')

  # Check whether the operating point solution converged.
  convergence_regex = re.compile(r'.*VISCAL:  Convergence failed')

  # Check whether the local speed is too large for the compressibility
  # correction to be valid.
  speed_too_large_regex = re.compile(r'.*CPCALC: Local speed too large.')

  num_convergence = 0
  num_speed_too_large = 0
  num_bl_initialized = 0
  for line in iter(log.splitlines()):
    if file_not_found_regex.match(line):
      raise RuntimeError('\n\033[1m\033[91mXFOIL error: "%s"' % line.strip())

    if convergence_regex.match(line):
      num_convergence += 1

    if speed_too_large_regex.match(line):
      num_speed_too_large += 1

    if bl_initialized_regex.match(line):
      num_bl_initialized += 1

  if num_convergence:
    logging.warning('Some solutions failed to converge (%s occurrences).',
                    str(num_convergence))

  if num_speed_too_large:
    logging.warning('Compressibility corrections invalid for some solutions '
                    '(%s occurrences).', str(num_speed_too_large))

  if num_bl_initialized:
    logging.warning('Boundary layer initialization was toggled to the wrong '
                    'state (%s occurrences).', str(num_bl_initialized))


def ParseXfoilLog(log):
  """Extracts calculated airfoil properties from XFOIL output."""

  # These regular expressions are used to extract values from a single
  # operating point of the airfoil.  It's a bit complicated, so here
  # are some key features:
  #
  # - fp_regex matches a floating point number, or a sequence of nine
  #   '*' which Drela sometimes uses for an unconverged value.
  # - The (?P<var>expr) notation lets Python store the result in a
  #   dictionary.
  # - re.DOTALL lets the '.' character match newlines, so this regular
  #   expression will operate across multiple lines.
  # - The '.*?' expression does a non-greedy match of characters,
  #   i.e. the minimum number of characters, until it matches what
  #   comes next.
  fp_regex = r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?|\*\*\*\*\*\*\*\*\*'
  case_regex = re.compile(
      r'HERE command not recognized.*?'
      r'transition at x/c =\s*(?P<bot_xtr>' + fp_regex + r').*?'
      r'transition at x/c =\s*(?P<top_xtr>' + fp_regex + r').*?'
      r'rms:\s*(?P<rms>' + fp_regex + r').*?'
      r'max:\s*(?P<max>' + fp_regex + r').*?'
      r'a =\s*(?P<a>' + fp_regex + r').*?'
      r'CL =\s*(?P<CL>' + fp_regex + r').*?'
      r'Cm =\s*(?P<Cm>' + fp_regex + r').*?'
      r'CD =\s*(?P<CD>' + fp_regex + r').*?'
      r'CDf =\s*(?P<CDf>' + fp_regex + r').*?'
      r'CDp =\s*(?P<CDp>' + fp_regex + r').*?'
      r'Hinge moment/span =\s*(?P<hinge_moment>' + fp_regex + r')',
      re.DOTALL)

  # Make list of dictionaries that describe each run case.
  cases = [{k: numpy.nan if v == '*********' else float(v)
            for k, v in case.groupdict().iteritems()}
           for case in case_regex.finditer(log)]

  return cases


def RunXfoil(airfoil_file, input_str):
  """Runs XFOIL with specified conditions."""

  try:
    # XFOIL cannot handle long paths, so run from the directory the
    # airfoil is in.
    airfoil_dir = os.path.dirname(airfoil_file) if airfoil_file else '.'
    airfoil_base = os.path.basename(airfoil_file) if airfoil_file else ''
    with os_util.ChangeDir(airfoil_dir):
      process = subprocess.Popen([_XFOIL_BIN, airfoil_base],
                                 stdin=subprocess.PIPE, stdout=subprocess.PIPE)
      log = process.communicate(input=input_str)[0]

      # Clean-up after XFOIL.
      try:
        os.remove(':00.bl')
        os.remove(_TEMP_FILENAME)
      except OSError:
        pass

  except subprocess.CalledProcessError as e:
    raise RuntimeError('\n\033[1m\033[91mXFOIL error: exited with return '
                       'code %d: %s' % (e.returncode, e.output))

  if FLAGS.display_log:
    print log

  # Check for common errors in the log.
  ValidateXfoilLog(log)

  # Convert the log output to an array of dictionaries of results for
  # each run case.
  return ParseXfoilLog(log)


def CalcPostStallLiftCoeff(alpha, alpha_0):
  # [Fluid Dynamic Lift, Ch. 4-23, Eq. 16].
  return 2.0 * numpy.sin(alpha - alpha_0) * numpy.cos(alpha - alpha_0)


def CalcPostStallDragCoeff(alpha, alpha_0):
  # [Fluid Dynamic Lift, Ch. 4-23, Eq. 17].
  return 2.0 * numpy.sin(alpha - alpha_0)**2.0


def CalcPostStallMomentCoeff(alpha, alpha_0):
  # [Fluid Dynamic Lift, Ch. 4-23, Eq. 18].
  return -0.25 * 2.0 * numpy.sin(alpha - alpha_0)


def MakeDatabase(flap_deflections, alphas, xfoil_inds, xfoil_output, params):
  """Collects data used in database in single dictionary."""

  def IsAtOperatingPoint(case, alpha):
    return numpy.abs(case['a'] - alpha * 180.0 / numpy.pi) < 1e-3

  nan_array = numpy.empty((len(flap_deflections), len(alphas)))
  nan_array[:] = numpy.nan

  database = collections.OrderedDict(
      [('name', params['name']),
       ('reynolds_number', params['reynolds_number']),
       ('mach_number', params['mach_number']),
       ('num_iter', params['num_iter']),
       ('n_crit', params['n_crit']),
       ('trip_position', params['trip_position']),
       ('hinge_position', params['hinge_position']),
       ('flap_deflections', flap_deflections),
       ('alphas', alphas),
       ('lift_coeffs', copy.deepcopy(nan_array).tolist()),
       ('drag_coeffs', copy.deepcopy(nan_array).tolist()),
       ('moment_coeffs', copy.deepcopy(nan_array).tolist()),
       ('hinge_moment_coeffs', copy.deepcopy(nan_array).tolist()),
       ('converged', copy.deepcopy(nan_array).tolist())])

  index = 0
  for i, flap_deflection in enumerate(flap_deflections):
    # Calculate the negative of the angle-of-attack of the chord line.
    alpha_0 = -numpy.arctan2((1.0 - params['hinge_position'][0]) *
                             numpy.sin(flap_deflection),
                             params['hinge_position'][0] +
                             (1.0 - params['hinge_position'][0]) *
                             numpy.cos(flap_deflection))
    for j, alpha in enumerate(alphas):
      post_stall_lift_coeff = CalcPostStallLiftCoeff(alpha, alpha_0)
      post_stall_drag_coeff = CalcPostStallDragCoeff(alpha, alpha_0)
      post_stall_moment_coeff = CalcPostStallMomentCoeff(alpha, alpha_0)
      post_stall_hinge_moment_coeff = 0.0

      if j in xfoil_inds:
        # Check that the case has the expected angle-of-attack.
        assert IsAtOperatingPoint(xfoil_output[index], alpha)

        angle_past_stall = max(max(params['stall_angles'][0] - alpha,
                                   alpha - params['stall_angles'][1]),
                               0.0)
        weight = 1.0 - min(angle_past_stall / _STALL_BLENDING_ANGLE, 1.0)

        lift_coeff = (weight * xfoil_output[index]['CL'] +
                      (1.0 - weight) * post_stall_lift_coeff)
        drag_coeff = (weight * xfoil_output[index]['CD'] +
                      (1.0 - weight) * post_stall_drag_coeff)
        moment_coeff = (weight * xfoil_output[index]['Cm'] +
                        (1.0 - weight) * post_stall_moment_coeff)
        hinge_moment_coeff = (weight * xfoil_output[index]['hinge_moment'] +
                              (1.0 - weight) * post_stall_hinge_moment_coeff)
        # This is the value used to check convergence in XFOIL.
        converged = xfoil_output[index]['rms'] < 1e-4
        index += 1
      else:
        lift_coeff = post_stall_lift_coeff
        drag_coeff = post_stall_drag_coeff
        moment_coeff = post_stall_moment_coeff
        hinge_moment_coeff = post_stall_hinge_moment_coeff
        converged = True

      database['lift_coeffs'][i][j] = lift_coeff
      database['drag_coeffs'][i][j] = drag_coeff
      database['moment_coeffs'][i][j] = moment_coeff
      database['hinge_moment_coeffs'][i][j] = hinge_moment_coeff
      database['converged'][i][j] = converged

  return database


def WriteJsonDatabase(filename, airfoil_database):
  """Writes airfoil database in JSON format."""
  with open(filename, 'w') as f:
    f.write(json.dumps(airfoil_database, indent=2, separators=(',', ': ')))


def GetAirfoilName(airfoil_file, naca):
  if airfoil_file:
    return os.path.splitext(os.path.basename(airfoil_file))[0]
  elif naca:
    return 'naca' + str(10000 + naca)[1:]
  else:
    assert False


def main(argv):
  """Runs XFOIL over all desired conditions and output to a JSON file."""
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n' % e
    sys.exit(1)

  logging.basicConfig(stream=sys.stdout,
                      format='%(asctime)s %(levelname)-8s %(message)s',
                      level=logging.INFO)

  params = dict({
      'name': GetAirfoilName(FLAGS.airfoil_file, FLAGS.naca),
      'airfoil_file': FLAGS.airfoil_file,
      'naca': FLAGS.naca,
      'reynolds_number': FLAGS.reynolds_number,
      'mach_number': FLAGS.mach_number,
      'n_crit': FLAGS.n_crit,
      'num_iter': FLAGS.num_iter,
      'trip_position': [float(FLAGS.trip_position[0]),
                        float(FLAGS.trip_position[1])],
      'hinge_position': [float(FLAGS.hinge_position[0]),
                         float(FLAGS.hinge_position[1])],
      'stall_angles': [float(FLAGS.stall_angles_deg[0]) * numpy.pi / 180.0,
                       float(FLAGS.stall_angles_deg[1]) * numpy.pi / 180.0]
  })

  flap_deflections = [numpy.pi / 180.0 * x for x in FLAGS.flap_deflections_deg]
  alphas = [numpy.pi / 180.0 * x for x in FLAGS.alphas_deg]

  xfoil_inds, = numpy.where(numpy.logical_and(
      params['stall_angles'][0] - _STALL_BLENDING_ANGLE < numpy.array(alphas),
      numpy.array(alphas) < params['stall_angles'][1] + _STALL_BLENDING_ANGLE))
  xfoil_alphas = [alphas[i] for i in xfoil_inds]

  input_str = GetXfoilInput(flap_deflections, xfoil_alphas, params)

  # Call XFOIL with input_str piped into stdin.  Parse the output.
  logging.info('Running XFOIL on %s using the viscous formulation for %s cases.'
               ' This may take awhile.', params['name'],
               str(len(FLAGS.flap_deflections_deg) * len(FLAGS.alphas_deg)))
  xfoil_output = RunXfoil(FLAGS.airfoil_file, input_str)

  database = MakeDatabase(flap_deflections, alphas, xfoil_inds, xfoil_output,
                          params)

  # Write the airfoil database into the standard JSON format.
  logging.info('Writing database to %s.', FLAGS.json_output_file)
  WriteJsonDatabase(FLAGS.json_output_file, database)

  logging.shutdown()


if __name__ == '__main__':
  main(sys.argv)
