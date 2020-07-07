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

r"""Generates a rotor database using XROTOR.

Runs XROTOR (http://web.mit.edu/drela/Public/web/xrotor/) to generate
thrust and power coefficients over a tensor grid of angular rates and
freestream velocities. The results are stored as a JSON file to be
used by the flight simulator.

Example Usage:

  ./generate_rotor_database.py \
      --rotor_file=drela.xrot \
      --thrust_corrections=1.0,0.95 \
      --power_corrections=1.05,0.9 \
      --angular_rates="22.0, 260.0, 50" \
      --freesteam_vels="0.1, 90.0, 50" \
      --json_output_file=drela.json

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
import numpy

# Full path to the XROTOR binary.
_XROTOR_BIN = os.path.join(makani.HOME, 'third_party/xrotor/xrotor')

gflags.DEFINE_string('rotor_file', None, 'Name of the rotor definition file.')
gflags.RegisterValidator('rotor_file', os.path.exists,
                         'rotor_file does not exist.')

# TODO: Eventually, I'd like to incorporate empirical
# corrections like this directly in the XROTOR file.
gflags.DEFINE_list('thrust_corrections', [1.0, 1.0],
                   'Multipliers to the thrust when [dragging, thrusting]')
gflags.DEFINE_list('power_corrections', [1.0, 1.0],
                   'Multipliers to the power when [motoring, generating]')
gflags.DEFINE_float('blade_pitch_correction_deg', 0.0,
                    'Pitch angle [deg] change to rotor geometry')

gflags.DEFINE_boolean('display_log', False, 'Displays output of XROTOR.')
gflags.DEFINE_boolean('display_input', False, 'Displays input to XROTOR.')

gflags.DEFINE_string('json_output_file', None, 'Name of output JSON file.')
gflags.RegisterValidator('json_output_file',
                         lambda f: f and os.path.basename(f),
                         'json_output_file mush have a valid file name.')

flag_types.DEFINE_linspace('freestream_vels', '0.1, 90.0, 50',
                           'Linspace range of freestream velocities [m/s].')
flag_types.DEFINE_linspace('angular_rates', '22.0, 260.0, 50',
                           'Linspace range of angular rates [rad/s].')

FLAGS = gflags.FLAGS


def GetXrotorInput(freestream_vels, angular_rates, solution_type):
  """Creates string of input commands to XROTOR."""

  if solution_type == 'graded_momentum':
    solution_str = 'grad'
  elif solution_type == 'potential':
    solution_str = 'pot'
  elif solution_type == 'vortex':
    solution_str = 'vrtx'
  else:
    assert False

  input_str = (
      # Choose default number of stations (if this is a prompt).
      '\n'

      # Disable plotting.
      'plop\n'
      'G\n\n'

      # Set fluid properties from standard atmosphere at 0 km.
      'atmo 0\n'

      # Enter the operating point menu.
      'oper\n'

      # Use terse outputs.
      'ters\n'

      # Change blade pitch angle.
      'angl %f\n'

      # Use potential (Goldstein) formulation with rigid wake.
      'form\n'
      '%s\n'
      'wake\n\n' % (FLAGS.blade_pitch_correction_deg, solution_str))

  for freestream_vel in freestream_vels:
    # Set flight speed [m/s].
    input_str += 'velo %f\n' % freestream_vel
    for angular_rate in angular_rates:
      # Set angular velocity [rpm] and perform run.
      input_str += 'rpm %f\n' % (angular_rate * 60.0 / (2.0 * numpy.pi))

      # Display the result again.  This is necessary if we want to
      # capture the unconverged solutions too.
      input_str += 'disp\n'

  # Exit.
  input_str += '\n\nquit'

  if FLAGS.display_input:
    print input_str

  return input_str


def ValidateXrotorLog(log):
  """Looks for common errors in the log file."""

  # Check whether XROTOR had trouble locating data files; it will
  # happily charge ahead with default data if that happens.
  file_not_found_regex = re.compile(r'.*File.*not found')

  # When a solution fails to converge after a specified number of
  # iterations, XROTOR displays "Iteration limit exceeded".
  iteration_limit_regex = re.compile(r'.*Iteration limit exceeded')

  # If part of the blade is past the critical Mach number of the
  # airfoil, XROTOR displays: "CLFUNC: Local Mach number limited".
  mach_limited_regex = re.compile(r'.*CLFUNC: Local Mach number limited')

  num_iteration_limit = 0
  num_mach_limit = 0
  for line in iter(log.splitlines()):
    if file_not_found_regex.match(line):
      raise RuntimeError('\n\033[1m\033[91mXROTOR error: "%s"' % line.strip())

    if iteration_limit_regex.match(line):
      num_iteration_limit += 1

    if mach_limited_regex.match(line):
      num_mach_limit += 1

  if num_iteration_limit:
    logging.warning('Iteration limit exceeded and solution did not converge '
                    '(%s occurrences).', str(num_iteration_limit))

  if num_mach_limit:
    logging.warning('Local Mach number limited (%s occurrences).',
                    str(num_mach_limit))


def ParseXrotorLog(log):
  """Extracts calculated rotor properties from XROTOR output."""

  # These regular expressions are used to extract values from a single
  # operating point of the rotor.  It's a bit complicated, so here are
  # some key features:
  #
  # - fp_regex matches a floating point number.
  # - The (?P<var>expr) notation lets Python store the result in a
  #   dictionary.
  # - re.DOTALL lets the '.' character match newlines, so this regular
  #   expression will operate across multiple lines.
  # - The '.*?' expression does a non-greedy match of characters,
  #   i.e. the minimum number of characters, until it matches what
  #   comes next.
  fp_regex = r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?'
  case_regex = re.compile(
      r'radius\(m\)  :\s*(?P<radius>' + fp_regex + r').*?'
      r'thrust\(N\)  :\s*(?P<thrust>' + fp_regex + r').*?'
      r'power\(W\)   :\s*(?P<power>' + fp_regex + r').*?'
      r'speed\(m/s\) :\s*(?P<speed>' + fp_regex + r').*?'
      r'rpm        :\s*(?P<rpm>' + fp_regex + r').*?'
      r'Ct:\s*(?P<Ct>' + fp_regex + r').*?'
      r'Cp:\s*(?P<Cp>' + fp_regex + r')', re.DOTALL)

  # Make list of dictionaries that describe each run case.
  cases = [{k: float(v) for k, v in case.groupdict().iteritems()}
           for case in case_regex.finditer(log)]

  # Skip the first case, which is automatically run when XROTOR loads.
  return cases[1:]


def RunXrotor(rotor_file, input_str):
  """Runs XROTOR with specified conditions."""

  try:
    process = subprocess.Popen([_XROTOR_BIN, rotor_file],
                               stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE)
    log = process.communicate(input=input_str)[0]
  except subprocess.CalledProcessError as e:
    raise RuntimeError('\n\033[1m\033[91mXROTOR error: exited with return '
                       'code %d: %s' % (e.returncode, e.output))

  if FLAGS.display_log:
    print log

  # Check for common errors in the log.
  ValidateXrotorLog(log)

  # Convert the log output to an array of dictionaries of results for
  # each run case.
  return ParseXrotorLog(log)


def MakeDatabase(rotor_file, freestream_vels, angular_rates, xrotor_output,
                 corrections):
  """Collects data used in database in single dictionary."""

  def ApplyCorrections(thrust, power, corrections):
    return (thrust * corrections['thrust'][0 if thrust < 0.0 else 1],
            power * corrections['power'][0 if power < 0.0 else 1])

  def IsAtOperatingPoint(case, freestream_vel, angular_rate):
    return (numpy.abs(case['rpm'] * 2.0 * numpy.pi / 60.0 - angular_rate) < 1e-3
            and numpy.abs(case['speed'] - freestream_vel) < 1e-3)

  # TODO: Change rotor database format so the matrix rows
  # are freestream velocities and the columns are angular rates.  This
  # is more consistent with how XROTOR treats these values,
  # i.e. setting an angular rate triggers a calculation.
  nan_array = numpy.empty((len(angular_rates), len(freestream_vels)))
  nan_array[:] = numpy.nan

  database = collections.OrderedDict(
      [('name', os.path.splitext(os.path.basename(rotor_file))[0]),
       ('diameter', 2.0 * xrotor_output[0]['radius']),
       ('corrections', corrections),
       ('num_omegas', len(angular_rates)),
       ('num_v_freestreams', len(freestream_vels)),
       ('omegas', angular_rates),
       ('v_freestreams', freestream_vels),
       ('thrust_coeffs', copy.deepcopy(nan_array).tolist()),
       ('power_coeffs', copy.deepcopy(nan_array).tolist()),
       ('converged', copy.deepcopy(nan_array).tolist()),
       ('thrust', copy.deepcopy(nan_array).tolist()),
       ('power', copy.deepcopy(nan_array).tolist())])

  index = 0
  for i, freestream_vel in enumerate(freestream_vels):
    for j, angular_rate in enumerate(angular_rates):
      # Check that the case has the expected angular rate and
      # freestream velocity; otherwise, our indexing is wrong.
      assert IsAtOperatingPoint(xrotor_output[index], freestream_vel,
                                angular_rate)

      # Because of the sequence of commands we use, each converged
      # operating point is displayed twice in the XROTOR log, while
      # the unconverged operating points are only displayed once.
      if (len(xrotor_output) == index + 1
          or not IsAtOperatingPoint(xrotor_output[index + 1], freestream_vel,
                                    angular_rate)):
        database['converged'][j][i] = False
      else:
        database['converged'][j][i] = True
        index += 1

      # Define power to be positive during generation.
      ct, cp = ApplyCorrections(xrotor_output[index]['Ct'],
                                -xrotor_output[index]['Cp'],
                                corrections)
      thrust, power = ApplyCorrections(xrotor_output[index]['thrust'],
                                       -xrotor_output[index]['power'],
                                       corrections)
      database['thrust_coeffs'][j][i] = ct
      database['power_coeffs'][j][i] = cp
      database['thrust'][j][i] = thrust
      database['power'][j][i] = power
      index += 1

  return database


def MergeDatabases(databases):
  """Merges rotor databases calculated with various formulations."""
  merged_database = copy.deepcopy(databases['potential'])
  for i in range(merged_database['num_v_freestreams']):
    for j in range(merged_database['num_omegas']):
      if not merged_database['converged'][j][i]:
        for field in ['thrust_coeffs', 'power_coeffs', 'thrust', 'power']:
          merged_database[field][j][i] = (
              databases['graded_momentum'][field][j][i])
  return merged_database


def WriteJsonDatabase(filename, rotor_database):
  """Writes rotor database in JSON format."""
  with open(filename, 'w') as f:
    f.write(json.dumps(rotor_database, indent=2, separators=(',', ': ')))


def main(argv):
  """Runs XROTOR over all desired conditions and output to a JSON file."""
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n' % e
    sys.exit(1)

  logging.basicConfig(stream=sys.stdout,
                      format='%(asctime)s %(levelname)-8s %(message)s',
                      level=logging.INFO)

  databases = dict()
  for solution_type in ['potential', 'graded_momentum']:
    # Generate a sequence of commands to be piped into XROTOR.
    input_str = GetXrotorInput(FLAGS.freestream_vels, FLAGS.angular_rates,
                               solution_type)

    # Call XROTOR with input_str piped into stdin.  Parse the output.
    logging.info('Running XROTOR on %s using the %s formulation for %s cases. '
                 'This may take awhile.', FLAGS.rotor_file, solution_type,
                 str(len(FLAGS.freestream_vels) * len(FLAGS.angular_rates)))
    xrotor_output = RunXrotor(FLAGS.rotor_file, input_str)

    # Make an ordered dictionary of rotor database values such as the
    # thrust and power coefficients and the angular rates and freestream
    # velocities at which they are evaluated.
    databases[solution_type] = MakeDatabase(
        FLAGS.rotor_file, FLAGS.freestream_vels.tolist(),
        FLAGS.angular_rates.tolist(), xrotor_output,
        {'thrust': map(float, FLAGS.thrust_corrections),
         'power': map(float, FLAGS.power_corrections),
         'blade_pitch_deg': FLAGS.blade_pitch_correction_deg})

  merged_database = MergeDatabases(databases)

  # Write the rotor database into the standard JSON format.
  logging.info('Writing database to %s.', FLAGS.json_output_file)
  WriteJsonDatabase(FLAGS.json_output_file, merged_database)

  logging.shutdown()


if __name__ == '__main__':
  main(sys.argv)
