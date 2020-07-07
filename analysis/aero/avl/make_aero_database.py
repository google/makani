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

r"""Runs AVL and writes a JSON file with aerodynamic stability derivatives.

This file runs AVL (http://web.mit.edu/drela/Public/web/avl/) to generate
stability derivatives over a tensor grid of alpha, beta, and control surface
deflections. The results are then output as a JSON file to be used at a later
time by the controls simulator.

Example Usage:
  [from the command line]
  ./make_aero_database.py --alpha=-2,2 --beta=-2,0,2 --delta=0 \
    --avl_output_file=tmp.out

A header is included in the JSON file with the entries:
 - num_alphas:  }
 - num_betas:   } The number of alpha, beta, and delta sample points used in the
 - num_deltas:  } tensor grid
 - alpha:  }
 - beta:   } The values of alpha, beta, and delta used to define the grid
 - delta:  } points.
 - flap_list:  The mask of which control surfaces were deflected by delta.
 - omega_hat:  The nondimensional rotational velocity
               (p*b/2*V, q*c/2*V, r*b/2*V)

The rest of the results are stored as fields of the form:

"field":
 [[[val,   val,   ...]           alpha_1   }
    ...                          ...       } delta_1
   [val,   val,   ...]]          alpha_n   }

   ...

  [[val,   val,   ...]           alpha_1   }
    ...                          ...       } delta_m
   [val,   val,   ...]]]         alpha_n   }

  beta_1  beta_2  ...

Presently, fields matching CLtot, CDtot, de1 (elevator deflection),
C[XYZlmn]tot, C[XYZlmn][pqr], and C[XYZlmn]d[1-8] are stored in the JSON file.
Note that C[XYZlmn][uvw] are not included in the present version along with a
number of other unused parameters output by AVL.
"""

import collections
import json
import os
import re
import shutil
import string
import subprocess
import sys
import tempfile

import gflags
import makani
from makani.lib.python import os_util
import numpy

gflags.DEFINE_string('avl_bin',
                     os.path.join(makani.HOME, 'third_party/avl/avl'),
                     'Path to the AVL binary.')
gflags.RegisterValidator('avl_bin', os.path.exists, '--avl_bin must exist')

gflags.DEFINE_string('avl_surface_file', None,
                     'Name of AVL input file where aerodynamic surfaces are '
                     'defined.')
gflags.MarkFlagAsRequired('avl_surface_file')
gflags.RegisterValidator('avl_surface_file', os.path.exists,
                         '--avl_surface_file must exist')

gflags.DEFINE_string('json_output_file', None,
                     'Name of output JSON file.')
gflags.RegisterValidator('json_output_file',
                         lambda f: f and os.path.basename(f),
                         '--json_output_file mush have a valid file name.')

gflags.DEFINE_string('avl_input_file', None,
                     'Name of AVL input file. Leave this empty to directly '
                     'pass the input arguments into AVL without creating an '
                     'intermediate file.')

_TEMP_DIR = os.path.join(makani.HOME, 'tmp/avl')
gflags.DEFINE_string('avl_output_file',
                     os.path.join(_TEMP_DIR, 'avl_output_file.txt'),
                     'Name of intermediate file for AVL to write results into.')
gflags.RegisterValidator('avl_output_file',
                         lambda f: f and os.path.basename(f),
                         '--avl_output_file mush have a valid file name.')

gflags.DEFINE_string('run_log_file',
                     os.path.join(_TEMP_DIR, 'avl.log'),
                     'Name of log file with which to record standard output.')
gflags.RegisterValidator('run_log_file',
                         lambda f: f and os.path.basename(f),
                         '--run_log_file must have a valid file name')

gflags.DEFINE_list('alphads', numpy.linspace(-10, 10, num=5).tolist(),
                   'List of angles [deg] of attack to run AVL at.')

gflags.DEFINE_list('betads', numpy.linspace(-15, 15, num=7).tolist(),
                   'List of side slip angles [deg] to run AVL at.')

# TODO: Support multiple deflection angles in the simulator.
gflags.DEFINE_list('deltads', [0.0],
                   'List of control surface deflection angles [deg] to run AVL '
                   'at.')

gflags.DEFINE_list('flap_list', [1, 1, 1, 1, 1, 1, 0, 0],
                   'List of control surfaces to deflect by delta_range.')
gflags.RegisterValidator(
    'flap_list', lambda mask: all([str(r) in ('0', '1') for r in mask]),
    '--flap_list must have integer elements of 0 or 1 with flap_list[i] '
    'corresponding to control surface i. In the current m600 avl file, the '
    '7th element corresponds to the elevator and the 8th index to the rudder.')

gflags.DEFINE_list('rotation_vector',
                   [0.0, 0.002213938111806962, -0.1652570298670424],
                   'Nominal nondimensional rotation rate of the wing in the '
                   'body coordinate system, i.e. (p*b/2*V, q*c/2*V, r*b/2*V).')
gflags.RegisterValidator(
    'rotation_vector', lambda r: len(r) == 3,
    '--rotation_vector must have exactly 3 elements.')

FLAGS = gflags.FLAGS


def GetAvlInput(alpha, beta, delta_flap, omega_hat, avl_output_file):
  """Get an ASCII input file for AVL."""

  # AVL is typically operated out of the command line. To automate this process,
  # the following string is piped through stdin to AVL to configure and start
  # the solver. Configuration of run cases is done using the pattern:
  #
  #   variable constraint value
  #
  # That is, choose variable such that constraint == value. For most of the
  # following parameters, the constraint and variable are the same, e.g. choose
  # alpha such that alpha == value.
  template = string.Template(
      'oper\n'  # Switch mode to compute operating-point run cases.
      'O\n'  # Switch mode to options.
      'r\n'  # Switch angular rate axis system to body coordinates.
      '\n'  # Return to operating-point menu.
      'a a $alp\n'  # Set the angle of attack, [a]lpha.
      'b b $bet\n'  # Set the side slip angle, [b]eta.
      'd7 pm 0\n'  # Choose elevator deflection such that pitching moment == 0.
      'd1 d1 $df1 \n'  # Set flap deflection 1 to delta_flap[0].
      'd2 d2 $df2 \n'  # Set flap deflection 2 to delta_flap[1].
      'd3 d3 $df3 \n'  # ...
      'd4 d4 $df4 \n'
      'd5 d5 $df5 \n'
      'd6 d6 $df6 \n'
      'd8 d8 $df8 \n'  # Set rudder deflection to delta_flap[7].
      'r r $roll_rate \n'  # Set the non-dimensional [r]oll rate pb/2V.
      'p p $pitch_rate \n'  # Set the non-dimensional [p]itch rate qc/2V.
      'y y $yaw_rate \n'  # Set the non-dimensional [y]aw rate rb/2V.
      'x\n'  # Start or e[x]ecute the simulation.
      'sb $output_file\n'  # Calculate [s]tability derivatives in [b]ody
                           # coordinates. Output to output_file,
      'o\n'                # [o]verwriting the file if necessary.
      '\n\nq\n')           # [q]uit AVL.
      # Note that a second \n before q is necessary to handle the case where
      # output_file does not exist such that AVL never prompts whether to
      # overwrite the file. In this case, 'o' is interpreted as [o]ption and
      # requires an extra new line to return from.

  # Fill in the parameters in the string and return.
  return template.substitute(alp=alpha,
                             bet=beta,
                             df1=delta_flap[0],
                             df2=delta_flap[1],
                             df3=delta_flap[2],
                             df4=delta_flap[3],
                             df5=delta_flap[4],
                             df6=delta_flap[5],
                             df8=delta_flap[7],
                             roll_rate=omega_hat[0],
                             pitch_rate=omega_hat[1],
                             yaw_rate=omega_hat[2],
                             output_file=avl_output_file)


def RunAvl(alpha, beta, delta_flap, omega, avl_files):
  """Run AVL with specified conditions."""

  # Use a temporary file for the output, which will be copied to the desired
  # location. AVL chokes when a path length exceeds 80 characters, which is a
  # common occurrence under Bazel.
  with tempfile.NamedTemporaryFile(suffix='.txt') as avl_output:
    # Generate a string with a sequence of commands to be piped into AVL.
    input_str = tempfile.TemporaryFile(mode='w+b')
    input_str.write(GetAvlInput(alpha, beta, delta_flap, omega,
                                avl_output.name))
    input_str.seek(0)

    # For debugging purposes and comparison with previous Matlab functions,
    # write the AVL input to a file.
    if avl_files['input'] is not None:
      with open(avl_files['input'], 'w') as f:
        f.write(input_str.read())
        input_str.seek(0)

    # Call AVL and have it solve for stability derivatives.
    surface_basename = os.path.basename(avl_files['surface'])
    surface_dir = os.path.dirname(avl_files['surface'])
    with open(avl_files['log'], 'w') as log, os_util.ChangeDir(surface_dir):
      popen = subprocess.Popen([FLAGS.avl_bin, surface_basename],
                               stdin=input_str,
                               stdout=log,
                               stderr=subprocess.PIPE)
      popen.wait()

    # Check whether AVL had trouble locating any data files. It happily charges
    # ahead with default data if that happens.
    with open(avl_files['log'], 'r') as log:
      error_regex = re.compile(
          r'.*(Airfoil file not found|Mass file.*open error).*')
      line_number = 0
      for line in log:
        line_number += 1
        if error_regex.match(line):
          raise RuntimeError('AVL error: "%s"\nSee %s:%d for more information.'
                             % (line.strip(), avl_files['log'], line_number))

    shutil.copyfile(avl_output.name, avl_files['output'])

  # Check for a failed run. "Killed" came up fairly frequently while debugging
  # due to a lack of memory.
  if popen.returncode != 0:
    if 'Killed' in avl_output.stderr.read():
      raise RuntimeError('Warning: AVL exited abnormally.',
                         ' Possible causes include:\n'
                         ' - insufficient system memory\n')
    raise RuntimeError('Warning: AVL exited abnormally.')

  with open(avl_files['output'], 'r+') as f:
    file_contents = f.read()

  return re.findall(r'(\S+)\s*=\s*([-+]?\d+\.\d*)', file_contents)


def ReshapeForOutput(avl_list):
  """Reorders the lists in avl_list to group stability derivatives together."""
  # Preallocate useful dimensions and lists.
  num_alphas = len(avl_list)
  num_betas = len(avl_list[0])
  num_deltas = len(avl_list[0][0])
  num_fields = len(avl_list[0][0][0])

  output_value = numpy.zeros((num_fields, num_deltas, num_alphas, num_betas))
  output_label = [''] * num_fields

  # Loop over elements in avl_list and rearrange them so that the first index is
  # the field number. For the field name, we'll reduce the array to a single
  # dimension and make sure that all labels corresponding to the same field are
  # actually the same.
  for i_field in range(num_fields):
    for i_alpha in range(num_alphas):
      for i_beta  in range(num_betas):
        for i_delta in range(num_deltas):
          output_value[i_field, i_delta, i_alpha, i_beta] = float(
              avl_list[i_alpha][i_beta][i_delta][i_field][1])

          if i_alpha == 0 and i_beta == 0 and i_delta == 0:
            # This is the first time we have reached this particular label, so
            # copy it into the output list.
            output_label[i_field] = (
                avl_list[i_alpha][i_beta][i_delta][i_field][0])
          else:
            # There should already be a label in output_label and it had better
            # be the same as all the other labels corresponding to that field
            # index otherwise a more complicated scheme needs to be implemented
            # to match keys.
            assert (output_label[i_field] ==
                    avl_list[i_alpha][i_beta][i_delta][i_field][0])
  return output_label, output_value.tolist()


def WriteJsonFile(alpha, beta, delta, omega_hat, flap_list,
                  label, value, filename):
  """Write a modified JSON file containing stability derivatives."""
  # The following are the fields to export to the simulator. Note that C*u, C*v,
  # and C*w (where * are wild cards) are not included.
  included_fields = ['CLtot', 'CDtot', 'de1',
                     'CXtot', 'CYtot', 'CZtot', 'Cltot', 'Cmtot', 'Cntot',
                     'CXp', 'CXq', 'CXr', 'Clp', 'Clq', 'Clr',
                     'CYp', 'CYq', 'CYr', 'Cmp', 'Cmq', 'Cmr',
                     'CZp', 'CZq', 'CZr', 'Cnp', 'Cnq', 'Cnr',
                     'CXd1', 'CYd1', 'CZd1', 'Cld1', 'Cmd1', 'Cnd1',
                     'CXd2', 'CYd2', 'CZd2', 'Cld2', 'Cmd2', 'Cnd2',
                     'CXd3', 'CYd3', 'CZd3', 'Cld3', 'Cmd3', 'Cnd3',
                     'CXd4', 'CYd4', 'CZd4', 'Cld4', 'Cmd4', 'Cnd4',
                     'CXd5', 'CYd5', 'CZd5', 'Cld5', 'Cmd5', 'Cnd5',
                     'CXd6', 'CYd6', 'CZd6', 'Cld6', 'Cmd6', 'Cnd6',
                     'CXd7', 'CYd7', 'CZd7', 'Cld7', 'Cmd7', 'Cnd7',
                     'CXd8', 'CYd8', 'CZd8', 'Cld8', 'Cmd8', 'Cnd8']
  included_params = ['Sref', 'Cref', 'Bref', 'Mach']

  label_indices = {}
  for i, label_i in enumerate(label):
    if label_i not in label_indices:
      label_indices[label_i] = i
  param_indices = [label_indices.get(p, []) for p in included_params]
  field_indices = [label_indices.get(f, []) for f in included_fields]

  # Condense constant parameters down to a single scalar value.
  for i_field in param_indices:
    tmp = numpy.array(value[i_field])
    assert numpy.all(tmp == tmp[(0,) * tmp.ndim])
    value[i_field] = tmp[(0,) * tmp.ndim]

  # Combine all the field and heading labels together to create the key for the
  # OrderedDict to come. Also do the same for values.
  keys = (['num_alphas', 'num_betas', 'num_deltas',
           'alphads', 'betads', 'deltads', 'flap_list', 'omega_hat']
          + [label[i] for i in param_indices]
          + [label[i] for i in field_indices])
  vals = ([len(alpha), len(beta), len(delta),
           alpha, beta, delta, flap_list, omega_hat]
          + [value[i] for i in param_indices]
          + [value[i] for i in field_indices])
  output_dict = collections.OrderedDict(zip(keys, vals))

  # Create a string from the standard Python JSON converter. However, the
  # converter is somewhat limited with respect to formatting lists. To get
  # around this, the lists are formatted manually with a series of passes by
  # replace.
  output_string = json.dumps(output_dict, separators=(', ', ':\n  '))
  output_string = (output_string
                   .replace(', \"', ',\n\"')
                   .replace('], [', '],\n   [')
                   .replace(' [[', '[[')
                   .replace('{', '{\n')
                   .replace('}', '\n}')) + '\n'

  filename_dir = os.path.dirname(filename)
  if filename_dir and not os.path.exists(filename_dir):
    os.makedirs(filename_dir)
  with open(filename, 'w') as output_file:
    output_file.write(output_string)


def main(argv):
  """Run AVL over all desired conditions and output to a JSON file."""
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  # Setup a directory in MAKANI_HOME/tmp to put the avl output file in.
  # Grab the path for the avl_output_file in case it has been overridden.
  temp_output_dir = os.path.dirname(FLAGS.avl_output_file)
  if temp_output_dir and not os.path.exists(temp_output_dir):
    os.makedirs(temp_output_dir)
  if os.path.isfile(FLAGS.avl_output_file):
    os.remove(FLAGS.avl_output_file)  # Clear the previous output if it exists.

  log_dir = os.path.dirname(FLAGS.run_log_file)
  if log_dir and not os.path.exists(log_dir):
    os.makedirs(log_dir)
  if os.path.isfile(FLAGS.run_log_file):
    os.remove(FLAGS.run_log_file)  # Clear the previous log if it exists.

  avl_files = {'surface': FLAGS.avl_surface_file,
               'input': FLAGS.avl_input_file,
               'output': FLAGS.avl_output_file,
               'log': FLAGS.run_log_file}

  print 'Using %s as input to AVL.' % avl_files['surface']
  print 'Logging AVL output to %s.' % avl_files['log']
  print 'Reducing results into the JSON file: %s.\n' % FLAGS.json_output_file

  # Convert to floats - DEFINE_list contains a list of strings when taking input
  # from the command line.
  omega_hat = [float(x) for x in FLAGS.rotation_vector]
  alpha_range = [float(x) for x in FLAGS.alphads]
  beta_range = [float(x) for x in FLAGS.betads]
  delta_range = [float(x) for x in FLAGS.deltads]
  flap_list = [int(x) for x in FLAGS.flap_list]

  num_alphas = len(alpha_range)
  num_betas = len(beta_range)
  num_deltas = len(delta_range)

  # Loop through each (alpha, beta, delta) configuration and run AVL.
  avl_output = [[[[] for i_delta in xrange(num_deltas)]
                 for i_beta in xrange(num_betas)]
                for i_alpha in xrange(num_alphas)]
  for i_alpha in range(num_alphas):
    for i_beta in range(num_betas):
      for i_delta in range(num_deltas):
        print ('Running AVL with '
               'alpha_{0} = {1:6.2f}, beta_{2} = {3:6.2f}, delta_{4} = {5:6.2f}'
               .format(repr(i_alpha).ljust(2), alpha_range[i_alpha],
                       repr(i_beta).ljust(2), beta_range[i_beta],
                       repr(i_delta).ljust(2), delta_range[i_delta]))
        output = RunAvl(alpha_range[i_alpha], beta_range[i_beta],
                        [x*delta_range[i_delta] for x in flap_list],
                        omega_hat, avl_files)
        avl_output[i_alpha][i_beta][i_delta] = output

  # Reformat and write to a JSON file.
  avl_label, avl_value = ReshapeForOutput(avl_output)
  WriteJsonFile(alpha_range, beta_range, delta_range, omega_hat, flap_list,
                avl_label, avl_value, FLAGS.json_output_file)


if __name__ == '__main__':
  main(sys.argv)
