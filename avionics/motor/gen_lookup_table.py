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

"""Lookup table generator for the motor client.

This script generates a lookup table from a Python file that contains top-level
attributes t_step, t_end, and a function Cmd(t) that maps time t to a list of
motor speeds.  To run an example,

echo "
import math

t_step = 0.1
t_end = 1.0

def Cmd(t):
  torque = [math.sin(i * t) for i in range(8)]
  omega_lower = [-2 for i in range(8)]
  omega_upper = [2 for i in range(8)]
  command = torque + omega_lower + omega_upper
  return command" > /tmp/command.py
${MAKANI_HOME}/avionics/motor/gen_lookup_table.py --input_file /tmp/command.py
"""

import os
import sys

import gflags
import numpy as np

gflags.DEFINE_string('input_file', None,
                     'Name of the Python file containing the desired function '
                     'Cmd(t), which returns an array of motor speeds '
                     'corresponding to time t.')

gflags.DEFINE_bool('binary', False, 'Specifies whether output is binary.')

FLAGS = gflags.FLAGS


class LookupTableGenerator(object):
  """Builds and outputs a lookup table from a user provided function."""

  def __init__(self):
    self._cmd_func = lambda _: None
    self._cmd = None
    self._t = None
    self._t_step = None
    self._t_end = None

  def LoadFunction(self, filename):
    """Execute filename to get a function handle to evaluate.

    Try to read the Python file 'filename' looking for a function 'Cmd(t)',
    and the flags t_end and t_step.

    Args:
      filename: Python file containing the function Cmd(t) which takes time
          since the beginning of a run and outputs a list of motor speeds.

    Raises:
      ValueError: `filename` points to a valid Python file that does not contain
          necessary information.
    """

    try:
      namespace = {}
      execfile(filename, namespace)
    except Exception:
      sys.stderr.write('Invalid command Python file: %s. Raising original '
                       'exception' % filename)
      raise

    self._cmd_func = namespace.get('Cmd', None)
    if self._cmd_func is None:
      raise ValueError('Invalid command Python file: \'%s\'\n'
                       'Missing function Cmd(t).' % filename)

    self._t_step = namespace.get('t_step', None)
    if self._t_step is None:
      raise ValueError('t_step must be provided as a top-level attribute in '
                       '%s.' % filename)

    self._t_end = namespace.get('t_end', None)
    if self._t_end is None:
      raise ValueError('t_end must be provided as a top-level attribute in '
                       '%s.' % filename)

  def GenerateTable(self):
    """Generate a (time, command) lookup table.

    Execute self._cmd_func(t) at points of time in the interval
    [0, self._t_end] with a step size of self._t_step to generate an array of
    commands.
    """

    self._t = np.arange(0.0, self._t_end + np.finfo(np.float32).eps,
                        self._t_step)
    try:
      t = self._t[0]
      self._cmd = np.zeros((len(self._t), len(self._cmd_func(t))))
      for i, t in enumerate(self._t):
        self._cmd[i, :] = self._cmd_func(t)
    except Exception:
      sys.stderr.write(
          'Cmd(t) failed at t = %f. Raising original exception.\n' % t)
      raise

  def WriteTable(self, binary):
    """Output a table of time and speeds to stdout."""
    data = np.concatenate((self._t.reshape(len(self._t), 1), self._cmd),
                          axis=1)

    if binary:
      np.save(sys.stdout, data)
    else:
      print '# t_step = %f' % self._t_step
      print '# t_end = %f' % self._t_end
      header_str = ('time       ' +
                    ' '.join(['command%s    ' % i for i in
                              range(np.size(self._cmd, axis=1))]))
      np.savetxt(sys.stdout, data, fmt='%10.6f', header=header_str)


def main(argv):
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, argv[0], FLAGS)
    sys.exit(1)

  generator = LookupTableGenerator()
  generator.LoadFunction(FLAGS.input_file)
  generator.GenerateTable()
  generator.WriteTable(FLAGS.binary)


if __name__ == '__main__':
  gflags.RegisterValidator('input_file',
                           os.path.isfile,
                           '--input_file must point to a file.')
  main(sys.argv)
