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

r"""Produce transfer functions from ServoStatusMessages in an h5 log file.

Example usage.  First, generate a logfile where a servo is commanded
at a broad range of frequencies.  One way to do this is to generate a
filtered random sequence and apply this as a servo command using the
servo client:

  ./make_random_waveform.py --mean 0.0 \
                            --rms 10.0 \
                            --columns 1 \
                            --cutoff_frequency 1.0 \
                            --duration 100.0 \
                            --use_motor_client_format=false \
                            --output_file servo_waveform.txt

  ${MAKANI_HOME}/lib/scripts/developer/servo_client
  (servo_client) move_file servo_waveform.txt

Finally, process the data using this program:

  ./servo_sys_id.py --servo_name E1 ${MAKANI_HOME}/logs/mylog.h5

By default, output files will be created in the current directory.
Command line arguments are available to change this default.

"""

# TODO: Merge the functionality of this script with sys_id.py.

# TODO: Add plots of some time-domain data, such as the
# original timeseries, and histograms of commanded and measured
# values.
import os
import sys

import gflags
import h5py
from makani.analysis.sys_id import sys_id_util
from makani.lib.python import dict_util
import numpy as np
import pylab


gflags.DEFINE_string('output_dir', '.',
                     'Directory into which output files will be written.')
gflags.DEFINE_boolean('save', True,
                      'Whether to save the plots into PNG and PDF files.')
gflags.DEFINE_boolean('interactive', False,
                      'Whether to display the plots interactively.')
gflags.DEFINE_integer('nfft', 1024,
                      'Number of points at which to compute the Fourier '
                      'transform.')
gflags.DEFINE_string('basename', '',
                     'Base filename for output files.')
gflags.DEFINE_float('min_coherence', 0.8,
                    'Minimum coherence required to include data point on '
                    'transfer function plot.')
gflags.DEFINE_enum('detrend', 'mean', ['none', 'mean', 'linear'],
                   'Detrending algorithm to be applied before taking the '
                   'Fourier transform.')
gflags.DEFINE_boolean('do_unwrap', True, 'Whether phase should be unwrapped.')
gflags.DEFINE_float('settling_time', 5.0,
                    'Duration [s] to skip at beginning of time series.')
gflags.DEFINE_float('shutdown_time', 5.0,
                    'Duration [s] to skip at end of time series.')
gflags.DEFINE_string('servo_name', None, 'Name of servo to process.')
gflags.MarkFlagAsRequired('servo_name')

FLAGS = gflags.FLAGS


def main(argv):
  def PrintUsage(argv, error=None):
    if error:
      print '\nError: %s\n' % error
    print 'Usage: %s --servo_name E1 logfile.h5 [logfile2.h5 ...]\n%s' % (
        argv[0], FLAGS)

  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    PrintUsage(argv, e)
    sys.exit(1)

  if len(argv) <= 1:
    PrintUsage(argv, 'Must specify at least one logfile.')
    sys.exit(1)

  # Create the desired output directory if it doesn't already exist.
  output_dir = FLAGS.output_dir
  if not os.path.isdir(output_dir):
    os.mkdir(output_dir)

  fs = 100.0

  plot_filename = os.path.join(output_dir,
                               (FLAGS.basename + '_' if FLAGS.basename
                                else '') + 'servo_' + FLAGS.servo_name)

  # Process the servo position data.
  servo_message_path = ['messages', 'kAioNodeServo' + FLAGS.servo_name,
                        'kMessageTypeServoStatus', 'message']
  servo_cmd_path = servo_message_path + ['angle_desired']
  servo_val_path = servo_message_path + ['angle_estimate']

  axes = None
  txy_collected = None
  coh_collected = None

  for filename in argv[1:]:
    print 'Reading %s...' % filename

    log_name = os.path.basename(filename)
    log_name = os.path.splitext(log_name)[0]

    try:
      log = h5py.File(filename, 'r')
    except IOError, e:
      print 'Error: Could not open h5 log file "%s".\n' % filename
      raise e

    servo_cmd = dict_util.GetByPath(log, servo_cmd_path)
    servo_val = dict_util.GetByPath(log, servo_val_path)

    servo_cmd = servo_cmd.reshape(len(servo_cmd), 1)
    servo_val = servo_val.reshape(len(servo_val), 1)

    axes, f, txy, coh = sys_id_util.PlotTransferFunction(
        servo_cmd, servo_val, [log_name], fs, axes,
        settling_time=FLAGS.settling_time,
        shutdown_time=FLAGS.shutdown_time,
        nfft=FLAGS.nfft,
        detrend=FLAGS.detrend,
        min_coherence=FLAGS.min_coherence,
        do_unwrap=FLAGS.do_unwrap)

    # Reshape into 2D arrays.
    txy = txy[np.newaxis, :]
    coh = coh[np.newaxis, :]

    if txy_collected is None:
      txy_collected = txy
    else:
      txy_collected = np.vstack((txy_collected, txy))

    if coh_collected is None:
      coh_collected = coh
    else:
      coh_collected = np.vstack((coh_collected, coh))

  sys_id_util.SetAxes(axes)
  pylab.legend(fontsize=10, loc='best')

  # Turn the matrix of transfer functions into interleaved rows of
  # real and imaginary parts and coherence data.
  tf_data = np.empty((0, txy_collected.shape[1]))
  for row in range(txy_collected.shape[0]):
    tf_data = np.vstack((tf_data,
                         np.real(txy_collected[row, :]),
                         np.imag(txy_collected[row, :]),
                         coh_collected[row, :]))

  if FLAGS.save:
    print 'Writing to %s.pdf' % plot_filename
    pylab.savefig(plot_filename + '.pdf')
    print 'Writing to %s.png' % plot_filename
    pylab.savefig(plot_filename + '.png')
    print 'Writing to %s.txt' % plot_filename
    np.savetxt(plot_filename + '.txt', np.vstack((f, tf_data)).T)

  if FLAGS.interactive:
    pylab.show()
  else:
    pylab.close()


if __name__ == '__main__':
  main(sys.argv)
