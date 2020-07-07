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


"""Generates a wind turbulence database using TurbSim."""

import os
import subprocess
import sys

import gflags
import h5py
from makani.lib.python import os_util
import numpy as np
from scipy import io


gflags.DEFINE_integer('random_seed', 42,
                      'RandSeed1')
gflags.DEFINE_string('output_file', None,
                     'Destination to write to.')
gflags.DEFINE_float('time_step', 0.1,
                    'Time resolution [s].')
gflags.DEFINE_integer('num_z', 41,
                      'Number of z grid points [#].')
gflags.DEFINE_integer('num_y', 41,
                      'Number of y grid points [#].')
gflags.DEFINE_float('duration', 240.0,
                    'Duration [s].')
gflags.DEFINE_float('width', 400.0,
                    'Width [m] of the grid.')
gflags.DEFINE_float('height', 400.0,
                    'Height [m] of the grid.')
gflags.DEFINE_float('mean_wind_speed', 5.0,
                    'Mean wind speed [m/s] at reference height (80m).')
gflags.DEFINE_float('v_flow_ang', 0.0,
                    'Vertical mean flow (uptilt) angle [deg].')
gflags.DEFINE_float('wind_shear_exp', 0.2,
                    'PLExp, wind shear exponent [#].')
gflags.DEFINE_enum('turb_model', 'IECKAI',
                   ['IECKAI', 'IECVKM', 'SMOOTH', 'NONE'],
                   'Set the spectral model to use for turbulence.')
gflags.DEFINE_enum('iec_standard', '3',
                   ['1-ED3', '3'],
                   'Set the IEC standard for spectral model definition.')
# TODO: Figure out a way to also accept TI numerical inputs for turbc
gflags.DEFINE_enum('iec_turbc', 'C',
                   ['A', 'B', 'C'],
                   'Set the IEC turbulence characteristic.')
gflags.DEFINE_enum('iec_wind_type', 'NTM',
                   ['NTM', '1ETM', '2ETM', '3ETM',
                    '1EWM1', '2EWM1', '3EWM1',
                    '1EWM50', '2EWM50', '3EWM50'],
                   'Set the wind type for IEC cases.')


FLAGS = gflags.FLAGS


def main(argv):
  # Parse flags.
  try:
    argv = FLAGS(argv)
  except gflags.FlagsError, e:
    print '\nError: %s\n\nUsage: %s ARGS\n%s' % (e, sys.argv[0], FLAGS)
    sys.exit(1)

  # If the grid is wider than it is tall, place the hub-height [m] at
  # half-way up the grid to have the grid reach the ground level,
  # otherwise, increase height by delta between width and height
  # (see Figure 5 in the TurbSim manual).
  if FLAGS.width > FLAGS.height:
    hub_height = FLAGS.height / 2.0 + 0.01
  else:
    hub_height = FLAGS.height - 0.5 * FLAGS.width + 0.01

  output_file = os.path.abspath(FLAGS.output_file)
  if not os.path.exists(os.path.dirname(output_file)):
    print 'Output path points at a non-existent directory.'
    sys.exit(-1)

  print 'Reading in template...'
  script_dir = os.path.dirname(__file__)
  with open(os.path.join(script_dir, 'TurbSim.inp.template'),
            'r') as turbsim_input_template:
    turbsim_input = turbsim_input_template.read().format(
        random_seed=FLAGS.random_seed,
        num_z=FLAGS.num_z,
        num_y=FLAGS.num_y,
        time_step=FLAGS.time_step,
        duration=FLAGS.duration,
        hub_height=hub_height,
        width=FLAGS.width,
        height=FLAGS.height,
        v_flow_ang=FLAGS.v_flow_ang,
        wind_shear_exp=FLAGS.wind_shear_exp,
        turb_model=FLAGS.turb_model,
        standard=FLAGS.iec_standard,
        turbc=FLAGS.iec_turbc,
        wind_type=FLAGS.iec_wind_type,
        mean_wind_speed=FLAGS.mean_wind_speed)

  print 'Writing TurbSim input...'
  with os_util.TempDir() as turbsim_dir:
    with os_util.ChangeDir(turbsim_dir):
      input_file = os.path.join(turbsim_dir, 'TurbSim.inp')
      with open(input_file, 'w') as f:
        f.write(turbsim_input)

      print 'Running TurbSim...'
      try:
        subprocess.check_call(['wine',
                               '/opt/makani/third_party/TurbSim/TurbSim64.exe',
                               'TurbSim.inp'])
      except:
        raise Exception('Could not find 64bit version of TurbSim. '
                        'To update, run a new build or try:\n'
                        '  cd /opt/makani/third_party && git fetch && '
                        ' git checkout origin/master')
      wnd_file = os.path.join(turbsim_dir, 'TurbSim.wnd')
      mat_file = os.path.join(turbsim_dir, 'TurbSim.mat')

    print 'Post-processing with Octave...'
    with os_util.ChangeDir(script_dir):
      subprocess.check_call([
          'octave', '--eval',
          'convert_turbsim(\'%s\', \'%s\');' % (wnd_file, mat_file)])

    print 'Generating HDF5 file...'
    mat_contents = io.loadmat(mat_file)
    num_t = mat_contents['velocity'].shape[0]
    num_y = mat_contents['velocity'].shape[2]
    num_z = mat_contents['velocity'].shape[3]
    sz = num_t * num_y * num_z

    u = np.reshape(mat_contents['velocity'][:, 0, :, :], (sz,))
    v = np.reshape(mat_contents['velocity'][:, 1, :, :], (sz,))
    w = np.reshape(mat_contents['velocity'][:, 2, :, :], (sz,))

    f = h5py.File(output_file, 'w')
    f.create_dataset('num_t', data=np.array([num_t]), dtype='<i4')
    f.create_dataset('num_y', data=np.array([num_y]), dtype='<i4')
    f.create_dataset('num_z', data=np.array([num_z]), dtype='<i4')
    f.create_dataset('mean_wind_speed', data=np.array([FLAGS.mean_wind_speed]))
    f.create_dataset('duration', data=np.array([FLAGS.time_step * (num_t - 1)]))
    f.create_dataset('width', data=np.array([FLAGS.width]))
    f.create_dataset('height', data=np.array([FLAGS.height]))
    f.create_dataset('u', data=u, compression='gzip', compression_opts=5)
    f.create_dataset('v', data=v, compression='gzip', compression_opts=5)
    f.create_dataset('w', data=w, compression='gzip', compression_opts=5)
    f.close()


if __name__ == '__main__':
  main(sys.argv)
